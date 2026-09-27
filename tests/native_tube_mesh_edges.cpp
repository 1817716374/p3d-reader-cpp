#include "native_tube_mesh_edges.hpp"
#include <p3d/polyface.hpp>
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
TubeMeshRegularVertices marked(double base, std::size_t nu = 2, std::size_t nv = 2) {
    TubeMeshRegularVertices g{nu, nv, {}};
    for (std::size_t j = 0; j < nv; ++j)
        for (std::size_t i = 0; i < nu; ++i) {
            double x = base + j * nu + i;
            g.vertices.push_back({{x, 0, 0}, {0, x, 1}, {x, -x}, false});
        }
    return g;
}
std::vector<double> ids(const std::vector<Point3> &points) {
    std::vector<double> v;
    for (auto p : points)
        v.push_back(p[0]);
    return v;
}
Json snapshot(const TubeMeshEdgeState &s) {
    return {{"rows", s.rows},
            {"column", s.column},
            {"profile", s.profile_seam},
            {"path", s.path_seam},
            {"start", s.start_points},
            {"end", s.end_points},
            {"next", {s.next_patch, s.next_strip}}};
}
Json surface() {
    return {{"_type", "BsplineSurface"},
            {"orderU", 2},
            {"orderV", 2},
            {"numPolesU", 2},
            {"numPolesV", 2},
            {"closedU", false},
            {"closedV", false},
            {"poles", {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0}},
            {"weights", nullptr},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"numRulesU", 0},
            {"numRulesV", 0},
            {"boundaries", nullptr},
            {"holeOrigin", 0}};
}
template <class T> Json flatten(const std::vector<T> &values) {
    Json out = Json::array();
    for (auto v : values)
        for (auto x : v)
            out.push_back(x);
    return out;
}
} // namespace
unsigned native_tube_mesh_edges_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool threw = false;
        try {
            fn();
        } catch (const std::exception &) {
            threw = true;
        }
        check(threw, "native mesh edge invalid state/budget rejects");
    };
    for (bool pc : {false, true})
        for (bool vc : {false, true}) {
            TubeBudget b;
            auto state = make_tube_mesh_edge_state(2, 2, pc, vc, b);
            std::vector<TubeMeshRegularMesh> meshes;
            for (std::size_t pi = 0; pi < 2; ++pi)
                for (std::size_t si = 0; si < 2; ++si) {
                    auto source = marked(100 * pi + 10 * si);
                    auto m = connect_tube_mesh_regular_vertices(
                        source, state, {true, true, pi == 0 && !vc, pi == 1 && !vc}, b);
                    check(
                        m.normal_indices == m.point_indices &&
                            m.parameter_indices == m.point_indices,
                        "normal and parameter signed index lists copy original coordinate indices");
                    for (std::size_t i = 0; i < 4; ++i)
                        check(m.normals[i] == source.vertices[i].normal &&
                                  m.parameters[i] == source.vertices[i].parameter,
                              "position replacement never inherits neighbour normal or parameter");
                    check(m.point_indices == std::vector<std::int32_t>{1, 2, 3, 0, 2, 4, 3, 0},
                          "native conversion keeps exact two triangle order and positive edge "
                          "visibility");
                    meshes.push_back(std::move(m));
                }
            check(ids(meshes[0].points) == std::vector<double>{0, 1, 2, 3},
                  "first open tile stores raw positions");
            check(ids(meshes[1].points) ==
                      (pc ? std::vector<double>{1, 0, 3, 2} : std::vector<double>{1, 11, 3, 13}),
                  "next strip inherits preceding column and optional profile closure");
            check(ids(meshes[2].points) ==
                      (vc ? std::vector<double>{2, 3, 0, 1} : std::vector<double>{2, 3, 102, 103}),
                  "next patch inherits prior row and optional path closure");
            const auto last = pc ? (vc ? std::vector<double>{3, 2, 1, 102, 11}
                                       : std::vector<double>{3, 2, 103, 102})
                                 : (vc ? std::vector<double>{3, 13, 1, 11}
                                       : std::vector<double>{3, 13, 103, 113});
            check(ids(meshes[3].points) == last,
                  "four-way seam priority preserves native double-closed append");
            check(
                meshes[3].report["unused_tail_coordinates"] == std::size_t(pc && vc) &&
                    meshes[3].normals.size() == 4 && meshes[3].parameters.size() == 4,
                "double closure keeps extra original coordinate and independent attribute counts");
            check(state.next_patch == 2 && state.next_strip == 0,
                  "successful calls advance source patch then strip positions");
            if (!vc) {
                check(
                    ids(state.start_points) ==
                        (pc ? std::vector<double>{0, 1, 1, 0} : std::vector<double>{0, 1, 1, 11}),
                    "start boundary collection retains repeated strip corners and profile closure");
                check(ids(state.end_points) == (pc ? std::vector<double>{102, 103, 103, 102}
                                                   : std::vector<double>{102, 103, 103, 113}),
                      "end boundary uses original point correspondence without creating caps");
            }
            const auto before = snapshot(state);
            rejects([&] { connect_tube_mesh_regular_vertices(marked(999), state, {}, b); });
            check(snapshot(state) == before, "completed state cannot be reused silently");
        }
    // V parameter counts may vary between patches; each prior row still has
    // the original U width. Columns stay local to a patch's shared V list.
    TubeBudget b;
    auto varying = make_tube_mesh_edge_state(2, 2, false, false, b);
    connect_tube_mesh_regular_vertices(marked(0, 3, 2), varying, {}, b);
    connect_tube_mesh_regular_vertices(marked(10, 3, 2), varying, {}, b);
    auto a = connect_tube_mesh_regular_vertices(marked(100, 3, 4), varying, {}, b);
    auto c = connect_tube_mesh_regular_vertices(marked(200, 3, 4), varying, {}, b);
    check(ids(a.points) ==
              std::vector<double>{3, 4, 5, 103, 104, 105, 106, 107, 108, 109, 110, 111},
          "new V row count preserves three original U coordinates from previous patch");
    check(ids(c.points) ==
              std::vector<double>{5, 14, 15, 105, 204, 205, 108, 207, 208, 111, 210, 211},
          "new shared V list governs adjacent columns within the current patch");
    check(c.point_indices.size() == 48 && c.report["triangles"] == 12,
          "rectangular index count derives from complete coordinate rows");
    for (bool pc : {false, true})
        for (bool vc : {false, true}) {
            TubeBudget one;
            auto state = make_tube_mesh_edge_state(1, 1, pc, vc, one);
            auto m = connect_tube_mesh_regular_vertices(marked(0), state, {}, one);
            const auto expected =
                pc ? (vc ? std::vector<double>{0, 0, 0, 2, 1} : std::vector<double>{0, 0, 2, 2})
                   : (vc ? std::vector<double>{0, 1, 0, 1} : std::vector<double>{0, 1, 2, 3});
            check(ids(m.points) == expected, "single tile keeps native self closure ordering");
        }
    // Real preparation -> source evaluation -> connection -> public Polyface
    // decoding. The actual library's public consumer verifies usable channels.
    SweptBodyPatchGroup group;
    group.patches = {{surface(), {}}};
    TubeBudget real;
    auto prepared = prepare_tube_mesh_grid(group, false, .01, .1, 10000, real);
    check(prepared.success, "planar surface prepares original grid parameters");
    auto nodes =
        evaluate_tube_mesh_regular_vertices(prepared.patches[0], prepared.section, 0, 0, 1, real);
    auto state = make_tube_mesh_edge_state(1, 1, false, false, real);
    auto m = connect_tube_mesh_regular_vertices(nodes, state, {}, real);
    auto decoded = mesh_bgfb_polyface({{"_type", "Polyface"},
                                       {"meshStyle", 1},
                                       {"numPerFace", 0},
                                       {"point", flatten(m.points)},
                                       {"pointIndex", m.point_indices},
                                       {"normal", flatten(m.normals)},
                                       {"normalIndex", m.normal_indices},
                                       {"param", flatten(m.parameters)},
                                       {"paramIndex", m.parameter_indices}});
    check(decoded.status == "meshed" && decoded.geometry.vertices == m.points &&
              decoded.geometry.faces == std::vector<Triangle>{{0, 1, 2}, {1, 3, 2}},
          "public Polyface consumer preserves native connected strip faces and point pool");
    for (std::size_t i = 0; i < 2; ++i) {
        check(decoded.geometry.face_normal_indices[i] ==
                      std::optional<Triangle>(decoded.geometry.faces[i]) &&
                  decoded.geometry.face_uv_indices[i] ==
                      std::optional<Triangle>(decoded.geometry.faces[i]),
              "public normal and UV index binding matches original face order");
    }
    for (const auto &edge : decoded.source_edges)
        check(edge.visible, "all native triangle-grid edges including diagonal are visible");
    TubeBudget empty;
    auto no_channels = make_tube_mesh_edge_state(1, 1, false, false, empty);
    auto none = connect_tube_mesh_regular_vertices(marked(0), no_channels,
                                                   {false, false, false, false}, empty);
    check(none.normals.empty() && none.parameters.empty() && none.normal_indices.empty() &&
              none.parameter_indices.empty() && none.point_indices.size() == 8,
          "optional channels remain absent");
    TubeBudget invalid_budget;
    auto invalid = make_tube_mesh_edge_state(2, 1, false, false, invalid_budget);
    invalid.next_patch = 1;
    const auto before = snapshot(invalid);
    rejects([&] { connect_tube_mesh_regular_vertices(marked(4), invalid, {}, invalid_budget); });
    check(snapshot(invalid) == before, "missing predecessor fails without changing caller state");
    TubeBudget low;
    auto rollback = make_tube_mesh_edge_state(1, 1, false, false, low);
    const auto rollback_before = snapshot(rollback);
    low.max_control_points = 7; // vertices/cache fit; eight index entries do not.
    rejects([&] { connect_tube_mesh_regular_vertices(marked(0), rollback, {}, low); });
    check(snapshot(rollback) == rollback_before,
          "late index-budget failure rolls back cached edge mutations");
    TubeBudget closed_budget;
    auto closed_state = make_tube_mesh_edge_state(1, 1, true, true, closed_budget);
    const auto closed_before = snapshot(closed_state);
    auto bad_normal = marked(0);
    bad_normal.vertices.back().normal[0] = std::numeric_limits<double>::quiet_NaN();
    rejects([&] {
        connect_tube_mesh_regular_vertices(bad_normal, closed_state, {true, true, true, true},
                                           closed_budget);
    });
    check(snapshot(closed_state) == closed_before,
          "late attribute failure restores both seam vectors and optional boundary collections");
    rejects([&] {
        TubeBudget x;
        x.max_work = 0;
        make_tube_mesh_edge_state(1, 1, false, false, x);
    });
    rejects([&] {
        TubeBudget x;
        make_tube_mesh_edge_state(0, 1, false, false, x);
    });
    auto work = []() {
        TubeBudget x;
        auto s = make_tube_mesh_edge_state(1, 1, false, false, x);
        return connect_tube_mesh_regular_vertices(marked(9), s, {}, x).points;
    };
    auto async = std::async(std::launch::async, work);
    check(async.get() == work(), "parallel mesh connection owns independent caches");
    return n;
}
