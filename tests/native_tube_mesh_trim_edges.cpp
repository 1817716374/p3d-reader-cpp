#include "native_tube_mesh_trim_edges.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
TubeMeshTrimVertices marked(double base,
                            std::initializer_list<std::pair<int, int>> ranges = {{1, 0}, {1, 0}}) {
    TubeMeshTrimVertices g;
    g.plan.success = true;
    for (auto [l, u] : ranges) {
        TubeMeshTrimColumn c;
        c.source_u = c.local_u = double(g.plan.columns.size());
        c.first_interior = l;
        c.last_interior = u;
        c.lower = double(l - 1) / 10;
        c.upper = double(u + 1) / 10;
        c.count = std::size_t(std::max(0, u - l + 3));
        c.offset = g.plan.vertex_count;
        g.plan.columns.push_back(c);
        for (std::size_t j = 0; j < c.count; ++j) {
            double id = base + 10 * double(j) + c.source_u;
            g.vertices.push_back({std::int32_t(l - 1 + int(j)),
                                  double(l - 1 + int(j)) / 10,
                                  {id, 0, 0},
                                  {0, id, 1},
                                  false});
        }
        g.plan.vertex_count += c.count;
    }
    return g;
}
std::vector<double> ids(const std::vector<Point3> &v) {
    std::vector<double> out;
    for (auto p : v)
        out.push_back(p[0]);
    return out;
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
            {"holeOrigin", 0},
            {"boundaries", nullptr}};
}
} // namespace
unsigned native_tube_mesh_trim_edges_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool thrown = false;
        try {
            fn();
        } catch (const std::exception &) {
            thrown = true;
        }
        check(thrown, "trimmed edge invalid state/correspondence/budget rejects");
    };
    const std::vector<double> v{0, .1, .2, .3, .4, .5, .6, .7, .8, .9, 1};
    for (bool pc : {false, true})
        for (bool vc : {false, true}) {
            TubeBudget b;
            auto state = make_tube_mesh_edge_state(2, 2, pc, vc, b);
            std::vector<TubeMeshTrimConnected> meshes;
            for (std::size_t pi = 0; pi < 2; ++pi)
                for (std::size_t si = 0; si < 2; ++si) {
                    auto g = marked(double(1000 * pi + 100 * si));
                    auto uv = evaluate_tube_mesh_trim_parameters(g.plan, v, pi, 2, b);
                    auto m = connect_tube_mesh_trim_vertices(
                        g, v, state, {true, true, pi == 0 && !vc, pi == 1 && !vc}, b);
                    check(m.parameters == uv,
                          "shared coordinates never replace independent attribute parameters");
                    for (std::size_t k = 0; k < g.vertices.size(); ++k)
                        check(m.normals[k] == g.vertices[k].normal,
                              "trimmed shared coordinates never inherit neighbour normal");
                    check(m.facets.facets.size() == 1 &&
                              m.facets.facets[0].indices == std::vector<std::int32_t>{1, 3, 4, 2},
                          "connected trimmed strip keeps original column-major quad arguments");
                    meshes.push_back(std::move(m));
                }
            check(ids(meshes[0].points) == std::vector<double>{0, 10, 1, 11},
                  "first trimmed strip traverses whole columns in order");
            check(ids(meshes[1].points) == (pc ? std::vector<double>{1, 11, 0, 10}
                                               : std::vector<double>{1, 11, 101, 111}),
                  "trimmed strip uses previous last column and source profile seam");
            check(ids(meshes[2].points) == (vc ? std::vector<double>{10, 0, 11, 1}
                                               : std::vector<double>{10, 1010, 11, 1011}),
                  "trimmed patch uses prior last row and source path seam");
            const auto expected = pc ? (vc ? std::vector<double>{11, 1, 10, 1010, 101}
                                           : std::vector<double>{11, 1011, 10, 1010})
                                     : (vc ? std::vector<double>{11, 1, 111, 101}
                                           : std::vector<double>{11, 1011, 111, 1111});
            check(ids(meshes[3].points) == expected,
                  "trimmed double closure retains original extra corner coordinate and precedence");
            check(meshes[3].points.size() == 4 + std::size_t(pc && vc) &&
                      meshes[3].normals.size() == 4 && meshes[3].parameters.size() == 4,
                  "trimmed coordinate and attribute pool sizes remain independent");
            check(state.next_patch == 2 && state.next_strip == 0,
                  "trimmed original cursor advances in patch/strip order");
            if (!vc) {
                check(ids(state.start_points) == (pc ? std::vector<double>{0, 1, 1, 0}
                                                     : std::vector<double>{0, 1, 1, 101}),
                      "trimmed start collection preserves repeated original strip corners");
                check(ids(state.end_points) == (pc ? std::vector<double>{1010, 1011, 1011, 1010}
                                                   : std::vector<double>{1010, 1011, 1011, 1111}),
                      "trimmed end collection applies native coordinate correspondence");
            }
            auto before = snapshot(state);
            rejects([&] { connect_tube_mesh_trim_vertices(marked(0), v, state, {}, b); });
            check(snapshot(state) == before,
                  "invalid completed cursor does not mutate shared state");
        }
    for (bool pc : {false, true})
        for (bool vc : {false, true}) {
            TubeBudget b;
            auto state = make_tube_mesh_edge_state(1, 1, pc, vc, b);
            auto m = connect_tube_mesh_trim_vertices(marked(0), v, state, {}, b);
            const auto expected =
                pc ? (vc ? std::vector<double>{0, 0, 0, 10, 1} : std::vector<double>{0, 10, 0, 10})
                   : (vc ? std::vector<double>{0, 0, 1, 1} : std::vector<double>{0, 10, 1, 11});
            check(ids(m.points) == expected,
                  "single trimmed strip applies self-closure in U-major order");
        }
    TubeBudget b;
    auto varying = make_tube_mesh_edge_state(2, 2, false, false, b);
    connect_tube_mesh_trim_vertices(marked(0, {{1, 1}, {2, 1}, {1, 2}}), v, varying, {}, b);
    auto m =
        connect_tube_mesh_trim_vertices(marked(100, {{3, 4}, {2, 0}, {1, 1}}), v, varying, {}, b);
    check(ids(m.points) == std::vector<double>{2, 12, 22, 32, 101, 102, 112, 122},
          "adjacent variable columns reuse relative row positions despite different absolute row "
          "labels");
    m = connect_tube_mesh_trim_vertices(marked(200, {{1, 0}, {1, 1}, {1, 1}}), v, varying, {}, b);
    check(ids(m.points) == std::vector<double>{20, 210, 11, 211, 221, 32, 212, 222},
          "new patch keeps previous per-column upper positions with changed column heights");
    m = connect_tube_mesh_trim_vertices(marked(300, {{1, 1}, {1, 3}, {1, 1}}), v, varying, {}, b);
    check(ids(m.points) ==
              std::vector<double>{32, 212, 222, 101, 311, 321, 331, 341, 122, 312, 322},
          "unequal trimmed columns combine adjacent-column and prior-patch correspondence");
    check(ids(varying.rows[1]) == std::vector<double>{222, 341, 322},
          "variable column last-row state is assembled from original relative endpoints");
    // A regular strip can precede a trimmed strip in the very same native
    // state: they must not acquire independent caches during refactoring.
    auto mixed = make_tube_mesh_edge_state(1, 2, false, false, b);
    TubeMeshRegularVertices regular{2, 2, {}};
    for (double x : {0, 1, 10, 11})
        regular.vertices.push_back({{x, 0, 0}, {0, 0, 1}, {0, 0}, false});
    connect_tube_mesh_regular_vertices(regular, mixed, {}, b);
    m = connect_tube_mesh_trim_vertices(marked(100), v, mixed, {}, b);
    check(ids(m.points) == std::vector<double>{1, 11, 101, 111},
          "regular and trimmed native branches share original coordinate caches");
    auto reverse = make_tube_mesh_edge_state(1, 2, false, false, b);
    connect_tube_mesh_trim_vertices(marked(0), v, reverse, {}, b);
    auto rm = connect_tube_mesh_regular_vertices(regular, reverse, {}, b);
    check(ids(rm.points) == std::vector<double>{1, 1, 11, 11},
          "trimmed caches can feed a following regular branch without conversion");
    auto initial = make_tube_mesh_edge_state(1, 2, true, false, b);
    connect_tube_mesh_trim_vertices(marked(0), v, initial, {true, true, true, true}, b);
    auto before = snapshot(initial);
    auto failed = initial;
    rejects([&] {
        connect_tube_mesh_trim_vertices(marked(100, {{1, 1}, {1, 0}}), v, failed,
                                        {true, true, true, true}, b);
    });
    check(snapshot(failed) == before,
          "late correspondence failure rolls back caches and collected edges");
    auto measured = initial;
    TubeBudget measure;
    connect_tube_mesh_trim_vertices(marked(100), v, measured, {true, true, true, true}, measure);
    TubeBudget limited;
    limited.max_work = measure.work - 1;
    failed = initial;
    rejects([&] {
        connect_tube_mesh_trim_vertices(marked(100), v, failed, {true, true, true, true}, limited);
    });
    check(snapshot(failed) == before && limited.work > 0,
          "late budget failure restores state while retaining consumed work");
    auto off = marked(0);
    for (auto &node : off.vertices)
        node.normal[0] = std::numeric_limits<double>::quiet_NaN();
    auto optional = make_tube_mesh_edge_state(1, 1, false, false, b);
    m = connect_tube_mesh_trim_vertices(off, {}, optional, {false, false, false, false}, b);
    check(m.normals.empty() && m.parameters.empty() && m.points.size() == 4,
          "disabled attribute paths neither read missing samples nor validate unused normals");
    auto bad = marked(0);
    ++bad.vertices.back().row;
    failed = initial;
    rejects([&] { connect_tube_mesh_trim_vertices(bad, v, failed, {}, b); });
    check(snapshot(failed) == before,
          "mismatched original row label rejects before state mutation");
    auto empty = make_tube_mesh_edge_state(1, 1, false, false, b);
    m = connect_tube_mesh_trim_vertices(marked(0, {{3, 0}, {3, 0}}), v, empty, {}, b);
    check(m.points.empty() && m.facets.facets.empty() && empty.next_patch == 1,
          "empty native column loops do not invent coordinates or faces");
    SweptBodyPatchGroup group;
    group.patches = {
        {surface(), {{{0, .1}, {.5, .2}, {1, .1}, {1, .9}, {.5, .8}, {0, .9}, {0, .1}}}}};
    auto prepared = prepare_tube_mesh_grid(group, false, .01, .1, 10000, b);
    auto nodes = evaluate_tube_mesh_trim_vertices(prepared.patches[0], prepared.section, 0, b);
    auto actual = make_tube_mesh_edge_state(1, 1, false, false, b);
    m = connect_tube_mesh_trim_vertices(nodes, prepared.patches[0].path.parameters, actual, {}, b);
    check(
        m.facets.indices_in_range && m.facets.column_correspondence_valid &&
            m.points.size() == nodes.vertices.size() && !m.facets.facets.empty(),
        "actual runtime bounds feed prepared vertices, shared state and original facet arguments");
    check(m.report["shared_coordinates_applied"] == true &&
              m.facets.report["shared_coordinates_applied"] == true &&
              m.report["triangulated"] == false,
          "connected facet reports distinguish shared coordinates from final triangulation");
    for (std::size_t i = 0; i < nodes.vertices.size(); ++i)
        check(m.points[i] == nodes.vertices[i].point && m.normals[i] == nodes.vertices[i].normal,
              "actual first open strip retains original geometric values");
    auto task_fn = [&] {
        TubeBudget local;
        auto state = initial;
        return connect_tube_mesh_trim_vertices(marked(100), v, state, {}, local).points;
    };
    auto task = std::async(std::launch::async, task_fn);
    check(task.get() == task_fn() && snapshot(initial) == before,
          "independent concurrent connections preserve shared immutable source state");
    return n;
}
