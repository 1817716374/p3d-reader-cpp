#include "native_polyface_mesh_face_data.hpp"
#include "native_polyface_edge_chains.hpp"
#include "native_polyface_prepare.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_mesh_records_tests() {
    unsigned checks = 0;
    auto check = [&](bool v, const char *why) {
        ++checks;
        require(v, why);
    };
    auto rejects = [&](auto f) {
        bool caught = false;
        try {
            f();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "unsafe original reads or exhausted budgets are rejected");
    };
    auto base = [] {
        auto m = make_native_polyface_mesh();
        m.data.coordinates.points = {{0, 0, 0}, {2, 0, 0}, {2, 1, 0}, {0, 1, 0}};
        m.data.indices.indices[point_channel] = {1, 2, 3, 0, 1, 3, 4, 0};
        return m;
    };
    auto faces = [](const NativePolyfaceMesh &m) {
        TubeBudget b;
        return build_native_polyface_mesh_face_data(m, b);
    };
    auto edges = [](const NativePolyfaceMesh &m) {
        TubeBudget b;
        return build_native_polyface_mesh_edge_chains(m, 99, b);
    };
    auto set = [](const NativePolyfaceMesh &m, std::size_t end) {
        TubeBudget b;
        return set_native_polyface_mesh_face_data(m, std::nullopt, end, b);
    };
    auto same_face = [&](const NativeBuilderFaceData &a, const NativeBuilderFaceData &b) {
        check(a.parameter_range == b.parameter_range &&
                  a.parameter_distance_range == b.parameter_distance_range &&
                  a.point_range == b.point_range && a.normal_range == b.normal_range &&
                  a.source_index == b.source_index && a.face_indices == b.face_indices,
              "typed face ranges and native statistics match established indexed arithmetic");
    };
    for (bool params : {false, true})
        for (bool active : {false, true})
            for (unsigned fixed : {0u, 4u}) {
                auto m = base();
                m.data.num_per_face = fixed;
                if (params) {
                    m.data.coordinates.parameters = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
                    m.pool_active[native_parameter_pool] = active;
                }
                NativePolyfaceFaceDataState old;
                old.mesh = m.data;
                old.parameter_pool_active = active;
                TubeBudget b;
                auto ref = build_native_polyface_face_data(old, b);
                auto r = faces(m);
                check(r.complete == ref.complete &&
                          r.output.data.indices.indices == ref.output.mesh.indices.indices &&
                          r.output.data.face_data.size() == ref.output.mesh.face_data.size(),
                      "indexed face grouping and completeness remain unchanged");
                for (std::size_t i = 0; i < r.output.data.face_data.size(); ++i)
                    same_face(r.output.data.face_data[i], ref.output.mesh.face_data[i]);
                check(!r.output.pool_active[native_face_data_pool] &&
                          r.output.data.indices.active[face_channel],
                      "building records activates only face indices");
            }
    {
        auto m = base();
        m.data.indices.active[face_channel] = true;
        auto r = set(m, 4);
        check(!r.complete && r.output.data.face_data.size() == 1 &&
                  r.output.data.indices.indices[face_channel] ==
                      std::vector<std::int32_t>{1, 1, 1, 0},
              "partial SetNewFace preserves its explicit end");
        r = set(r.output, 0);
        check(r.complete && r.output.data.face_data.size() == 2,
              "subsequent set starts at existing face-index length");
        m.data.coordinates.normals = {{0, 0, 1}};
        m.data.indices.indices[normal_channel] = {1, 1, 1, 0, 1};
        rejects([&] { set(m, 4); }); // Loads the next face before checking its read position.
        m = base();
        m.double_colors = {{1, 1, 1}};
        m.integer_colors.resize(4);
        m.pool_active[native_integer_color_pool] = true;
        m.data.indices.indices[color_channel] = {4, 4, 4, 0, 1, 1, 1, 0};
        TubeBudget b;
        std::size_t calls = 0;
        auto v =
            consume_native_polyface_from(m, b, 4, [&](std::size_t at, const auto &, const auto &d) {
                ++calls;
                check(at == 4 && d.double_colors.size() == 3,
                      "direct seek reads only the requested safe facet");
                return false;
            });
        check(calls == 1 && v.complete && v.report["complete_scope"] == "visited_range" &&
                  v.report["stopped_by_consumer"] == true,
              "explicit range completion does not claim earlier source coverage");
        rejects([&] {
            TubeBudget q;
            visit_native_polyface(m, q);
        });
    }
    for (unsigned style : {3u, 4u}) {
        auto m = make_native_polyface_mesh(0, style);
        m.data.coordinates.points =
            style == 3 ? std::vector<Point3>{{0, 0, 0}, {2, 0, 0}, {0, 1, 0}}
                       : std::vector<Point3>{{0, 0, 0}, {2, 0, 0}, {2, 1, 0}, {0, 1, 0}};
        const auto first = m.data.coordinates.points;
        m.data.coordinates.points.insert(m.data.coordinates.points.end(), first.begin(),
                                         first.end());
        m.data.indices.indices[point_channel].assign(style * 2, 1000);
        m.texture_id = 42;
        m.face_material_ids = {9, 10};
        m.illumination_name = u"lamp";
        auto e = edges(m);
        check(e.complete && e.output.data.edge_chains.size() == style * 2 &&
                  !e.output.edge_chain_active,
              "raw blocks derive independent visible edges from client identities without setting "
              "vector activity");
        for (std::size_t i = 0; i < e.output.data.edge_chains.size(); ++i) {
            const auto &c = e.output.data.edge_chains[i];
            check(c.topology_type == 24 && c.topology_ids[0] == i &&
                      c.point_indices[0] == int(i + 1),
                  "raw edge vertices ignore arbitrary source index values");
        }
        check(e.output.texture_id == 42 && e.output.face_material_ids == m.face_material_ids &&
                  e.output.illumination_name == m.illumination_name,
              "edge generation preserves independent material metadata");
        auto r = faces(m);
        check(!r.complete && r.output.data.face_data.size() == 1 &&
                  r.output.data.indices.indices[face_channel].size() == style + 1,
              "raw ordinal-based SetNewFace keeps native grouping instead of inventing per-face "
              "references");
        m.data.coordinates.parameters.resize(style * 2);
        for (std::size_t i = 0; i < m.data.coordinates.points.size(); ++i)
            m.data.coordinates.parameters[i] = {m.data.coordinates.points[i][0] / 2,
                                                m.data.coordinates.points[i][1]};
        m.pool_active[native_parameter_pool] = true;
        r = faces(m);
        check(r.complete && r.output.data.face_data.size() == 1 &&
                  r.output.data.face_data[0].parameter_distance_range ==
                      std::array<Point2, 2>{Point2{0, 0}, Point2{2, 1}},
              "raw UV-bearing data groups all facets and preserves physical parameter distances");
        auto bad = m;
        bad.data.coordinates.normals.resize(1);
        rejects([&] { edges(bad); });
        auto existing = e.output;
        existing.mesh_style = 999;
        auto no_op = edges(existing);
        check(no_op.native_status == 1 && no_op.output.data.edge_chains.size() == style * 2,
              "existing chains return before attaching even an unsupported visitor");
        m.data.indices.indices[point_channel].clear();
        r = faces(m);
        check(!r.complete && r.report["raw_source_has_no_face_index_storage"] == true,
              "raw geometry without index storage is not reported fully assigned");
    }
    for (unsigned style : {5u, 6u}) {
        auto m = make_native_polyface_mesh(0, style);
        m.num_per_row = 2;
        m.data.coordinates.points = {{0, 0, 0}, {2, 0, 0}, {0, 1, 0}, {2, 1, 0}};
        m.data.indices.indices[point_channel] = {99, 99, 99, 99};
        auto e = edges(m);
        check(e.complete && e.output.data.edge_chains.size() == (style == 5 ? 5 : 4) &&
                  e.report["emitted"][0]["read_index"].is_null(),
              "grid edge generation needs client vertices but no index-position array");
        m.data.coordinates.parameters = {{0, 0}, {1, 0}, {0, 1}, {1, 1}};
        m.pool_active[native_parameter_pool] = true;
        auto r = faces(m);
        check(r.complete && r.output.data.face_data.size() == 1 &&
                  r.output.data.face_data[0].parameter_distance_range ==
                      std::array<Point2, 2>{Point2{0, 0}, Point2{2, 1}},
              "grid face ranges use values and do not invent missing visitor positions");
    }
    {
        auto m = base();
        m.data.indices.indices[point_channel] = {1, 2, 99, 0};
        rejects([&] { edges(m); });
        m.data.indices.indices[point_channel] = {-1, -2, -99, 0};
        rejects([&] { edges(m); });
        m = base();
        m.data.indices.indices[point_channel] = {99, 1, 2, 0};
        auto r = edges(m);
        check(r.native_succeeded && !r.complete && r.output.data.edge_chains.empty(),
              "failure before native wrap acceptance stops safely");
    }
    {
        NativePolyfacePreparationOptions o;
        o.normals_required = false;
        o.parameters_required = false;
        auto m = make_native_polyface_mesh(0, 4);
        m.data.coordinates.points = base().data.coordinates.points;
        TubeBudget b;
        auto r = prepare_native_polyface_for_builder(m, o, b);
        check(r.complete && r.output.mesh_style == 1 &&
                  r.output.data.indices.indices[point_channel].size() == 8,
              "raw quad reaches the original conditional conversion and triangulation");
        o.max_edges_per_face = 4;
        b = TubeBudget{};
        auto assembled = assemble_native_prepared_polyfaces({m}, o, {}, b);
        check(assembled.native_succeeded && !assembled.complete &&
                  assembled.report["untransferred_raw_layout_sources"] == Json::array({0}),
              "native passthrough of raw layout is not misreported as complete indexed assembly");
    }
    const auto source = base();
    TubeBudget b;
    b.max_control_points = 1;
    rejects([&] { build_native_polyface_mesh_face_data(source, b); });
    b = TubeBudget{};
    b.max_work = 1;
    rejects([&] { build_native_polyface_mesh_edge_chains(source, 0, b); });
    auto job = [&] { return faces(source).output.data.indices.indices; };
    auto a = std::async(std::launch::async, job), c = std::async(std::launch::async, job);
    check(a.get() == c.get() && source.data.face_data.empty(),
          "face generation is concurrent, deterministic and source-preserving");
    return checks;
}
