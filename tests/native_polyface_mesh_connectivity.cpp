#include "native_polyface_connectivity.hpp"
#include "native_polyface_mesh_attributes.hpp"
#include "native_polyface_smooth_normals.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_mesh_connectivity_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "typed connectivity rejects unsafe original reads and exhausted budgets");
    };
    auto graph = [](const NativePolyfaceMesh &m, bool filter = false) {
        TubeBudget b;
        return build_native_polyface_mesh_connectivity(m, filter, b);
    };
    auto smooth = [](const NativePolyfaceMesh &m, double edge, double acc, bool hide) {
        TubeBudget b;
        return build_native_polyface_mesh_approximate_normals(m, edge, acc, hide, b);
    };
    auto compare = [&](const NativePolyfaceConnectivity &a, const NativePolyfaceConnectivity &b) {
        check(a.native_succeeded == b.native_succeeded && a.points == b.points &&
                  a.read_to_half_edge == b.read_to_half_edge && a.nodes.size() == b.nodes.size() &&
                  a.half_edges.size() == b.half_edges.size(),
              "typed and prepared graph topology sizes and source maps match");
        for (std::size_t i = 0; i < a.nodes.size(); ++i) {
            const auto &x = a.nodes[i], &y = b.nodes[i];
            check(x.face_successor == y.face_successor &&
                      x.vertex_successor == y.vertex_successor && x.mask == y.mask &&
                      x.vertex_index == y.vertex_index && x.read_index == y.read_index,
                  "shared assembler preserves exact native node order, masks and labels");
        }
        for (std::size_t i = 0; i < a.half_edges.size(); ++i) {
            const auto &x = a.half_edges[i], &y = b.half_edges[i];
            check(x.vertex0 == y.vertex0 && x.vertex1 == y.vertex1 &&
                      x.read_index == y.read_index &&
                      x.successor_read_index == y.successor_read_index && x.node == y.node &&
                      x.visible == y.visible,
                  "half-edge sorting, direction, read positions and visibility are preserved");
        }
    };
    NativePolyfaceMesh square;
    square.data.coordinates.points = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    square.data.indices.indices[point_channel] = {1, 2, 3, 0, 1, 3, 4, 0};
    const std::vector<std::vector<std::int32_t>> layouts = {{1, -2, 3, 0},
                                                            {1, 2, 3, 0, 1, 3, 4, 0},
                                                            {1, 2, 3, 0, 1, 2, 4, 0},
                                                            {1, 2, 2, 3, 1, 0},
                                                            {1, 2, 1, 0},
                                                            {1, 1, 1, 0},
                                                            {1, 2, 2, 0, 2, 1, 3, 0, 2, 1, 4, 0},
                                                            {1, 2, 3, 0, 2, 1, 4, 0, 1, 2, 4, 0},
                                                            {1, 2, 3, 0, 99, 1, 2, 0},
                                                            {0, 0, 1, 2, 3},
                                                            {},
                                                            {1, 2, 0}};
    for (const auto &indices : layouts)
        for (bool filter : {false, true}) {
            auto m = square;
            m.data.indices.indices[point_channel] = indices;
            NativePolyfaceFaceDataState legacy;
            legacy.mesh = m.data;
            TubeBudget b;
            const auto old = build_native_polyface_connectivity(legacy, filter, b);
            const auto now = graph(m, filter);
            compare(old, now);
            check(old.complete == now.complete &&
                      old.report["filtered_corners"] == now.report["filtered_corners"] &&
                      old.report["skipped_degenerate_faces"] ==
                          now.report["skipped_degenerate_faces"],
                  "typed traversal preserves degenerate filtering and partial-source status");
        }
    for (unsigned style : {3u, 4u}) {
        NativePolyfaceMesh raw;
        raw.mesh_style = style;
        raw.data.num_per_face = 19;
        const std::vector<Point3> polygon =
            style == 3 ? std::vector<Point3>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}
                       : std::vector<Point3>{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
        raw.data.coordinates.points = polygon;
        raw.data.coordinates.points.insert(raw.data.coordinates.points.end(), polygon.begin(),
                                           polygon.end());
        raw.data.indices.indices[point_channel].assign(style * 2, -12345);
        raw.texture_id = 50;
        raw.face_material_ids = {6, 7};
        raw.face_smooth_groups = {8, 9};
        raw.illumination_name = u"material";
        raw.pool_rows.fill(7);
        raw.index_rows.fill(6);
        NativePolyfaceFaceDataState fixed;
        fixed.mesh.coordinates = raw.data.coordinates;
        fixed.mesh.num_per_face = style;
        for (unsigned i = 0; i < style * 2; ++i)
            fixed.mesh.indices.indices[point_channel].push_back(int(i + 1));
        for (bool filter : {false, true}) {
            TubeBudget b;
            const auto reference = build_native_polyface_connectivity(fixed, filter, b);
            const auto g = graph(raw, filter);
            compare(reference, g);
            check(g.complete && g.report["paired_edges"] == 0 &&
                      g.report["boundary_edges"] == style * 2,
                  "coincident raw faces keep separate native vertex identities, including visible "
                  "edges");
            for (const auto &e : g.half_edges)
                check(e.visible && e.vertex0 == e.read_index + 1 &&
                          e.vertex1 == e.successor_read_index + 1,
                      "raw client ids ignore arbitrary source index values and preserve true "
                      "positions");
        }
        for (bool hide : {false, true}) {
            TubeBudget b;
            const auto reference = build_native_polyface_approximate_normals(fixed, 1, 1, hide, b);
            const auto r = smooth(raw, 1, 1, hide);
            check(r.native_succeeded &&
                      r.output.data.coordinates.normals ==
                          reference.output.mesh.coordinates.normals &&
                      r.output.data.indices.indices[normal_channel] ==
                          reference.output.mesh.indices.indices[normal_channel],
                  "raw per-face generation feeds exact shared smoothing without regeneration or "
                  "coordinate welding");
            check(r.output.data.coordinates.normals.size() == style * 2 &&
                      r.output.data.indices.indices[point_channel] ==
                          std::vector<std::int32_t>(style * 2, hide ? 12345 : -12345),
                  "native per-vertex sectors allocate separate equal normals and optional sign "
                  "replacement");
            check(!r.complete &&
                      r.report["connectivity"]["visitor"]["incomplete_attribute_reads"] != 0,
                  "raw visitor's partial normal-value reads remain explicit despite native "
                  "smoothing success");
            check(r.output.texture_id == 50 &&
                      r.output.face_material_ids == raw.face_material_ids &&
                      r.output.face_smooth_groups == raw.face_smooth_groups &&
                      r.output.illumination_name == raw.illumination_name &&
                      r.output.pool_rows == raw.pool_rows &&
                      r.output.index_rows == raw.index_rows && r.output.mesh_style == style,
                  "smoothing preserves independent material and layout metadata");
        }
        raw.data.indices.indices[point_channel].clear();
        check(graph(raw).native_succeeded,
              "raw connectivity itself does not require source point indices");
        const auto fail = smooth(raw, 1, 1, false);
        check(!fail.native_succeeded && fail.report["reason"] == "per_face_generation_failed" &&
                  fail.output.data.coordinates.normals.empty(),
              "approximate normal generation retains its earlier empty-index guard");
    }
    for (double theta : {0.0, .2, .8})
        for (double edge : {0.0, .3, 1.0})
            for (double accumulated : {0.0, 1.0})
                for (bool hide : {false, true}) {
                    auto m = square;
                    m.data.coordinates.points = {
                        {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, -std::cos(theta), std::sin(theta)}};
                    m.data.indices.indices[point_channel] = {1, 2, 3, 0, 2, 1, 4, 0};
                    NativePolyfaceFaceDataState legacy;
                    legacy.mesh = m.data;
                    TubeBudget b;
                    const auto ref = build_native_polyface_approximate_normals(
                        legacy, edge, accumulated, hide, b);
                    const auto now = smooth(m, edge, accumulated, hide);
                    check(now.native_succeeded == ref.native_succeeded &&
                              now.complete == ref.complete &&
                              now.output.data.coordinates.normals ==
                                  ref.output.mesh.coordinates.normals &&
                              now.output.data.indices.indices == ref.output.mesh.indices.indices,
                          "typed indexed smoothing preserves angle thresholds, allocation order "
                          "and visibility");
                }
    for (unsigned style : {5u, 6u}) {
        auto m = square;
        m.mesh_style = style;
        m.num_per_row = 2;
        rejects([&] { graph(m); });
        rejects([&] { smooth(m, 1, 1, false); });
        check(m.data.coordinates.normals.empty(), "unsafe grid positions never mutate the input");
    }
    {
        auto bad = square;
        bad.integer_colors = {1, 2, 3, 4};
        bad.pool_active[native_integer_color_pool] = true;
        bad.data.indices.indices[color_channel] = {1};
        rejects([&] { graph(bad); });
        rejects([&] { smooth(bad, 1, 1, false); });
        bad = square;
        bad.data.coordinates.normals = {{0, 0, 1}};
        const auto r = graph(bad);
        check(r.native_succeeded && !r.complete,
              "incomplete attribute queries do not silently become complete topology input");
        TubeBudget b;
        b.max_control_points = 1;
        rejects([&] { build_native_polyface_mesh_connectivity(square, false, b); });
        b = TubeBudget{};
        b.max_work = 1;
        rejects([&] { build_native_polyface_mesh_approximate_normals(square, 1, 1, false, b); });
        rejects([&] { smooth(square, std::numeric_limits<double>::quiet_NaN(), 1, false); });
    }
    auto job = [&] { return smooth(square, 1, 1, true).output.data.indices.indices; };
    auto a = std::async(std::launch::async, job), b = std::async(std::launch::async, job);
    check(a.get() == b.get() && square.data.coordinates.normals.empty(),
          "typed topology and smoothing are immutable and deterministic concurrently");
    return checks;
}
