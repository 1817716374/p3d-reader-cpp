#include "native_polyface_mesh_attributes.hpp"
#include "native_polyface_attributes.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_mesh_attributes_tests() {
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
        check(caught, "typed generation rejects unsafe native reads or exhausted budgets");
    };
    auto make = [](unsigned style) {
        NativePolyfaceMesh m;
        m.mesh_style = style;
        m.data.coordinates.points = {{0, 0, 0}, {2, 0, 0}, {0, 1, 0},
                                     {0, 0, 1}, {2, 0, 1}, {0, 1, 1}};
        m.data.indices.indices[point_channel] =
            style == 1 ? std::vector<std::int32_t>{1, 2, 3, 0, 4, 5, 6, 0}
                       : std::vector<std::int32_t>{9, -9, 9, 9, 9, 9};
        m.data.indices.active[point_channel] = true;
        m.texture_id = 123;
        m.face_material_ids = {70, 80};
        m.face_uv_points = {{.1, .2}};
        m.face_smooth_groups = {3, 4};
        m.illumination_name = u"light";
        m.pool_rows.fill(9);
        m.index_rows.fill(7);
        return m;
    };
    auto normal = [](const NativePolyfaceMesh &m) {
        TubeBudget b;
        return build_native_polyface_mesh_normals(m, b);
    };
    auto uv = [](const NativePolyfaceMesh &m, int selector) {
        TubeBudget b;
        return build_native_polyface_mesh_parameters(m, selector, b);
    };
    for (auto style : {1u, 3u}) {
        const auto m = make(style);
        const auto n = normal(m);
        check(n.native_succeeded && n.complete &&
                  n.output.data.coordinates.normals == std::vector<Point3>{{0, 0, 1}, {0, 0, 1}},
              "indexed and raw triangle normals preserve per-face allocation");
        check(n.output.data.indices.indices[normal_channel] ==
                  (style == 1 ? std::vector<std::int32_t>{1, 1, 1, 0, 2, 2, 2, 0}
                              : std::vector<std::int32_t>{1, 1, 1, 2, 2, 2}),
              "normal writes use actual index positions rather than raw facet ordinal");
        for (int selector : {-1, 0, 1, 2, 3, 90}) {
            const auto p = uv(m, selector);
            check(p.native_succeeded && p.complete &&
                      p.output.data.coordinates.parameters.size() == 6,
                  "UV generation handles every original selector on both layouts");
            check(p.output.data.indices.indices[parameter_channel] ==
                      (style == 1 ? std::vector<std::int32_t>{1, 2, 3, 0, 4, 5, 6, 0}
                                  : std::vector<std::int32_t>{1, 2, 3, 4, 5, 6}),
                  "UV vertices are distinct and preserve zero separators only in indexed sources");
            check(p.output.texture_id == m.texture_id &&
                      p.output.face_material_ids == m.face_material_ids &&
                      p.output.face_uv_points == m.face_uv_points &&
                      p.output.face_smooth_groups == m.face_smooth_groups &&
                      p.output.illumination_name == m.illumination_name &&
                      p.output.pool_rows == m.pool_rows && p.output.index_rows == m.index_rows &&
                      p.output.mesh_style == style,
                  "typed generation preserves all independent metadata and raw style");
            if (style == 1) {
                NativePolyfaceFaceDataState s;
                s.mesh = m.data;
                TubeBudget b;
                auto reference = build_native_polyface_parameters(s, selector, b);
                check(p.output.data.coordinates.parameters ==
                              reference.output.mesh.coordinates.parameters &&
                          p.output.data.indices.indices == reference.output.mesh.indices.indices,
                      "typed indexed path matches established projection and index generation");
            }
        }
        check(m.data.coordinates.normals.empty() && m.data.coordinates.parameters.empty(),
              "generators preserve input on successful calls");
    }
    {
        auto m = make(4);
        m.data.coordinates.points = {{0, 0, 0}, {2, 0, 0}, {2, 1, 0}, {0, 1, 0},
                                     {0, 0, 1}, {2, 0, 1}, {2, 1, 1}, {0, 1, 1}};
        m.data.indices.indices[point_channel].assign(8, 123);
        auto p = uv(m, 2);
        check(p.complete && p.output.data.coordinates.parameters ==
                                std::vector<Point2>{
                                    {0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}, {1, 0}, {1, 1}, {0, 1}},
              "raw quads project each face independently with separate parameter slots");
        auto n = normal(m);
        check(n.complete && n.output.data.indices.indices[normal_channel] ==
                                std::vector<std::int32_t>{1, 1, 1, 1, 2, 2, 2, 2},
              "quad normals use eight point-index positions");
        m.data.indices.indices[point_channel].resize(2);
        n = normal(m);
        p = uv(m, 0);
        check(n.native_succeeded && !n.complete && n.report["skipped_index_writes"] == 6 &&
                  n.output.data.coordinates.normals.size() == 2 &&
                  n.output.data.indices.indices[normal_channel] == std::vector<std::int32_t>{1, 1},
              "out-of-range destination positions skip writes while generated normals remain");
        check(p.native_succeeded && !p.complete && p.report["skipped_index_writes"] == 6 &&
                  p.output.data.coordinates.parameters.size() == 8,
              "UV append precedes its destination bound check");
        m.data.indices.indices[point_channel].assign(10, 1);
        n = normal(m);
        check(!n.complete && n.report["unassigned_nonzero_indices"] == 2,
              "unused source index tail is explicitly incomplete");
    }
    for (unsigned style : {1u, 3u, 4u, 5u, 6u, 99u}) {
        auto m = make(style);
        m.data.indices.indices[point_channel].clear();
        m.data.coordinates.normals = {{7, 8, 9}};
        m.data.coordinates.parameters = {{3, 4}};
        m.data.face_data.resize(1);
        m.data.face_data[0].parameter_range = {Point2{2, 3}, Point2{4, 5}};
        const auto n = normal(m), p = uv(m, 2);
        check(!n.native_succeeded && !p.native_succeeded &&
                  n.output.data.coordinates.normals == m.data.coordinates.normals &&
                  p.output.data.coordinates.parameters == m.data.coordinates.parameters &&
                  p.output.data.face_data[0].parameter_range == m.data.face_data[0].parameter_range,
              "empty point indices return before visitor creation, clearing or range mutation");
    }
    for (unsigned style : {5u, 6u}) {
        auto m = make(style);
        m.num_per_row = 2;
        m.data.coordinates.points = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}};
        rejects([&] { normal(m); });
        rejects([&] { uv(m, 0); });
        check(m.data.coordinates.normals.empty() && m.data.coordinates.parameters.empty(),
              "missing raw grid index positions reject safely without mutating the source");
    }
    {
        auto m = make(3);
        m.data.indices.indices[face_channel] = {0, 0, 0, 1, 1, 1};
        m.data.indices.active[face_channel] = true;
        m.data.face_data.resize(2);
        m.data.face_data[1].source_index = 888;
        auto p = uv(m, 2);
        check(p.report["face_range_updates"] == 6 &&
                  p.output.data.face_data[0].parameter_range ==
                      std::array<Point2, 2>{Point2{0, 0}, Point2{1, 1}} &&
                  p.output.data.face_data[1].parameter_range ==
                      p.output.data.face_data[0].parameter_range &&
                  p.output.data.face_data[1].source_index == 888,
              "raw UV ranges use direct stored face indices including zero and preserve other face "
              "metadata");
        m.data.indices.indices[face_channel] = {-1, 2, 0};
        p = uv(m, 2);
        check(!p.complete && p.report["face_range_updates_skipped"] == 2 &&
                  p.report["face_range_updates"] == 1,
              "invalid direct face references skip range writes independently");
        m = make(1);
        m.integer_colors = {1, 2, 3, 4, 5, 6};
        m.pool_active[native_integer_color_pool] = true;
        m.data.indices.indices[color_channel] = {1};
        rejects([&] { normal(m); });
        rejects([&] { uv(m, 0); });
        m = make(3);
        m.float_colors.resize(6);
        check(!normal(m).complete && !uv(m, 0).complete,
              "unconsumed original color payload is not masked by successful generation");
    }
    // These checks distinguish a live query from precomputing all source faces.
    for (unsigned style : {1u, 3u}) {
        auto m = make(style);
        m.data.indices.indices[normal_channel].assign(m.data.indices.indices[point_channel].size(),
                                                      0);
        TubeBudget b;
        std::size_t calls = 0;
        const auto v = visit_native_polyface_attribute_updates(
            m, b, normal_channel,
            [&](std::size_t, const NativePolyfaceVisitorFacet &,
                const NativePolyfaceVisitedData &d) {
                if (!calls) {
                    check(d.normals.empty(), "first advance precedes generated normals");
                    m.data.coordinates.normals.assign(6, Point3{7, 8, 9});
                    if (style == 1)
                        m.data.indices.indices[normal_channel] = {0, 0, 0, 0, 2, 2, 2, 0};
                } else
                    check(d.normals == std::vector<Point3>(3, Point3{7, 8, 9}),
                          "second advance reads updated counts, pools and normal index values");
                ++calls;
            });
        check(calls == 2 && v.complete && v.facets.empty() && v.data.empty() &&
                  v.read_indices.size() == 2,
              "live query visits each face exactly once without retaining consumed value arrays");
    }
    {
        auto m = make(5);
        m.num_per_row = 2;
        m.data.coordinates.points = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}};
        TubeBudget b;
        std::size_t calls = 0;
        rejects([&] {
            visit_native_polyface_attribute_updates(m, b, parameter_channel,
                                                    [&](std::size_t,
                                                        const NativePolyfaceVisitorFacet &,
                                                        const NativePolyfaceVisitedData &) {
                                                        ++calls;
                                                        m.data.coordinates.parameters.resize(3);
                                                    });
        });
        check(calls == 1,
              "updated UV count exposes native second triangle boundary read before next callback");
    }
    const auto source = make(3);
    {
        NativePolyfaceMesh m;
        m.mesh_style = 3;
        m.data.coordinates.points.resize(300);
        TubeBudget limited;
        limited.max_control_points = 750;
        std::size_t count = 0;
        auto v = visit_native_polyface_attribute_updates(
            m, limited, normal_channel,
            [&](std::size_t, const NativePolyfaceVisitorFacet &,
                const NativePolyfaceVisitedData &) { ++count; });
        check(count == 100 && v.complete && v.facets.empty(),
              "incremental traversal reuses per-face storage within a bounded budget");
        limited = TubeBudget{};
        limited.max_control_points = 750;
        rejects([&] { visit_native_polyface(m, limited, true, 0); });
    }
    TubeBudget b;
    b.max_control_points = 1;
    rejects([&] { build_native_polyface_mesh_normals(source, b); });
    b = TubeBudget{};
    b.max_work = 1;
    rejects([&] { build_native_polyface_mesh_parameters(source, 2, b); });
    rejects([&] { normal(make(2)); });
    auto task = [&] { return uv(source, 3).output.data.coordinates.parameters; };
    auto a = std::async(std::launch::async, task), c = std::async(std::launch::async, task);
    check(a.get() == c.get(),
          "typed generators are deterministic across concurrent independent calls");
    return checks;
}
