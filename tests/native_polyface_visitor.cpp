#include "native_polyface_visitor.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_visitor_tests() {
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
        check(caught, "visitor rejects unsafe native read or resource exhaustion");
    };
    auto base = [] {
        NativePolyfaceMesh s;
        s.data.coordinates.points = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
        s.data.indices.indices[point_channel] = {1, -2, 3, 4, 0};
        s.pool_active[native_point_pool] = true;
        return s;
    };
    auto visit = [](const NativePolyfaceMesh &s, bool all = true, std::uint32_t wrap = 1) {
        TubeBudget b;
        return visit_native_polyface(s, b, all, wrap);
    };
    auto tri = [](const NativePolyfaceMesh &s, std::size_t max_edges = 3) {
        TubeBudget b;
        return triangulate_native_polyface_mesh(s, b, max_edges);
    };
    const auto source = base();
    const auto v = visit(source);
    check(v.complete && v.facets.size() == 1 && v.read_indices == std::vector<std::size_t>{0},
          "indexed source produces one facet with original read position");
    check(v.facets[0].points == source.data.coordinates.points &&
              v.facets[0].client_indices[point_channel] ==
                  std::vector<std::int32_t>{0, 1, 2, 3, 0} &&
              v.facets[0].visible == std::vector<std::uint8_t>{1, 0, 1, 1, 1},
          "point values exclude wrap, client indices and visibility retain it");
    for (std::uint32_t wrap = 0; wrap < 12; ++wrap) {
        const auto r = visit(source, true, wrap);
        check(r.complete && r.facets[0].client_indices[point_channel].size() == 4 + wrap &&
                  r.facets[0].visible.size() == 4 + wrap,
              "wrap count is independent of all-data selection");
        for (std::size_t i = 0; i < wrap; ++i)
            check(r.facets[0].client_indices[point_channel][4 + i] == std::int32_t(i % 4) &&
                      r.facets[0].visible[4 + i] == std::uint8_t(i % 4 != 1),
                  "large wraps repeat the original client indices and visibility");
    }
    auto attrs = source;
    attrs.data.coordinates.normals = {{0, 0, 1}, {0, 1, 0}, {1, 0, 0}, {0, 0, -1}};
    attrs.data.coordinates.parameters = {{.1, .2}, {.3, .4}, {.5, .6}, {.7, .8}};
    attrs.integer_colors = {11, 22, 33, 44};
    attrs.pool_active[native_integer_color_pool] = true;
    attrs.data.face_data.resize(4);
    attrs.data.face_data[2].source_index = 987654;
    attrs.data.indices.active.fill(true);
    attrs.data.indices.indices[normal_channel] = {4, 3, -2, 1, 0};
    attrs.data.indices.indices[parameter_channel] = {2, 4, 1, 3, 0};
    attrs.data.indices.indices[color_channel] = {3, 1, 4, 2, 0};
    attrs.data.indices.indices[face_channel] = {2, -3, 1, 4, 0};
    const auto a = visit(attrs);
    check(a.complete &&
              a.facets[0].client_indices[normal_channel] ==
                  std::vector<std::int32_t>{3, 2, 1, 0, 3} &&
              a.facets[0].client_indices[parameter_channel] ==
                  std::vector<std::int32_t>{1, 3, 0, 2, 1} &&
              a.facets[0].client_indices[color_channel] ==
                  std::vector<std::int32_t>{2, 0, 3, 1, 2} &&
              a.facets[0].client_indices[face_channel] == std::vector<std::int32_t>{1, 2, 0, 3, 1},
          "all attribute client indices remain independent, zero-based and unsigned");
    const auto point_only = visit(attrs, false);
    check(point_only.complete && point_only.facets[0].client_indices[normal_channel].empty() &&
              point_only.facets[0].client_indices[face_channel].empty(),
          "point-only mode does not access attributes");
    {
        auto s = attrs;
        s.data.indices.indices[normal_channel].clear();
        s.data.indices.indices[parameter_channel].clear();
        s.data.indices.active.fill(false);
        auto r = visit(s);
        check(r.complete &&
                  r.facets[0].client_indices[normal_channel] ==
                      v.facets[0].client_indices[point_channel] &&
                  r.facets[0].client_indices[parameter_channel] ==
                      v.facets[0].client_indices[point_channel],
              "normal and UV pointer/count fallback ignores index activity");
        s.data.coordinates.normals.pop_back();
        r = visit(s);
        check(!r.complete && r.facets[0].client_indices[normal_channel].empty(),
              "different pool count prevents point-index fallback");
        s.data.indices.indices[color_channel].clear();
        s.data.indices.indices[face_channel].clear();
        r = visit(s);
        check(!r.complete && r.facets[0].client_indices[color_channel].empty() &&
                  r.facets[0].client_indices[face_channel].empty(),
              "colors and face data never inherit point indices");
    }
    {
        auto s = attrs;
        s.data.indices.indices[face_channel] = {2, 3};
        auto r = visit(s);
        check(!r.complete &&
                  r.facets[0].client_indices[face_channel] == std::vector<std::int32_t>{1, 2, 1},
              "face index reads use their own actual index count and wrap the prefix");
        s.data.indices.indices[normal_channel] = {1};
        rejects([&] { visit(s); });
        check(visit(s, false).complete, "point-only visitor avoids unsafe short attribute read");
        s.data.indices.indices[normal_channel] = {1, 0};
        r = visit(s);
        check(!r.complete &&
                  r.facets[0].client_indices[normal_channel] == std::vector<std::int32_t>{0, 0},
              "attribute zero stops before later out-of-array reads");
        s.data.indices.indices[normal_channel] = {1, 2, 3, 999, 0};
        r = visit(s);
        check(!r.complete && r.facets[0].client_indices[normal_channel] ==
                                 std::vector<std::int32_t>{0, 1, 2, 0},
              "invalid last normal is wrapped but still diagnosed as incomplete");
    }
    {
        auto s = source;
        s.data.indices.indices[point_channel] = {0, 0, 1, -2, 3, 0, 0, 4, 1, 2};
        auto r = visit(s);
        check(r.complete && r.read_indices == std::vector<std::size_t>{2, 7} &&
                  r.facets.size() == 2,
              "variable faces skip leading zeros and accept missing final terminator");
        s.data.num_per_face = 5;
        s.data.indices.indices[point_channel] = {1, 2, 3, 0, 0, 4, 3, 2, 0, 0};
        r = visit(s);
        check(r.complete && r.read_indices == std::vector<std::size_t>{0, 5},
              "fixed face stride skips zero padding");
        s.data.indices.indices[point_channel][5] = 0;
        r = visit(s);
        check(!r.complete && r.facets.size() == 1, "fixed leading zero stops subsequent traversal");
        s.data.indices.indices[point_channel] = {1, 2, 3, 0, 4};
        r = visit(s);
        check(!r.complete && r.facets.size() == 1,
              "unvisited nonzero fixed padding prevents complete");
        s.data.num_per_face = 0;
        s.data.indices.indices[point_channel] = {1, 2, 3, 999, 0, 1, 2, 3, 0};
        r = visit(s);
        check(
            !r.complete && r.facets.size() == 2 && r.facets[0].points.size() == 3 &&
                r.facets[0].client_indices[point_channel] == std::vector<std::int32_t>{0, 1, 2, 0},
            "native post-wrap size gate accepts truncated final point but does not prove complete");
        r = visit(s, true, 0);
        check(!r.complete && r.facets.empty(),
              "without wrap the same invalid face stops traversal");
        s.data.indices.indices[point_channel][1] = INT32_MIN;
        r = visit(s);
        check(!r.complete && r.facets.empty(), "signed minimum invalid point safely stops advance");
    }
    // Every combination of active color pools and physically present arrays:
    // pointer chooses double/float/int/table; count chooses active float/double/int/table.
    for (unsigned present = 0; present < 16; ++present)
        for (unsigned active = 0; active < 16; ++active) {
            auto s = source;
            if (present & 1)
                s.double_colors.resize(4);
            if (present & 2)
                s.float_colors.resize(4);
            if (present & 4)
                s.integer_colors.resize(4);
            if (present & 8)
                s.color_table.resize(4);
            for (unsigned k = 0; k < 4; ++k)
                s.pool_active[native_double_color_pool + k] = (active & (1u << k)) != 0;
            s.data.indices.indices[color_channel] = {4, 2, 3, 1, 0};
            unsigned selected = 8, count_source = 8;
            for (unsigned k = 0; k < 4; ++k)
                if (present & (1u << k)) {
                    selected = native_double_color_pool + k;
                    break;
                }
            for (unsigned k : {1u, 0u, 2u, 3u})
                if (active & (1u << k)) {
                    count_source = native_double_color_pool + k;
                    break;
                }
            const bool usable = selected != 8 && count_source != 8 &&
                                (present & (1u << (count_source - native_double_color_pool)));
            const auto r = visit(s);
            check(r.report["color_value_pool"] == selected &&
                      r.report["color_count_pool"] == count_source &&
                      r.facets[0].client_indices[color_channel] ==
                          (usable ? std::vector<std::int32_t>{3, 1, 2, 0, 3}
                                  : std::vector<std::int32_t>{}),
                  "color pointer presence and active-count priorities are independent");
            check(r.complete == usable, "unresolved color references prevent complete status");
        }
    {
        auto s = source;
        s.double_colors.resize(2);
        s.float_colors.resize(4);
        s.pool_active[native_float_color_pool] = true;
        s.data.indices.indices[color_channel] = {1, 2, 3, 4, 0};
        rejects([&] { visit(s); });
        check(visit(s, false).complete,
              "color pointer/count mismatch is irrelevant to point-only queries");
        s.double_colors.resize(4);
        s.float_colors.resize(2);
        auto r = visit(s);
        check(!r.complete &&
                  r.facets[0].client_indices[color_channel] == std::vector<std::int32_t>{0, 1, 0},
              "smaller native color count truncates even when selected pool has more elements");
    }
    {
        auto s = attrs;
        s.num_per_row = 71;
        s.index_rows.fill(13);
        s.pool_rows.fill(19);
        s.data.two_sided = true;
        s.data.edge_chains.push_back({24, {0, 1}, {1, 2}});
        const auto r = tri(s);
        check(r.native_succeeded && r.complete &&
                  r.output.data.indices.indices[point_channel].size() == 8,
              "typed raw quad conversion and visitor feed triangle consumer");
        const auto &out = r.output.data.indices.indices;
        for (std::size_t i = 0; i < out[point_channel].size(); ++i) {
            const auto point = out[point_channel][i];
            if (!point) {
                for (std::size_t c = 1; c < polyface_channel_count; ++c)
                    check(out[c][i] == 0, "independent channels retain matching face delimiters");
            } else {
                const auto k = std::size_t(std::abs(point) - 1);
                for (std::size_t c = 1; c < polyface_channel_count; ++c)
                    check(out[c][i] == std::abs(s.data.indices.indices[c][k]),
                          "triangle output preserves per-corner source attribute mapping");
                if (k == 1)
                    check(point < 0, "original hidden edge sign survives triangulation");
            }
        }
        check(r.output.data.coordinates.points == s.data.coordinates.points &&
                  r.output.data.coordinates.normals == s.data.coordinates.normals &&
                  r.output.data.coordinates.parameters == s.data.coordinates.parameters &&
                  r.output.integer_colors == s.integer_colors &&
                  r.output.data.face_data[2].source_index == 987654 &&
                  r.output.data.edge_chains[0].point_indices == std::vector<std::int32_t>{1, 2} &&
                  r.output.data.two_sided && r.output.pool_rows == s.pool_rows &&
                  r.output.index_rows == s.index_rows && r.output.num_per_row == 0,
              "triangulation preserves source pools and metadata except native global row reset");
        const auto retained = tri(s, 4);
        check(retained.complete && retained.output.data.indices.indices[point_channel] ==
                                       s.data.indices.indices[point_channel],
              "larger edge limit retains source quad");
        check(s.num_per_row == 71 && s.data.indices.indices[point_channel] ==
                                         source.data.indices.indices[point_channel],
              "typed pipeline leaves source untouched");
    }
    for (auto style : {3u, 4u, 5u, 6u}) {
        auto s = attrs;
        s.mesh_style = style;
        if (style == 3) {
            s.data.coordinates.points.pop_back();
            s.data.coordinates.normals.pop_back();
            s.data.coordinates.parameters.pop_back();
            s.integer_colors.pop_back();
        }
        s.pool_active.fill(true);
        // Face indices are not synthesized by native non-indexed layout conversion.
        s.data.face_data.clear();
        s.data.indices.indices[face_channel].clear();
        s.pool_rows[native_point_pool] = 2;
        const auto r = tri(s);
        check(r.native_succeeded && r.complete && r.output.mesh_style == 1 &&
                  r.output.data.num_per_face == 0 && r.output.num_per_row == 0,
              "all supported coordinate styles reach typed triangulation");
    }
    {
        auto s = source;
        s.mesh_style = 17;
        rejects([&] { visit(s); });
        rejects([&] { tri(s); });
        TubeBudget b;
        b.max_control_points = 5;
        rejects([&] { visit_native_polyface(source, b); });
        b = TubeBudget{};
        b.max_work = 1;
        rejects([&] { visit_native_polyface(source, b); });
        b = TubeBudget{};
        rejects([&] { visit_native_polyface(source, b, true, UINT32_MAX); });
        s = source;
        s.data.indices.indices[point_channel] = {1, 999, 3, 4, 0};
        auto r = tri(s);
        check(r.native_succeeded && !r.complete &&
                  r.output.data.indices.indices[point_channel].empty(),
              "native successful empty output after failed advance is not complete");
        s = attrs;
        s.data.indices.indices[normal_channel] = {999, 2, 3, 4, 0};
        r = tri(s);
        check(r.native_succeeded && !r.complete && !r.output.data.indices.active[normal_channel] &&
                  r.output.data.indices.indices[normal_channel].empty(),
              "raw visitor reaches native missing-normal drop");
        s = attrs;
        s.data.indices.indices[color_channel] = {999, 2, 3, 4, 0};
        r = tri(s);
        check(!r.native_succeeded && !r.complete && !r.output.data.indices.active[color_channel],
              "raw visitor reaches distinct missing-color error");
    }
    auto worker = [attrs, tri] { return tri(attrs).output.data.indices.indices; };
    auto f0 = std::async(std::launch::async, worker), f1 = std::async(std::launch::async, worker);
    check(f0.get() == f1.get(), "typed visitor and triangulation have no shared mutable state");
    return checks;
}
