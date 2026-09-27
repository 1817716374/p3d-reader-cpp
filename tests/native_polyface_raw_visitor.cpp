#include "native_polyface_visitor.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_raw_visitor_tests() {
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
        check(caught, "unsafe original reads and exhausted budgets are rejected");
    };
    auto make = [](unsigned style, unsigned count, unsigned width = 0) {
        NativePolyfaceMesh m;
        m.mesh_style = style;
        m.num_per_row = width;
        m.pool_rows[native_point_pool] = 99; // Raw visitor uses global width.
        m.data.num_per_face = 17;            // Raw blocks use the style's own width.
        for (unsigned i = 0; i < count; ++i)
            m.data.coordinates.points.push_back({double(i), double(i % 3), 0});
        m.data.indices.indices[point_channel] = {INT32_MIN}; // Ignored by raw visitors.
        return m;
    };
    auto visit = [](const NativePolyfaceMesh &m, bool all = true, unsigned wrap = 1) {
        TubeBudget b;
        return visit_native_polyface(m, b, all, wrap);
    };
    auto attributes = [](NativePolyfaceMesh &m) {
        for (std::size_t i = 0; i < m.data.coordinates.points.size(); ++i) {
            m.data.coordinates.normals.push_back({double(i), 1, 2});
            m.data.coordinates.parameters.push_back({double(i), .25});
            m.double_colors.push_back({double(i), .5, .75});
            m.integer_colors.push_back(unsigned(i + 100));
            m.color_table.push_back(unsigned(i + 200));
        }
        m.pool_active[native_double_color_pool] = true;
    };
    for (unsigned style : {3u, 4u}) {
        auto m = make(style, style * 2);
        attributes(m);
        for (unsigned wrap : {0u, 1u, 9u, 12u}) {
            const auto v = visit(m, true, wrap);
            check(v.complete && v.facets.size() == 2 && v.data.size() == 2 &&
                      v.read_indices == std::vector<std::size_t>{0, 1},
                  "blocked visitor advances by face ordinal and ignores source indices");
            for (std::size_t face = 0; face < 2; ++face) {
                const auto &f = v.facets[face];
                const auto &d = v.data[face];
                check(f.points.size() == style && f.visible.size() == style + wrap &&
                          d.normals.size() == style + wrap && d.parameters.size() == style + wrap &&
                          d.double_colors.size() == style + wrap &&
                          d.integer_colors.size() == style + wrap &&
                          d.color_table.size() == style + wrap,
                      "blocked values append independent color pools and repeated wrap");
                for (auto c : {normal_channel, parameter_channel, color_channel, face_channel})
                    check(f.client_indices[c].empty(),
                          "blocked attribute index arrays stay inactive");
                for (std::size_t i = 0; i < style + wrap; ++i) {
                    const auto k = face * style + i % style;
                    check(f.client_indices[point_channel][i] == int(k) && f.visible[i] &&
                              d.normals[i][0] == double(k) && d.parameters[i][0] == double(k) &&
                              d.integer_colors[i] == k + 100 && d.color_table[i] == k + 200 &&
                              d.index_positions[i] == face * style + i,
                          "blocked wrap repeats values but index positions keep increasing");
                }
            }
        }
        auto v = visit(m, false);
        check(v.complete && v.data[0].normals.empty() && v.data[0].integer_colors.empty(),
              "blocked all-data false skips attributes");
        m.float_colors.resize(style * 2);
        m.data.face_data.resize(style * 2);
        v = visit(m);
        check(!v.complete && v.data[0].float_colors.empty() && v.data[0].face_data.empty() &&
                  v.report["ignored_float_color_values"] == style * 2 &&
                  v.report["ignored_face_data_values"] == style * 2,
              "unsupported raw block payloads are reported without fabricated indices");
        check(visit(m, false).complete,
              "unrequested block attributes do not reduce point-only completeness");
    }
    {
        auto m = make(3, 4);
        check(!visit(m).complete && visit(m).report["unused_source_points"] == 1,
              "partial point block remains unconsumed");
        m = make(3, 6);
        m.data.coordinates.normals = {{10, 0, 0}, {20, 0, 0}, {30, 0, 0}, {40, 0, 0}};
        auto v = visit(m, true, 3);
        check(!v.complete && v.data[1].normals.size() == 4 && v.data[1].normals.back()[0] == 40,
              "partial attribute block wraps its own available prefix");
        m.data.coordinates.normals.resize(3);
        v = visit(m, true, 0);
        check(!v.complete && v.data[1].normals.empty(),
              "empty later attribute prefix is safe without wrap");
        rejects([&] { visit(m); });
        check(visit(m, false).complete, "blocked point-only avoids unsafe attribute wrap");
        m = make(3, 3);
        m.double_colors.resize(1);
        m.integer_colors.resize(3);
        m.pool_active[native_integer_color_pool] = true;
        rejects([&] { visit(m); });
    }
    for (unsigned style : {5u, 6u}) {
        for (unsigned width : {2u, 3u}) {
            for (unsigned height : {2u, 3u}) {
                auto m = make(style, width * height, width);
                attributes(m);
                m.color_table.clear();
                m.data.face_data.resize(width * height);
                for (unsigned i = 0; i < width * height; ++i)
                    m.data.face_data[i].source_index = i + 300;
                for (unsigned requested : {0u, 1u, 10u, 25u}) {
                    const auto v = visit(m, false, requested);
                    const auto wrap = std::min(requested, 10u);
                    const auto faces = (width - 1) * (height - 1) * (style == 5 ? 2 : 1);
                    check(
                        v.complete && v.facets.size() == faces &&
                            v.report["all_data_effective"] == true &&
                            v.report["actual_wrap"] == wrap,
                        "grid reads all attributes regardless of all-data request and clamps wrap");
                    for (unsigned face = 0; face < faces; ++face) {
                        const auto quad = style == 5 ? face / 2 : face;
                        const auto start = quad + quad / (width - 1), lower = start + width;
                        std::vector<unsigned> expected =
                            style == 6 ? std::vector<unsigned>{start, start + 1, lower + 1, lower}
                            : face % 2 ? std::vector<unsigned>{lower, start + 1, lower + 1}
                                       : std::vector<unsigned>{start, start + 1, lower};
                        const auto n = expected.size();
                        for (unsigned i = 0; i < wrap; ++i) {
                            const auto k = expected[i];
                            expected.push_back(k);
                        }
                        const auto &f = v.facets[face];
                        const auto &d = v.data[face];
                        check(v.read_indices[face] == face && f.points.size() == n &&
                                  d.index_positions.empty() &&
                                  f.client_indices[color_channel].size() == 2 * expected.size(),
                              "grid ordinals, empty positions and concatenated double/integer "
                              "color indices are preserved");
                        for (std::size_t i = 0; i < expected.size(); ++i) {
                            const auto k = expected[i];
                            check(f.client_indices[point_channel][i] == int(k) &&
                                      f.client_indices[normal_channel][i] == int(k) &&
                                      f.client_indices[color_channel][i] == int(k) &&
                                      f.client_indices[color_channel][i + expected.size()] ==
                                          int(k) &&
                                      d.normals[i][0] == double(k) &&
                                      d.parameters[i][0] == double(k) &&
                                      d.integer_colors[i] == k + 100 &&
                                      d.face_data[i].source_index == k + 300,
                                  "grid preserves exact triangle starts, attribute values and "
                                  "closure");
                        }
                    }
                }
                check(m.data.indices.indices[point_channel] ==
                              std::vector<std::int32_t>{INT32_MIN} &&
                          m.num_per_row == width && m.pool_rows[native_point_pool] == 99,
                      "raw visits leave input values and layout unchanged");
            }
        }
    }
    {
        auto m = make(5, 4, 2);
        attributes(m);
        auto v = visit(m);
        check(!v.complete && v.data[0].color_table.empty() &&
                  v.report["ignored_color_table_values"] == 4,
              "integer pointer suppresses table payload with an explicit omission report");
        m.integer_colors.clear();
        v = visit(m);
        check(v.complete &&
                  v.data[0].color_table == std::vector<std::uint32_t>{200, 201, 202, 200} &&
                  v.facets[0].client_indices[color_channel].size() == 8,
              "without integer pointer double and table are both appended");
        m.float_colors.resize(4);
        m.pool_active[native_float_color_pool] = true;
        check(!visit(m).complete && visit(m).data[0].float_colors.empty(),
              "grid never reads float colors");
        m.double_colors.resize(2);
        rejects([&] { visit(m, false); });
        for (unsigned style : {5u, 6u}) {
            rejects([&] { visit(make(style, 7, 3)); });
            rejects([&] { visit(make(style, 0, 1)); });
            check(visit(make(style, 0, 0)).complete, "empty zero-width grid stops before reading");
            rejects([&] { visit(make(style, 1, 0)); });
            auto bad = make(style, 4, 2);
            bad.data.coordinates.normals.resize(3);
            rejects([&] { visit(bad, false); });
            bad.data.coordinates.normals.clear();
            bad.data.face_data.resize(3);
            rejects([&] { visit(bad); });
        }
    }
    {
        auto m = make(1, 3);
        attributes(m);
        m.data.num_per_face = 0;
        m.data.indices.indices[point_channel] = {3, 1, 2, 0};
        m.data.indices.indices[normal_channel] = {2, 3, 1, 0};
        m.data.indices.indices[parameter_channel] = {1, 2, 3, 0};
        m.data.indices.indices[color_channel] = {3, 2, 1, 0};
        const auto v = visit(m);
        check(v.complete && v.data[0].normals[0][0] == 1 && v.data[0].normals.back()[0] == 1 &&
                  v.data[0].parameters[0][0] == 0 && v.data[0].double_colors[0][0] == 2 &&
                  v.data[0].integer_colors.empty() &&
                  v.data[0].index_positions == std::vector<std::size_t>{0, 1, 2, 0},
              "indexed values follow independent indices and retain wrapped index positions");
    }
    const auto source = make(5, 6, 3);
    for (unsigned style : {3u, 4u, 5u, 6u}) {
        const auto m = make(style, 12, 3);
        rejects([&] { visit(m, true, UINT32_MAX); });
        TubeBudget b;
        b.max_control_points = 1;
        rejects([&] { visit_native_polyface(m, b); });
        b = TubeBudget{};
        b.max_work = 1;
        rejects([&] { visit_native_polyface(m, b); });
    }
    rejects([&] { visit(make(2, 0)); });
    auto task = [&] { return visit(source).report; };
    auto a = std::async(std::launch::async, task), b = std::async(std::launch::async, task);
    check(a.get() == b.get(), "concurrent raw visitors have independent deterministic state");
    return checks;
}
