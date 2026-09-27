#include "native_polyface_layout.hpp"
#include "native_polygon_convexity.hpp"
#include "native_polyface_attributes.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_layout_tests() {
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
        check(caught, "layout conversion respects storage and work budgets");
    };
    auto convert = [](const NativePolyfaceLayout &s) {
        TubeBudget b;
        return convert_native_polyface_layout(s, b);
    };
    auto activate = [](const NativePolyfaceLayout &s) {
        TubeBudget b;
        return activate_native_polyface_layout(s, b);
    };
    auto original = [] {
        NativePolyfaceLayout s;
        s.num_per_face = 4;
        s.num_per_row = 77;
        s.pools[native_point_pool] = {4, 3, true};
        s.indices.indices[point_channel] = {1, -2, 3, 0, 1, 3, 4, 0};
        s.index_structs_per_row[point_channel] = 4;
        return s;
    };
    const auto source = original();
    const auto base = convert(source);
    check(base.native_succeeded && base.complete && base.output.num_per_face == 0 &&
              base.output.mesh_style == 1 && base.output.num_per_row == 77 &&
              base.output.pools[native_point_pool].structs_per_row == 3 &&
              base.output.index_structs_per_row[point_channel] == 0 &&
              base.output.indices.indices[point_channel] == source.indices.indices[point_channel],
          "blocked padded faces preserve signed indices and unrelated metadata while clearing "
          "channel block width");
    {
        auto s = source;
        s.indices.indices[point_channel] = {0, 0, 0, 0, 1, 2, 0, 0, 3, 4, 5, 6, 8, 9};
        const auto r = convert(s);
        check(r.native_succeeded && !r.complete && r.report["discarded_tail_indices"] == 2 &&
                  r.output.indices.indices[point_channel] ==
                      std::vector<std::int32_t>{0, 1, 2, 0, 3, 4, 5, 6, 0},
              "blocked conversion collapses repeated zeros including across rows and drops "
              "incomplete tail");
        s.indices.indices[point_channel] = {1, 2};
        check(convert(s).output.indices.indices[point_channel].empty() && !convert(s).complete,
              "partial first block becomes empty instead of an invented short face");
        s.indices.indices[point_channel] = {INT32_MIN, INT32_MAX, 0, 0};
        check(convert(s).output.indices.indices[point_channel] ==
                  std::vector<std::int32_t>{INT32_MIN, INT32_MAX, 0},
              "layout conversion preserves raw signed index bits without claiming valid point "
              "references");
        s = source;
        s.indices.indices[point_channel] = {1, 2, 3, 4, 5, 6, 7, 8};
        check(convert(s).output.indices.indices[point_channel] ==
                  std::vector<std::int32_t>{1, 2, 3, 4, 0, 5, 6, 7, 8, 0},
              "dense fixed blocks gain one terminator each");
        s = source;
        s.num_per_face = 9;
        s.indices.active.fill(true);
        s.indices.indices[normal_channel] = {11, 12, 13, 14, 15, 16};
        s.index_structs_per_row[normal_channel] = 3;
        s.indices.indices[parameter_channel] = {21, -22, 23};
        s.index_structs_per_row[parameter_channel] = 1;
        s.indices.indices[face_channel] = {31, 32, 33, 34};
        s.index_structs_per_row[face_channel] = 2;
        const auto independent = convert(s);
        check(independent.complete &&
                  independent.output.indices.indices[normal_channel] ==
                      std::vector<std::int32_t>{11, 12, 13, 0, 14, 15, 16, 0} &&
                  independent.output.indices.indices[parameter_channel] ==
                      std::vector<std::int32_t>{21, -22, 23} &&
                  independent.output.indices.indices[face_channel] ==
                      std::vector<std::int32_t>{31, 32, 0, 33, 34, 0},
              "each active index buffer uses its own row width rather than global numPerFace");
        check(independent.output.index_structs_per_row[parameter_channel] == 1,
              "block helper with width zero or one leaves the vector and row metadata unchanged");
        s.indices.active[normal_channel] = false;
        check(convert(s).output.indices.indices[normal_channel] ==
                      s.indices.indices[normal_channel] &&
                  convert(s).output.index_structs_per_row[normal_channel] == 3,
              "inactive fixed index channel is preserved even while global face layout becomes "
              "variable");
        s = source;
        s.num_per_face = 1;
        s.indices.indices[point_channel] = {1, 2, 3};
        const auto variable = convert(s);
        check(variable.complete &&
                  variable.output.indices.indices[point_channel] ==
                      std::vector<std::int32_t>{1, 2, 3} &&
                  variable.output.index_structs_per_row[point_channel] == 4,
              "native variable layout does not add newer-reference missing final terminators");
    }
    {
        // Independent block oracle: concatenate complete blocks, then collapse zero runs.
        for (std::uint32_t width = 2; width < 8; ++width)
            for (std::size_t n = 0; n < 25; ++n) {
                auto s = source;
                s.index_structs_per_row[point_channel] = width;
                s.indices.indices[point_channel].clear();
                for (std::size_t i = 0; i < n; ++i)
                    s.indices.indices[point_channel].push_back(
                        i % 5 < 2 ? 0 : -static_cast<int>(i + 1));
                std::vector<std::int32_t> expected;
                for (std::size_t row = 0; row < n / width; ++row) {
                    for (std::size_t k = 0; k < width; ++k)
                        expected.push_back(s.indices.indices[point_channel][row * width + k]);
                    expected.push_back(0);
                }
                expected.erase(std::unique(expected.begin(), expected.end(),
                                           [](auto a, auto b) { return a == 0 && b == 0; }),
                               expected.end());
                const auto r = convert(s);
                check(r.output.indices.indices[point_channel] == expected &&
                          r.complete == (n % width == 0),
                      "blocked conversion agrees with independent row-and-separator construction");
            }
    }
    {
        for (std::uint32_t style : {3, 4}) {
            NativePolyfaceLayout s;
            s.mesh_style = style;
            s.num_per_face = 9;
            s.num_per_row = 73;
            const std::size_t width = style == 3 ? 3 : 4;
            s.pools[native_point_pool] = {2 * width + 1, 71, true};
            s.pools[native_normal_pool] = {0, 63, true};
            s.pools[native_parameter_pool] = {1, 19, false};
            s.indices.active.fill(false);
            s.index_structs_per_row.fill(17);
            for (auto &i : s.indices.indices)
                i = {91, -92};
            const auto r = convert(s);
            std::vector<std::int32_t> expected;
            for (std::size_t row = 0; row < 2; ++row) {
                for (std::size_t k = 0; k < width; ++k)
                    expected.push_back(static_cast<std::int32_t>(1 + row * width + k));
                expected.push_back(0);
            }
            check(r.native_succeeded && !r.complete &&
                      r.report["unreferenced_partial_row_points"] == 1 &&
                      r.output.indices.indices[point_channel] == expected &&
                      r.output.indices.indices[normal_channel] == expected,
                  "coordinate-face generation uses point count and pool activity even for an empty "
                  "active normal pool");
            check(r.output.indices.indices[parameter_channel] ==
                          s.indices.indices[parameter_channel] &&
                      r.output.indices.indices[face_channel] == s.indices.indices[face_channel] &&
                      r.output.indices.active == s.indices.active &&
                      r.output.index_structs_per_row == s.index_structs_per_row &&
                      r.output.pools[native_point_pool].structs_per_row == 71 &&
                      r.output.num_per_row == 73,
                  "generation changes only selected buffers and global style/face width, not flags "
                  "or row metadata");
            const auto active = activate(r.output);
            check(active.output.indices.active[point_channel] &&
                      !active.output.indices.active[normal_channel] &&
                      active.output.indices.active[parameter_channel],
                  "separate availability step uses actual pool presence rather than earlier "
                  "generation gates");
        }
    }
    {
        for (std::uint32_t style : {5, 6}) {
            NativePolyfaceLayout s;
            s.mesh_style = style;
            s.num_per_row = 99;
            s.pools[native_point_pool] = {6, 3, true};
            const auto r = convert(s);
            const auto expected =
                style == 5
                    ? std::vector<std::int32_t>{1, 2, 4, 0, 2, 5, 4, 0, 2, 3, 5, 0, 3, 6, 5, 0}
                    : std::vector<std::int32_t>{1, 2, 5, 4, 0, 2, 3, 6, 5, 0};
            check(r.complete && r.output.indices.indices[point_channel] == expected &&
                      r.output.num_per_row == 99 &&
                      r.output.pools[native_point_pool].structs_per_row == 3,
                  "grid uses point-vector row width and native winding/diagonal order, ignoring "
                  "global row metadata");
            auto tail = s;
            tail.pools[native_point_pool].count = 7;
            check(!convert(tail).complete &&
                      convert(tail).output.indices.indices[point_channel] == expected,
                  "grid partial final row is not turned into extra cells");
            auto tall = s;
            tall.pools[native_point_pool].structs_per_row = 2;
            const auto tall_expected =
                style == 5
                    ? std::vector<std::int32_t>{1, 2, 3, 0, 2, 4, 3, 0, 3, 4, 5, 0, 4, 6, 5, 0}
                    : std::vector<std::int32_t>{1, 2, 4, 3, 0, 3, 4, 6, 5, 0};
            check(
                convert(tall).output.indices.indices[point_channel] == tall_expected,
                "multiple grid rows advance by point-vector row width in original row-major order");
            for (std::uint32_t width : {0, 1, 7}) {
                auto empty = s;
                empty.pools[native_point_pool].structs_per_row = width;
                empty.indices.indices[point_channel] = {1, 2, 3, 0};
                const auto e = convert(empty);
                check(e.native_succeeded && !e.complete &&
                          e.output.indices.indices[point_channel].empty() &&
                          e.report["unreferenced_partial_row_points"] == 6,
                      "grid with no complete cell clears requested indices and reports uncovered "
                      "points");
            }
        }
    }
    {
        // All four SDK color representations participate in activation, but
        // float colors alone do not trigger the original layout-generation path.
        for (unsigned mask = 0; mask < 16; ++mask)
            for (std::uint32_t style : {3, 5}) {
                NativePolyfaceLayout s;
                s.mesh_style = style;
                s.pools[native_point_pool] = {6, 3, true};
                s.indices.active.fill(false);
                s.indices.indices[color_channel] = {99};
                for (std::size_t k = 0; k < 4; ++k)
                    s.pools[native_double_color_pool + k] = {mask & (1u << k) ? 6u : 0u, 17,
                                                             bool(mask & (1u << k))};
                const auto r = convert(s);
                const bool generates = bool(mask & 13);
                check((r.output.indices.indices[color_channel] ==
                       r.output.indices.indices[point_channel]) == generates &&
                          r.output.indices.active[color_channel] == false,
                      "double, integer and table color activity generates indices; float-only "
                      "activity does not");
                const auto a = activate(r.output);
                check(a.complete && a.output.indices.active[color_channel] == bool(mask) &&
                          a.output.indices.indices[color_channel] ==
                              r.output.indices.indices[color_channel],
                      "available-data activation includes float colors but never repairs or "
                      "generates indices");
            }
        NativePolyfaceLayout s;
        s.pools[native_face_data_pool] = {1, 4, false};
        s.indices.indices[face_channel] = {999};
        s.indices.indices[normal_channel] = {1};
        s.indices.active.fill(true);
        s.pools[native_normal_pool].active = true;
        const auto a = activate(s);
        check(a.complete && a.output.indices.active[face_channel] &&
                  !a.output.indices.active[normal_channel] &&
                  !a.output.indices.active[point_channel] &&
                  a.report["references_checked"] == false &&
                  a.output.indices.indices[face_channel] == std::vector<std::int32_t>{999},
              "active flags reflect pool/buffer presence, not index validity or prior flag values");
        for (std::uint32_t style : {0u, 2u, 7u, UINT32_MAX}) {
            auto invalid = source;
            invalid.mesh_style = style;
            const auto r = convert(invalid);
            check(!r.native_succeeded && !r.complete && r.output.mesh_style == style &&
                      r.output.num_per_face == source.num_per_face &&
                      r.output.indices.indices == source.indices.indices,
                  "unsupported native styles return false without rewriting layout");
        }
    }
    {
        NativePolyfaceLayout s;
        s.mesh_style = 6;
        s.pools[native_point_pool] = {6, 3, true};
        const auto a = activate(convert(s).output);
        NativePolyfaceFaceDataState mesh;
        mesh.mesh.coordinates.points = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0},
                                        {0, 1, 0}, {1, 1, 0}, {2, 1, 0}};
        mesh.mesh.indices = a.output.indices;
        mesh.mesh.num_per_face = a.output.num_per_face;
        TubeBudget b;
        const auto query = query_native_polyface_facets(mesh, b);
        check(query.complete && query.has_convex_facets && query.max_facet_size == 4,
              "converted and activated grid indices feed the existing point visitor without "
              "inferred sharing");
        const auto normal = build_native_polyface_normals(mesh, b);
        check(normal.complete &&
                  normal.output.mesh.coordinates.normals == std::vector<Point3>(2, Point3{0, 0, 1}),
              "native grid layout feeds per-face geometry and preserves winding");
    }
    {
        for (bool activation : {false, true}) {
            TubeBudget measured;
            auto run = [&](TubeBudget &b) {
                return activation ? activate_native_polyface_layout(source, b)
                                  : convert_native_polyface_layout(source, b);
            };
            run(measured);
            for (auto n : {std::size_t(0), measured.work / 2, measured.work - 1}) {
                TubeBudget b;
                b.max_work = n;
                rejects([&] { run(b); });
            }
            TubeBudget b;
            b.max_control_points = 11;
            rejects([&] { run(b); });
        }
        auto growing = source;
        growing.mesh_style = 5;
        growing.pools[native_point_pool] = {6, 3, true};
        TubeBudget limited;
        limited.max_control_points = 18;
        rejects([&] { convert_native_polyface_layout(growing, limited); });
        auto task = [&] { return convert(source); };
        auto a = std::async(std::launch::async, task), b = std::async(std::launch::async, task);
        const auto x = a.get(), y = b.get();
        check(x.complete && y.complete && x.report == y.report &&
                  x.output.indices.indices == y.output.indices.indices &&
                  source.indices.indices == original().indices.indices && source.num_per_face == 4,
              "parallel conversions are deterministic and all failures leave input unchanged");
    }
    return checks;
}
