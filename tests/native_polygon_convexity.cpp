#include "native_polygon_convexity.hpp"
#include "native_polyface_triangulate.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polygon_convexity_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto near = [](double a, double b) { return std::abs(a - b) < 1e-12; };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "facet query rejects unsafe arithmetic and exhausted budgets");
    };
    auto convex = [](const std::vector<Point3> &p) {
        TubeBudget b;
        return native_polygon_convexity(p, b);
    };
    auto query = [](const NativePolyfaceFaceDataState &s) {
        TubeBudget b;
        return query_native_polyface_facets(s, b);
    };
    const std::vector<Point3> square{{0, 0, 0}, {2, 0, 0}, {2, 1, 0}, {0, 1, 0}};
    const auto original = convex(square);
    check(original.convex && original.unit_normal == Point3{0, 0, 1} &&
              original.positive_turn_sum == 8 && original.negative_turn_sum == 0,
          "rectangle has analytical normal and positive turning sum");
    {
        auto reversed = square;
        std::reverse(reversed.begin(), reversed.end());
        const auto r = convex(reversed);
        check(r.convex && r.unit_normal == Point3{0, 0, -1} && r.positive_turn_sum == 8,
              "reversing polygon winding reverses the reference normal without changing convexity");
        auto spatial = square;
        for (auto &p : spatial)
            p = {5 + p[0], 7, 9 + p[1]};
        const auto q = convex(spatial);
        check(q.convex && q.unit_normal == Point3{0, -1, 0} && q.positive_turn_sum == 8,
              "convexity uses spatial cross products, not XY-only projection");
        auto closed = square;
        closed.push_back(square.front());
        closed.push_back(square.front());
        const auto c = convex(closed);
        check(c.convex && c.report["trailing_copies_removed"] == 2 && c.positive_turn_sum == 8,
              "all exact trailing copies of the first point are ignored");
        closed.back()[0] = 1e-15;
        check(convex(closed).report["trailing_copies_removed"] == 0,
              "nearby closing coordinates are not removed by a geometric tolerance");
        const auto warped = convex({{0, 0, 0}, {1, 0, 0}, {1, 1, 1}, {0, 1, 0}});
        check(warped.convex && near(warped.unit_normal[0], 0) &&
                  near(warped.unit_normal[1], -std::sqrt(.5)) &&
                  near(warped.unit_normal[2], std::sqrt(.5)),
              "first largest cross wins ties and a nonplanar quad can pass the native test");
    }
    {
        for (double dent : {.1, 4e-12, 2e-12}) {
            const auto r = convex({{0, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1, 1 - dent, 0}, {0, 1, 0}});
            check(
                r.convex == (dent == 2e-12) && near(r.positive_turn_sum, 6) &&
                    near(r.negative_turn_sum, -2 * dent),
                "relative signed-turn criterion tolerates tiny concavity but rejects larger dents");
        }
        for (const auto &p :
             std::vector<std::vector<Point3>>{{},
                                              {{0, 0, 0}},
                                              {{0, 0, 0}, {1, 0, 0}},
                                              {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}},
                                              {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}},
                                              std::vector<Point3>(4, Point3{7, 8, 9})}) {
            check(!convex(p).convex,
                  "short or completely collapsed polygon does not pass strict area ratio");
        }
        const auto r = convex({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}});
        check(
            r.unit_normal == Point3{1, 0, 0} && r.positive_turn_sum == 0 &&
                r.negative_turn_sum == 0,
            "zero reference vector uses native positive-X fallback but zero turn sum still fails");
    }
    NativePolyfaceFaceDataState mesh;
    mesh.mesh.coordinates.points = square;
    mesh.mesh.indices.indices[point_channel] = {1, 2, 3, 4, 0};
    {
        const auto r = query(mesh);
        check(r.complete && r.has_convex_facets && r.max_facet_size == 4 &&
                  r.report["polygon_tests"].size() == 1,
              "prepared indexed mesh queries both largest face and spatial convexity");
        auto s = mesh;
        s.mesh.coordinates.normals = {{0, 0, 1}};
        s.mesh.indices.indices[normal_channel] = {999};
        s.mesh.coordinates.parameters = {{7, 8}};
        s.mesh.indices.indices[parameter_channel] = {-99};
        const auto ignored = query(s);
        check(ignored.complete && ignored.has_convex_facets && ignored.max_facet_size == 4,
              "point-only native queries do not access unsafe unrelated attribute channels");
        s = mesh;
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 2, 0};
        s.mesh.coordinates.points = std::vector<Point3>(4);
        const auto degenerate = query(s);
        check(degenerate.complete && degenerate.has_convex_facets &&
                  degenerate.max_facet_size == 3 && degenerate.report["polygon_tests"].empty(),
              "hasConvexFacets accepts short and triangle faces without calling polygon convexity");
        s = mesh;
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 2, 99, 0, 1, 2, 3, 4, 0};
        const auto partial = query(s);
        check(
            !partial.complete && partial.has_convex_facets && partial.max_facet_size == 3 &&
                partial.report["unvisited_nonzero_indices"] == 7,
            "early invalid-point termination can retain native true while the query is incomplete");
        s = mesh;
        s.mesh.num_per_face = 4;
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 3, 4, 0};
        check(query(s).complete && query(s).max_facet_size == 3,
              "maximum face size comes from actual visitor corners, not fixed block width");
        s.mesh.indices.indices[point_channel] = {0, 0, 0, 0, 1, 2, 3, 0};
        check(!query(s).complete && query(s).max_facet_size == 0,
              "fixed empty block stops traversal and exposes unvisited later corners");
        s = mesh;
        s.mesh.indices.indices[point_channel] = {0, 1, 2, 3, 4, 0, 0, 1, 2, 3};
        check(query(s).complete && query(s).max_facet_size == 4,
              "variable zero padding and unterminated final face retain original visitor behavior");
        s.mesh.indices.indices[point_channel].clear();
        check(query(s).complete && query(s).has_convex_facets && query(s).max_facet_size == 0,
              "empty mesh has maximum zero and native vacuous convexity");
        s = mesh;
        s.mesh.coordinates.points = {{0, 0, 0}, {2, 0, 0}, {.5, .5, 0}, {0, 2, 0}};
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 4, 0, 1, 2, 3, 4, 1, 2, 0};
        const auto early = query(s);
        check(early.complete && !early.has_convex_facets && early.max_facet_size == 6 &&
                  early.report["max_query_visited_facets"] == 2 &&
                  early.report["convex_query_visited_facets"] == 1,
              "maximum scan finishes independently before convex query returns false on the first "
              "failing face");
    }
    {
        const std::vector<Point3> pentagon{{0, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1, .8, 0}, {0, 1, 0}};
        auto plan = [](const std::vector<Point3> &p, std::size_t max) {
            TubeBudget b;
            return native_facet_index_plan(p, b, max);
        };
        for (std::size_t limit : {0, 1, 2, 3}) {
            const auto r = plan(square, limit);
            check(r.completed && r.route == NativeFacetIndexPlan::Route::quad &&
                      r.indices.size() == 8,
                  "edge limits below three clamp to three and preserve original quad splitting");
        }
        for (std::size_t limit :
             {std::size_t(4), std::size_t(5), std::numeric_limits<std::size_t>::max() - 1}) {
            const auto r = plan(square, limit);
            check(r.completed && r.native_succeeded &&
                      r.route == NativeFacetIndexPlan::Route::passthrough &&
                      r.indices == std::vector<std::int32_t>{1, 2, 3, 4, 0},
                  "within-limit quad is retained as one polygon without introducing a diagonal");
        }
        const auto keep = plan(pentagon, 5), split = plan(pentagon, 4);
        check(keep.completed && keep.indices == std::vector<std::int32_t>{1, 2, 3, 4, 5, 0} &&
                  split.completed && split.indices.size() == 12,
              "limit retains even concave polygons but larger faces use the native triangle path");
        const auto wrapped = plan(square, std::numeric_limits<std::size_t>::max());
        check(wrapped.completed && wrapped.route == NativeFacetIndexPlan::Route::quad,
              "SIZE_MAX preserves native unsigned wrapped-count overflow rather than acting "
              "unlimited");
        const auto short_overflow =
            plan({{0, 0, 0}, {1, 0, 0}}, std::numeric_limits<std::size_t>::max());
        check(short_overflow.native_succeeded && short_overflow.completed &&
                  short_overflow.indices.empty() &&
                  plan({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}, std::numeric_limits<std::size_t>::max())
                          .indices == std::vector<std::int32_t>{1, 2, 3, 0},
              "overflowed threshold retains native explicit triangle branch and short-face "
              "projected empty region");
        NativePolyfaceVisitorFacet facet;
        facet.points = pentagon;
        facet.visible = {1, 0, 1, 1, 0, 1};
        for (std::size_t c = 0; c < polyface_channel_count; ++c)
            facet.client_indices[c] = {
                static_cast<std::int32_t>(c * 10),     static_cast<std::int32_t>(c * 10 + 1),
                static_cast<std::int32_t>(c * 10 + 2), static_cast<std::int32_t>(c * 10 + 3),
                static_cast<std::int32_t>(c * 10 + 4), static_cast<std::int32_t>(c * 10)};
        NativePolyfaceIndexState state;
        state.active.fill(true);
        TubeBudget b;
        const auto r = triangulate_native_polyface_facets({facet}, state, b, 5);
        check(r.complete && r.native_succeeded && r.report["retained_polygon_facets"] == 1 &&
                  r.report["all_output_faces_triangular"] == false &&
                  r.output.indices[point_channel] == std::vector<std::int32_t>{1, -2, 3, 4, -5, 0},
              "prepared consumer preserves polygon visibility and distinguishes completion from "
              "triangular output");
        for (std::size_t c = 1; c < polyface_channel_count; ++c)
            check(r.output.indices[c] ==
                      std::vector<std::int32_t>{static_cast<std::int32_t>(c * 10 + 1),
                                                static_cast<std::int32_t>(c * 10 + 2),
                                                static_cast<std::int32_t>(c * 10 + 3),
                                                static_cast<std::int32_t>(c * 10 + 4),
                                                static_cast<std::int32_t>(c * 10 + 5), 0},
                  "retained polygons keep independent attribute-to-source mappings");
        facet.points.resize(2);
        const auto short_face = triangulate_native_polyface_facets({facet}, state, b, 4);
        check(short_face.complete && short_face.report["all_output_faces_triangular"] == false,
              "successful short-face passthrough is not labelled triangular output");
    }
    {
        TubeBudget measured;
        native_polygon_convexity(square, measured);
        for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1}) {
            TubeBudget b;
            b.max_work = limit;
            rejects([&] { native_polygon_convexity(square, b); });
        }
        TubeBudget limited;
        limited.max_control_points = 3;
        rejects([&] { native_polygon_convexity(square, limited); });
        TubeBudget q;
        query_native_polyface_facets(mesh, q);
        for (auto limit : {std::size_t(0), q.work / 2, q.work - 1}) {
            TubeBudget b;
            b.max_work = limit;
            rejects([&] { query_native_polyface_facets(mesh, b); });
        }
        auto p = square;
        p[0][0] = std::numeric_limits<double>::infinity();
        rejects([&] { convex(p); });
        p = square;
        p[1][0] = 1e200;
        rejects([&] { convex(p); });
        auto task = [&] { return query(mesh); };
        auto a = std::async(std::launch::async, task), b = std::async(std::launch::async, task);
        const auto x = a.get(), y = b.get();
        check(
            x.complete && y.complete && x.report == y.report &&
                mesh.mesh.coordinates.points == square,
            "independent parallel queries are deterministic and keep source coordinates unchanged");
    }
    return checks;
}
