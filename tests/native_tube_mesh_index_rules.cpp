#include "native_tube_mesh_index_rules.hpp"
#include "native_tube_mesh_trim_edges.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_tube_mesh_index_rules_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "native index rule invalid input or budget rejects");
    };
    auto plan = [&](std::vector<Point3> p) {
        TubeBudget b;
        return native_facet_index_plan(p, b);
    };
    auto a = plan({{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}});
    check(a.route == NativeFacetIndexPlan::Route::quad &&
              a.indices == std::vector<std::int32_t>{1, -2, 4, 0, -4, 2, 3, 0},
          "equal normal dot products choose the native second diagonal and signed hidden edges");
    auto q = std::vector<Point3>{{0, 0, 0}, {3, 0, 0}, {2, 1, 0}, {0, 2, 0}};
    check(plan(q).indices == std::vector<std::int32_t>{1, 2, -3, 0, -1, 3, 4, 0},
          "strictly greater first normal dot product chooses original first diagonal");
    auto concave = std::vector<Point3>{{0, 0, 0}, {3, 0, 0}, {3, 2, 0}, {2, 1, 0}};
    check(plan(concave).indices == a.indices,
          "native concave quad selection keeps original signed triangle order");
    for (const auto &points : {q, concave}) {
        auto result = plan(points);
        double polygon_area = 0, triangle_area = 0;
        for (std::size_t i = 0; i < 4; ++i) {
            auto x = points[i], y = points[(i + 1) % 4];
            polygon_area += x[0] * y[1] - x[1] * y[0];
        }
        for (std::size_t i = 0; i < result.indices.size(); i += 4) {
            auto x = points[std::abs(result.indices[i]) - 1],
                 y = points[std::abs(result.indices[i + 1]) - 1],
                 z = points[std::abs(result.indices[i + 2]) - 1];
            triangle_area += (y[0] - x[0]) * (z[1] - x[1]) - (y[1] - x[1]) * (z[0] - x[0]);
        }
        check(polygon_area == triangle_area,
              "selected native triangles preserve independent planar signed area");
    }
    auto translated = q;
    auto spatial = q;
    spatial[3][2] = 4;
    check(plan(spatial).indices == std::vector<std::int32_t>{1, 2, -3, 0, -1, 3, 4, 0},
          "nonplanar quad compares original spatial cross products without projecting to XY");
    for (auto &p : translated) {
        p[0] += 10;
        p[1] -= 20;
        p[2] = 4;
    }
    check(plan(translated).indices == plan(q).indices,
          "quad selection is invariant under exact translation");
    auto degenerate = plan({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}});
    check(degenerate.indices == a.indices,
          "zero-area native quad does not invent a different diagonal or drop records");
    check(plan({{0, 0, 0}}).indices == std::vector<std::int32_t>{1, 0} &&
              plan({{0, 0, 0}, {1, 0, 0}}).indices == std::vector<std::int32_t>{1, 2, 0},
          "low-count native face branch preserves even degenerate index loops");
    check(plan({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}).indices == std::vector<std::int32_t>{1, 2, 3, 0},
          "triangle branch preserves original vertex order");
    auto large = plan({{0, 0, 0}, {1, 0, 0}, {2, 1, 0}, {1, 2, 0}, {0, 1, 0}});
    check(large.route == NativeFacetIndexPlan::Route::projected_loops &&
              large.completed && large.indices.size() == 12,
          "larger polygon completes native projected-loop triangulation and source indices");
    TubeMeshVisibilityLayout rect{3, 2, 6, 0, 0, false, true, true};
    std::vector<std::int32_t> grid{1, 2, 4, 0, 2, 5, 4, 0, 2, 3, 5, 0, 3, 6, 5, 0};
    auto visible = [&](const std::vector<std::int32_t> &input, TubeMeshVisibilityLayout l) {
        TubeBudget b;
        return apply_tube_mesh_edge_visibility(input, l, b);
    };
    auto show = visible(grid, rect);
    check(show.indices ==
              std::vector<std::int32_t>{-1, -2, 4, 0, -2, -5, -4, 0, -2, -3, -5, 0, 3, -6, -5, 0},
          "rectangular first and last U edges alone receive positive signs");
    check(show.report["selected_edges"] == 2 && show.report["attribute_indices_changed"] == false,
          "visibility report separates coordinate signs from prior attribute indices");
    for (bool first : {false, true})
        for (bool last : {false, true}) {
            auto layout = rect;
            layout.first_column_visible = first;
            layout.last_column_visible = last;
            auto r = visible(grid, layout);
            check((r.indices[2] > 0) == first && (r.indices[12] > 0) == last,
                  "native caller flags independently select the original two U boundaries");
            for (std::size_t i = 0; i < grid.size(); ++i)
                check(std::abs(r.indices[i]) == std::abs(grid[i]),
                      "visibility never remaps vertices or face delimiters");
        }
    auto inverted = grid;
    for (auto &x : inverted)
        x = -x;
    check(visible(inverted, rect).indices == show.indices,
          "selected edge set overrides prior signs including originally hidden boundary edges");
    check(visible(show.indices, rect).indices == show.indices,
          "native visibility assignment is idempotent");
    TubeMeshVisibilityLayout trim{2, 2, 4, 2, 2, true, true, true};
    std::vector<std::int32_t> tri{1, -3, 2, 0, -2, 3, 4, 0};
    auto attribute_copy = tri;
    show = visible(tri, trim);
    check(show.indices == std::vector<std::int32_t>{-1, -3, 2, 0, -2, 3, -4, 0} &&
              tri == attribute_copy,
          "trimmed column boundaries change coordinate signs after independent attribute copies");
    trim.coordinate_count = 5;
    trim.first_column_visible = false;
    check(visible({3, 4, 5, 0}, trim).indices == std::vector<std::int32_t>{-3, 4, -5, 0},
          "extra closing coordinate shifts native last-column visibility using actual point count");
    trim.first_column_count = -1;
    trim.last_column_count = 1;
    trim.first_column_visible = true;
    check(visible({1, 2, 3, 0}, trim).indices == std::vector<std::int32_t>{-1, -2, -3, 0},
          "negative or singleton declared column lengths produce no visibility edges");
    auto none = rect;
    none.first_column_visible = none.last_column_visible = false;
    check(visible({0, 1, 0, 0, -2, 0}, none).indices ==
              std::vector<std::int32_t>{0, -1, 0, 0, -2, 0},
          "empty loops and one-record loops retain native zero handling");
    check(visible({1, 2, 3}, none).indices == std::vector<std::int32_t>{-1, -2, 3} &&
              visible({7}, none).indices == std::vector<std::int32_t>{7} &&
              visible({}, none).indices.empty(),
          "native scan leaves final unpaired token unchanged instead of adding a terminator");
    // Actual rectangular connector -> final point visibility. Normals and
    // parameters were copied before sign rewriting and must stay untouched.
    TubeBudget b;
    auto state = make_tube_mesh_edge_state(1, 1, false, false, b);
    TubeMeshRegularVertices nodes{3, 2, {}};
    for (int y = 0; y < 2; ++y)
        for (int x = 0; x < 3; ++x)
            nodes.vertices.push_back(
                {{double(x), double(y), 0}, {0, 0, 1}, {double(x) / 2, double(y)}, false});
    auto mesh = connect_tube_mesh_regular_vertices(nodes, state, {}, b);
    auto final = apply_tube_mesh_edge_visibility(mesh.point_indices, rect, b);
    check(final.indices == visible(grid, rect).indices && mesh.normal_indices == grid &&
              mesh.parameter_indices == grid,
          "actual generated regular strip accepts final visibility without changing attribute "
          "index pools");
    rejects([&] { plan({}); });
    rejects([&] {
        auto p = q;
        p[0][0] = std::numeric_limits<double>::infinity();
        plan(p);
    });
    rejects([&] {
        TubeBudget z;
        z.max_work = 4;
        native_facet_index_plan(q, z);
    });
    rejects([&] {
        TubeBudget z;
        z.max_control_points = 4;
        native_facet_index_plan(q, z);
    });
    rejects([&] { visible({INT32_MIN, 0}, rect); });
    rejects([&] {
        TubeBudget z;
        z.max_work = grid.size();
        apply_tube_mesh_edge_visibility(grid, rect, z);
    });
    rejects([&] {
        auto l = rect;
        l.u_count = 0;
        visible(grid, l);
    });
    auto before = grid;
    rejects([&] {
        TubeBudget z;
        z.max_work = grid.size() + 10;
        apply_tube_mesh_edge_visibility(grid, rect, z);
    });
    check(grid == before, "late visibility budget failure leaves original signed indices intact");
    auto work = [&] { return visible(grid, rect).indices; };
    auto task = std::async(std::launch::async, work);
    check(task.get() == work() && grid == before,
          "parallel index rule evaluation has no shared mutable state");
    return n;
}
