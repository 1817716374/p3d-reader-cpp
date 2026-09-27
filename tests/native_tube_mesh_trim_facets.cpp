#include "native_tube_mesh_trim_facets.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
TubeMeshTrimPlan columns(std::initializer_list<std::pair<int, int>> ranges) {
    TubeMeshTrimPlan p;
    p.success = true;
    for (auto [l, u] : ranges) {
        TubeMeshTrimColumn c;
        c.first_interior = l;
        c.last_interior = u;
        c.count = std::size_t(std::max(0, u - l + 3));
        c.offset = p.vertex_count;
        c.source_u = c.local_u = double(p.columns.size());
        c.lower = double(l - 1) / 4;
        c.upper = double(u + 1) / 4;
        p.columns.push_back(c);
        p.vertex_count += c.count;
    }
    return p;
}
TubeMeshTrimFacetPlan facets(const TubeMeshTrimPlan &p) {
    TubeBudget b;
    return prepare_tube_mesh_trim_facets(p, b);
}
std::vector<std::vector<std::int32_t>> indices(const TubeMeshTrimFacetPlan &p) {
    std::vector<std::vector<std::int32_t>> out;
    for (auto &f : p.facets)
        out.push_back(f.indices);
    return out;
}
BsplineCurve scalar(double a, double b) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", 2},
                                    {"closed", false},
                                    {"knots", nullptr},
                                    {"poles", {a, 0, 0, b, 0, 0}},
                                    {"weights", nullptr}});
}
} // namespace
unsigned native_tube_mesh_trim_facets_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *msg) {
        ++n;
        require(ok, msg);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "trimmed facet invalid input or exhausted budget rejects");
    };
    const auto rectangular = columns({{1, 3}, {1, 3}});
    auto f = facets(rectangular);
    check(indices(f) ==
              std::vector<std::vector<std::int32_t>>{
                  {1, 6, 7, 2}, {2, 7, 8, 3}, {3, 8, 9, 4}, {4, 9, 10, 5}},
          "native trimmed common rows are column-major quads in original order");
    check(f.indices_in_range && f.column_correspondence_valid &&
              f.report["triangulated"] == false && f.report["corners"] == 16,
          "facet arguments are not advertised as triangulated mesh");
    auto narrowing = columns({{1, 3}, {2, 2}});
    f = facets(narrowing);
    check(indices(f) ==
              std::vector<std::vector<std::int32_t>>{
                  {2, 6, 7, 3}, {3, 7, 8, 4}, {2, 1, 6}, {5, 4, 8}},
          "narrowing emits shared row quads followed by ordered bottom and top triangles");
    check(f.facets[0].kind == TubeMeshTrimFacetKind::common_rows &&
              f.facets[2].kind == TubeMeshTrimFacetKind::lower_boundary &&
              f.facets[3].kind == TubeMeshTrimFacetKind::upper_boundary,
          "boundary facets retain native emission roles");
    f = facets(columns({{2, 2}, {1, 3}}));
    check(indices(f) ==
              std::vector<std::vector<std::int32_t>>{
                  {1, 5, 6, 2}, {2, 6, 7, 3}, {1, 4, 5}, {3, 7, 8}},
          "widening retains native winding and right-column offset");
    f = facets(columns({{1, 5}, {4, 4}}));
    check(indices(f) ==
              std::vector<std::vector<std::int32_t>>{
                  {4, 8, 9, 5}, {5, 9, 10, 6}, {4, 3, 2, 1, 8}, {7, 6, 10}},
          "large lower offset remains one original n-gon instead of an invented fan");
    f = facets(columns({{1, 3}, {2, 2}, {1, 3}}));
    check(f.facets.size() == 8 && f.facets[4].column_pair == 1 &&
              f.facets[4].indices == std::vector<std::int32_t>{6, 10, 11, 7} &&
              f.facets[7].indices == std::vector<std::int32_t>{8, 12, 13},
          "consecutive pairs accumulate original variable column sizes");
    check(facets(columns({{2, 0}, {2, 0}})).facets.empty(),
          "two singleton columns do not manufacture a face");
    check(facets(columns({{3, 0}, {3, 0}})).facets.empty(),
          "two empty columns retain empty native face loops");
    f = facets(columns({{1, 0}, {4, 3}}));
    check(!f.column_correspondence_valid && f.facets.size() == 2 &&
              f.report["invalid_column_correspondences"].get<std::size_t>() > 0,
          "disjoint row ranges preserve and diagnose native cross-column references");
    f = facets(columns({{4, 0}, {1, 1}, {1, 1}}));
    check(
        !f.indices_in_range && !f.column_correspondence_valid &&
            f.facets[f.facets.size() - 2].indices == std::vector<std::int32_t>{0, 3, 4, 1},
        "negative declared count affects native signed offsets even though actual column is empty");
    // Independent planar area check across every overlapping pair in a small
    // row lattice. Coordinates come from row labels, not the index formula.
    for (int al = 1; al <= 4; ++al)
        for (int au = al - 1; au <= 3; ++au)
            for (int bl = 1; bl <= 4; ++bl)
                for (int bu = bl - 1; bu <= 3; ++bu) {
                    if (std::max(al, bl) > std::min(au, bu) + 2)
                        continue;
                    auto p = columns({{al, au}, {bl, bu}});
                    auto q = facets(p);
                    check(q.indices_in_range && q.column_correspondence_valid,
                          "overlapping planar columns preserve valid local correspondence");
                    std::vector<Point2> points;
                    for (const auto &c : p.columns)
                        for (int row = c.first_interior - 1; row <= c.last_interior + 1; ++row)
                            points.push_back({c.source_u, double(row) / 4});
                    double total = 0;
                    for (const auto &face : q.facets) {
                        double area = 0;
                        for (std::size_t j = 0; j < face.indices.size(); ++j) {
                            const auto &a = points.at(face.indices[j] - 1);
                            const auto &b =
                                points.at(face.indices[(j + 1) % face.indices.size()] - 1);
                            area += a[0] * b[1] - a[1] * b[0];
                        }
                        check(area >= 0, "all original facet windings agree in a planar strip");
                        total += area / 2;
                    }
                    const double expected = double((au - al + 2) + (bu - bl + 2)) / 8;
                    check(total == expected,
                          "native polygon areas cover the exact planar trapezoid");
                }
    const std::vector<double> v{0, .25, .5, .75, 1};
    TubeMeshBoundaryCurves bounds;
    bounds.success = true;
    bounds.lower = scalar(.2, .4);
    bounds.upper = scalar(.8, .6);
    TubeBudget b;
    auto p = prepare_tube_mesh_trim_columns(bounds, {0, 1}, v, {0, 1}, b);
    auto uv = evaluate_tube_mesh_trim_parameters(p, v, 1, 3, b);
    check(uv.size() == p.vertex_count && uv.front() == Point2{0, 1.0 / 3 + .2 / 3},
          "lower attribute uses the boundary curve value and separate patch divisions");
    check(uv[p.columns[0].count - 1] == Point2{0, 1.0 / 3 + 1.0 / 3} && p.columns[0].upper == .8,
          "upper attribute uses the bracketing sample rather than geometry boundary value");
    check(uv.back() == Point2{1, 1.0 / 3 + .75 / 3} && p.columns[1].upper == .6,
          "different columns preserve their own upper row lookups");
    check(uv[1] == Point2{0, 1.0 / 3 + .25 / 3},
          "interior temporary point lifetime does not alter the appended attribute vector");
    auto single = columns({{2, 0}, {2, 0}});
    single.columns[0].lower = .6;
    single.columns[1].lower = .7;
    uv = evaluate_tube_mesh_trim_parameters(single, v, 0, 1, b);
    check(uv == std::vector<Point2>{{0, .6}, {1, .7}},
          "singleton parameter uses lower precedence without a shared-V read");
    auto negrow = columns({{0, -1}, {0, -1}});
    negrow.columns[0].lower = -.2;
    uv = evaluate_tube_mesh_trim_parameters(negrow, v, 0, 1, b);
    check(uv[0] == Point2{0, -.2} && uv[1] == Point2{0, 0},
          "negative lower row is evaluated without indexing before the V vector");
    rejects([&] {
        auto q = p;
        q.success = false;
        facets(q);
    });
    rejects([&] {
        auto q = p;
        ++q.columns[1].offset;
        facets(q);
    });
    rejects([&] {
        auto q = p;
        ++q.vertex_count;
        facets(q);
    });
    rejects([&] {
        auto q = p;
        ++q.columns[0].count;
        facets(q);
    });
    rejects([&] {
        TubeBudget z;
        z.max_work = 2;
        prepare_tube_mesh_trim_facets(p, z);
    });
    rejects([&] {
        TubeBudget z;
        z.max_control_points = p.vertex_count;
        prepare_tube_mesh_trim_facets(p, z);
    });
    rejects([&] {
        TubeBudget z;
        evaluate_tube_mesh_trim_parameters(p, v, 0, 0, z);
    });
    rejects([&] {
        TubeBudget z;
        evaluate_tube_mesh_trim_parameters(p, v, 3, 3, z);
    });
    rejects([&] {
        TubeBudget z;
        evaluate_tube_mesh_trim_parameters(p, {0, .25}, 0, 1, z);
    });
    auto concurrent = [&] { return indices(facets(narrowing)); };
    auto task = std::async(std::launch::async, concurrent);
    check(task.get() == concurrent() && narrowing.columns[1].offset == 5,
          "parallel facet planning leaves source columns immutable");
    return n;
}
