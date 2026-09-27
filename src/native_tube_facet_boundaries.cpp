#include "native_tube_facet_boundaries.hpp"
#include "native_curve_segment.hpp"
namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
std::size_t measure(const TubeFacetBoundarySegments &segments, const std::vector<Point3> &polygon,
                    TubeBudget &b) {
    charge(b, segments.size());
    require(segments.size() <= INT32_MAX, "native facet boundary segment count overflow");
    if (segments.empty())
        return 2;
    std::size_t n = 1;
    for (const auto &segment : segments) {
        require(&segment != &polygon, "native facet boundary output aliases an input segment");
        require(!segment.empty() && segment.size() <= INT32_MAX,
                "native facet boundary empty segment has undefined native iteration");
        require(segment.size() - 1 <= b.max_control_points &&
                    n <= b.max_control_points - (segment.size() - 1),
                "native facet boundary point budget exceeded");
        n += segment.size() - 1;
        charge(b, segment.size());
    }
    return n;
}
} // namespace
Json compose_tube_facet_boundary(std::vector<Point3> &polygon,
                                 const TubeFacetBoundarySegments &first,
                                 TubeFacetBoundarySegments &second, TubeBudget &b) {
    Json report{{"scope", "native_facet_boundary_polygon"},
                {"shared_segment_list", &first == &second},
                {"second_segments_reversed", false},
                {"output_unchanged", false}};
    if (first.empty() && second.empty()) {
        report["output_unchanged"] = true;
        report["points"] = polygon.size();
        report["work_used"] = b.work;
        return report;
    }
    const auto n1 = measure(first, polygon, b), n2 = measure(second, polygon, b);
    require(n1 < b.max_control_points && n2 <= b.max_control_points - n1 - 1 &&
                n1 + n2 + 1 <= INT32_MAX,
            "native facet boundary combined point budget exceeded");
    charge(b, n1 + n2 + 1);
    std::vector<Point3> out;
    out.reserve(n1 + n2 + 1);
    if (first.empty()) {
        out.push_back({0, 0, 0});
        out.push_back({1, 0, 0});
    } else {
        for (const auto &segment : first)
            out.insert(out.end(), segment.begin(), segment.end() - 1);
        out.push_back(first.back().back());
    }
    if (second.empty()) {
        out.push_back({1, 1, 0});
        out.push_back({0, 1, 0});
    } else {
        for (auto &segment : second)
            std::reverse(segment.begin(), segment.end());
        for (auto i = second.rbegin(); i != second.rend(); ++i)
            out.insert(out.end(), i->begin(), i->end() - 1);
        out.push_back(second.front().back());
        report["second_segments_reversed"] = true;
    }
    out.push_back(out.front()); // Native closure is unconditional, even if already equal.
    polygon = std::move(out);
    report["points"] = polygon.size();
    report["work_used"] = b.work;
    return report;
}
bool append_tube_facet_uv_boundary(std::vector<std::vector<Point2>> &boundaries,
                                   const std::vector<Point3> &polygon, TubeBudget &b) {
    if (polygon.empty())
        return false;
    require(polygon.size() <= INT32_MAX && boundaries.size() < INT32_MAX,
            "native facet UV boundary count overflow");
    charge(b, boundaries.size());
    std::size_t count = polygon.size();
    require(count <= b.max_control_points, "native facet UV boundary point budget exceeded");
    for (const auto &boundary : boundaries) {
        require(boundary.size() <= b.max_control_points - count,
                "native facet UV boundary cumulative point budget exceeded");
        count += boundary.size();
    }
    charge(b, polygon.size());
    std::vector<Point2> points;
    points.reserve(polygon.size());
    for (const auto &p : polygon)
        points.push_back({p[0], p[1]});
    boundaries.push_back(std::move(points));
    return true;
}
TubeCurve prepare_tube_facet_trim_path(const TubeFacetComposition &chain, TubeBudget &b) {
    require(chain.prepared && chain.report.value("seams_applied", false),
            "native facet trim path requires completed seam processing");
    require(chain.working_prefix_path || chain.working_suffix_path,
            "native facet trim path has no nonempty source branch");
    auto account = [&](const BsplineCurve &c) {
        require(c.poles().size() <= b.max_control_points && b.max_control_points <= UINT32_MAX,
                "native facet trim path source budget");
        charge(b, c.knots().size());
        for (unsigned pass = 0; pass < 8; ++pass)
            charge(b, c.poles().size());
    };
    Json report{{"scope", "native_facet_trim_path"},
                {"chain_finalized", false},
                {"prefix_reversal", nullptr},
                {"combination", nullptr}};
    if (!chain.working_prefix_path) {
        account(*chain.working_suffix_path);
        report["method"] = "suffix_copy";
        report["work_used"] = b.work;
        return {*chain.working_suffix_path, std::move(report)};
    }
    account(*chain.working_prefix_path);
    auto path = *chain.working_prefix_path;
    Json reversal;
    // Native caller ignores the reversal return code and uses the resulting
    // working curve. Preserve that distinction instead of silently retrying.
    const bool reversed = curve_detail::reverse_native_working_curve(
        path, unsigned(b.max_control_points), {b.work, b.max_work}, reversal);
    report["prefix_reversal"] = {{"success", reversed}, {"detail", std::move(reversal)}};
    report["method"] = "reversed_prefix_copy";
    if (chain.working_suffix_path) {
        account(*chain.working_suffix_path);
        auto combined = combine_tube_curves(path, *chain.working_suffix_path, false, false, b);
        path = std::move(combined.curve);
        report["combination"] = std::move(combined.report);
        report["method"] = "reversed_prefix_then_suffix";
    }
    report["work_used"] = b.work;
    return {std::move(path), std::move(report)};
}
} // namespace p3d::swept_detail
