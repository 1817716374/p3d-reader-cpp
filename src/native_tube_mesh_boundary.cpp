#include "native_tube_mesh_patch.hpp"
#include "native_bezier_support.hpp"
namespace p3d::swept_detail {
TubeMeshBoundaryCurves tube_mesh_boundary_curves(const std::vector<std::vector<Point2>> &bounds,
                                                 TubeBudget &budget) {
    using curve_detail::bezier_support::finite;
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    TubeMeshBoundaryCurves out;
    out.report = {{"scope", "native_mesh_boundary_graphs"},
                  {"status", "native_failure"},
                  {"source_records", bounds.size()},
                  {"consumed_records", 0}};
    if (bounds.empty() || bounds.front().empty()) {
        out.success = true;
        out.report["status"] = "native_defaults";
        return out;
    }
    const auto &points = bounds.front();
    require(points.size() <= budget.max_control_points, "native mesh boundary point budget");
    std::vector<double> lower_knots{0, 0}, upper_knots{1, 1};
    std::vector<Point3> lower, upper;
    auto near = [&](double a, double b) {
        return std::abs(finite(a - b)) <= finite(finite((std::abs(a) + 1) + std::abs(b)) * 1e-10);
    };
    auto fail = [&](const char *why, std::size_t index) {
        out.report["failure"] = why;
        out.report["point_index"] = index;
        out.report["work_used"] = budget.work;
        return std::move(out);
    };
    bool rising = true;
    out.report["consumed_records"] = 1;
    for (std::size_t i = 0; i < points.size(); ++i) {
        work.charge(24);
        const double u = finite(points[i][0]), v = finite(points[i][1]);
        if (rising) {
            if (i == 0) {
                if (!near(u, 0))
                    return fail("first_u_not_zero", i);
                lower.push_back({v, 0, 0});
            } else if (!near(u, 1)) {
                if (!(u > lower_knots.back()))
                    return fail("lower_u_not_increasing", i);
                lower_knots.push_back(u);
                lower.push_back({v, 0, 0});
            } else {
                if (!(1 > lower_knots.back()))
                    return fail("lower_endpoint_order", i);
                lower_knots.insert(lower_knots.end(), {1, 1});
                lower.push_back({v, 0, 0});
                ++i;
                // The native routine directly reads the successor at the
                // turn. Reject malformed input instead of reading past it.
                require(i < points.size(), "native mesh boundary missing upper turn point");
                work.charge(8);
                if (!near(finite(points[i][0]), 1))
                    return fail("upper_turn_u_not_one", i);
                upper.push_back({finite(points[i][1]), 0, 0});
                rising = false;
            }
        } else if (points.size() >= 2 && i == points.size() - 2) {
            if (!near(u, 0))
                return fail("upper_endpoint_u_not_zero", i);
            if (!(upper_knots.back() > 0))
                return fail("upper_endpoint_order", i);
            upper_knots.insert(upper_knots.end(), {0, 0});
            upper.push_back({v, 0, 0});
            break; // Last saved point is intentionally not inspected.
        } else {
            if (!(upper_knots.back() > u))
                return fail("upper_u_not_decreasing", i);
            upper_knots.push_back(u);
            upper.push_back({v, 0, 0});
        }
    }
    auto build = [&](std::vector<Point3> &p, std::vector<double> &knots,
                     bool reverse) -> std::optional<BsplineCurve> {
        if (knots.size() == 4)
            return {}; // Native discards even non-default endpoint values.
        require(p.size() >= 2 && knots.size() == p.size() + 2,
                "native mesh boundary incomplete scalar-curve storage");
        work.charge(p.size() * 4 + knots.size());
        if (reverse) {
            std::reverse(p.begin(), p.end());
            std::reverse(knots.begin(), knots.end());
        }
        Json xyz = Json::array();
        for (const auto &point : p)
            for (double x : point)
                xyz.push_back(x);
        return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                        {"order", 2},
                                        {"closed", false},
                                        {"poles", std::move(xyz)},
                                        {"knots", knots},
                                        {"weights", nullptr}});
    };
    out.lower = build(lower, lower_knots, false);
    out.upper = build(upper, upper_knots, true);
    out.success = true;
    out.report["status"] = "prepared";
    out.report["lower_curve_present"] = out.lower.has_value();
    out.report["upper_curve_present"] = out.upper.has_value();
    out.report["work_used"] = budget.work;
    return out;
}
} // namespace p3d::swept_detail
