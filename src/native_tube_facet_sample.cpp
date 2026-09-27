#include "native_tube_facet_sample.hpp"
#include "native_curve_plane.hpp"
#include "native_tube_path_placement.hpp"
namespace p3d::swept_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native facet sample nonfinite arithmetic");
    return x;
}
Point2 snap(Point2 p) {
    // Strict comparisons, not endpoint clamping. Values exactly at the
    // tolerance remain unchanged, and no other endpoints are snapped.
    if (std::abs(finite(p[0] - 1.)) < 1e-10)
        p[0] = 1;
    if (std::abs(p[1]) < 1e-10)
        p[1] = 0;
    return p;
}
} // namespace
TubeFacetPlaneSample sample_tube_facet_plane(const BsplineSurface &first,
                                             const BsplineSurface &second, double u,
                                             const std::array<Point3, 2> &plane,
                                             bool require_same_point, TubeBudget &b) {
    const auto n1 = first.v().pole_count(), n2 = second.v().pole_count();
    require(n1 <= b.max_control_points && n2 <= b.max_control_points - n1,
            "native facet isocurve cumulative control budget exceeded");
    const curve_detail::BezierWork work{b.work, b.max_work};
    TubeFacetPlaneSample out{detail::native_iso_u_curve(first, u, work, b.max_control_points),
                             detail::native_iso_u_curve(second, u, work, b.max_control_points),
                             std::nullopt, Json::object()};
    for (const auto &p : plane)
        for (double x : p)
            finite(x);
    work.charge(16);
    const auto &p = plane[0], &n = plane[1];
    // Base3500 uses Y + X + Z, without normalizing the plane normal.
    const double w = finite(-((n[1] * p[1] + n[0] * p[0]) + n[2] * p[2]));
    const curve_detail::BezierPole equation{n[0], n[1], n[2], w};
    const auto a = curve_detail::native_curve_plane_intersections(out.first.curve, equation,
                                                                  b.max_control_points, work);
    const auto c = curve_detail::native_curve_plane_intersections(out.second.curve, equation,
                                                                  b.max_control_points, work);
    std::string reason = "missing_plane_intersection";
    Json same = nullptr;
    if (!a.intersections.empty() && !c.intersections.empty()) {
        const auto &last = a.intersections.back(), &head = c.intersections.front();
        bool accept = true;
        if (require_same_point) {
            work.charge(32);
            accept = native_path_points_equal(last.point, head.point);
            same = accept;
        }
        if (accept) {
            out.parameters = snap({last.fraction, head.fraction});
            reason = "last_first_plane_intersections";
        } else {
            reason = "plane_intersection_points_differ";
        }
    }
    out.report = {{"scope", "native_facet_plane_sample"},
                  {"status", out.parameters ? "resolved" : "fallback_required"},
                  {"reason", reason},
                  {"fraction_u", u},
                  {"require_same_point", require_same_point},
                  {"selected_points_equal", std::move(same)},
                  {"first_intersections", a.intersections.size()},
                  {"second_intersections", c.intersections.size()},
                  {"first_all_parameter_segments", a.all_parameter_segments},
                  {"second_all_parameter_segments", c.all_parameter_segments},
                  {"first_zero_weight_fallbacks", out.first.zero_weight_fallbacks},
                  {"second_zero_weight_fallbacks", out.second.zero_weight_fallbacks},
                  {"parameters", out.parameters ? Json(*out.parameters) : Json()},
                  {"work_used", b.work}};
    return out;
}
std::optional<Point2> select_tube_facet_fallback_parameters(const std::vector<double> &first,
                                                            const std::vector<double> &second,
                                                            TubeBudget &b) {
    if (first.empty() || second.empty())
        return std::nullopt;
    curve_detail::BezierWork work{b.work, b.max_work};
    work.charge(first.size());
    work.charge(second.size());
    Point2 p{-1, 2};
    for (double t : first)
        if (finite(t) > p[0])
            p[0] = t;
    for (double t : second)
        if (finite(t) < p[1])
            p[1] = t;
    // Native caller considers nonempty arrays successful even when the
    // sentinels survive. Preserve that behavior instead of clamping to [0,1].
    return snap(p);
}
} // namespace p3d::swept_detail
