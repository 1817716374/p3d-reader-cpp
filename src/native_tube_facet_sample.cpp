#include "native_tube_facet_sample.hpp"
#include "native_curve_plane.hpp"
#include "native_tube_path_placement.hpp"
#include "native_curve_planarity.hpp"
#include "native_curve_xy.hpp"
#include "native_tube_transform.hpp"
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
TubeFacetParameters sample_tube_facet_pair(const BsplineCurve &first, const BsplineCurve &second,
                                           TubeBudget &b) {
    const auto n1 = first.poles().size(), n2 = second.poles().size();
    require(n1 <= b.max_control_points && n2 <= b.max_control_points - n1,
            "native facet fallback cumulative curve control budget exceeded");
    const curve_detail::BezierWork work{b.work, b.max_work};
    auto table = [&](const BsplineCurve &c) {
        work.charge(c.knots().size());
        for (unsigned i = 0; i < 8; ++i)
            work.charge(c.poles().size());
        Json poles = Json::array();
        for (const auto &p : c.poles())
            for (double x : p)
                poles.push_back(x);
        return Json{
            {"_type", "BsplineCurve"}, {"order", c.order()},
            {"closed", c.closed()},    {"poles", std::move(poles)},
            {"knots", c.knots()},      {"weights", c.rational() ? Json(c.weights()) : Json()}};
    };
    // Native frame queries operate on clones in a type-0 group. Keep the
    // original two curves for the later independent transform operation.
    Json members = Json::array();
    members.push_back({{"_type", "VariantGeometry"}, {"geometry", table(first)}});
    members.push_back({{"_type", "VariantGeometry"}, {"geometry", table(second)}});
    const Json group{{"_type", "CurveVector"}, {"type", 0}, {"curves", std::move(members)}};
    const auto planar =
        curve_detail::native_curve_vector_planarity(group, b.max_control_points, work);
    TubeFacetParameters out;
    out.report = {{"scope", "native_facet_pair_fallback"},
                  {"status", "native_failure"},
                  {"planarity", planar},
                  {"transform", nullptr},
                  {"intersections", nullptr},
                  {"parameters", nullptr},
                  {"work_used", b.work}};
    if (!planar.value("planar", false)) {
        out.report["reason"] = "pair_not_planar_or_frame_failed";
        return out;
    }
    const auto transform = planar.at("world_to_local").get<Matrix4>();
    // Two native wrappers are created even when both references share source
    // identity. clone=true keeps each transformed occurrence independent.
    const TubeCurveViews curves{std::make_shared<const BsplineCurve>(first),
                                std::make_shared<const BsplineCurve>(second)};
    const auto transformed = transform_tube_curve_list(curves, &transform, true, b);
    require(transformed.report.at("native_result") == true && transformed.curves.size() == 2 &&
                transformed.curves[0] && transformed.curves[1],
            "native facet fallback transform did not produce both curve copies");
    const auto hits = curve_detail::native_curve_xy_intersections(
        *transformed.curves[0], *transformed.curves[1], b.max_control_points, work);
    std::vector<double> first_parameters, second_parameters;
    work.charge(hits.intersections.size());
    first_parameters.reserve(hits.intersections.size());
    second_parameters.reserve(hits.intersections.size());
    for (const auto &hit : hits.intersections) {
        first_parameters.push_back(hit.fractions[0]);
        second_parameters.push_back(hit.fractions[1]);
    }
    out.parameters = select_tube_facet_fallback_parameters(first_parameters, second_parameters, b);
    out.report.update(
        {{"status", out.parameters ? "resolved" : "native_failure"},
         {"reason", out.parameters ? "independent_parameter_extrema" : "no_pair_intersections"},
         {"transform", transformed.report},
         {"intersections",
          {{"count", hits.intersections.size()},
           {"span_pairs", hits.span_pairs},
           {"candidates", hits.candidates},
           {"discarded_candidates", hits.discarded_candidates},
           {"failed_newton", hits.failed_newton},
           {"outside_parameters", hits.outside_parameters}}},
         {"parameters", out.parameters ? Json(*out.parameters) : Json()},
         {"work_used", b.work}});
    return out;
}
TubeFacetParameters sample_tube_facet_seam(const BsplineSurface &first,
                                           const BsplineSurface &second, double u,
                                           const std::array<Point3, 2> &plane,
                                           bool require_same_point, TubeBudget &b) {
    auto primary = sample_tube_facet_plane(first, second, u, plane, require_same_point, b);
    TubeFacetParameters out{primary.parameters,
                            {{"scope", "native_facet_seam_sample"},
                             {"method", "plane"},
                             {"plane_query", std::move(primary.report)},
                             {"pair_query", nullptr}}};
    if (!out.parameters) {
        auto fallback = sample_tube_facet_pair(primary.first.curve, primary.second.curve, b);
        out.parameters = fallback.parameters;
        out.report["method"] = "curve_pair_fallback";
        out.report["pair_query"] = std::move(fallback.report);
    }
    out.report["status"] = out.parameters ? "resolved" : "native_failure";
    out.report["native_result"] = bool(out.parameters);
    out.report["parameters"] = out.parameters ? Json(*out.parameters) : Json();
    out.report["work_used"] = b.work;
    return out;
}
} // namespace p3d::swept_detail
