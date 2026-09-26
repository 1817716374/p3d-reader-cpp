#include "native_tube_path_selection.hpp"
#include "native_curve_closest.hpp"
#include "native_curve_plane.hpp"
namespace p3d::swept_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native path selection nonfinite arithmetic");
    return x;
}
double distance2(const Point3 &a, const Point3 &b) {
    const double dx = finite(b[0] - a[0]), dy = finite(b[1] - a[1]), dz = finite(b[2] - a[2]);
    return finite((dy * dy + dx * dx) + dz * dz);
}
} // namespace
TubePathSelection select_tube_path_candidate(const TubeFacetSources &sources, TubeBudget &budget) {
    const auto &path = sources.path.path;
    const auto &reference = sources.reference;
    require(path.is_object() && path.value("_type", "") == "CurveVector",
            "native path selection requires a CurveVector");
    const auto &type = path.at("type");
    require(type.is_number_integer() && type >= 0 && type <= 3,
            "native path selection supports boundary types 0..3");
    const auto &members = path.at("curves");
    require(members.is_array() && !members.empty(), "native path selection has no working members");
    curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(members.size());
    for (double x : reference.reference)
        finite(x);
    for (const auto &p : sources.path.source_endpoints)
        for (double x : p)
            finite(x);
    TubePathSelection out;
    out.curves.reserve(members.size());
    out.candidate_point = sources.path.source_endpoints[0];
    out.candidate_squared_distance = distance2(reference.reference, out.candidate_point);
    std::string reason = "original_start";
    bool projection_succeeded = true;
    std::size_t controls = 0, candidates = 0;
    Json reports = Json::array();
    auto consider = [&](const Point3 &p, double fraction, std::size_t index, const char *why,
                        bool projection) {
        work.charge(16);
        ++candidates;
        const double d2 = distance2(reference.reference, p);
        if (d2 < out.candidate_squared_distance) {
            out.candidate_squared_distance = d2;
            out.candidate_point = p;
            out.index = index;
            out.fraction = finite(fraction);
            reason = why;
            projection_succeeded = projection;
        }
    };
    for (std::size_t i = 0; i < members.size(); ++i) {
        auto converted = convert_tube_primitive(members[i].at("geometry"), budget);
        require(controls <= budget.max_control_points &&
                    converted.curve.poles().size() <= budget.max_control_points - controls,
                "native path selection cumulative controls exceeded");
        controls += converted.curve.poles().size();
        auto &report = converted.report;
        report["working_index"] = i;
        if (!reference.area_valid) {
            const auto hit = curve_detail::native_curve_closest_point(converted.curve,
                                                                      reference.reference, work);
            require(hit.found, "native path selection closest point is undefined");
            consider(hit.point, hit.fraction, i, "closest_point", true);
            report["query"] = "closest_point";
            report["candidate_count"] = 1;
        } else {
            // Base3500 evaluates the plane constant with Y then X then Z.
            for (double x : reference.plane_origin)
                finite(x);
            for (double x : reference.plane_normal)
                finite(x);
            const auto &n = reference.plane_normal, &p = reference.plane_origin;
            const double w = finite(-((n[1] * p[1] + n[0] * p[0]) + n[2] * p[2]));
            const auto hits = curve_detail::native_curve_plane_intersections(
                converted.curve, {n[0], n[1], n[2], w}, budget.max_work, work);
            for (const auto &hit : hits.intersections)
                consider(hit.point, hit.fraction, i, "plane_intersection",
                         hit.projection_succeeded);
            report["query"] = "plane_intersection";
            report["candidate_count"] = hits.intersections.size();
            report["all_parameter_segments"] = hits.all_parameter_segments;
        }
        out.curves.push_back(std::move(converted.curve));
        reports.push_back(std::move(report));
    }
    consider(sources.path.source_endpoints[1], 1., members.size() - 1, "original_end", true);
    const auto raw_index = out.index;
    const double raw_fraction = out.fraction;
    work.charge(16);
    const bool at_end =
        std::abs(finite(1. - out.fraction)) <= finite(((std::abs(out.fraction) + 1.) + 1.) * 1e-10);
    const bool moved = at_end && out.index + 1 < out.curves.size();
    if (moved) {
        ++out.index;
        out.fraction = 0;
    }
    out.report = {{"scope", "native_facet_path_selection"},
                  {"query", reference.area_valid ? "plane_intersection" : "closest_point"},
                  {"candidate_source", reason},
                  {"candidate_index", raw_index},
                  {"candidate_fraction", raw_fraction},
                  {"candidate_point", out.candidate_point},
                  {"candidate_squared_distance", out.candidate_squared_distance},
                  {"candidate_projection_succeeded", projection_succeeded},
                  {"candidate_count", candidates},
                  {"index", out.index},
                  {"fraction", out.fraction},
                  {"moved_to_next_member", moved},
                  {"converted_control_points", controls},
                  {"members", std::move(reports)},
                  {"work_used", budget.work}};
    return out;
}
TubeFacetPath prepare_tube_facet_path(const Json &profile, const Json &path, TubeBudget &budget) {
    auto sources = prepare_tube_facet_sources(profile, path, budget);
    auto selection = select_tube_path_candidate(sources, budget);
    return {std::move(sources), std::move(selection)};
}
} // namespace p3d::swept_detail
