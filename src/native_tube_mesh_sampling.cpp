#include "native_tube_mesh_sampling.hpp"
#include "native_surface_iso.hpp"
#include "native_pcurve_points.hpp"
#include "native_tube_path_placement.hpp"
#include "native_bezier_support.hpp"
namespace p3d::swept_detail {
TubeSectionMeshSamples sample_tube_mesh_section(const BsplineSurface &first_patch,
                                                bool source_profile_closed, double chord_tolerance,
                                                double angle_tolerance,
                                                std::size_t max_sample_nodes, TubeBudget &budget) {
    using curve_detail::bezier_support::finite;
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    TubeSectionMeshSamples out;
    out.report = {{"scope", "native_swept_mesh_section_sampling"},
                  {"status", "native_failure"},
                  {"intervals", Json::array()},
                  {"mesh_generated", false}};
    auto fail = [&](const char *step) {
        out.report["failure_step"] = step;
        out.report["work_used"] = budget.work;
        return std::move(out);
    };
    auto iso = detail::native_iso_v_curve(first_patch, 0, work, budget.max_control_points);
    out.report["iso_weight_fallbacks"] = iso.zero_weight_fallbacks;
    out.reference = std::move(iso.curve);
    const auto &curve = *out.reference;
    out.knots = curve_detail::native_curve_knot_data(curve, budget.max_control_points, work);
    // Native calls IsWellOrdered but ignores its bool result here. The later
    // Bezier/interval count guard is separate and must not be replaced by it.
    auto beziers = curve_detail::native_curve_make_beziers(curve, budget.max_control_points, work);
    if (beziers.empty())
        return fail("no_bezier_segments");
    out.breaks = curve_detail::native_curve_c1_breaks(curve, budget.max_control_points,
                                                      budget.max_control_points, work);
    work.charge(curve.knots().size() + 16 * std::size_t(curve.order()) * curve.order());
    auto start = detail::pcurve_point_tangent(curve, 0),
         end = detail::pcurve_point_tangent(curve, 1);
    const auto domain = curve.knot_domain();
    const double span = finite(domain[1] - domain[0]);
    for (auto *t : {&start.tangent, &end.tangent})
        for (auto &v : *t)
            v = finite(v * span);
    std::optional<bool> parallel;
    auto suppress_end = [&] {
        if (!source_profile_closed)
            return false;
        if (!parallel)
            parallel = native_path_vectors_parallel(start.tangent, end.tangent);
        return *parallel;
    };
    constexpr double epsilon = 1e-5;
    for (double f : out.breaks.parameters) {
        work.charge(1);
        const bool near_start = f >= -epsilon && finite(f - epsilon) <= 0;
        const bool near_end = f >= .99999 && finite(f - epsilon) <= 1;
        if ((near_start || near_end) && suppress_end())
            continue;
        if (near_end)
            out.end_discontinuity = true;
        // Native ordered-set comparison uses value-epsilon, including its
        // rounding. Do not round samples onto a grid or compare abs(a-b).
        auto at = std::lower_bound(out.break_set.begin(), out.break_set.end(), f,
                                   [&](double a, double b) { return finite(b - epsilon) > a; });
        if (at == out.break_set.end() || finite(*at - epsilon) > f)
            out.break_set.insert(at, f);
    }
    if (out.knots.right < out.knots.left || beziers.size() != out.knots.right - out.knots.left)
        return fail("bezier_knot_interval_count");
    std::size_t sampled = 0;
    for (std::size_t i = 0; i < beziers.size(); ++i) {
        work.charge(1);
        const auto knot = out.knots.compressed[out.knots.left + i];
        const auto at =
            std::lower_bound(out.break_set.begin(), out.break_set.end(), knot,
                             [&](double a, double b) { return finite(b - epsilon) > a; });
        if (at != out.break_set.end() && finite(*at - epsilon) <= knot)
            out.discontinuity_intervals.push_back(i);
        auto sample = curve_detail::native_curve_sample_tree(
            {&beziers[i]}, {0, 1}, chord_tolerance, angle_tolerance, budget.max_control_points,
            max_sample_nodes, work);
        out.report["intervals"].push_back(std::move(sample.report));
        if (!sample.success)
            return fail("adaptive_sampling");
        require(sample.parameters.size() <= budget.max_control_points - sampled,
                "native mesh section cumulative sample budget");
        sampled += sample.parameters.size();
        out.interval_samples.push_back(std::move(sample.parameters));
    }
    out.success = true;
    out.report["status"] = "sampled";
    out.report["knot_data_well_ordered"] = out.knots.well_ordered;
    out.report["source_profile_closed"] = source_profile_closed;
    out.report["end_tangents_parallel"] = parallel ? Json(*parallel) : Json();
    out.report["sample_count"] = sampled;
    out.report["work_used"] = budget.work;
    return out;
}
} // namespace p3d::swept_detail
