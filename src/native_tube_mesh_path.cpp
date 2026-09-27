#include "native_tube_mesh_path.hpp"
#include "native_bezier_support.hpp"
#include "native_surface_iso.hpp"
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
bool near_zero(double v) {
    return std::abs(v) <= finite((std::abs(v) + 1) * 1e-10);
}
} // namespace
TubeMeshControlRange tube_mesh_control_range(const BsplineCurve &curve, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    require(curve.poles().size() <= budget.max_control_points, "native mesh control-range budget");
    TubeMeshControlRange out;
    const double maximum = std::numeric_limits<double>::max();
    out.low = {maximum, maximum, maximum};
    out.high = {-maximum, -maximum, -maximum};
    for (std::size_t i = 0; i < curve.poles().size(); ++i) {
        work.charge(24);
        auto p = curve.poles()[i];
        if (std::find(p.begin(), p.end(), maximum) != p.end()) {
            ++out.disconnected;
            continue;
        }
        if (curve.rational()) {
            const auto w = curve.weights()[i];
            if (std::abs(w) <= 1e-12) {
                ++out.rejected_weights;
                continue;
            }
            const auto inverse = finite(1 / w);
            for (auto &x : p)
                x = finite(x * inverse);
        }
        ++out.accepted;
        for (unsigned k = 0; k < 3; ++k) {
            if (p[k] < out.low[k])
                out.low[k] = p[k];
            if (p[k] > out.high[k])
                out.high[k] = p[k];
        }
    }
    out.native_null = !out.accepted;
    for (unsigned k = 0; k < 3; ++k)
        if (std::abs(out.low[k]) >= 1e100 || std::abs(out.high[k]) >= 1e100)
            out.native_null = true;
    if (!out.native_null) {
        const double x = finite(out.high[0] - out.low[0]), y = finite(out.high[1] - out.low[1]),
                     z = finite(out.high[2] - out.low[2]);
        out.squared_extent = finite(finite(y * y + x * x) + z * z);
    }
    out.degenerate = near_zero(out.squared_extent);
    return out;
}
TubeMeshPathSamples sample_tube_mesh_path(const std::vector<BsplineSurface> &strips,
                                          bool source_profile_closed, double chord_tolerance,
                                          double angle_tolerance, std::size_t max_sample_nodes,
                                          TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    TubeMeshPathSamples out;
    out.report = {{"scope", "native_swept_mesh_path_sampling"},
                  {"status", "native_failure"},
                  {"selections", Json::array()},
                  {"mesh_generated", false}};
    require(strips.size() <= budget.max_control_points, "native path sample strip count budget");
    if (strips.empty()) {
        out.report["failure"] = "empty_strip_list";
        return out;
    }
    std::size_t controls = 0;
    // Preserve every native selection before range filtering. Equal source
    // positions and equal curves do not imply native reusable identity.
    auto append = [&](std::size_t strip, double u) {
        work.charge(1);
        const auto &s = strips[strip];
        require(s.v().pole_count() <= budget.max_control_points - controls,
                "native path sample cumulative curve-control budget");
        auto iso = detail::native_iso_u_curve(s, u, work, budget.max_control_points);
        controls += iso.curve.poles().size();
        out.report["selections"].push_back(
            {{"strip", strip}, {"u", u}, {"iso_zero_weight_fallbacks", iso.zero_weight_fallbacks}});
        out.curves.push_back({strip, out.curves.size(), u, std::move(iso.curve)});
    };
    if (strips.size() == 1) {
        append(0, 0);
        if (source_profile_closed) {
            append(0, .325);
            append(0, .75);
        } else {
            append(0, .5);
            append(0, 1);
        }
    } else {
        for (std::size_t i = 0; i < strips.size(); ++i) {
            append(i, 0);
            append(i, .5);
        }
        if (!source_profile_closed)
            append(strips.size() - 1, 1);
    }
    std::vector<TubeMeshPathCurve> retained;
    retained.reserve(out.curves.size());
    for (auto &selected : out.curves) {
        const auto range = tube_mesh_control_range(selected.curve, budget);
        auto &entry = out.report["selections"][selected.selection_index];
        entry["range"] = {{"low", range.low},
                          {"high", range.high},
                          {"native_null", range.native_null},
                          {"accepted", range.accepted},
                          {"disconnected", range.disconnected},
                          {"rejected_weights", range.rejected_weights},
                          {"squared_extent", range.squared_extent}};
        entry["removed"] = range.degenerate;
        if (!range.degenerate)
            retained.push_back(std::move(selected));
    }
    out.curves = std::move(retained);
    out.report["retained_curves"] = out.curves.size();
    out.report["source_profile_closed"] = source_profile_closed;
    if (out.curves.empty()) {
        out.report["failure"] = "all_selected_curves_degenerate";
        out.report["work_used"] = budget.work;
        return out;
    }
    std::vector<const BsplineCurve *> curves;
    for (const auto &c : out.curves)
        curves.push_back(&c.curve);
    auto sampled =
        curve_detail::native_curve_sample_tree(curves, {0, 1}, chord_tolerance, angle_tolerance,
                                               budget.max_control_points, max_sample_nodes, work);
    out.report["sampling"] = std::move(sampled.report);
    out.success = sampled.success;
    if (out.success) {
        out.parameters = std::move(sampled.parameters);
        out.report["status"] = "sampled";
    } else
        out.report["failure"] = "adaptive_sampling";
    out.report["work_used"] = budget.work;
    return out;
}
} // namespace p3d::swept_detail
