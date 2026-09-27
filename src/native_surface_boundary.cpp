#include "native_surface_boundary.hpp"
#include "native_surface_iso.hpp"
#include "native_curve_segment.hpp"
#include "native_pcurve_points.hpp"

namespace p3d::swept_detail {
namespace {
curve_detail::BezierWork work(TubeBudget &b) {
    return {b.work, b.max_work};
}
double finite(double x) {
    require(std::isfinite(x), "native surface boundary nonfinite arithmetic");
    return x;
}
unsigned point_edge(Point2 p, double low, double high) {
    return (p[0] <= low    ? 1u
            : p[0] >= high ? 2u
                           : 0u) |
           (p[1] <= low    ? 4u
            : p[1] >= high ? 8u
                           : 0u);
}
std::optional<BsplineCurve> iso_span(const BsplineSurface &surface, const std::vector<Point2> &uv,
                                     std::size_t start, std::size_t count, TubeBudget &b,
                                     Json &report) {
    const auto first = uv[start], last = uv[start + count - 1];
    auto low = first, high = first;
    for (std::size_t i = start; i < start + count; ++i) {
        work(b).charge(1);
        for (unsigned k = 0; k < 2; ++k) {
            low[k] = std::min(low[k], uv[i][k]);
            high[k] = std::max(high[k], uv[i][k]);
        }
    }
    // b23c0 prefers constant U when both extents are small. No averaging of
    // nearly equal UVs: the first point supplies the fixed coordinate.
    const bool constant_u = std::abs(finite(high[0] - low[0])) < 1e-7;
    if (!constant_u && !(std::abs(finite(high[1] - low[1])) < 1e-7)) {
        report["iso_attempted"] = false;
        return std::nullopt;
    }
    report["iso_attempted"] = true;
    auto iso = constant_u
                   ? detail::native_iso_u_curve(surface, first[0], work(b), b.max_control_points)
                   : detail::native_iso_v_curve(surface, first[1], work(b), b.max_control_points);
    const auto domain = iso.curve.knot_domain();
    const unsigned varying = constant_u ? 1 : 0;
    const double span = finite(domain[1] - domain[0]);
    const double a = finite(finite(first[varying] - domain[0]) / span);
    const double z = finite(finite(last[varying] - domain[0]) / span);
    report["iso_axis"] = constant_u ? "U" : "V";
    report["iso_zero_weight_fallbacks"] = iso.zero_weight_fallbacks;
    report["segment_fractions"] = {a, z};
    auto segment = curve_detail::native_curve_segment(
        iso.curve, a, z, static_cast<unsigned>(b.max_control_points), work(b));
    report["segment"] = std::move(segment.report);
    report["segment_success"] = segment.success;
    return segment.success ? std::move(segment.curve) : std::nullopt;
}
BsplineCurve sampled_span(const BsplineSurface &surface, const std::vector<Point2> &uv,
                          std::size_t start, std::size_t count, TubeBudget &b) {
    require(count <= b.max_control_points, "native surface boundary sampled control budget");
    Json poles = Json::array();
    for (std::size_t i = start; i < start + count; ++i) {
        work(b).charge(std::size_t(surface.u().order()) * surface.v().order());
        const auto p = detail::pcurve_surface_point(surface, uv[i][0], uv[i][1]);
        for (double x : p)
            poles.push_back(x);
    }
    // afaa0 copies every vertex, including duplicates, and asks 127af0 for
    // uniform knots. No chord-length fit or simplification in this branch.
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", 2},
                                    {"closed", false},
                                    {"poles", std::move(poles)},
                                    {"weights", nullptr},
                                    {"knots", nullptr}});
}
} // namespace

NativeBoundarySpan native_boundary_span(const std::vector<Point2> &points, std::size_t first,
                                        double tolerance, TubeBudget &b) {
    require(first < points.size() && points.size() <= INT32_MAX,
            "native surface boundary span input bounds");
    require(std::isfinite(tolerance) && tolerance >= 0 && tolerance < .5,
            "native surface boundary edge tolerance");
    const double high = 1 - tolerance;
    const auto p = points[first];
    for (unsigned edge : {1u, 2u, 4u, 8u}) {
        const unsigned axis = edge < 4 ? 0 : 1;
        const bool lower = edge == 1 || edge == 4;
        work(b).charge(1);
        if (!(lower ? p[axis] <= tolerance : p[axis] >= high))
            continue;
        std::size_t end = first + 1;
        // Native low-edge membership is strict AFTER the initial vertex;
        // high-edge membership remains inclusive for every vertex.
        while (end < points.size()) {
            work(b).charge(1);
            if (!(lower ? points[end][axis] < tolerance : points[end][axis] >= high))
                break;
            ++end;
        }
        if (end - first > 1)
            return {end - first, edge};
    }
    std::size_t end = first + 1;
    while (end < points.size()) {
        work(b).charge(1);
        const auto edge = point_edge(points[end++], tolerance, high);
        if (edge)
            break; // Include this edge vertex in the preceding general span.
    }
    return end - first > 1 ? NativeBoundarySpan{end - first, 0} : NativeBoundarySpan{};
}

NativeSurfaceBoundary native_surface_boundary(const BsplineSurface &surface,
                                              const std::vector<std::vector<Point2>> &boundaries,
                                              bool include_outer, TubeBudget &b,
                                              std::size_t max_curves) {
    require(b.max_control_points > 0 && b.max_control_points <= UINT32_MAX && max_curves > 0,
            "native surface boundary resource limits");
    require(surface.poles().size() <= b.max_control_points &&
                boundaries.size() <= b.max_control_points,
            "native surface boundary input control/record budget");
    require(surface.boundaries().is_null(), "native surface boundary requires runtime UV caches");
    NativeSurfaceBoundary out;
    out.report = {{"scope", "native_unstructured_boundary_without_fitting"},
                  {"spans", Json::array()},
                  {"outer", Json::array()},
                  {"skipped_short_records", Json::array()}};
    std::size_t points = 0;
    for (const auto &record : boundaries) {
        require(record.size() <= b.max_control_points - points && record.size() <= INT32_MAX,
                "native surface boundary cumulative UV point budget");
        points += record.size();
        for (auto p : record) {
            work(b).charge(1);
            finite(p[0]);
            finite(p[1]);
        }
    }
    auto append = [&](BsplineCurve curve) {
        require(out.curves.size() < max_curves, "native surface boundary output curve budget");
        work(b).charge(1);
        out.curves.push_back(std::move(curve));
    };
    for (std::size_t r = 0; r < boundaries.size(); ++r) {
        const auto &uv = boundaries[r];
        work(b).charge(1);
        if (uv.size() < 2) {
            out.report["skipped_short_records"].push_back(r);
            continue;
        }
        std::size_t first = 0;
        while (first + 1 < uv.size()) {
            const auto span = native_boundary_span(uv, first, 1e-10, b);
            if (span.count == 0)
                break;
            Json report{{"record", r},
                        {"first", first},
                        {"count", span.count},
                        {"edge", span.edge},
                        {"curve_index", out.curves.size()}};
            auto curve = iso_span(surface, uv, first, span.count, b, report);
            report["branch"] = curve ? "isocurve_segment" : "sampled_polyline";
            append(curve ? std::move(*curve) : sampled_span(surface, uv, first, span.count, b));
            out.report["spans"].push_back(std::move(report));
            first += span.count - 1;
        }
    }
    const bool outer = include_outer && (boundaries.empty() || surface.hole_origin() == 0);
    out.report["outer_requested"] = outer;
    auto add_iso = [&](bool constant_u, double fraction, bool reverse) {
        auto iso =
            constant_u
                ? detail::native_iso_u_curve(surface, fraction, work(b), b.max_control_points)
                : detail::native_iso_v_curve(surface, fraction, work(b), b.max_control_points);
        Json report{{"axis", constant_u ? "U" : "V"},
                    {"fraction", fraction},
                    {"reverse_requested", reverse},
                    {"curve_index", out.curves.size()},
                    {"zero_weight_fallbacks", iso.zero_weight_fallbacks}};
        if (reverse)
            report["reverse_result"] = curve_detail::reverse_native_working_curve(
                iso.curve, static_cast<unsigned>(b.max_control_points), work(b),
                report["reversal"]);
        append(std::move(iso.curve));
        out.report["outer"].push_back(std::move(report));
    };
    if (outer) {
        if (surface.u().closed() && !surface.v().closed()) {
            add_iso(false, 0, false);
            add_iso(false, 1, true);
        } else if (!surface.u().closed() && surface.v().closed()) {
            add_iso(true, 1, false);
            add_iso(true, 0, true);
        } else {
            // Native uses this order both when neither and when BOTH are closed.
            add_iso(false, 0, false);
            add_iso(true, 1, false);
            add_iso(false, 1, true);
            add_iso(true, 0, true);
        }
    }
    out.report["native_nonnull"] = !out.curves.empty();
    out.report["work_used"] = b.work;
    return out;
}
} // namespace p3d::swept_detail
