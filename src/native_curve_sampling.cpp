// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// C1 query and knot-data flow adapted from MSBsplineCurve.cpp and bsputil.cpp.
// P3D arithmetic, independent spans and bounded local storage replace runtime
// types/counters. See THIRD_PARTY.md.
#include "native_curve_sampling.hpp"
#include "native_bezier_support.hpp"
#include "native_curve_conversion.hpp"
namespace p3d::curve_detail {
namespace {
using bezier_support::finite;
double magnitude(const Point3 &p) {
    return finite(std::sqrt(finite(finite(p[0] * p[0] + p[1] * p[1]) + p[2] * p[2])));
}
double real_distance(const BezierPole &a, const BezierPole &b) {
    double denominator = 1;
    Point3 delta{};
    if (a[3] == 1 && b[3] == 1) {
        for (unsigned i = 0; i < 3; ++i)
            delta[i] = finite(a[i] - b[i]);
    } else {
        const double product = finite(b[3] * a[3]);
        if (product == 0)
            return 0; // Also preserves underflow-to-zero of the weight product.
        denominator = finite(product * product);
        for (unsigned i = 0; i < 3; ++i)
            delta[i] = finite(finite(b[3] * a[i]) - finite(a[3] * b[i]));
    }
    const double squared =
        finite(finite(delta[1] * delta[1] + delta[0] * delta[0]) + delta[2] * delta[2]);
    return finite(std::sqrt(finite(squared / denominator)));
}
void validate(const BsplineCurve &c, std::size_t limit) {
    require(c.order() >= 2 && c.order() <= 26 && c.poles().size() <= limit,
            "native curve sampling order/control limit");
}
} // namespace
double native_curve_vector_angle(const Point3 &a, const Point3 &b) {
    Point3 cross{finite(finite(a[1] * b[2]) - finite(a[2] * b[1])),
                 finite(finite(b[0] * a[2]) - finite(a[0] * b[2])),
                 finite(finite(a[0] * b[1]) - finite(b[0] * a[1]))};
    const double dot = finite(finite(a[0] * b[0] + b[1] * a[1]) + a[2] * b[2]);
    const double length = finite(
        std::sqrt(finite(finite(cross[1] * cross[1] + cross[0] * cross[0]) + cross[2] * cross[2])));
    return dot == 0 && length == 0 ? 0. : finite(std::atan2(length, dot));
}
NativeKnotData native_curve_knot_data(const BsplineCurve &curve, std::size_t max_controls,
                                      BezierWork work) {
    require(curve.poles().size() <= max_controls, "native knot-data control limit");
    NativeKnotData out;
    out.order = curve.order();
    out.closed = curve.closed();
    work.charge(curve.knots().size());
    out.all = curve.knots();
    if (out.all.empty())
        return out;
    out.compressed.push_back(out.all.front());
    out.multiplicities.push_back(1);
    for (std::size_t i = 1; i < out.all.size(); ++i) {
        work.charge(8);
        const auto u = out.all[i];
        if (bezier_support::null_interval(out.compressed.back(), u))
            ++out.multiplicities.back();
        else {
            out.compressed.push_back(u);
            out.multiplicities.push_back(1);
        }
        if (i + 1 == out.order) {
            out.left = out.compressed.size() - 1;
            out.compressed.back() = u;
        }
        if (i == out.all.size() - out.order) {
            out.right = out.compressed.size() - 1;
            out.compressed.back() = u;
        }
    }
    // Fresh arrays have matching multiplicity counts and sums by construction.
    // Native IsWellOrdered checks source order; it does not demand strictly
    // increasing compressed representatives or inspect the closed flag.
    work.charge(out.all.size());
    out.well_ordered = out.order != 0 && out.order <= out.all.size() / 2 &&
                       std::is_sorted(out.all.begin(), out.all.end()) && out.left < out.right &&
                       out.right < out.compressed.size();
    return out;
}
NativeCurveBreaks native_curve_c1_breaks(const BsplineCurve &curve, std::size_t max_controls,
                                         std::size_t max_output, BezierWork work) {
    validate(curve, max_controls);
    NativeCurveBreaks out;
    out.report = {{"scope", "native_curve_c1_discontinuities"}, {"segments", Json::array()}};
    const auto order = curve.order(), degree = order - 1;
    const auto candidates =
        curve.closed() ? curve.poles().size() : curve.poles().size() - order + 1;
    std::optional<BezierPointTangent> previous;
    double end_knot = 0;
    auto append = [&](double v) {
        require(out.parameters.size() < max_output, "native curve break output limit");
        work.charge(1);
        out.parameters.push_back(finite(v));
    };
    std::size_t skipped = 0;
    for (std::size_t i = 0; i < candidates; ++i) {
        work.charge(1);
        const auto low = curve.knots()[i + degree], high = curve.knots()[i + order];
        if (bezier_support::null_interval(low, high)) {
            ++skipped;
            continue;
        }
        work.charge(8 * std::size_t(order) * order + 32 * order);
        const auto poles = bezier_support::extract(curve, i);
        double length = 0;
        for (std::size_t j = 1; j < poles.size(); ++j)
            length = finite(length + real_distance(poles[j - 1], poles[j]));
        const double tolerance = finite(length * 1e-8);
        std::string reason = "first_segment";
        if (previous) {
            const auto start = native_bezier_point_tangent(poles, 0, work);
            if (!endpoint_pair_closed(previous->point, start.point))
                reason = "position";
            else if (native_curve_vector_angle(previous->tangent, start.tangent) > 1e-12)
                reason = "angle";
            else if (magnitude(previous->tangent) < tolerance ||
                     magnitude(start.tangent) < tolerance)
                reason = "near_zero_tangent";
            else
                reason = "continuous";
        }
        if (reason != "continuous")
            append(finite(finite(high - low) * 0. + low));
        end_knot = finite(finite(high - low) + low);
        previous = native_bezier_point_tangent(poles, 1, work);
        out.report["segments"].push_back({{"support", i},
                                          {"knot_interval", {low, high}},
                                          {"polygon_length", length},
                                          {"tangent_tolerance", tolerance},
                                          {"reason", reason},
                                          {"end_weight_fallback", previous->weight_fallback}});
    }
    if (!out.parameters.empty())
        append(end_knot);
    const auto domain = curve.knot_domain();
    const double span = finite(domain[1] - domain[0]);
    const bool divided = std::abs(span) > 1e-15;
    const double factor = divided ? finite(1 / span) : 1.;
    for (auto &v : out.parameters) {
        work.charge(4);
        // Native adds knotA back AFTER scaling. Do not silently turn this
        // query into the usual [0,1] mapping on a non-unit source domain.
        v = finite(finite(finite(v - domain[0]) * factor) + domain[0]);
    }
    out.report["skipped_intervals"] = skipped;
    out.report["knot_domain"] = domain;
    out.report["reciprocal_applied"] = divided;
    out.report["normalization_factor"] = factor;
    out.report["status"] = "complete";
    out.report["work_used"] = work.used;
    return out;
}
std::vector<BsplineCurve> native_curve_make_beziers(const BsplineCurve &curve,
                                                    std::size_t max_controls, BezierWork work) {
    validate(curve, max_controls);
    const auto order = curve.order(), degree = order - 1;
    const auto candidates = curve.closed() ? curve.poles().size() : curve.poles().size() - degree;
    std::vector<BsplineCurve> result;
    std::size_t count = 0;
    for (std::size_t i = 0; i < candidates; ++i) {
        work.charge(1);
        if (bezier_support::null_interval(curve.knots()[i + degree], curve.knots()[i + order]))
            continue;
        require(order <= max_controls - count, "native makeBeziers output control budget");
        count += order;
        work.charge(8 * std::size_t(order) * order + 16 * order);
        const auto poles = bezier_support::extract(curve, i);
        Json xyz = Json::array(), weights = Json::array();
        bool unit = true;
        for (const auto &p : poles) {
            unit = unit && std::abs(finite(p[3] - 1)) <= 1e-8;
            for (unsigned j = 0; j < 3; ++j)
                xyz.push_back(p[j]);
            weights.push_back(p[3]);
        }
        result.push_back(BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                                  {"order", order},
                                                  {"closed", false},
                                                  {"poles", std::move(xyz)},
                                                  {"weights", unit ? Json() : std::move(weights)},
                                                  {"knots", nullptr}}));
    }
    return result;
}
} // namespace p3d::curve_detail
