// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Adapted from bezierDPoint4d.cpp and MSBsplineCurve_ByBezier.cpp.
// Native P3D root/weight rules, immutable independent spans, work budgets.
// See THIRD_PARTY.md.
#include "native_curve_range.hpp"
#include "native_curve_closest.hpp"
#include "native_bezier_support.hpp"
namespace p3d::curve_detail {
namespace {
using bezier_support::finite;
void extend(NativeCurveRange &out, const Point3 &point) {
    if (!out.present) {
        out.low = out.high = point;
        out.present = true;
    } else
        for (unsigned axis = 0; axis < 3; ++axis) {
            if (point[axis] < out.low[axis])
                out.low[axis] = point[axis];
            if (point[axis] > out.high[axis])
                out.high[axis] = point[axis];
        }
}
void extend(NativeCurveRange &out, const BezierPole &h) {
    if (!(std::abs(h[3]) > 1e-12)) {
        ++out.rejected_weights;
        return;
    }
    const double inverse = finite(1. / h[3]);
    extend(out, Point3{finite(h[0] * inverse), finite(h[1] * inverse), finite(h[2] * inverse)});
}
BezierPole evaluate(std::vector<BezierPole> p, double u, BezierWork work) {
    work.charge(4 * p.size() * p.size());
    const double v = finite(1 - u);
    for (std::size_t j = 1; j < p.size(); ++j)
        for (std::size_t k = p.size() - 1; k >= j; --k)
            for (unsigned axis = 0; axis < 4; ++axis)
                p[k][axis] = finite(v * p[k - 1][axis] + u * p[k][axis]);
    return p.back();
}
} // namespace
NativeCurveRange native_bezier_range(const std::vector<BezierPole> &p, BezierWork work) {
    const auto tangent = native_bezier_pseudo_tangent(p, work);
    NativeCurveRange out;
    out.segments = 1;
    out.segment_controls = p.size();
    extend(out, p.front());
    extend(out, p.back());
    if (p.size() > 2)
        for (const auto &a : tangent.coefficients) {
            const auto roots = native_bezier_roots(a, work);
            // This caller ignores convergence and evaluates the all-parameter
            // sentinel; plane/perpendicular callers have different rules.
            for (double u : roots.parameters) {
                extend(out, evaluate(p, u, work));
                ++out.extrema_evaluations;
            }
        }
    return out;
}
NativeCurveRange native_curve_range(const BsplineCurve &curve, std::size_t max_segment_controls,
                                    BezierWork work) {
    const auto order = curve.order();
    const auto n = curve.poles().size();
    require(order >= 2 && order <= 26 && n <= INT32_MAX, "native curve range order/control size");
    const auto candidates = curve.closed() ? n : n - order + 1;
    const auto &knots = curve.knots();
    work.charge(knots.size() + candidates);
    NativeCurveRange out;
    for (std::size_t i = 0; i < candidates; ++i) {
        if (bezier_support::null_interval(knots[i + order - 1], knots[i + order])) {
            ++out.skipped_intervals;
            continue;
        }
        require(out.segment_controls <= max_segment_controls &&
                    order <= max_segment_controls - out.segment_controls,
                "native curve range segment control budget exceeded");
        work.charge(std::size_t(8) * order * order + 24 * order);
        const auto segment = native_bezier_range(bezier_support::extract(curve, i), work);
        if (segment.present) {
            extend(out, segment.low);
            extend(out, segment.high);
        }
        out.segments += segment.segments;
        out.segment_controls += segment.segment_controls;
        out.extrema_evaluations += segment.extrema_evaluations;
        out.rejected_weights += segment.rejected_weights;
    }
    return out;
}
} // namespace p3d::curve_detail
