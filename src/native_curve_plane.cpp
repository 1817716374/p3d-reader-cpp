// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Adapted from MSBsplineCurve_ByBezier.cpp; P3D parameter/projection arithmetic,
// immutable source spans and work/output budgets. See THIRD_PARTY.md.
#include "native_curve_plane.hpp"
#include "native_bezier_support.hpp"
namespace p3d::curve_detail {
CurvePlaneIntersections native_curve_plane_intersections(const BsplineCurve &curve,
                                                         const BezierPole &plane,
                                                         std::size_t max_output, BezierWork work) {
    using bezier_support::finite;
    const auto order = curve.order(), degree = order - 1;
    const auto n = curve.poles().size();
    require(order >= 2 && order <= 26 && n <= INT32_MAX,
            "native curve plane order or control count exceeded");
    for (double x : plane)
        finite(x);
    const auto candidates = curve.closed() ? n : n - order + 1;
    const auto &knots = curve.knots();
    work.charge(knots.size() + candidates);
    const auto domain = curve.knot_domain();
    const double range = finite(domain[1] - domain[0]);
    CurvePlaneIntersections out;
    for (std::size_t i = 0; i < candidates; ++i) {
        const double low = knots[i + degree], high = knots[i + order];
        if (bezier_support::null_interval(low, high)) {
            ++out.skipped_intervals;
            continue;
        }
        work.charge(std::size_t(8) * order * order + 24 * order);
        const auto p = bezier_support::extract(curve, i);
        const auto hits = native_bezier_plane_intersections(p, plane, 26, work);
        ++out.segments;
        if (hits.all_parameters)
            ++out.all_parameter_segments;
        for (std::size_t j = 0; j < hits.parameters.size(); ++j) {
            require(out.intersections.size() < max_output,
                    "native curve plane output budget exceeded");
            work.charge(24);
            CurvePlaneIntersection hit;
            hit.fraction = finite(
                (finite(finite((high - low) * hits.parameters[j]) + low) - domain[0]) / range);
            const auto &h = hits.points[j];
            hit.weight = h[3];
            hit.source_span = i;
            hit.projection_succeeded = h[3] != 0;
            if (h[3] == 1)
                hit.point = {h[0], h[1], h[2]};
            else if (h[3] != 0) {
                const double inverse = finite(1. / h[3]);
                for (unsigned axis = 0; axis < 3; ++axis)
                    hit.point[axis] = finite(inverse * h[axis]);
            }
            out.intersections.push_back(hit);
        }
    }
    return out;
}
} // namespace p3d::curve_detail
