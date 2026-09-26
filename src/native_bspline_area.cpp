// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from cv_properties.cpp and MSBsplineCurve_ByBezier.cpp.
// Changes: native P3D arithmetic, streaming immutable supports and work limits.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_bspline_area.hpp"
#include "native_bezier_support.hpp"
#include "native_pcurve_points.hpp"

namespace p3d::curve_detail {
namespace {
using bezier_support::finite;
// The area visitor executes the homogeneous transform even when it is the
// identity. Preserve its addition order, including signed-zero behavior.
BezierPole identity_product(const BezierPole &p) {
    BezierPole q{};
    for (unsigned r = 0; r < 3; ++r) {
        const double translation = p[3] == 1 ? 0. : p[3] * 0.;
        q[r] = finite(((translation + p[0] * (r == 0 ? 1. : 0.)) + p[1] * (r == 1 ? 1. : 0.)) +
                      p[2] * (r == 2 ? 1. : 0.));
    }
    q[3] = p[3];
    return q;
}
} // namespace
BsplineArea native_bspline_area(const BsplineCurve &curve, BezierWork work) {
    const auto order = curve.order(), degree = order - 1;
    const auto n = curve.poles().size();
    require(order >= 2 && order <= 26 && n <= INT32_MAX,
            "native B-spline area order or control count exceeded");
    const auto candidates = curve.closed() ? n : n - order + 1;
    // Charge reference evaluation before its allocations and knot scan. Range
    // queries in the native visitor are read-only and unused by its quadrature.
    work.charge(curve.knots().size());
    work.charge(std::size_t(8) * order * order);
    work.charge(candidates);
    BsplineArea result;
    const auto reference = detail::pcurve_point(curve, 0);
    result.reference = result.centroid = reference.point;
    result.reference_weight_fallback = reference.zero_weight_fallback;
    const auto &knots = curve.knots();
    for (std::size_t i = 0; i < candidates; ++i) {
        if (bezier_support::null_interval(knots[i + degree], knots[i + order])) {
            ++result.skipped_intervals;
            continue;
        }
        work.charge(std::size_t(8) * order * order + 24 * order);
        auto p = bezier_support::extract(curve, i);
        for (auto &h : p)
            h = identity_product(h);
        ++result.segments;
        const auto edges = native_bezier_edge_count(p, !curve.rational(), 0, .3, 0, work);
        result.edges += edges;
        const double step = 1. / edges;
        for (unsigned j = 0; j < edges; ++j) {
            const auto m =
                native_bezier_moments(p, j * step, (j + 1) * step, result.reference, work);
            result.evaluations += m.evaluations;
            result.weight_fallbacks += m.weight_fallbacks;
            // Native adds every integration interval straight to the visitor,
            // without first grouping sums by Bezier segment.
            for (unsigned r = 0; r < 3; ++r) {
                result.normal_sum[r] = finite(result.normal_sum[r] + m.normal[r]);
                for (unsigned c = 0; c < 3; ++c)
                    result.centroid_tensor[r][c] =
                        finite(result.centroid_tensor[r][c] + m.centroid_tensor[r][c]);
            }
        }
    }
    const auto &v = result.normal_sum;
    result.area = finite(std::sqrt(finite((v[0] * v[0] + v[1] * v[1]) + v[2] * v[2])));
    // The B-spline visitor does not update absAreaSum. Its final native test
    // therefore has threshold zero; do not introduce a polygon area tolerance.
    if (result.area > 0) {
        result.valid = true;
        const double inverse = finite(1. / result.area);
        for (unsigned r = 0; r < 3; ++r)
            result.normal[r] = finite(v[r] * inverse);
        for (unsigned r = 0; r < 3; ++r) {
            const auto &t = result.centroid_tensor[r];
            const double dot = finite((t[1] * result.normal[1] + t[0] * result.normal[0]) +
                                      t[2] * result.normal[2]);
            result.centroid[r] = finite(result.reference[r] + dot * inverse);
        }
    }
    return result;
}
} // namespace p3d::curve_detail
