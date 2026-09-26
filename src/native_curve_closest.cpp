// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Adapted from bezierDPoint4d.cpp and MSBsplineCurve_ByBezier.cpp.
// Native P3D arithmetic, immutable spans and bounded work. See THIRD_PARTY.md.
#include "native_curve_closest.hpp"
#include "native_bezier_support.hpp"
namespace p3d::curve_detail {
namespace {
using bezier_support::finite;
void validate(const std::vector<BezierPole> &p, BezierWork work) {
    require(p.size() >= 2 && p.size() <= 26, "native closest curve order must be 2..26");
    work.charge(8 * p.size());
    for (const auto &h : p)
        for (double x : h)
            finite(x);
}
BezierPole evaluate(std::vector<BezierPole> p, double s, BezierWork work) {
    work.charge(4 * p.size() * p.size());
    const double v = finite(1 - s);
    for (std::size_t j = 1; j < p.size(); ++j)
        for (std::size_t k = p.size() - 1; k >= j; --k)
            for (unsigned axis = 0; axis < 4; ++axis)
                p[k][axis] = finite(v * p[k - 1][axis] + s * p[k][axis]);
    return p.back();
}
} // namespace
BezierPseudoTangent native_bezier_pseudo_tangent(const std::vector<BezierPole> &p,
                                                 BezierWork work) {
    validate(p, work);
    const auto n = p.size(), degree = n - 1;
    BezierPseudoTangent out;
    out.unit_weight_branch =
        std::all_of(p.begin(), p.end(), [](const auto &h) { return std::abs(h[3] - 1.) <= 1e-8; });
    std::array<std::vector<double>, 4> source, derivative;
    work.charge(32 * n);
    for (unsigned axis = 0; axis < 4; ++axis) {
        for (const auto &h : p)
            source[axis].push_back(h[axis]);
        for (std::size_t i = 0; i < degree; ++i)
            derivative[axis].push_back(finite((p[i + 1][axis] - p[i][axis]) * double(degree)));
    }
    for (unsigned axis = 0; axis < 3; ++axis) {
        auto &tangent = out.coefficients[axis];
        if (out.unit_weight_branch)
            tangent = derivative[axis];
        else {
            tangent = native_bezier_product(derivative[axis], source[3], work);
            const auto other = native_bezier_product(source[axis], derivative[3], work);
            for (std::size_t i = 0; i < tangent.size(); ++i)
                tangent[i] = finite(tangent[i] - other[i]);
        }
    }
    return out;
}
BezierPerpendiculars native_bezier_perpendiculars(const std::vector<BezierPole> &p,
                                                  const BezierPole &fixed, BezierWork work) {
    for (double x : fixed)
        finite(x);
    const auto tangent = native_bezier_pseudo_tangent(p, work);
    BezierPerpendiculars out;
    out.unit_weight_branch = tangent.unit_weight_branch;
    for (unsigned axis = 0; axis < 3; ++axis) {
        std::vector<double> eye;
        if (fixed[3] == 0)
            eye.push_back(fixed[axis]);
        else
            for (const auto &h : p)
                eye.push_back(finite(h[axis] * fixed[3] -
                                     (out.unit_weight_branch ? fixed[axis] : fixed[axis] * h[3])));
        const auto product = native_bezier_product(tangent.coefficients[axis], eye, work);
        if (axis == 0)
            out.coefficients = product;
        else
            for (std::size_t i = 0; i < product.size(); ++i)
                out.coefficients[i] = finite(out.coefficients[i] + product[i]);
    }
    // Direction against a unit-weight line yields a constant polynomial. The
    // scalar native wrapper returns no roots for order 1, including zero.
    if (out.coefficients.size() >= 2) {
        auto roots = native_bezier_roots(out.coefficients, work, true);
        out.all_parameters = roots.parameters.size() == out.coefficients.size();
        if (!out.all_parameters)
            out.parameters = std::move(roots.parameters);
    }
    return out;
}
BezierClosestPoint native_bezier_closest_point(const std::vector<BezierPole> &p,
                                               const Point3 &fixed, double s0, double s1,
                                               BezierWork work) {
    for (double x : fixed)
        finite(x);
    finite(s0);
    finite(s1);
    const auto perpendiculars =
        native_bezier_perpendiculars(p, {fixed[0], fixed[1], fixed[2], 1}, work);
    BezierClosestPoint out;
    auto update = [&](double s) {
        const auto h = evaluate(p, s, work);
        if (h[3] == 0)
            return;
        const double dx = finite(h[0] / h[3] - fixed[0]);
        const double dy = finite(h[1] / h[3] - fixed[1]);
        const double dz = finite(h[2] / h[3] - fixed[2]);
        const double d2 = finite((dy * dy + dx * dx) + dz * dz);
        if (!out.found || d2 < out.squared_distance)
            out = {true, s, d2, h};
    };
    update(s0);
    update(s1);
    for (double s : perpendiculars.parameters)
        if (s > std::min(s0, s1) && s < std::max(s0, s1))
            update(s);
    return out;
}
CurveClosestPoint native_curve_closest_point(const BsplineCurve &curve, const Point3 &fixed,
                                             BezierWork work) {
    const auto order = curve.order();
    const auto n = curve.poles().size();
    require(order >= 2 && order <= 26 && n <= INT32_MAX, "native closest curve size exceeded");
    for (double x : fixed)
        finite(x);
    const auto candidates = curve.closed() ? n : n - order + 1;
    const auto &knots = curve.knots();
    work.charge(knots.size() + candidates);
    const auto domain = curve.knot_domain();
    CurveClosestPoint out;
    BezierPole selected{};
    // Object36a70 initializes its distance ceiling before scanning spans.
    double best = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < candidates; ++i) {
        const double low = knots[i + order - 1], high = knots[i + order];
        if (bezier_support::null_interval(low, high)) {
            ++out.skipped_intervals;
            continue;
        }
        work.charge(std::size_t(8) * order * order + 24 * order);
        const auto h =
            native_bezier_closest_point(bezier_support::extract(curve, i), fixed, 0, 1, work);
        ++out.segments;
        if (h.found && h.squared_distance < best) {
            best = h.squared_distance;
            out.found = true;
            out.fraction = finite((finite((high - low) * h.parameter + low) - domain[0]) /
                                  finite(domain[1] - domain[0]));
            out.weight = h.homogeneous[3];
            out.source_span = i;
            out.squared_distance = best;
            selected = h.homogeneous;
        }
    }
    // Project only the final winner. Direct coordinate/W distance evaluation
    // can stay finite when the inverse of an earlier candidate's W overflows.
    if (out.found) {
        const double inverse = finite(1. / out.weight);
        for (unsigned axis = 0; axis < 3; ++axis)
            out.point[axis] = out.weight == 1 ? selected[axis] : finite(inverse * selected[axis]);
    }
    // The native caller has undefined output when no candidate wins. Explicitly
    // report this case instead of manufacturing a point from uninitialized data.
    return out;
}
} // namespace p3d::curve_detail
