// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Adapted from newton.cpp; P3D's separate restart/return rules, fixed caller
// settings, finite arithmetic and work accounting. See THIRD_PARTY.md.
#include "native_xy_newton.hpp"
namespace p3d::curve_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native XY Newton nonfinite arithmetic");
    return x;
}
bool divide(double n, double d, double &out) {
    if (std::abs(d) > finite(std::abs(n) * 1e-15)) {
        out = finite(n / d);
        return true;
    }
    out = 0;
    return false;
}
double norm(double a, double b) {
    return finite(std::sqrt(finite(finite(a * a) + finite(b * b))));
}
bool full_step(const XYNewtonValue &q, Point2 &step) {
    const auto &j = q.jacobian;
    const double ad = finite(j[0][0] * j[1][1]), bc = finite(j[0][1] * j[1][0]);
    const double det = finite(ad - bc);
    const double au = finite(finite(q.residual[0] * j[1][1]) - finite(q.residual[1] * j[0][1]));
    const double av = finite(finite(j[0][0] * q.residual[1]) - finite(q.residual[0] * j[1][0]));
    const double hf = norm(j[0][0], j[0][1]), hg = norm(j[1][0], j[1][1]);
    if (hf <= finite(hg * 1e-10) || hg <= finite(hf * 1e-10))
        return false;
    if (!(std::abs(det) > finite(finite(std::abs(bc) + std::abs(ad)) * 1e-15)))
        return false;
    return divide(au, det, step[0]) && divide(av, det, step[1]);
}
bool converged(const Point2 &p, const Point2 &step, unsigned &count) {
    if (std::abs(step[0]) < finite(std::abs(finite(1e-12 * p[0])) + 1e-14) &&
        std::abs(step[1]) < finite(std::abs(finite(1e-12 * p[1])) + 1e-14))
        return ++count >= 2;
    count = 0;
    return false;
}
bool retain_update(const XYNewtonValue &q, const Point2 &initial, bool fuzzy) {
    for (unsigned axis = 0; axis < 2; ++axis) {
        double bound = std::abs(initial[axis]);
        if (fuzzy)
            bound = finite(finite(bound + std::abs(finite(q.jacobian[axis][0] * 1e-14))) +
                           std::abs(finite(q.jacobian[axis][1] * 1e-14)));
        if (finite(std::abs(q.residual[axis]) - std::abs(bound)) > 0)
            return false;
    }
    return true;
}
XYNewtonResult run(const XYNewtonEvaluator &evaluate, Point2 seed, bool diagonal, BezierWork work) {
    XYNewtonResult result;
    result.parameters = seed;
    result.diagonal_fallback = diagonal;
    Point2 p = seed, initial{};
    XYNewtonValue q;
    unsigned small_steps = 0;
    auto finish = [&](const char *why, bool fuzzy) {
        result.success = true;
        result.reason = why;
        result.converged = fuzzy;
        result.parameters_applied = retain_update(q, initial, fuzzy);
        if (result.parameters_applied)
            result.parameters = p;
        return result;
    };
    for (unsigned iteration = 0; iteration < 20; ++iteration) {
        work.charge(96);
        ++result.iterations;
        if (diagonal)
            ++result.diagonal_iterations;
        else
            ++result.full_iterations;
        if (!evaluate(p[0], p[1], q)) {
            result.reason = "evaluation_failed";
            return result;
        }
        for (double x : q.residual)
            finite(x);
        for (const auto &row : q.jacobian)
            for (double x : row)
                finite(x);
        if (iteration == 0)
            initial = q.residual;
        Point2 step{}, factor{1, 1};
        if (diagonal) {
            const bool a = divide(q.residual[0], q.jacobian[0][0], step[0]);
            const bool b = divide(q.residual[1], q.jacobian[1][1], step[1]);
            if (!a && !b)
                return finish("diagonal_stalled", false);
            const double d = norm(q.jacobian[0][0], q.jacobian[1][1]);
            divide(std::abs(q.jacobian[0][0]), d, factor[0]);
            divide(std::abs(q.jacobian[1][1]), d, factor[1]);
        } else if (!full_step(q, step)) {
            // The original uu/vv have not been written by the full route.
            auto fallback = run(evaluate, seed, true, work);
            fallback.full_iterations = result.full_iterations;
            fallback.iterations += result.iterations;
            return fallback;
        }
        for (unsigned axis = 0; axis < 2; ++axis)
            p[axis] = finite(p[axis] - finite(step[axis] * factor[axis]));
        // Native diagonal convergence uses the unscaled quotients above.
        if (converged(p, step, small_steps))
            return finish("step_converged", true);
    }
    // The last residual is from BEFORE the last update; no extra evaluation,
    // best-iterate selection or residual-zero requirement is introduced.
    return finish("iteration_limit", false);
}
bool unit_weights(const std::vector<BezierPole> &p, BezierWork work) {
    require(p.size() >= 2 && p.size() <= 26, "native XY Bezier order must be 2..26");
    work.charge(p.size());
    bool unit = true;
    for (const auto &pole : p) {
        for (double x : pole)
            finite(x);
        if (std::abs(finite(pole[3] - 1.)) > 1e-15)
            unit = false;
    }
    return unit;
}
bool bezier_value(const std::vector<BezierPole> &p, bool unit, double u, Point2 &xy,
                  Point2 &derivative, BezierWork work) {
    const auto h = native_bezier_homogeneous_tangent(p, u, work);
    xy = {h.point[0], h.point[1]};
    derivative = {h.tangent[0], h.tangent[1]};
    if (unit)
        return true;
    double inverse;
    if (!divide(1., h.point[3], inverse))
        return false;
    const double square = finite(inverse * inverse);
    for (unsigned i = 0; i < 2; ++i) {
        xy[i] = finite(xy[i] * inverse);
        derivative[i] = finite(
            finite(finite(h.point[3] * h.tangent[i]) - finite(h.tangent[3] * h.point[i])) * square);
    }
    return true;
}
} // namespace
XYNewtonResult native_xy_newton(const XYNewtonEvaluator &evaluate, Point2 seed, BezierWork work) {
    require(bool(evaluate), "native XY Newton requires an evaluator");
    finite(seed[0]);
    finite(seed[1]);
    return run(evaluate, seed, false, work);
}
XYNewtonResult native_bezier_xy_newton(const std::vector<BezierPole> &a,
                                       const std::vector<BezierPole> &b, Point2 seed,
                                       BezierWork work) {
    const bool ua = unit_weights(a, work), ub = unit_weights(b, work);
    return native_xy_newton(
        [&](double u, double v, XYNewtonValue &q) {
            Point2 pa, pb, da, db;
            if (!bezier_value(a, ua, u, pa, da, work) || !bezier_value(b, ub, v, pb, db, work))
                return false;
            for (unsigned i = 0; i < 2; ++i) {
                q.residual[i] = finite(pa[i] - pb[i]);
                q.jacobian[i] = {da[i], -db[i]};
            }
            return true;
        },
        seed, work);
}
} // namespace p3d::curve_detail
