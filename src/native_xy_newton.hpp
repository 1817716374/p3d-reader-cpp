#pragma once
#include "native_bezier.hpp"
#include <functional>
namespace p3d::curve_detail {
struct XYNewtonValue {
    Point2 residual{};
    // Rows are f/g, columns are u/v.
    std::array<Point2, 2> jacobian{};
};
using XYNewtonEvaluator = std::function<bool(double, double, XYNewtonValue &)>;
struct XYNewtonResult {
    Point2 parameters{};
    bool success = false, parameters_applied = false, converged = false, diagonal_fallback = false;
    unsigned iterations = 0, full_iterations = 0, diagonal_iterations = 0;
    std::string reason;
};
// Native chordal-intersection settings: 20 iterations per route, two successive
// strict step checks, absolute tolerance 1e-14 and relative tolerance 1e-12.
// Singular full steps restart the diagonal route at the ORIGINAL parameters.
// Native success may retain the original parameters or exhaust the iterations;
// it is not a guarantee of geometric coincidence or residual convergence.
XYNewtonResult native_xy_newton(const XYNewtonEvaluator &, Point2, BezierWork);
// Same Newton kernel with native rational Bezier XY evaluators. Z is ignored;
// poles near unit weight use raw XY, otherwise |evaluated W|<=1e-15 fails.
XYNewtonResult native_bezier_xy_newton(const std::vector<BezierPole> &first,
                                       const std::vector<BezierPole> &second, Point2, BezierWork);
} // namespace p3d::curve_detail
