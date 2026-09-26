#pragma once
#include "internal.hpp"

namespace p3d::curve_detail {
using BezierPole = std::array<double, 4>;
struct BezierWork {
    std::size_t &used;
    std::size_t limit;
    void charge(std::size_t) const;
};
struct BezierPointTangent {
    Point3 point{}, tangent{};
    bool weight_fallback = false;
};
// Native homogeneous de Casteljau and Cartesian quotient. Fractions are not
// clamped. |evaluated W| <= 1e-12 uses reciprocal zero, not a singularity repair.
BezierPointTangent native_bezier_point_tangent(const std::vector<BezierPole> &, double, BezierWork);
// Native control-polygon estimate. Unlike newer upstream versions, collinear
// higher-order inputs retain at least degree edges; zero polygon edges use X.
unsigned native_bezier_edge_count(const std::vector<BezierPole> &, bool weights_are_one,
                                  double chord_tolerance, double angle_tolerance,
                                  double maximum_edge_length, BezierWork);
struct BezierMoments {
    Point3 normal{};
    Matrix3 centroid_tensor{};
    std::size_t evaluations = 0, weight_fallbacks = 0;
    unsigned edges = 1;
};
// Integral of curve-to-chord strips plus the origin/endpoints triangle. Native
// five-point Gauss evaluates a coarse interval and both halves, retaining only
// the sum from the halves. No adaptive tolerance or Richardson extrapolation.
BezierMoments native_bezier_moments(const std::vector<BezierPole> &, double u0, double u1,
                                    const Point3 &origin, BezierWork);
// Full [0,1] segment with the area visitor's angle-0.3 edge estimate. Raw moment
// sums do not certify closure, planarity, denominator regularity or a solid.
BezierMoments native_bezier_area(const std::vector<BezierPole> &, bool weights_are_one,
                                 const Point3 &origin, BezierWork);
} // namespace p3d::curve_detail
