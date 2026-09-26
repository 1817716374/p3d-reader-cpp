#pragma once
#include "native_bezier_roots.hpp"
namespace p3d::curve_detail {
struct BezierPerpendiculars {
    bool all_parameters = false, unit_weight_branch = false;
    std::vector<double> coefficients, parameters;
};
// Unextended 3D perpendicular query. A zero fixed W denotes a direction.
BezierPerpendiculars native_bezier_perpendiculars(const std::vector<BezierPole> &,
                                                  const BezierPole &fixed, BezierWork);
struct BezierClosestPoint {
    bool found = false;
    double parameter = 0, squared_distance = 0;
    BezierPole homogeneous{};
};
// Query endpoints first, then strict interior roots; equal distance keeps first.
// W==0 candidates are skipped, without an epsilon. Bounds may be reversed.
BezierClosestPoint native_bezier_closest_point(const std::vector<BezierPole> &, const Point3 &,
                                               double s0, double s1, BezierWork);
struct CurveClosestPoint {
    bool found = false;
    double fraction = 0, squared_distance = 0, weight = 0;
    Point3 point{};
    std::size_t source_span = 0, segments = 0, skipped_intervals = 0;
};
CurveClosestPoint native_curve_closest_point(const BsplineCurve &, const Point3 &, BezierWork);
} // namespace p3d::curve_detail
