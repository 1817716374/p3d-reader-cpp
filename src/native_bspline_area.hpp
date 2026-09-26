#pragma once
#include "native_bezier.hpp"

namespace p3d::curve_detail {
struct BsplineArea {
    Point3 reference{}, centroid{}, normal{}, normal_sum{};
    Matrix3 centroid_tensor{};
    double area = 0;
    bool valid = false, reference_weight_fallback = false;
    std::size_t segments = 0, skipped_intervals = 0, edges = 0;
    std::size_t evaluations = 0, weight_fallbacks = 0;
};
// Native area visitor for a single complete B-spline, identity world transform.
// Closure and planarity are not prerequisites imposed by the native visitor;
// valid means nonzero accumulated area, not a certified bounded region.
BsplineArea native_bspline_area(const BsplineCurve &, BezierWork);
} // namespace p3d::curve_detail
