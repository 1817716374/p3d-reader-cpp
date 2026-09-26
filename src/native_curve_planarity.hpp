#pragma once
#include "native_curve_range.hpp"
namespace p3d::curve_detail {
// Native transformed range of the original primitive, not its spline conversion.
NativeCurveRange native_primitive_range(const Json &, const Matrix4 &, std::size_t, BezierWork);
// Native angle-in-sweep predicate used by ellipse extrema, including its tolerance.
bool native_range_angle_in_sweep(double angle, double start, double sweep);
struct NativeRangeZ {
    double scale = 0, rounded_span = 0, tolerance = 0;
    bool planar = false;
};
NativeRangeZ native_range_z(const NativeCurveRange &);
// Single original working member. Native frame/inverse/empty-range failure is
// a computed false; unsupported or unsafe arithmetic throws, never false.
Json native_primitive_planarity(const Json &, std::size_t max_controls, BezierWork);
} // namespace p3d::curve_detail
