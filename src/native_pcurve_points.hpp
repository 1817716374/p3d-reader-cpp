#pragma once
#include "p3d/pcurve.hpp"
namespace p3d::detail {
struct NativePCurvePoint {
    Point3 point;
    bool zero_weight_fallback = false;
    // Raw evaluated W is returned before the point division's zero-W fallback.
    double weight = 1;
};
NativePCurvePoint pcurve_point(const BsplineCurve &, double fraction);
struct NativePCurvePointTangent {
    NativePCurvePoint value;
    Point3 tangent{}; // Derivative with respect to the raw knot parameter.
    double weight_derivative = 0;
};
// Native point/tangent query: no knot tolerance or control-weight round trip.
// Closed fractions clamp, exactly as point-only queries do; no period wrapping.
NativePCurvePointTangent pcurve_point_tangent(const BsplineCurve &, double fraction);
Point3 pcurve_surface_point(const BsplineSurface &, double u, double v);
PCurveLoopStrokes sample_initial_pcurve_loops(const BsplineSurface &,
                                              const std::vector<std::vector<BsplineCurve>> &,
                                              const PCurveLoopStrokeOptions &);
} // namespace p3d::detail
