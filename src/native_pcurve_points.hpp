#pragma once
#include "p3d/reader.hpp"
namespace p3d::detail {
struct NativePCurvePoint {
    Point3 point;
    bool zero_weight_fallback = false;
};
NativePCurvePoint pcurve_point(const BsplineCurve &, double fraction);
Point3 pcurve_surface_point(const BsplineSurface &, double u, double v);
} // namespace p3d::detail
