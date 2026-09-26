#pragma once
#include "internal.hpp"
namespace p3d::curve_detail {
// GeEllipse3d conversion shared by native trim boundaries and swept sources.
// Preserves the native angle adjustment and closed-knot representation.
BsplineCurve ellipse_to_bspline(const Json &value);
} // namespace p3d::curve_detail
