#pragma once
#include "internal.hpp"
namespace p3d::curve_detail {
// GeEllipse3d conversion shared by native trim boundaries and swept sources.
// Preserves the native angle adjustment and closed-knot representation.
BsplineCurve ellipse_to_bspline(const Json &value);
bool normalized_domain(double low, double high);
void fraction_knots(std::vector<double> &knots, double low, double high);
// Source primitive endpoints: polyline vertices and analytic ellipse angles
// precede conversion. Other primitives use their native B-spline point kernel.
std::optional<std::array<Point3, 2>> primitive_endpoints(const Json &,
                                                         const BsplineCurve * = nullptr);
bool endpoint_pair_closed(const Point3 &, const Point3 &);
} // namespace p3d::curve_detail
