#pragma once
#include "native_bspline_area.hpp"
#include "native_tube.hpp"
namespace p3d::curve_detail {
struct CurveVectorArea {
    BsplineArea value;
    Json report;
};
// Native identity-transform source visitor: line, polyline, analytic ellipse,
// B-spline and nested CurveVector. Region types retain their native dispatch;
// this does not repair closure, intersections or infer geometric containment.
CurveVectorArea native_curve_vector_area(const Json &, swept_detail::TubeBudget &);
// Native source-ring orientation flags for face-patch construction. Empty or
// partial flag arrays are meaningful; report maps each flag to its source ring.
Json native_facet_orientation_flags(const Json &, Point3 tangent, swept_detail::TubeBudget &);
} // namespace p3d::curve_detail
