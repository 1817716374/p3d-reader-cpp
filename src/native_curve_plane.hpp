#pragma once
#include "native_bezier_roots.hpp"
namespace p3d::curve_detail {
struct CurvePlaneIntersection {
    double fraction = 0;
    Point3 point{};
    double weight = 1;
    std::size_t source_span = 0;
    bool projection_succeeded = false;
};
struct CurvePlaneIntersections {
    std::vector<CurvePlaneIntersection> intersections;
    std::size_t segments = 0, skipped_intervals = 0, all_parameter_segments = 0;
};
// Native unextended B-spline/plane query: independent Bezier spans, original
// whole-curve fractions and source order. Shared span endpoints are repeated.
// W==0 projects to zero and remains in the result with an explicit flag.
// The output limit is a caller resource bound, not a native truncation request.
CurvePlaneIntersections native_curve_plane_intersections(const BsplineCurve &,
                                                         const BezierPole &plane,
                                                         std::size_t max_output, BezierWork);
} // namespace p3d::curve_detail
