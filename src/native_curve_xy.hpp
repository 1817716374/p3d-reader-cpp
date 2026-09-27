#pragma once
#include "native_xy_newton.hpp"
namespace p3d::curve_detail {
struct BezierXYIntersections {
    std::vector<Point2> parameters;
    std::size_t candidates = 0, retained_candidates = 0, failed_newton = 0, outside_parameters = 0,
                samples_a = 0, samples_b = 0;
};
// Native chordal route, with 100 retained polyline candidates per Bezier pair.
// Higher orders sample -0.1+i*0.05 (25 points); lines only sample endpoints.
// Preserves repeated intersections and the native successful Newton result,
// including unchanged seeds. This is not an exact all-roots guarantee.
BezierXYIntersections native_bezier_xy_intersections(const std::vector<BezierPole> &,
                                                     const std::vector<BezierPole> &,
                                                     std::size_t max_output, BezierWork);
struct CurveXYIntersection {
    Point2 fractions{};
    std::size_t first_span = 0, second_span = 0;
};
struct CurveXYIntersections {
    std::vector<CurveXYIntersection> intersections;
    std::size_t span_pairs = 0, candidates = 0, discarded_candidates = 0, failed_newton = 0,
                outside_parameters = 0;
};
// Native untransformed XY curve-pair query. The caller must supply the desired
// frame; Z is not used for intersection. No overlap intervals or deduplication.
CurveXYIntersections native_curve_xy_intersections(const BsplineCurve &, const BsplineCurve &,
                                                   std::size_t max_output, BezierWork);
} // namespace p3d::curve_detail
