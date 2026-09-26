#pragma once
#include "native_bezier.hpp"
namespace p3d::curve_detail {
struct BezierRoots {
    bool success = false;
    // Native all-parameter sentinel is parameters.size()==coefficient count;
    // its equispaced entries are not separate isolated intersections.
    std::vector<double> parameters;
    std::vector<double> working_coefficients;
};
// Native default non-analytic branch, order 2..78, within [0,1]. Retains the
// working-coefficient changes made by monotonic-polygon correction. Source
// coefficients stay immutable. Does not add near-endpoint roots by default.
BezierRoots native_bezier_roots(const std::vector<double> &, BezierWork,
                                bool add_endpoint_roots = false);
struct BezierPlaneIntersections {
    bool success = false, all_parameters = false;
    std::vector<double> parameters;
    std::vector<BezierPole> points;
};
// Native unextended curve/plane query, order 2..26. Plane is homogeneous XYZW.
// Near-endpoint roots may lie just outside [0,1]; preserve returned order.
// Coplanar sentinel yields no discrete intersections. A full output buffer
// returns false with the previously written prefix retained.
BezierPlaneIntersections native_bezier_plane_intersections(const std::vector<BezierPole> &,
                                                           const BezierPole &plane,
                                                           std::size_t max_output, BezierWork);
} // namespace p3d::curve_detail
