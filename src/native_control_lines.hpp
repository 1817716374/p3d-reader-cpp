#pragma once
#include "internal.hpp"
namespace p3d::swept_detail {
struct NativeControlLinePair {
    bool success = false;
    Point3 first{}, second{};
    double first_fraction = 0, second_fraction = 0;
    // 0 outside, 1 near start, 2 interior, 4 near end. These do not clamp points.
    int first_location = 0, second_location = 0;
};
// Native closest points on two infinite control lines, with the separate
// segment-location classifications used by native consumers.
NativeControlLinePair native_control_line_pair(const Point3 &a0, const Point3 &a1, const Point3 &b0,
                                               const Point3 &b1, double distance_tolerance);
struct NativeRayPlaneIntersection {
    Point3 point{};
    double parameter = 0;
    bool divided = false;
};
// Neither direction nor plane normal is normalized. Failed protected division
// returns parameter zero and the input origin, as the native helper does.
NativeRayPlaneIntersection native_ray_plane_intersection(const Point3 &origin,
                                                         const Point3 &direction,
                                                         const std::array<Point3, 2> &plane);
} // namespace p3d::swept_detail
