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
} // namespace p3d::swept_detail
