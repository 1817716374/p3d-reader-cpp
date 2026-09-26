#pragma once
#include "internal.hpp"

namespace p3d::swept_detail {
// Internal stage of path-sweep reconstruction. Coordinates in section are
// already in the caller's profile frame. This is not a complete P3DSweptBody.
struct TubeBudget {
    std::size_t max_control_points = 1000000;
    std::size_t max_work = 10000000;
    std::size_t work = 0;
};
struct TubePatch {
    Json surface;
    Matrix3 final_frame; // Rows: profile X, profile Y, path tangent.
    Json report;
};
// Native zero-vector normalization is retained; no substitute reference axis.
Matrix3 advance_tube_frame(const Matrix3 &previous, Point3 tangent, bool rigid);
// Executes the Bezier callback and its tube-patch construction together.
// segment must be an open, normalized single Bezier segment, order 2..26.
// Frame is passed by value: callers explicitly carry the returned frame forward.
TubePatch tube_patch(const BsplineCurve &section, const BsplineCurve &segment, Matrix3 frame,
                     bool rigid, TubeBudget &budget);
} // namespace p3d::swept_detail
