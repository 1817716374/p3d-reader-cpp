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
struct TubeAssembly {
    Json surface;
    Json report;
};
// General tube-surface assembly callback, not the separate native face-patch
// enumeration. Inputs come from one trace/profile: matching U layout, V order
// and rational flags. patch is one normalized Bezier V span; assembled contains
// prior_segments spans. close_trace is requested only for the final append.
// Miter/closure follow native control-line rules, not a watertightness repair.
TubeAssembly append_tube_patch(const BsplineSurface &assembled, const BsplineSurface &patch,
                               std::size_t prior_segments, bool close_trace, TubeBudget &budget);
struct TubeTrace {
    std::vector<BsplineCurve> segments; // Normalized Beziers in native processing order.
    Json report;
};
// Native prepareCurve storage shares each previous end control with the next
// segment, even across source full-multiplicity discontinuities. Source stays
// unchanged; report records discarded incoming leading controls.
TubeTrace prepare_tube_trace(const BsplineCurve &trace, TubeBudget &budget);
// General tubeSurface route for an already local-frame section and world-space
// B-spline trace. This is not the P3DSweptBody profile-placement/face API.
TubePatch tube_surface(const BsplineCurve &section, const BsplineCurve &trace, bool rigid,
                       TubeBudget &budget);
} // namespace p3d::swept_detail
