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
struct TubeCurve {
    BsplineCurve curve;
    Json report;
};
// Native LineString-to-B-spline conversion used for both swept paths and
// sections: two points form a line; longer inputs use the degree-one fit.
// This can remove source vertices. It is not a general polyline simplifier.
// Singular interpolation and unrepresentable dense-knot removal states fail
// explicitly; they are not repaired by merging points or changing tolerances.
TubeCurve fit_tube_linestring(const std::vector<Point3> &points, TubeBudget &budget);
// One decoded BGFB primitive, not a VariantGeometry or a CurveVector. Native
// swept sources accept line, LineString, ellipse and B-spline primitives only.
// B-spline controls, weights, knots and closed state are copied without opening
// or normalization; successful conversion does not prove sweepability.
TubeCurve convert_tube_primitive(const Json &value, TubeBudget &budget);
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
// Separate closed-trace postprocessing used by the swept-body route. The
// general tube_surface above deliberately keeps its open V representation.
// Open V must be clamped and normalized; input is an untrimmed tube surface.
// Native closure failures retain the entire input, with applied=false in the
// report. A native zero return code alone does not prove that closure applied.
TubeAssembly close_tube_surface_v(const BsplineSurface &surface, TubeBudget &budget);
} // namespace p3d::swept_detail
