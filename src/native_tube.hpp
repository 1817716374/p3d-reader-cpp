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
    // Only tube_surface fills this. Carry into subsequent operations on the
    // same native trace; tube_patch has no shared source-curve frame query.
    std::vector<Point3> working_trace_poles{};
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
// Native combine kernel for two open curves of equal order, including gaps.
// force_contiguous drops the incoming first control even if endpoints differ.
// Otherwise native Cartesian endpoint tolerance decides that choice. This does
// not perform the outer CurveArray wrapper's periodic opening/degree elevation.
// Reparameterization uses control-polygon lengths, with native weighted-control
// round trips and short-curve copy branches; source objects remain unchanged.
TubeCurve combine_open_tube_curves(const BsplineCurve &left, const BsplineCurve &right,
                                   bool force_contiguous, bool reparameterize, TubeBudget &budget);
// Native degree elevation of an already open curve. Same-degree input is copied
// exactly; elevation preserves native knot grouping and endpoint reassignment.
// Non-clamped layouts leaving native knot storage unwritten fail explicitly.
TubeCurve elevate_open_tube_curve(const BsplineCurve &, unsigned degree, TubeBudget &);
// Outer native two-curve combination: open closed inputs at source parameter
// zero, elevate the lower degree, then combine with native endpoint rules.
TubeCurve combine_tube_curves(const BsplineCurve &, const BsplineCurve &, bool force_contiguous,
                              bool reparameterize, TubeBudget &);
// Source CurveVector types 0..3, converted in original member order. Source
// endpoint agreement requests native closure, whose failure retains the open
// combined curve. Internal member gaps are not repaired or rejected by closure.
TubeCurve convert_tube_curve_array(const Json &, TubeBudget &);
struct TubeProfile {
    std::vector<BsplineCurve> curves;
    Json report;
};
// Simple group or parity group (type 4) of closed type-2/3 children. Source ring
// order is retained; no nesting inference, orientation or spatial sorting.
TubeProfile convert_tube_profile(const Json &, TubeBudget &);
struct TubePlacement {
    BsplineCurve trace;
    std::vector<BsplineCurve> sections;
    Json report;
};
// Frame at the converted trace start, then profile conversion and homogeneous
// world-to-profile transformation. Singular frame uses native identity fallback.
// The returned trace includes frame-query control round trips.
TubePlacement place_tube_profile(const Json &profile, const BsplineCurve &trace, TubeBudget &);
struct TubeSurfaces {
    std::vector<Json> surfaces;
    BsplineCurve trace;
    Json report;
};
// Internal preparation: carries native trace state across source rings, closes
// V for closed traces, queries the carried trace's native start tangent, and
// applies area orientation. Report distinguishes native success from visiting
// every ring. No source isValidGeom/isValidNum dispatch, caps or native face
// enumeration is applied here; failure retains generated/partly oriented data.
TubeSurfaces prepare_swept_tube_surfaces(const Json &profile, const Json &path, TubeBudget &);
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
