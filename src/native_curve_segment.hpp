#pragma once
#include "loft_curve.hpp"
#include "native_bezier.hpp"
namespace p3d::curve_detail {
struct NativeCurveSegment {
    std::optional<BsplineCurve> curve;
    // Tolerance evaluation changes the native source's weighted working poles.
    // Carry these to a subsequent query; the caller's source remains immutable.
    std::vector<Point3> working_poles;
    bool success = false;
    Json report;
};
// Raw open working storage, no domain normalization or end-knot correction.
bool insert_open_native_knot(loft_detail::Curve &, double knot, double tolerance,
                             unsigned multiplicity, unsigned limit, BezierWork, Json &);
// Native fraction-to-knot mapping, full-copy/periodic/partial branches and
// source tolerance side effects. Native failures can retain a whole-copy result.
NativeCurveSegment native_curve_segment(const BsplineCurve &, double first, double last,
                                        unsigned limit, BezierWork);
// Native reversal itself, without segment(1,0)'s preliminary tolerance query.
// Only the caller-owned working curve is changed.
bool reverse_native_working_curve(BsplineCurve &, unsigned limit, BezierWork, Json &report);
} // namespace p3d::curve_detail
