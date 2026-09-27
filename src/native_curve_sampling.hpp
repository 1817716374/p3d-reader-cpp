#pragma once
#include "native_bezier.hpp"
#include <deque>
namespace p3d::curve_detail {
struct NativeKnotData {
    std::vector<double> all, compressed;
    std::vector<std::size_t> multiplicities;
    std::size_t order = 0, left = 0, right = 0;
    bool closed = false, well_ordered = false;
};
// Fresh native knot-data object. Does not normalize, modify or merge source knots.
NativeKnotData native_curve_knot_data(const BsplineCurve &, std::size_t max_controls, BezierWork);
// Native makeBeziers: independent normalized spans. Near-unit weights are
// omitted without deweighting XYZ, using the native default 1e-8 test.
std::vector<BsplineCurve> native_curve_make_beziers(const BsplineCurve &, std::size_t max_controls,
                                                    BezierWork);
struct NativeCurveBreaks {
    std::vector<double> parameters;
    Json report;
};
// Native C1 query, including its non-unit-domain offset and guarded reciprocal.
// Independent saturated spans retain both sides of positional discontinuities.
NativeCurveBreaks native_curve_c1_breaks(const BsplineCurve &, std::size_t max_controls,
                                         std::size_t max_output, BezierWork);
double native_curve_vector_angle(const Point3 &, const Point3 &);
struct NativeCurveSample {
    double parameter = 0;
    std::vector<Point3> points, tangents; // Fraction derivatives; curves remain separate.
};
struct NativeCurveSampleNode {
    NativeCurveSample sample;
    std::optional<std::size_t> parent, left, right;
    unsigned depth = 0;
    bool refined = false;
};
struct NativeCurveSampleTree {
    bool success = false;
    // Native provisional quarter samples are present even at accepted leaves.
    // Only refined nodes contribute to collected interior parameters.
    std::deque<NativeCurveSampleNode> nodes;
    std::vector<double> parameters;
    Json report;
};
// Native adaptive shared-parameter sampling (89120 / 88d50 / 63070). Each
// accepted interval tests midpoint, quarters and four tangent comparisons.
// The right-quarter positional test is native XY-only, not spatial distance.
// Native depth/zero-tolerance failure clears output; unsupported/budget throws.
// Tolerances retain native signs/near-zero tests, and are not error certificates.
NativeCurveSampleTree native_curve_sample_tree(const std::vector<const BsplineCurve *> &,
                                               std::array<double, 2> interval,
                                               double chord_tolerance, double angle_tolerance,
                                               std::size_t max_controls, std::size_t max_nodes,
                                               BezierWork);
} // namespace p3d::curve_detail
