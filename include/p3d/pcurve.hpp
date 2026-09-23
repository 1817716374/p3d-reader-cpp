#pragma once
#include "reader.hpp"

namespace p3d {
struct PCurveSample {
    Point3 parameter; // X/Y are surface fractions; Z is retained by native sampling.
    Point3 position;
};
struct PCurveStrokeOptions {
    double uv_tolerance = 0.001;
    double spatial_tolerance = 1e-7;
    unsigned minimum_points = 2;
    // Finite higher-order curve fractions may extend beyond [0,1]; point
    // evaluation clamps to the knot domain. Order two ignores this interval.
    double start_fraction = 0;
    double end_fraction = 1;
    unsigned max_points = 200000;
    unsigned max_evaluations = 1000000;
};
struct PCurveStrokes {
    std::vector<PCurveSample> samples;
    Json report;
};
// Reconstructs the native append-PCurve sampling rules. The input curve's X/Y
// coordinates must already be surface fractions. Optional previous is the last
// sample of the same native stream, not a deduplication or inferred association.
// Nonunit surface knot domains are normalized in a private evaluation copy.
// Only newly appended samples are returned; incomplete results contain none.
// Native stopping tests concern sampled triples, not a continuous error bound.
// Point kernels support orders up to 26. Curve zero-weight fallback and surface
// parameter clamping are recorded separately; stored source data is not changed.
PCurveStrokes sample_native_pcurve(const BsplineSurface &surface, const BsplineCurve &curve,
                                   const PCurveStrokeOptions &options = {},
                                   const std::optional<PCurveSample> &previous = std::nullopt);

struct PCurveLoopStrokeOptions {
    // Both negative: UV 0.01, spatial control-range diagonal * 0.0001.
    // Otherwise each nonpositive tolerance uses the single-curve fallback.
    double uv_tolerance = -1;
    double spatial_tolerance = -1;
    unsigned max_points = 200000;
    unsigned max_evaluations = 1000000;
    unsigned max_loops = 10000;
    unsigned max_curves = 100000;
};
struct PCurveLoopStrokes {
    std::vector<std::vector<PCurveSample>> loops;
    Json report;
};
// Input loops explicitly identify their ordered members. Each curve must already
// be converted to an open B-spline with knot domain [0,1] and XY surface fractions.
// Reconstructs the restroke stage, not source region conversion or mesh closure.
// Stream state is shared within each loop and reset between loops. No deduplication,
// gap repair, implicit closing edge or reordering is performed. Budgets are global;
// any failed member clears every returned loop (stricter than native partial caches).
PCurveLoopStrokes sample_native_pcurve_loops(const BsplineSurface &surface,
                                             const std::vector<std::vector<BsplineCurve>> &loops,
                                             const PCurveLoopStrokeOptions &options = {});
} // namespace p3d
