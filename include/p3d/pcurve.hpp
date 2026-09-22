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
PCurveStrokes sample_native_pcurve(const BsplineSurface &surface, const BsplineCurve &curve,
                                   const PCurveStrokeOptions &options = {},
                                   const std::optional<PCurveSample> &previous = std::nullopt);
} // namespace p3d
