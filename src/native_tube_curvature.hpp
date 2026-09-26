#pragma once
#include "native_tube.hpp"
namespace p3d::swept_detail {
struct TubeCurvatureSample {
    double fraction = 0, curvature = 0;
    std::optional<Point3> point;
    std::optional<std::size_t> left, right;
    unsigned depth = 0;
};
struct TubeCurvatureSampling {
    std::vector<TubeCurvatureSample> tree;
    // Only refined interior nodes, in native traversal order. Quarter-point
    // probes used for acceptance do not themselves become insertion knots.
    std::vector<std::array<double, 2>> interior;
    std::vector<Point3> working_poles;
    bool success = false;
    Json report;
};
// Native facet callback curvature tree, including ordered working-curve frame
// queries and the native depth failure. Infinity is the native zero-range
// tolerance; negative or NaN tolerance is rejected.
TubeCurvatureSampling sample_tube_facet_curvature(const BsplineCurve &, double tolerance,
                                                  TubeBudget &);
struct TubeFacetSampling {
    TubeCurvatureSampling sampling;
    double start_curvature = 0, end_curvature = 0;
    bool success = false, ruled_fallback = false;
    Json report;
};
// ce4a0's range-scaled tree and subsequent endpoint curvature queries. The
// caller supplies its private callback trace, after the callback's first frame
// query. This is not the tensor patch or complete face-chain generator.
TubeFacetSampling prepare_tube_facet_sampling(const BsplineCurve &, TubeBudget &);
} // namespace p3d::swept_detail
