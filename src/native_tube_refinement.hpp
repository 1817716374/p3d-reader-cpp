#pragma once
#include "native_tube_curvature.hpp"
namespace p3d::swept_detail {
// Native bulk refinement used by the independent facet callback. Open curves
// only: this native helper allocates open knot storage even with a closed flag.
// Inserts raw knots in supplied order without normalization or deduplication.
TubeCurve refine_tube_facet_curve(const BsplineCurve &, const std::vector<double> &, TubeBudget &);
struct TubeFacetRefinedTrace {
    TubeFacetSampling sampling;
    std::optional<BsplineCurve> trace;
    std::vector<std::array<double, 2>> curvature_knots, greville;
    bool success = false;
    Json report;
};
// The facet callback passes an open normalized Bezier. Carries curvature-query
// working poles into bulk refinement, then computes ordered parameter/curvature
// Greville means. Ruled fallback does not construct those tensor-row inputs.
TubeFacetRefinedTrace prepare_tube_facet_trace(const BsplineCurve &, TubeBudget &);
} // namespace p3d::swept_detail
