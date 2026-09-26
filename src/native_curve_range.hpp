#pragma once
#include "native_bezier.hpp"
namespace p3d::curve_detail {
struct NativeCurveRange {
    bool present = false;
    Point3 low{}, high{};
    std::size_t segments = 0, skipped_intervals = 0, segment_controls = 0;
    std::size_t extrema_evaluations = 0, rejected_weights = 0;
};
// Native endpoint + pseudo-tangent roots range. Uses |W|>1e-12, without
// near-endpoint root addition; all-parameter root grids are evaluated too.
NativeCurveRange native_bezier_range(const std::vector<BezierPole> &, BezierWork);
// Independent native source spans, without endpoint repair or normalization.
NativeCurveRange native_curve_range(const BsplineCurve &, std::size_t max_segment_controls,
                                    BezierWork);
} // namespace p3d::curve_detail
