#pragma once
#include "native_tube.hpp"

namespace p3d::swept_detail {
struct NativeBoundarySpan {
    std::size_t count = 0;
    unsigned edge = 0; // 1/2: low/high U; 4/8: low/high V; 0: general span
};
// 125a60: count until a parameter-domain edge break. Adjacent spans share one
// point. The native diagnostic counter has no effect on the returned span.
NativeBoundarySpan native_boundary_span(const std::vector<Point2> &, std::size_t first,
                                        double tolerance, TubeBudget &);
struct NativeSurfaceBoundary {
    std::vector<BsplineCurve> curves;
    Json report;
};
// b26e0 with curve fitting disabled, as used by swept getFace. Reads the active
// UV caches, not pcurve lists. Iso spans use native subcurve extraction; other
// spans become open degree-one B-splines of evaluated UV vertices. Appended
// outer isocurves follow the native closed-U/V and hole-origin rules.
NativeSurfaceBoundary native_surface_boundary(const BsplineSurface &,
                                              const std::vector<std::vector<Point2>> &,
                                              bool include_outer, TubeBudget &,
                                              std::size_t max_curves = 1000000);
} // namespace p3d::swept_detail
