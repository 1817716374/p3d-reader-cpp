#pragma once
#include "native_tube.hpp"
#include "native_surface_iso.hpp"
namespace p3d::swept_detail {
struct TubeFacetPlaneSample {
    // Retained for the native curve-pair fallback; never recreate them using
    // different weight or knot arithmetic after a failed plane query.
    detail::NativeIsoCurve first, second;
    std::optional<Point2> parameters;
    Json report;
};
// f6360's initial route: constant-U curves, then last/first plane hits.
// A missing result explicitly requires f5f20; it is NOT native failure.
TubeFacetPlaneSample sample_tube_facet_plane(const BsplineSurface &first,
                                             const BsplineSurface &second, double fraction_u,
                                             const std::array<Point3, 2> &plane,
                                             bool require_same_point, TubeBudget &);
// f5f20's selection AFTER successful transformed curve-pair intersection.
// The arrays are selected independently, without sorting or paired indexing.
std::optional<Point2> select_tube_facet_fallback_parameters(const std::vector<double> &first,
                                                            const std::vector<double> &second,
                                                            TubeBudget &);
struct TubeFacetParameters {
    std::optional<Point2> parameters;
    Json report;
};
// Full f5f20 curve-pair fallback: original two-curve frame/range planarity,
// independent transformed copies, native XY query and independent extrema.
// Computed failure is distinct from unsupported inputs/resource exceptions.
TubeFacetParameters sample_tube_facet_pair(const BsplineCurve &, const BsplineCurve &,
                                           TubeBudget &);
// Full f6360 single-U sample, including the curve-pair fallback when required.
TubeFacetParameters sample_tube_facet_seam(const BsplineSurface &, const BsplineSurface &,
                                           double fraction_u, const std::array<Point3, 2> &plane,
                                           bool require_same_point, TubeBudget &);
} // namespace p3d::swept_detail
