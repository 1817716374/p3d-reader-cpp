#pragma once
#include "native_tube_facet_uv.hpp"
namespace p3d::swept_detail {
struct TubeFacetUvBoundaryState {
    // Native boundary storage is separate from a BGFB CurveVector. Only the
    // first active_count entries participate; trailing allocated entries are
    // retained for diagnosis, never implicitly activated. Generated pcurves
    // are null: these boundaries contain raw UV polylines only.
    std::vector<std::vector<Point2>> allocated;
    std::size_t active_count = 0;
};
struct TubeFacetTrimResult {
    bool success = false;
    // Untrimmed geometry tables with the final native holeOrigin. Actual trim
    // storage is in boundaries, not an invented BGFB boundary serialization.
    std::vector<Json> surfaces;
    std::vector<TubeFacetUvBoundaryState> boundaries;
    Json report;
};
// f81c0 after seam processing. Supports the native initially untrimmed facet
// chain produced by composition. The chain remains read-only. Native success
// does not imply every seam sampled or every surface received a boundary.
TubeFacetTrimResult trim_tube_facet_chain(const TubeFacetComposition &,
                                          const BsplineCurve &combined_path,
                                          const BsplineCurve &profile, TubeBudget &);
// Uses f95b0's existing combined-path preparation, then f81c0.
TubeFacetTrimResult finalize_tube_facet_boundaries(const TubeFacetComposition &,
                                                   const BsplineCurve &profile, TubeBudget &);
} // namespace p3d::swept_detail
