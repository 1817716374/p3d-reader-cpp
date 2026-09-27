#pragma once
#include "native_tube_facet_generation.hpp"
namespace p3d::swept_detail {
struct TubeFacetSurface {
    Json geometry;
    // Independent native UV polylines. Every entry here is active; unused
    // allocation slots from the generation chain are not copied.
    std::vector<std::vector<Point2>> boundaries;
    // Optional native trim curves parallel to boundaries. An empty outer list
    // means all boundary records have null pcurve lists. These are runtime
    // curves, not an inferred BGFB boundary tree.
    std::vector<std::vector<BsplineCurve>> pcurves{};
};
// 1135e0 / 121190 for generated facets: deep-copy geometry and active UV
// polylines, generating missing knot arrays. No BGFB trim-tree invention.
TubeFacetSurface copy_tube_facet_surface(const Json &, const TubeFacetUvBoundaryState &,
                                         TubeBudget &);
// d00f0 appends to existing collections. The optional flags are the native
// unsigned classifier <= 1 predicate, not material indices or cap semantics.
// Its return value is independent of f95b0's generation result: an empty
// native-failure chain still completes the wrapper without appending anything.
struct TubeFacetSurfaceOutput {
    TubeFacetGeneration generation;
    Json report;
};
TubeFacetSurfaceOutput append_tube_facet_surfaces(std::vector<TubeFacetSurface> &surfaces,
                                                  std::vector<bool> *flags,
                                                  const BsplineCurve *prefix_path,
                                                  const BsplineCurve *prefix_section,
                                                  const BsplineCurve *suffix_path,
                                                  const BsplineCurve *suffix_section, TubeBudget &);
} // namespace p3d::swept_detail
