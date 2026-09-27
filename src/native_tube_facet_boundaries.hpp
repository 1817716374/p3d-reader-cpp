#pragma once
#include "native_tube_facet_composition.hpp"
namespace p3d::swept_detail {
using TubeFacetBoundarySegments = std::vector<std::vector<Point3>>;
// Native f7b80: first is traversed forward, second backwards. Each segment of
// second is reversed IN PLACE; the segment list itself keeps its order. Segment
// ends are omitted positionally, without comparison or welding. Both lists
// empty leave polygon unchanged. first and second may reference the same list.
Json compose_tube_facet_boundary(std::vector<Point3> &polygon,
                                 const TubeFacetBoundarySegments &first,
                                 TubeFacetBoundarySegments &second, TubeBudget &);
// Native 122a90 polygon-boundary append: copies XY and drops Z. Empty input
// returns false without changing the list. Does not set holeOrigin or select
// the active boundary count; those are separate operations in the caller.
bool append_tube_facet_uv_boundary(std::vector<std::vector<Point2>> &boundaries,
                                   const std::vector<Point3> &polygon, TubeBudget &);
// f95b0's path preparation AFTER the complete seam pass: reverse a private
// prefix copy, combine with suffix without forced continuity/length reparam,
// or copy the sole suffix. This is input preparation, not f81c0 finalization.
TubeCurve prepare_tube_facet_trim_path(const TubeFacetComposition &, TubeBudget &);
} // namespace p3d::swept_detail
