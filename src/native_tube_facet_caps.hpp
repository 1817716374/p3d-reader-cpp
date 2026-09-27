#pragma once
#include "native_tube_facet_assembly.hpp"
#include "native_tube_caps.hpp"
namespace p3d::swept_detail {
struct TubeFacetCapPair {
    // Ordered members of one profile group; borrowed native working surfaces.
    std::vector<const TubeFacetSurface *> first, last;
};
// 84cb0: constant-V endpoints of every member, outer/inner region assembly,
// endpoint closure checks and start-region reversal. Partial caps can remain
// when native_result is false; no planar face or triangulation is invented.
TubeCaps tube_facet_cap_regions(const std::vector<TubeFacetCapPair> &, TubeBudget &);
struct CappedTubeFacets {
    TubeFacetGroupAssembly sides;
    TubeCaps attempted;
    std::vector<Json> caps;
    Json face_indices;
    bool success = false;
    Json report;
};
// 857f0: preserve completed side groups, select each member's first/last output
// surface by native identity, generate caps only for capped nonclosed SOURCE
// paths, and publish both caps only after successful region construction.
CappedTubeFacets cap_tube_facet_assembly(TubeFacetGroupAssembly, bool capped, TubeBudget &);
CappedTubeFacets prepare_swept_tube_facets_with_caps(const Json &profile, const Json &path,
                                                     bool capped, TubeBudget &);
} // namespace p3d::swept_detail
