#pragma once
#include "native_tube_facet_plane.hpp"
namespace p3d::swept_detail {
// Native surface combine with forceContinuity=true, reparamSurface=false,
// joining along V. No transverse compatibility conversion or endpoint repair.
// Requires untrimmed surfaces and open, equal-order V curves.
TubeAssembly combine_tube_surfaces_v(const BsplineSurface &, const BsplineSurface &, TubeBudget &);
// Consumes the higher-order plane preparation, preserving self-surface identity.
// Only publishes the updated preparation after both sides complete.
void extend_tube_facet_plane_seam(TubeFacetPlanePreparation &, const TubeFacetSeamReferences &,
                                  TubeBudget &);
} // namespace p3d::swept_detail
