#pragma once
#include "native_tube_facet_seams.hpp"
namespace p3d::swept_detail {
struct TubeFacetPlanePreparation {
    Json first, second;
    std::vector<Point3> first_projected, second_projected;
    std::vector<Point3> first_original, second_original;
    std::vector<double> first_weights, second_weights;
    double first_extension = 0, second_extension = 0;
    int classifier = -2;
    TubeFacetSeamStatus status = TubeFacetSeamStatus::pending_general;
    Json report;
};
// f4100 plane branch through projection, early exits and extension sizing.
// Both V-order-2 surfaces complete here. Higher-order cases retain the exact
// carried working rows and auxiliary curves for subsequent surface extension.
// Inputs are read-only; first/second address identity preserves self-seam state.
TubeFacetPlanePreparation prepare_tube_facet_plane_seam(const Json &first, const Json &second,
                                                        const TubeFacetSeamReferences &,
                                                        TubeBudget &);
// Direct-line branch followed by the plane branch when native control flow
// requests it. Pending extension never commits a half-applied current seam.
TubeFacetSeamResult apply_tube_facet_seam(Json &first, Json &second, TubeFacetSeamReferences &,
                                          TubeBudget &);
} // namespace p3d::swept_detail
