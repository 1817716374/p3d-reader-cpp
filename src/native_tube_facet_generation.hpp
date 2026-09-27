#pragma once
#include "native_tube_facet_trim.hpp"
#include "native_tube_facet_seams.hpp"
namespace p3d::swept_detail {
struct TubeFacetGeneration {
    TubeFacetSeamStatus status = TubeFacetSeamStatus::native_failure;
    // Includes authoritative seam references and updated shared source objects,
    // even when native generation has failed after updating a source curve.
    TubeFacetComposition composition;
    TubeFacetTrimResult trimmed;
    Json report;
};
// Complete f95b0 with the cf5a0 facet callback: ordered branch construction,
// composition, all seams, combined path and final native boundary processing.
// Prefix section is selected whenever its input pointer is present, including
// the case with no prefix path. The selected UPDATED source is used for trim.
// No caps, outer grouping, final surface-copy wrapper or mesh conversion here.
TubeFacetGeneration generate_tube_facet_boundaries(const BsplineCurve *prefix_path,
                                                   const BsplineCurve *prefix_section,
                                                   const BsplineCurve *suffix_path,
                                                   const BsplineCurve *suffix_section, bool rigid,
                                                   TubeBudget &);
} // namespace p3d::swept_detail
