#pragma once
#include "native_tube_facet_composition.hpp"
namespace p3d::swept_detail {
enum class TubeFacetSeamStatus { complete, native_failure, pending_general };
struct TubeFacetSeamResult {
    TubeFacetSeamStatus status = TubeFacetSeamStatus::pending_general;
    Json report;
};
// f4100's direct control-line branch for surfaces of V order 2. A pending
// general branch is explicitly distinguished from a native failure. Changes
// occur only after every column has passed the native intersection checks.
TubeFacetSeamResult apply_tube_ruled_facet_seam(Json &first, Json &second,
                                                TubeFacetSeamReferences &, TubeBudget &);
// f8130 visits current/next in order and wraps the final node to the head.
// Includes the native plane fallback; higher-order surface extension is pending.
// On pending, prior mutations persist; next_seam in the report identifies the
// unprocessed node. A native failure clears the entire chain as f95b0 does.
// Completion here still does not perform the subsequent f81c0 finalization.
TubeFacetSeamResult process_tube_facet_seams(TubeFacetComposition &, TubeBudget &,
                                             std::size_t start_seam = 0);
} // namespace p3d::swept_detail
