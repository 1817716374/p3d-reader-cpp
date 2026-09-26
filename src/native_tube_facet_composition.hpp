#pragma once
#include "native_tube_facet_chain.hpp"
#include <memory>
namespace p3d::swept_detail {
template <class T> struct TubeFacetSeamStorage {
    T value;
    bool alive = true;
};
struct TubeFacetSeamReferences {
    int classifier = -2;
    std::shared_ptr<TubeFacetSeamStorage<Point3>> incoming, outgoing;
    std::shared_ptr<TubeFacetSeamStorage<std::array<Point3, 2>>> plane;
};
struct TubeFacetComposition {
    std::vector<TubeFacetNode> nodes;
    // Authoritative native allocation identities for subsequent seam processing.
    // nodes[].end_seam is only a value snapshot. Native deletion through one
    // reference can invalidate another: alive=false is retained, never dereferenced.
    std::vector<TubeFacetSeamReferences> seams;
    std::optional<BsplineCurve> working_prefix_path, working_suffix_path;
    // Only the pre-seam composition has completed, not the entire native caller.
    bool prepared = false;
    Json report;
};
// f95b0 through branch composition, immediately before f8130 seam processing.
// Paths must carry the state after BOTH branch-chain builders have run.
TubeFacetComposition compose_tube_facet_nodes(const std::vector<TubeFacetNode> &prefix,
                                              const std::vector<TubeFacetNode> &suffix,
                                              const BsplineCurve *prefix_path,
                                              const BsplineCurve *suffix_path, TubeBudget &);
// Null path means absent branch. Present paths require corresponding sections.
// Pointer identity preserves native sharing in the private working state; equal
// curves at different addresses remain independent. All input objects are read-only.
TubeFacetComposition prepare_tube_facet_composition(const BsplineCurve *prefix_path,
                                                    const BsplineCurve *prefix_section,
                                                    const BsplineCurve *suffix_path,
                                                    const BsplineCurve *suffix_section, bool rigid,
                                                    TubeBudget &);
} // namespace p3d::swept_detail
