#pragma once
#include "native_tube_facet_groups.hpp"
#include "native_tube_path_groups.hpp"
namespace p3d::swept_detail {
// 82f40: positional UV boundary splice. A nonempty side needs at least five
// points. No endpoint comparison, welding, closure repair or domain inference.
std::vector<Point2> splice_tube_facet_boundaries(const std::vector<Point2> &,
                                                 const std::vector<Point2> &, TubeBudget &);
struct TubeFacetGroupAssembly {
    // Private working copies, including source surfaces mutated by combination.
    TubeFacetGroupGeneration generation;
    TubeFacetPathClassification classification;
    // [profile group][member][output surface] -> that member's working surface
    // index in generation. Keeps native references, not geometry deduplication.
    std::vector<std::vector<std::vector<std::size_t>>> groups;
    bool success = false;
    Json report;
};
// Consumes 835c0's path classes, splits at changes of V-order==2, combines
// compatible runs and rebuilds the first UV boundary. Completed profile groups
// survive native failure in a later group; the unfinished group is discarded.
// Does not perform caps, final entity construction or tessellation.
TubeFacetGroupAssembly assemble_tube_facet_groups(TubeFacetGroupGeneration,
                                                  TubeFacetPathClassification, TubeBudget &);
TubeFacetGroupAssembly assemble_tube_facet_groups(TubeFacetGroupGeneration, TubeBudget &);
} // namespace p3d::swept_detail
