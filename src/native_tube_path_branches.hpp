#pragma once
#include "native_tube_path_selection.hpp"
namespace p3d::swept_detail {
struct TubePathPiece {
    std::size_t index;
    double first = 0, last = 1;
    bool whole = true;
};
struct TubePathBranchPlan {
    std::vector<TubePathPiece> prefix, suffix;
    Json report;
};
// Native 7c020 branch ordering, preserving whole-member reference reuse.
TubePathBranchPlan plan_tube_path_branches(std::size_t count, std::size_t selected, double fraction,
                                           bool whole_planar, bool member_planar, TubeBudget &);
struct TubePathBranch {
    // A single whole source is referenced by index rather than cloned or deduped.
    std::optional<std::size_t> reused_curve_index;
    std::optional<BsplineCurve> constructed;
    // Distinguishes an allocated native subcurve with null poles from no branch.
    bool empty_curve_object = false;
    Json report;
};
struct TubeFacetPathBranches {
    Json whole_path_planarity;
    TubeFacetPath path;
    TubePathBranchPlan plan;
    TubePathBranch prefix, suffix;
    Json report;
};
// Planarity on the original path precedes work-copy preparation. Whole-member
// branches are combined with native opening/elevation/length rules. Interior
// members use ordered native subcurve queries, including empty-object results.
TubeFacetPathBranches prepare_tube_facet_path_branches(const Json &, const Json &, TubeBudget &);
} // namespace p3d::swept_detail
