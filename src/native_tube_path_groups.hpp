#pragma once
#include "native_tube.hpp"
namespace p3d::swept_detail {
struct TubeFacetPathClassification {
    bool success = false;
    std::vector<std::vector<std::int32_t>> groups;
    Json report;
};
// 81db0: source-order patch indices grouped by primitive type and adjacent
// straight-segment direction. No connectivity test, fitting or point welding.
// Native failure preserves completed groups but discards the pending group.
TubeFacetPathClassification classify_tube_facet_path(const Json &, TubeBudget &);
// 82760: direct classification, or classify members before/after the selected
// source member and fill its generated patch count with independent singleton
// groups. Existing source grouping is preserved on both sides.
TubeFacetPathClassification partition_tube_facet_path(const Json &, bool direct,
                                                      std::size_t selected_member,
                                                      std::size_t generated_patch_count,
                                                      TubeBudget &);
struct TubeFacetGroupGeneration;
// 835c0's actual call context: prepared working path, selected SOURCE member
// planarity, selected index and first group's first member's surface count.
TubeFacetPathClassification classify_generated_tube_facet_groups(const TubeFacetGroupGeneration &,
                                                                 TubeBudget &);
} // namespace p3d::swept_detail
