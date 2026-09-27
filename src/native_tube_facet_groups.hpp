#pragma once
#include "native_tube_facet_sections.hpp"
#include "native_tube_facet_output.hpp"
namespace p3d::swept_detail {
struct TubeFacetMemberSurfaces {
    // Position in the partitioned source group before native orientation reversal.
    std::size_t source_member = 0;
    std::vector<TubeFacetSurface> surfaces;
};
struct TubeFacetGroupGeneration {
    // Includes working paths after all processed members. Shared source-index
    // branches continue to refer to the same entry in selection.curves.
    TubeFacetPreparation preparation;
    std::vector<std::vector<TubeFacetMemberSurfaces>> groups;
    bool success = false;
    Json report;
};
// 835c0 through 84108: ordered profile members -> native surface copies,
// carrying working path mutations, then positional orientation reversal of
// member lists. Internal surface order remains unchanged. Does not execute
// later path classification, group combination, caps or final getFacets.
TubeFacetGroupGeneration generate_tube_facet_groups(TubeFacetPreparation, TubeBudget &);
TubeFacetGroupGeneration generate_tube_facet_groups(const Json &profile, const Json &path,
                                                    TubeBudget &);
} // namespace p3d::swept_detail
