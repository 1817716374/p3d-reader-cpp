#pragma once
#include "native_tube_facet_output.hpp"
#include "native_tube_facet_sections.hpp"
namespace p3d::swept_detail {
// getPatches converts each complete section ring to one working curve. This
// is distinct from getFacets' primitive partition and three-level grouping.
TubeFacetSectionBranches place_tube_patch_sections(const TubeCurveViews &, const Matrix4 *prefix,
                                                   const Matrix4 *suffix, const Json &flags,
                                                   TubeBudget &);
struct TubePatchPreparation {
    std::optional<TubeFacetPathPlacement> placement;
    TubeFacetSectionBranches sections;
    Json orientation;
    bool success = false;
    Json report;
};
TubePatchPreparation prepare_tube_patch_inputs(const Json &profile, const Json &path, TubeBudget &);
struct TubePatchGroup {
    std::size_t source_curve = 0; // Whole-profile conversion index, never a face/material ID.
    std::vector<TubeFacetSurface> surfaces;
};
struct TubePatchGeneration {
    TubePatchPreparation preparation;
    std::vector<TubePatchGroup> groups;
    bool success = false;
    Json report;
};
// Native empty per-curve output rejects the call but retains previous groups.
// Unsupported/resource failures throw; working paths carry ordered mutations
// between rings and preserve actual branch aliases, never coordinate equality.
TubePatchGeneration generate_tube_patch_groups(TubePatchPreparation, TubeBudget &);
TubePatchGeneration generate_tube_patch_groups(const Json &profile, const Json &path, TubeBudget &);
} // namespace p3d::swept_detail
