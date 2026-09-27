#pragma once
#include "native_tube.hpp"
#include <memory>
namespace p3d::swept_detail {
struct TubeFacetMember {
    unsigned boundary_type = 1;
    std::size_t source_member = 0;
    std::optional<std::size_t> source_segment;
    std::string source_path;
    // Native line/ellipse/B-spline wrappers retain their source primitive.
    // Only polyline segments allocate a new line. The owning partition keeps
    // the immutable source tree alive through copies and moves of this view.
    const Json *source_geometry = nullptr;
    std::optional<Json> derived_line;
    const Json &geometry() const {
        return derived_line ? *derived_line : *source_geometry;
    }
};
struct TubeFacetProfile {
    std::shared_ptr<const Json> source;
    std::vector<std::vector<TubeFacetMember>> groups;
    Json report;
};
// Native face-patch profile partition, separate from whole-surface conversion.
// Non-four boundary kinds form one group; parity children form separate groups.
// Unknown/nonparticipating primitive kinds are skipped by this native helper.
// Native failure discards the current temporary group but retains earlier ones.
TubeFacetProfile partition_tube_facet_profile(std::shared_ptr<const Json>, TubeBudget &);
// Native facet conversion clones/converts the source group, then opens each
// closed curve at raw knot (0 - domain.low) / domain.span. The native caller
// ignores an opening status failure and keeps that curve closed. Unsupported
// arithmetic/layouts and resource limits still throw rather than fake success.
TubeProfile prepare_tube_facet_curves(const Json &, TubeBudget &);
TubeCurve prepare_tube_facet_member(const TubeFacetMember &, TubeBudget &);

using TubeFacetGroups = std::vector<std::vector<std::vector<Json>>>;
// Input must be the three-level native face-patch result, not the whole-sweep
// side surfaces. Does not infer this grouping from source or assign materials.
// Each returned side mapping retains its group/member/patch location.
Json enumerate_tube_facet_indices(const TubeFacetGroups &, std::size_t cap_count,
                                  bool generation_succeeded, TubeBudget &);
// Same native enumeration over assembled object references; never copies or
// inspects geometry merely to count output patches.
Json enumerate_tube_facet_reference_indices(
    const std::vector<std::vector<std::vector<std::size_t>>> &, std::size_t cap_count,
    bool generation_succeeded, TubeBudget &);
} // namespace p3d::swept_detail
