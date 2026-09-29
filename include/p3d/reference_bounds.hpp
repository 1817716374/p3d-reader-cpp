#pragma once
#include <p3d/reader.hpp>
#include <p3d/default_view_table.hpp>
namespace p3d {
struct NativeInlineReferenceClipInput {
    // Loaded runtime points; both coordinates DBL_MAX delimit loops.
    std::vector<std::array<double,2>> points;
    // Matrix selected by 15c7d0, before adding the corrected reference origin.
    // It is independent of the reference's geometry affine matrix and scale.
    std::optional<Matrix3> selected_matrix;
    std::optional<bool> depths_allowed;
    std::optional<double> lower_288, upper_280;
};
struct NativeReferenceBoundsNode {
    std::optional<bool> target_model_present;
    // Only relevant when the target model is absent. A present provider's
    // arbitrary range callback remains unresolved by this projection.
    std::optional<bool> no_model_provider_present;
    NativeCachedModelBoundsInput target_bounds;
    std::vector<std::size_t> children;
    bool children_complete = false;
    Json reference_input;
    ReferenceAffineContext affine_context;
    std::optional<std::uint32_t> runtime_flags_9c;
    std::optional<Point3> perspective_point_1b0;
    std::optional<double> perspective_distance_1c8;
    std::optional<std::uint32_t> clip_count_2d8;
    std::optional<bool> clip_pointer_278_present;
    std::optional<NativeInlineReferenceClipInput> inline_clip;
};
struct NativeModelReferenceBoundsInput {
    NativeCachedModelBoundsInput root_bounds;
    std::vector<std::size_t> root_references;
    bool root_references_complete = false;
    // Indices identify reference objects, not target model IDs. Repeated
    // indices are visited again in native order; no global deduplication.
    std::vector<NativeReferenceBoundsNode> references;
    std::size_t max_depth = 64; // additionally capped at 256 to protect the stack
    std::size_t max_visits = 100000;
};
struct NativeModelReferenceBoundsResult {
    bool resolved = false;
    std::string reason;
    std::optional<std::array<double,6>> range;
    std::vector<std::size_t> visit_order;
    std::optional<std::size_t> failed_reference;
};
// Default-view traversal: filter=null, recurse=true, both optional adjustment
// flags=false. Reuses reference_affine_transform with forced Z scaling.
// Covers absent providers, perspective, and loaded inline clipping points with
// a known selected clip matrix/depth gate. External clip objects, unknown clip
// state, provider callbacks,
// unresolved model/transform state, disconnect-marker translation/eye inputs,
// nonfinite arithmetic and cycles produce an unresolved result.
// The resulting range includes root and reference bounds, possibly the native
// empty sentinel. It can feed the default-table range selector once resolved.
NativeModelReferenceBoundsResult project_native_model_reference_bounds(const NativeModelReferenceBoundsInput &input);
} // namespace p3d
