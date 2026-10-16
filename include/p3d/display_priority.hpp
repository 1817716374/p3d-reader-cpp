#pragma once
#include <cstdint>
#include <vector>

namespace p3d {
// Actual current-model and ancestor contributions, before cached evaluation.
// Adds with native 32-bit wrap and clamps once after the complete sum. This
// does not discover parent models or substitute the logical structure tree.
std::int32_t combine_native_model_display_priorities(const std::vector<std::int32_t> &);
// The inherited-priority branch, after its caller has selected the actual
// parent, layer and model context. INT32_MIN means zero only for parent_priority.
// combined_model_priority must already be evaluated for the actual model chain.
std::int32_t calculate_native_inherited_display_priority(std::int32_t parent_priority,
                                                         std::int32_t layer_priority,
                                                         std::int32_t combined_model_priority);
// Optional subsequent view range operation. Preserves the native ordered
// comparisons even for an inverted range; does not select or enable a view.
std::int32_t apply_native_display_priority_range(std::int32_t prepared_priority,
                                                 std::int32_t minimum, std::int32_t maximum);
} // namespace p3d
