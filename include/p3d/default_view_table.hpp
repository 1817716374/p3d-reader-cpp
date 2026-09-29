#pragma once
#include <p3d/default_view.hpp>
#include <p3d/view_model.hpp>
namespace p3d {
struct NativeCachedModelBoundsInput {
    // Complete result of the cached bounds provider virtual +0x10. Unknown
    // is distinct from a known null pointer (known=true, no integer_range).
    bool range_pointer_known = false;
    std::optional<std::array<std::int64_t,6>> integer_range;
    std::optional<std::uint8_t> spatial_byte_78;
};
struct NativeCachedModelBoundsResult {
    bool resolved = false;
    std::string reason;
    std::optional<int> return_code;
    // Native code writes the destination only on return code zero.
    std::optional<std::array<double,6>> range;
};
// Numeric bounds extraction of 19c5e0/19c6a0 after obtaining the provider
// result. Does not construct/cache the provider or union child model bounds.
NativeCachedModelBoundsResult project_native_cached_model_bounds(const NativeCachedModelBoundsInput &input);

struct NativeDefaultRangeScaleContext {
    std::optional<bool> record_present;
    std::optional<double> value_28;
    std::optional<std::uint32_t> code_30, code_68;
    std::optional<double> value_38, value_40, value_70, value_78;
};
struct NativeDefaultRangeResult {
    bool resolved = false;
    std::string reason;
    std::array<double,6> range{};
    bool used_fallback = false;
};
// Range after native model bounds acquisition AND child/reference union.
// Missing context is not an empty range. Empty/large/nonfinite/point ranges
// select the original 33aa50 scale fallback; a reversed finite range alone
// does not trigger that fallback in this stage.
NativeDefaultRangeResult select_native_default_view_range(
    const std::array<double,6> &combined_range, const NativeDefaultRangeScaleContext &scale);

enum class NativeDefaultTableSlotAction { Keep, Copy, Create };
struct NativeDefaultTableSlot {
    NativeDefaultTableSlotAction action = NativeDefaultTableSlotAction::Keep;
    // Copy references the original view AND its layout, with flags +0x10
    // cleared by 0x84 and the destination slot number replacing +0xc8.
    std::optional<std::size_t> copy_source_slot;
    std::uint32_t copy_clear_flags_10 = 0;
    std::optional<NativeDefaultViewResult> created_view;
    std::array<std::int32_t,4> viewport{};
    std::array<double,4> normalized_layout{};
    std::array<std::uint16_t,7> layout_words{};
    std::int8_t orientation_preset = 0;
};
struct NativeDefaultViewTableInput {
    std::array<NativeViewSlotPresence,8> slots{};
    // Only the source layout in the no-model Copy branch is dereferenced.
    std::array<NativeViewSlotPresence,8> layouts{};
    std::optional<bool> model_present;
    NativeViewDirectoryContext directory;
    std::optional<bool> model_query_68;
    std::optional<std::int32_t> model_record_value_1c;
    std::optional<std::array<double,6>> combined_model_range;
    NativeDefaultRangeScaleContext fallback_scale;
    std::optional<bool> suppress_selection;
};
struct NativeDefaultViewTableResult {
    bool resolved = false;
    std::string reason;
    std::uint8_t preserved_slot_mask = 0, copied_slot_mask = 0, created_slot_mask = 0;
    std::optional<NativeDefaultRangeResult> range;
    std::array<NativeDefaultTableSlot,8> slots;
};
// R1.18 4acc80 slot actions and newly-created scalar/frame/layout projection.
// Keep/Copy preserve original source identity, not caller-provided substitutes.
// Requires unchanged model/directory context across the native calls.
// Does not clone objects/attributes or execute resource/host/GUI operations.
// Consume slot actions, masks and numeric fields only when resolved is true.
NativeDefaultViewTableResult project_native_default_view_table(const NativeDefaultViewTableInput &input);
} // namespace p3d
