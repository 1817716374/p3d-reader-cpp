#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
namespace p3d {
enum class NativeViewModelHostResult { Unknown, Absent, Reject, Accept };
struct NativeViewModelContext {
    // IDs whose resident registry entries contain non-null model pointers.
    std::vector<std::int32_t> resident_model_ids;
    // Complete means unlisted IDs have no non-null resident entry.
    bool resident_registry_complete = false;
    // Result for this source_id only, reached after a resident lookup miss.
    NativeViewModelHostResult host_result = NativeViewModelHostResult::Unknown;
    std::optional<std::int32_t> file_default_model_id;
};
enum class NativeViewModelIdSource { Unresolved, SystemContext, ResidentRegistry, HostAccepted, FileDefault };
struct NativeViewModelIdResult {
    std::optional<std::int32_t> model_id;
    NativeViewModelIdSource source = NativeViewModelIdSource::Unresolved;
    std::string reason;
};
// R1.18 type11 constructor 4b46b0, with a known owning resident file.
// Chooses the stored ID only: does not obtain a model object, load models,
// call the host, validate the fallback ID, or apply view/display resources.
NativeViewModelIdResult project_native_view_model_id(
    std::int32_t source_id, const NativeViewModelContext &context);

enum class NativeViewSlotPresence { Unknown, Absent, Present };
struct NativeViewModelSlot {
    NativeViewSlotPresence presence = NativeViewSlotPresence::Unknown;
    // Current constructor/runtime values, not unconditionally the source bytes.
    std::optional<std::int32_t> model_id;
    std::optional<std::uint8_t> flags_low;
};
using NativeViewModelSlots = std::array<NativeViewModelSlot, 8>;
struct NativeViewModelMatch {
    std::optional<bool> matches;
    std::string reason;
};
// R1.18 4adc70. all_same takes priority over slot. Out-of-range slots
// (including negative values) scan for the first view with flag 0x80.
NativeViewModelMatch match_native_view_table_model(const NativeViewModelSlots &views,
    std::int32_t requested_id, bool all_same, std::int32_t slot);
struct NativeViewTableModelQuery {
    std::optional<std::int32_t> model_id;
    std::optional<std::size_t> slot_index;
    // Bits mark slots that the original query writes to table+d0. A partial
    // scan reports only the proven prefix; this function mutates no input.
    std::uint8_t marked_slot_mask = 0;
    bool scan_complete = false;
    std::string reason;
};
// Slot phase of 12ef50 AFTER a table was obtained. No flagged view uses slot 0.
// Unlike the model-match query, this describes dirty-slot writes as well.
NativeViewTableModelQuery query_native_view_table_model(const NativeViewModelSlots &views);
} // namespace p3d
