#pragma once
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace p3d {
struct NativeViewModelPointer {
    bool known = false;
    // Caller graph identity, not a model ID. A known empty value means null.
    std::optional<std::size_t> object_index;
};
struct NativeViewModelObjectState {
    std::optional<bool> status_record_present; // model+138 pointer
    std::optional<std::uint8_t> status_record_byte14;
    std::optional<std::uint8_t> model_byte154;
};
struct NativeViewModelAccessContext {
    NativeViewModelPointer cached_model; // view+d0
    std::optional<bool> owning_file_present; // view+e0
    std::optional<std::int32_t> stored_model_id; // view+e8, AFTER construction
    std::map<std::int32_t, std::optional<std::size_t>> resident_models;
    bool resident_models_complete = false;
    // Identity of the owning file's embedded system context, file+6c8.
    // Missing means unknown identity, not a null system context.
    std::optional<std::size_t> system_context_object;
    std::map<std::size_t, NativeViewModelObjectState> object_states;
    std::optional<std::int32_t> file_state_680;
    // Return pointer from COMPLETE 12d0e0(file,null,id,true,true,false),
    // including any provider/model/host operations. Not a provider's raw output.
    NativeViewModelPointer forced_load_result;
};
struct NativeViewModelAccessResult {
    bool resolved = false;
    std::optional<std::size_t> object_index;
    // If true, the original getter stores the returned pointer (possibly null)
    // in view+d0. Unknown until enough evidence establishes a returning path.
    std::optional<bool> cache_write;
    // True once the known prefix reaches the helper. False on an unresolved
    // result does not prove that the unknown continuation avoids loading.
    bool forced_load_required = false;
    std::string decision;
    std::string reason;
};
// R1.18 4b5800. Describes returned identity and cache writes; mutates nothing,
// executes no loading/callbacks, and does not certify model contents as loaded.
NativeViewModelAccessResult resolve_native_view_model(
    const NativeViewModelAccessContext &context, bool force_load = false);
// R1.18 4b5900 checks view+d8 first, then follows 4b5800's force-load path.
// A preferred hit does not overwrite view+d0.
NativeViewModelAccessResult resolve_native_preferred_view_model(
    const NativeViewModelAccessContext &context, const NativeViewModelPointer &preferred);
} // namespace p3d
