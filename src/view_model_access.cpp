#include <p3d/view_model_access.hpp>

namespace p3d {
NativeViewModelAccessResult resolve_native_view_model(const NativeViewModelAccessContext &c, bool force_load) {
    NativeViewModelAccessResult out;
    auto finish = [&](std::optional<std::size_t> object, bool write, const char *decision) {
        out.resolved = true; out.object_index = object; out.cache_write = write; out.decision = decision;
        return out;
    };
    auto unknown = [&](const char *reason) { out.reason = reason; return out; };
    if (!c.cached_model.known) return unknown("view_model_cache_not_established");
    if (c.cached_model.object_index) return finish(c.cached_model.object_index, false, "cached_model");
    if (!c.owning_file_present) return unknown("owning_file_presence_not_established");
    if (!*c.owning_file_present) return finish(std::nullopt, false, "absent_file");
    if (!c.stored_model_id) return unknown("constructor_model_id_not_established");
    const auto id = *c.stored_model_id;
    if (id == -2) return finish(std::nullopt, false, "model_minus2");
    if (force_load) {
        out.forced_load_required = true;
        if (c.file_state_680 && *c.file_state_680 <= 0)
            return finish(std::nullopt, true, "file_state_guard");
        if (!c.forced_load_result.known) return unknown("complete_forced_model_acquisition_result_required");
        return finish(c.forced_load_result.object_index, true, "forced_load_result");
    }
    std::optional<std::size_t> object;
    if (id == -1) {
        if (!c.system_context_object) return unknown("system_context_identity_not_established");
        object = c.system_context_object;
    } else {
        const auto at = c.resident_models.find(id);
        if (at == c.resident_models.end()) {
            if (!c.resident_models_complete) return unknown("resident_model_lookup_not_established");
        } else object = at->second;
        if (!object) return finish(std::nullopt, false, "resident_model_missing");
    }
    const auto at = c.object_states.find(*object);
    if (at == c.object_states.end()) return unknown("resident_model_state_not_established");
    const auto &state = at->second;
    if (!state.status_record_present) return unknown("model_status_record_presence_not_established");
    if (!*state.status_record_present) return unknown("native_access_would_dereference_null_status_record");
    if (!state.status_record_byte14 || !state.model_byte154)
        return unknown("resident_model_state_bytes_not_established");
    // Both bytes are read before the combined value is tested against 6.
    if (!*state.status_record_byte14 || !*state.model_byte154)
        return finish(std::nullopt, false, "resident_model_state_not_ready");
    return finish(object, true, id == -1 ? "system_context" : "resident_registry");
}

NativeViewModelAccessResult resolve_native_preferred_view_model(
        const NativeViewModelAccessContext &context, const NativeViewModelPointer &preferred) {
    NativeViewModelAccessResult out;
    if (!preferred.known) { out.reason = "preferred_view_model_cache_not_established"; return out; }
    if (preferred.object_index) {
        out.resolved = true; out.object_index = preferred.object_index;
        out.cache_write = false; out.decision = "preferred_cached_model";
        return out;
    }
    return resolve_native_view_model(context, true);
}
} // namespace p3d
