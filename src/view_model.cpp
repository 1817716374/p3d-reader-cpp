#include <p3d/view_model.hpp>
#include <algorithm>
namespace p3d {
NativeViewModelIdResult project_native_view_model_id(std::int32_t source_id, const NativeViewModelContext &context) {
    NativeViewModelIdResult out;
    if (source_id == -1) out.source = NativeViewModelIdSource::SystemContext;
    else if (std::find(context.resident_model_ids.begin(), context.resident_model_ids.end(), source_id) !=
             context.resident_model_ids.end()) out.source = NativeViewModelIdSource::ResidentRegistry;
    else if (!context.resident_registry_complete) out.reason = "resident_model_lookup_not_established";
    else if (context.host_result == NativeViewModelHostResult::Unknown) out.reason = "host_model_acceptance_not_established";
    else if (context.host_result == NativeViewModelHostResult::Accept) out.source = NativeViewModelIdSource::HostAccepted;
    else if (!context.file_default_model_id) out.reason = "file_default_model_id_not_established";
    else {
        out.source = NativeViewModelIdSource::FileDefault;
        out.model_id = context.file_default_model_id;
        return out;
    }
    if (out.source != NativeViewModelIdSource::Unresolved) out.model_id = source_id;
    return out;
}
NativeViewModelMatch match_native_view_table_model(const NativeViewModelSlots &views,
        std::int32_t requested_id, bool all_same, std::int32_t slot) {
    NativeViewModelMatch out;
    auto present = [&](const NativeViewModelSlot &view) {
        if (view.presence == NativeViewSlotPresence::Present) return true;
        out.reason = view.presence == NativeViewSlotPresence::Absent ?
            "native_query_would_dereference_null_view" : "view_presence_not_established";
        return false;
    };
    auto compare = [&](const NativeViewModelSlot &view) {
        if (!view.model_id) out.reason = "constructor_model_id_not_established";
        else out.matches = *view.model_id == requested_id;
    };
    if (all_same) {
        std::int32_t common = -2;
        for (const auto &view : views) {
            if (view.presence == NativeViewSlotPresence::Absent) continue;
            if (!present(view)) return out;
            if (!view.model_id) { out.reason = "constructor_model_id_not_established"; return out; }
            // -2 is an accumulator sentinel on EVERY iteration, not just the
            // first. Thus [-2,7] and [7,-2] intentionally differ.
            if (common == -2) common = *view.model_id;
            else if (common != *view.model_id) { out.matches = false; return out; }
        }
        out.matches = common == requested_id;
    } else if (static_cast<std::uint32_t>(slot) <= 7) {
        const auto &view = views[static_cast<std::size_t>(slot)];
        if (present(view)) compare(view);
    } else {
        for (const auto &view : views) {
            if (!present(view)) return out;
            if (!view.flags_low) { out.reason = "view_flags_not_established"; return out; }
            if (*view.flags_low & 0x80u) { compare(view); return out; }
        }
        out.matches = false;
    }
    return out;
}

NativeViewTableModelQuery query_native_view_table_model(const NativeViewModelSlots &views) {
    NativeViewTableModelQuery out;
    std::size_t selected = 0;
    for (std::size_t i = 0; i < views.size(); ++i) {
        out.marked_slot_mask |= static_cast<std::uint8_t>(1u << i);
        const auto &view = views[i];
        if (view.presence != NativeViewSlotPresence::Present) {
            out.reason = view.presence == NativeViewSlotPresence::Absent ?
                "native_query_would_dereference_null_view" : "view_presence_not_established";
            return out;
        }
        if (!view.flags_low) { out.reason = "view_flags_not_established"; return out; }
        if (*view.flags_low & 0x80u) { selected = i; break; }
    }
    out.scan_complete = true;
    out.slot_index = selected;
    out.model_id = views[selected].model_id;
    if (!out.model_id) out.reason = "constructor_model_id_not_established";
    return out;
}
} // namespace p3d
