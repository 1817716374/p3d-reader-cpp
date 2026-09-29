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
} // namespace p3d
