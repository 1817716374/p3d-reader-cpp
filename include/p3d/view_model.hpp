#pragma once
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
} // namespace p3d
