#include "internal.hpp"
#include <p3d/view_model.hpp>
#include "view_model_oracle.hpp"
using namespace p3d;
unsigned view_model_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) { ++checks; require(value, message); };
    const auto oracle = Json::parse(view_model_oracle);
    for (const auto &row : oracle.at("cases")) {
        NativeViewModelContext context;
        context.resident_registry_complete = true;
        for (const auto &entry : row.at("registry_entries"))
            if (entry[1] == true) context.resident_model_ids.push_back(entry[0].get<std::int32_t>());
        context.file_default_model_id = row.at("file_default_id").get<std::int32_t>();
        const auto host = row.at("host_result").get<std::string>();
        context.host_result = host == "absent" ? NativeViewModelHostResult::Absent :
                              host == "reject" ? NativeViewModelHostResult::Reject : NativeViewModelHostResult::Accept;
        const auto result = project_native_view_model_id(row.at("source_id").get<std::int32_t>(), context);
        check(result.model_id && *result.model_id == row.at("constructor_model_id") && result.reason.empty(),
              "stored model ID matches the complete original constructor across signed tree lookup and host outcomes");
    }
    NativeViewModelContext c;
    check(project_native_view_model_id(-1, c).model_id == -1,
          "system context ID needs no registry, host or default-model evidence");
    c.resident_model_ids = {7};
    check(project_native_view_model_id(7, c).source == NativeViewModelIdSource::ResidentRegistry,
          "known non-null resident entry proves a hit even if other registry entries are unknown");
    c.host_result = NativeViewModelHostResult::Accept;
    check(!project_native_view_model_id(8, c).model_id,
          "an incomplete registry must not imply that a host query was reached");
    c.resident_registry_complete = true;
    check(project_native_view_model_id(8, c).source == NativeViewModelIdSource::HostAccepted,
          "known host acceptance preserves a nonresident source ID without obtaining its model object");
    c.host_result = NativeViewModelHostResult::Unknown; c.file_default_model_id = 99;
    check(!project_native_view_model_id(8, c).model_id,
          "unknown host acceptance cannot silently become a file-default fallback");
    c.host_result = NativeViewModelHostResult::Absent; c.file_default_model_id.reset();
    check(!project_native_view_model_id(8, c).model_id,
          "absent host still requires a known file-default field after lookup miss");
    c.file_default_model_id = 99;
    check(project_native_view_model_id(8, c).model_id == 99,
          "native fallback does not validate whether the file-default ID exists in the registry");
    c.file_default_model_id = -1;
    check(project_native_view_model_id(-2, c).source == NativeViewModelIdSource::FileDefault,
          "a fallback value of minus one retains its file-default provenance");
    return checks;
}
