#include "internal.hpp"
#include <p3d/reference_search.hpp>

namespace p3d {
ReferenceSearchQuery initial_reference_search_query(const Json &record,
                                                    const NativeFileReference &file,
                                                    NativeModelNameEqual equal) {
    require(record.at("element_type") == 13, "type_13_reference_record_required");
    require(record.at("reference_input").at("status") == "decoded",
            "decoded_reference_input_required");
    const auto &target = record.at("reference_target");
    const auto &state = target.at("initial_runtime_state");
    require(state.at("status") == "decoded", "initial_reference_runtime_state_unresolved");
    ReferenceSearchQuery query;
    query.stored_file_reference = file.stored_reference;
    query.lookup_file_reference = file.lookup_reference;
    query.runtime_flags = state.at("runtime_flags").get<std::uint32_t>();
    query.model_id = state.at("model_id").get<std::uint32_t>();
    query.equal = std::move(equal);
    for (const auto &slot : target.at("strings")) {
        if (slot.at("key") != 21)
            continue;
        if (slot.at("status") == "decoded" || slot.at("status") == "absent") {
            std::u16string name;
            for (const auto &unit : slot.at("utf16_code_units")) {
                const auto value = unit.get<std::uint32_t>();
                require(value <= 0xffff, "invalid_reference_model_name_code_unit");
                if (!value)
                    break;
                name.push_back(static_cast<char16_t>(value));
            }
            query.model_name = std::move(name);
        }
        break;
    }
    return query;
}
} // namespace p3d
