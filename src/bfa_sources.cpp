#include "blob_internal.hpp"
namespace p3d {
namespace {
using DataKey = std::pair<std::uint64_t, std::uint64_t>;
DataKey data_key(const Json &item) {
    return {item.at("class_id").get<std::uint64_t>(), item.at("object_id").get<std::uint64_t>()};
}
bool root_field(const Json &field) {
    const auto prefix = "/" + field.at("class_name").get<std::string>() + "/";
    const auto &path = field.at("path").get_ref<const std::string &>();
    return path.compare(0, prefix.size(), prefix) == 0 &&
           path.find('/', prefix.size()) == std::string::npos;
}
} // namespace
void bind_bfa_definition_sources(Json &binary_fields, const Json &objects) {
    std::set<DataKey> requested;
    for (auto &field : binary_fields) {
        if (field.at("class_name") != "BPParaHandleAndBfaDataMap" ||
            field.at("name") != "DataIdMap" || !root_field(field))
            continue;
        auto &value = field.at("value");
        if (!value.contains("decoded") || !value["decoded"].contains("native_lookup"))
            continue;
        auto &decoded = value["decoded"];
        decoded["definition_source_bindings"] = Json::array();
        for (const auto &index : decoded.at("native_lookup").at("selected_entry_indices"))
            requested.insert(data_key(decoded.at("entries").at(index.get<std::size_t>())));
    }
    if (requested.empty())
        return;
    std::map<DataKey, std::vector<std::size_t>> object_index, tree_fields;
    for (std::size_t i = 0; i < objects.size(); ++i) {
        const auto key = data_key(objects[i]);
        if (requested.count(key))
            object_index[key].push_back(i);
    }
    for (std::size_t i = 0; i < binary_fields.size(); ++i)
        if (binary_fields[i].at("name") == "BfaTree" && root_field(binary_fields[i]))
            tree_fields[data_key(binary_fields[i])].push_back(i);
    // One source tree may serve several native handles. Keep field/record
    // indices instead of copying or expanding that tree for each association.
    std::map<std::size_t, std::map<std::uint64_t, std::vector<std::size_t>>> node_indices;
    for (std::size_t i = 0; i < binary_fields.size(); ++i) {
        auto &field = binary_fields[i];
        if (field.at("class_name") != "BPParaHandleAndBfaDataMap" ||
            field.at("name") != "DataIdMap" || !root_field(field))
            continue;
        auto &value = field.at("value");
        if (!value.contains("decoded") || !value["decoded"].contains("native_lookup"))
            continue;
        auto &decoded = value["decoded"];
        Json bindings = Json::array();
        for (const auto &selected : decoded.at("native_lookup").at("selected_entry_indices")) {
            const auto entry_index = selected.get<std::size_t>();
            const auto &entry = decoded.at("entries").at(entry_index);
            const auto id = entry.at("parameter_handle").get<std::uint64_t>();
            const auto key = data_key(entry);
            Json binding = {{"entry_index", entry_index},
                            {"parameter_handle", id},
                            {"scope", "this_document"},
                            {"project_selection", "not_performed"}};
            const auto objects_found = object_index.find(key);
            if (objects_found == object_index.end())
                binding["target_status"] = "object_not_in_document";
            else if (objects_found->second.size() != 1) {
                binding["target_status"] = "ambiguous_object_identity";
                binding["object_candidates"] = objects_found->second;
            } else {
                binding["target_object_index"] = objects_found->second.front();
                const auto &object = objects[objects_found->second.front()];
                std::size_t named_properties = 0;
                if (object.contains("root") && object["root"].contains("children"))
                    for (const auto &property : object["root"]["children"])
                        if (property.value("name", std::string()) == "BfaTree")
                            ++named_properties;
                if (named_properties > 1) {
                    binding["target_status"] = "ambiguous_named_property";
                    bindings.push_back(std::move(binding));
                    continue;
                }
                const auto fields_found = tree_fields.find(key);
                if (fields_found == tree_fields.end())
                    binding["target_status"] = "bfa_binary_field_not_found";
                else if (fields_found->second.size() != 1) {
                    binding["target_status"] = "ambiguous_bfa_field";
                    binding["field_candidates"] = fields_found->second;
                } else {
                    const auto fi = fields_found->second.front();
                    binding["target_field_index"] = fi;
                    const auto &tree_value = binary_fields[fi].at("value");
                    if (!tree_value.contains("decoded") ||
                        !tree_value.at("decoded").contains("records"))
                        binding["target_status"] = "bfa_payload_unavailable";
                    else {
                        binding["target_status"] = "matched_bfa_field";
                        auto cached = node_indices.find(fi);
                        if (cached == node_indices.end()) {
                            auto &index = node_indices[fi];
                            const auto &records = tree_value.at("decoded").at("records");
                            for (std::size_t n = 0; n < records.size(); ++n)
                                index[records[n].at("id").get<std::uint64_t>()].push_back(n);
                            cached = node_indices.find(fi);
                        }
                        const auto found = cached->second.find(id);
                        if (found == cached->second.end()) {
                            binding["handle_node_status"] = "not_in_payload";
                            binding["handle_record_indices"] = Json::array();
                        } else {
                            binding["handle_node_status"] = found->second.size() == 1
                                                                ? "unique_node_in_payload"
                                                                : "ambiguous_node_id";
                            binding["handle_record_indices"] = found->second;
                        }
                    }
                }
            }
            bindings.push_back(std::move(binding));
        }
        decoded["definition_source_bindings"] = std::move(bindings);
    }
}
} // namespace p3d
