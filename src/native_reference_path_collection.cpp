#include "internal.hpp"

namespace p3d {
namespace {
struct PathFailure {
    std::string reason;
};
} // namespace

Json native_reference_path_collection(const Json &owner_ids, const Json &system_ids,
                                      const Json &records, std::uint64_t id) {
    Json out = {{"profile", "bimbase_2025_initial_owner_path_collection"},
                {"scope", "fresh_input_graph_before_runtime_callbacks"},
                {"status", "not_evaluated"},
                {"requested_id", id},
                {"lookups", Json::array()},
                {"collected", Json::array()},
                {"terminal_index", -1},
                {"owner_scope", nullptr},
                {"runtime_callbacks", "not_evaluated"},
                {"geometry_transformation", "not_evaluated"}};
    std::array<std::map<std::size_t, Json>, 2> nodes;
    std::array<bool, 2> built{};
    std::array<std::map<std::uint64_t, const Json *>, 2> registries;
    std::array<bool, 2> registry_built{};
    bool owner_set = false;
    std::size_t operations = 0;
    std::set<std::pair<unsigned, std::size_t>> active;
    auto spend = [&] { require(++operations <= 100000, "path_collection_operation_limit"); };
    auto node = [&](unsigned scope, std::size_t occurrence) -> const Json & {
        if (!built[scope]) {
            const auto &ids = scope ? system_ids : owner_ids;
            require(ids.at("status") == "resolved", "complete_initial_registry_required");
            for (const auto &root : ids.at("roots"))
                for (const auto &item : root.at("records")) {
                    auto value = item;
                    value["lookup_scope"] = scope ? "system" : "owner";
                    value["block_number"] = root.at("block_number");
                    require(nodes[scope]
                                .emplace(item.at("input_occurrence_index").get<std::size_t>(),
                                         std::move(value))
                                .second,
                            "ambiguous_initial_input_occurrence");
                }
            built[scope] = true;
        }
        return nodes[scope].at(occurrence);
    };
    auto flags = [&](const Json &value) {
        const auto &f = value.at("prepared_element_flags");
        require(f.is_number_integer() && f >= 0 && f <= UINT16_MAX,
                "prepared_element_flags_required");
        return f.get<unsigned>();
    };
    auto append = [&](const Json &value) {
        spend();
        out["collected"].push_back(value);
        out["terminal_index"] = out["collected"].size() - 1;
    };
    auto append_target = [&](unsigned scope, std::size_t occurrence) {
        const auto &target = node(scope, occurrence);
        if (flags(target) & 0x80) {
            const Json *ancestor = &target;
            std::set<std::size_t> parents;
            while (flags(*ancestor) & 0x80) {
                spend();
                const auto &parent = ancestor->at("parent_input_occurrence_index");
                if (parent.is_null()) {
                    ancestor = nullptr;
                    break;
                }
                const auto parent_index = parent.get<std::size_t>();
                require(parents.insert(parent_index).second, "cyclic_initial_parent_chain");
                ancestor = &node(scope, parent_index);
            }
            if (ancestor)
                append(*ancestor);
        }
        append(target);
    };
    auto lookup = [&](std::uint64_t target_id, bool current_owner) {
        spend();
        if (!current_owner)
            throw PathFailure{"path_lookup_owner_missing"};
        const Json *selected = nullptr;
        unsigned scope = 0;
        for (; scope < 2; ++scope) {
            const auto &ids = scope ? system_ids : owner_ids;
            require(ids.is_object() && ids.value("status", Json()) == "resolved",
                    scope ? "complete_system_registry_required"
                          : "complete_owner_registry_required");
            if (!registry_built[scope]) {
                for (const auto &entry : ids.at("registry")) {
                    const auto inserted =
                        registries[scope].emplace(entry.at("id").get<std::uint64_t>(), &entry);
                    if (!inserted.second)
                        inserted.first->second = nullptr;
                }
                registry_built[scope] = true;
            }
            const auto found = registries[scope].find(target_id);
            out["lookups"].push_back(
                {{"id", target_id},
                 {"scope", scope ? "system" : "owner"},
                 {"status", found == registries[scope].end() ? "missing" : "selected"}});
            if (found == registries[scope].end())
                continue;
            require(found->second != nullptr, "ambiguous_registered_id");
            selected = found->second;
            break;
        }
        if (!selected)
            throw PathFailure{"path_object_not_found"};
        const auto occurrence = selected->at("input_occurrence_index").get<std::size_t>();
        const auto &object = node(scope, occurrence);
        require(object.at("assigned_id") == target_id &&
                    object.at("native_record_index") == selected->at("native_record_index"),
                "path_registry_identity_mismatch");
        return std::make_pair(scope, occurrence);
    };
    std::function<void(unsigned, std::size_t, bool, unsigned)> expand;
    expand = [&](unsigned scope, std::size_t occurrence, bool current_owner, unsigned depth) {
        spend();
        require(depth <= 256, "owner_path_recursion_limit");
        const auto &object = node(scope, occurrence);
        const auto &record = records.at(object.at("native_record_index").get<std::size_t>());
        const auto &source_type = record.at("element_type");
        require(source_type.is_number_integer() && source_type >= 0 && source_type <= UINT16_MAX,
                "native_collected_record_type_required");
        const auto type = source_type.get<unsigned>();
        if (type == 13) {
            out["unresolved_reference"] = object;
            throw std::runtime_error("referenced_model_attachment_context_required");
        }
        if (type != 47) {
            append_target(scope, occurrence);
            if (current_owner)
                owner_set = true;
            return;
        }
        if (!record.contains("owner_reference_path"))
            throw PathFailure{"application_record_is_not_owner_reference_path"};
        const auto key = std::make_pair(scope, occurrence);
        require(active.insert(key).second, "cyclic_owner_reference_path");
        const auto &program = record.at("owner_reference_path");
        require(program.at("status") == "resolved", "invalid_owner_reference_path_program");
        for (const auto &step : program.at("steps")) {
            const auto selected = lookup(step.at("element_id").get<std::uint64_t>(), current_owner);
            const auto operation = step.at("operation").get<std::string>();
            if (operation == "resolve_and_append_target") {
                // Format 6's terminal target is appended without type dispatch
                // and without changing the collector's owner.
                append_target(selected.first, selected.second);
            } else {
                require(operation == "resolve_and_expand_reference",
                        "unknown_owner_path_operation");
                expand(selected.first, selected.second, current_owner, depth + 1);
                current_owner = owner_set;
            }
        }
        if (program.at("steps").empty()) {
            require(program.at("reference_format") == 0, "invalid_empty_owner_path_program");
            if (!owner_set)
                owner_set = current_owner;
        }
        active.erase(key);
    };
    try {
        const auto selected = lookup(id, true);
        expand(selected.first, selected.second, true, 0);
        out["status"] = "resolved";
        if (!out["collected"].empty())
            out["target"] = out["collected"].back();
    } catch (const PathFailure &e) {
        out.update({{"status", "native_failure"}, {"native_status", 1}, {"reason", e.reason}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    out["owner_scope"] = owner_set ? Json("owner") : Json();
    out["collection_complete"] = out.at("status") == "resolved";
    out["operation_count"] = operations;
    return out;
}

Json Document::native_model_reference_path(const StreamPath &model_storage,
                                           std::uint64_t initial_id_counter,
                                           std::uint64_t id) const {
    const auto containers = p3d::native_input_containers(streams(), index(), native_records());
    const auto owner = p3d::native_model_id_assignments(containers, native_records(), index(),
                                                        model_storage, initial_id_counter);
    bool known_model = false;
    for (const auto &input : owner.at("inputs"))
        known_model = known_model || input.at("status") != "not_loaded";
    Json system;
    const Json *selected = nullptr;
    bool ambiguous = false;
    for (const auto &container : containers)
        if (container.at("kind") == "P3D-SSYS") {
            ambiguous = ambiguous || selected != nullptr;
            selected = &container;
        }
    if (known_model && selected && !ambiguous)
        system = native_system_id_assignments(selected->at("list_preparation"), native_records(),
                                              file_header());
    auto result = native_reference_path_collection(owner, system, native_records(), id);
    result["model_storage"] = model_storage;
    result["initial_id_counter"] = initial_id_counter;
    if (!known_model) {
        result["status"] = "not_evaluated";
        result["reason"] = "model_record_container_unavailable";
        result["collection_complete"] = false;
        result.erase("target");
    }
    return result;
}
} // namespace p3d
