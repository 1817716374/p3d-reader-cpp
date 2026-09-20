#include "internal.hpp"
#include <p3d/view_sequence.hpp>
#include <set>

namespace p3d {
Json initial_model_link_registry(const Json &input, const Json &records) {
    Json out = {{"status", "unresolved"},
                {"scope", "fresh_control_link_registry_before_runtime_callbacks"},
                {"runtime_link_loading", "not_evaluated"},
                {"runtime_record_flags", 0}};
    try {
        auto integer = [](const Json &v) {
            require(v.is_number_unsigned() || (v.is_number_integer() && v.get<std::int64_t>() >= 0),
                    "model_link_registry_unsigned_integer_required");
            return v.get<std::uint64_t>();
        };
        require(input.at("scope") == "fresh_model_control_then_graphics_id_registration",
                "fresh_model_id_assignments_required");
        require(input.at("status") == "resolved" || input.at("status") == "partial",
                "model_id_assignment_results_required");
        const auto &inputs = input.at("inputs");
        require(inputs.is_array() && !inputs.empty() && inputs[0].at("kind") == "P3D-SMC",
                "initial_control_input_required");
        const auto &control = inputs[0];
        require(control.at("status") == "resolved" || control.at("status") == "not_loaded",
                "complete_control_input_required");
        const auto expected =
            control.at("status") == "not_loaded" ? 0 : integer(control.at("root_count"));
        const auto &roots = input.at("roots");
        require(roots.is_array() && records.is_array(), "model_link_registry_arrays_required");
        std::map<std::uint64_t, Json> registry;
        std::set<std::uint64_t> occurrences;
        Json replaced = Json::array(), saved = nullptr;
        std::size_t count = 0;
        for (std::size_t ri = 0; ri < roots.size(); ++ri) {
            const auto &root = roots[ri];
            if (root.at("kind") != "P3D-SMC")
                continue;
            require(integer(root.at("input_list_index")) == 0 &&
                        root.at("container") == control.at("container"),
                    "control_root_source_mismatch");
            ++count;
            const auto ni = integer(root.at("native_record_index"));
            require(ni < records.size(), "control_root_record_index_out_of_range");
            const auto &members = root.at("records");
            require(members.is_array() && !members.empty(), "assigned_control_root_required");
            const auto &member = members[0];
            require(integer(member.at("native_record_index")) == ni &&
                        member.at("parent_input_occurrence_index").is_null(),
                    "control_root_identity_mismatch");
            const auto occurrence = integer(member.at("input_occurrence_index"));
            require(occurrences.insert(occurrence).second, "duplicate_control_root_identity");
            const auto id = integer(member.at("assigned_id"));
            const auto &record = records[ni];
            require(member.at("source_id") == record.at("id"), "control_source_id_mismatch");
            Json source = {{"root_index", ri},
                           {"native_record_index", ni},
                           {"input_occurrence_index", occurrence},
                           {"block_number", root.at("block_number")},
                           {"container", root.at("container")},
                           {"source_id", record.at("id")},
                           {"assigned_id", id}};
            const auto type = integer(record.at("element_type"));
            if (type == 13) {
                auto at = registry.find(id);
                if (at != registry.end())
                    replaced.push_back({{"id", id},
                                        {"previous_source", at->second.at("source")},
                                        {"selected_source", source}});
                // Empty initial map: assignment overwrites both the status and
                // source pointer on a repeated key, including key zero.
                registry[id] = {{"id", id}, {"initial_loading_state", 0}, {"source", source}};
            } else if (type == 47) {
                const auto bytes = bytesof(record.at("data"));
                require(bytes.size() >= 20, "control_application_subtype_unavailable");
                if (Reader(bytes, 16).u32() == 33)
                    saved = std::move(source); // Last record, even if its sequence is malformed.
            }
        }
        require(count == expected, "incomplete_assigned_control_roots");
        Json entries = Json::array();
        for (auto &[id, entry] : registry)
            entries.push_back(std::move(entry));
        out.update({{"status", "resolved"},
                    {"model_storage", input.at("model_storage")},
                    {"initial_id_counter", input.at("initial_id_counter")},
                    {"control_root_count", count},
                    {"registry_order", "unsigned_link_id_ascending"},
                    {"registry", std::move(entries)},
                    {"replaced_sources", std::move(replaced)},
                    {"saved_sequence_source", std::move(saved)}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json initial_model_link_registry(const Document &document, const StreamPath &model_storage,
                                 std::uint64_t initial_id_counter) {
    return initial_model_link_registry(
        document.native_model_id_assignments(model_storage, initial_id_counter),
        document.native_records());
}
} // namespace p3d
