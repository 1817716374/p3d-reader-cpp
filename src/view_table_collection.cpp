#include "internal.hpp"

namespace p3d {
Json project_view_table_text_fields(const Json &record) {
    Json out = {{"status", "unresolved"}, {"fields", Json::array()}};
    try {
        const auto &links = record.at("links");
        require(links.is_array(), "complete_view_table_linkages_required");
        for (unsigned key : {1u, 2u}) {
            Json selected = {{"entry_index", nullptr}};
            for (std::size_t i = 0; i < links.size(); ++i) {
                const auto &link = links.at(i);
                if (link.at("app") != 0x56d2 || !(link.at("header").get<unsigned>() & 0x1000)) continue;
                const auto data = bytesof(link.at("payload"));
                require(data.size() >= 2, "truncated_view_table_string_key");
                if (Reader(data).u16() == key) {
                    selected["entry_index"] = i;
                    break; // Native occurrence zero; failed decoding does not try the next duplicate.
                }
            }
            Json field = {{"key", key}, {"source_linkage_index", selected.at("entry_index")},
                          {"status", "unresolved"}};
            auto reader = native_512_string_reader(selected, links, "R1.18_2c1f40_512_unit_buffer");
            const auto status = reader.at("status").get<std::string>();
            if (status == "missing" || status == "failure") {
                field.update({{"status", "resolved"}, {"action", "keep_constructor_empty_string"},
                              {"constructor_utf16_units", Json::array()}, {"constructor_text", ""}});
            } else if (status == "success") {
                Json units = Json::array();
                for (const auto &unit : reader.at("output_utf16_units")) {
                    if (unit == 0) break;
                    units.push_back(unit);
                }
                field.update({{"status", "resolved"}, {"action", "assign_reader_output"},
                              {"constructor_utf16_units", std::move(units)},
                              {"constructor_text", reader.at("value")}});
            }
            field["reader"] = std::move(reader);
            out["fields"].push_back(std::move(field));
        }
        out["status"] = "resolved";
        for (const auto &field : out.at("fields"))
            if (field.at("status") != "resolved") out["status"] = "unresolved";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json project_view_table_collection(const Json &tables) {
    Json out = {{"status", "unresolved"}, {"scope", "R1.18_initial_common_then_lite_view_collection"},
                {"active_table_selection", "not_evaluated"}};
    try {
        // Validate all candidates first. An unknown earlier name may change
        // every later replacement position, so do not publish a guessed list.
        for (const auto &table : tables) {
            if (table.at("status") == "not_a_view_table_input") continue;
            require(table.at("status") == "conditional", "complete_view_table_inputs_required");
            require(table.at("table_kind") == "ordinary" || table.at("table_kind") == "lite", "known_view_table_kind_required");
            require(table.at("text_fields").at("status") == "resolved", "complete_view_table_text_fields_required");
            require(table.at("text_fields").at("fields").size() == 2, "two_view_table_text_fields_required");
            for (unsigned key = 1; key <= 2; ++key) {
                const auto &field = table.at("text_fields").at("fields").at(key-1);
                require(field.at("key") == key && field.at("status") == "resolved", "resolved_ordered_view_table_text_pair_required");
            }
        }
        auto equal = [&](std::size_t a, std::size_t b) {
            const auto &x = tables.at(a).at("text_fields").at("fields");
            const auto &y = tables.at(b).at("text_fields").at("fields");
            return x[0].at("constructor_utf16_units") == y[0].at("constructor_utf16_units") &&
                   x[1].at("constructor_utf16_units") == y[1].at("constructor_utf16_units");
        };
        std::vector<std::size_t> entries;
        Json operations = Json::array();
        for (const auto *kind : {"ordinary", "lite"}) {
            for (std::size_t i = 0; i < tables.size(); ++i) {
                const auto &table = tables.at(i);
                if (table.at("status") == "not_a_view_table_input" || table.at("table_kind") != kind) continue;
                std::size_t at = entries.size();
                if (std::string(kind) == "lite")
                    for (std::size_t j = 0; j < entries.size(); ++j)
                        if (equal(i, entries[j])) { at = j; break; }
                Json operation = {{"table_input_index", i}, {"phase", kind}, {"collection_index", at}};
                if (at == entries.size()) {
                    operation["action"] = "append";
                    entries.push_back(i);
                } else {
                    operation["action"] = "replace_first_exact_text_pair";
                    operation["replaced_table_input_index"] = entries[at];
                    entries[at] = i;
                }
                operations.push_back(std::move(operation));
            }
        }
        Json result = Json::array();
        for (std::size_t i = 0; i < entries.size(); ++i) {
            const auto &table = tables.at(entries[i]);
            result.push_back({{"collection_index", i}, {"table_input_index", entries[i]},
                              {"source", table.at("source")}, {"table_kind", table.at("table_kind")}});
        }
        out.update({{"status", "conditional"}, {"entries", std::move(result)}, {"operations", std::move(operations)},
                    {"conditions", Json::array({"fresh_collection_uses_R1.18_4b2f70_constructor",
                        "prepared_file_system_root_order_preserved", "table_construction_and_allocation_succeed",
                        "no_host_or_plugin_mutation_during_collection_construction"})}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json project_file_view_table_selection(const Json &list, const Json &records,
                                       const Json &ids, const Json &collection) {
    Json out = {{"status", "unresolved"}, {"scope", "R1.18_initial_file_table_query"},
                {"runtime_application", "not_evaluated"},
                {"model_selection", "not_evaluated"}, {"fallback", "not_evaluated"}};
    try {
        require(list.at("status") == "resolved" &&
                    list.at("scope") == "empty_list_before_runtime_registration" &&
                    list.at("system_bootstrap_required") == true &&
                    ids.at("status") == "resolved" &&
                    ids.at("scope") == "first_system_input_with_empty_id_registry",
                "complete_fresh_system_input_and_id_assignments_required");
        require(!list.at("roots").empty(), "file_table_query_requires_first_system_root");
        const auto &root = list.at("roots").front();
        const auto ni = root.at("native_record_index").get<std::size_t>();
        const auto &record = records.at(ni);
        const auto data = bytesof(record.at("data"));
        require(record.at("element_type") == 46 && data.size() >= 20 &&
                    Reader(data, 16).u32() == 8,
                "file_table_query_requires_first_type46_subtype8_root");
        require(!ids.at("roots").empty() && ids.at("roots").front().at("native_record_index") == ni &&
                    ids.at("roots").front().at("block_number") == root.at("block_number"),
                "file_table_query_root_assignment_mismatch");
        const auto &source = ids.at("roots").front().at("records").at(0);
        require(source.at("native_record_index") == ni && source.at("parent_record_index").is_null(),
                "file_table_query_root_identity_mismatch");
        out["query_source"] = {{"native_record_index", ni},
            {"input_occurrence_index", source.at("input_occurrence_index")},
            {"source_id", source.at("source_id")}, {"assigned_id", source.at("assigned_id")}};
        // 12b570 accepts the first resident root only. 4b2350 then reads
        // body+248 (stream data includes a four-byte prefix).
        out["requested_id_data_offset"] = 0x24c;
        require(data.size() >= 0x254, "truncated_file_view_table_query_id");
        const auto requested = Reader(data, 0x24c).u64();
        out["requested_id"] = requested;
        require(collection.at("status") == "conditional" &&
                    collection.at("scope") == "R1.18_initial_common_then_lite_view_collection",
                "complete_initial_view_collection_required");
        for (const auto &entry : collection.at("entries")) {
            // 4af930 compares the post-registration ID in the table's record,
            // not its source ID and not its name. It returns the first match.
            if (entry.at("source").at("assigned_id").get<std::uint64_t>() != requested) continue;
            out.update({{"status", "conditional"}, {"lookup", "first_assigned_id_match"},
                {"selected_entry", entry}, {"fallback", "not_needed"},
                {"conditions", Json::array({"fresh_resident_collection_and_system_roots",
                    "R1.18_4b2350_file_table_query_path",
                    "no_intervening_record_id_or_collection_mutation"})}});
            return out;
        }
        out["lookup"] = "no_assigned_id_match";
        out["fallback"] = "required";
        out["reason"] = "default_table_and_model_fallback_not_established";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
