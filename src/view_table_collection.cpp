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
} // namespace p3d
