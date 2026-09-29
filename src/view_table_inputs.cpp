#include "display_style_xml.hpp"

namespace p3d {
Json initial_native_view_table_inputs(const Json &list, const Json &records, const Json &ids) {
    Json out = {{"scope", "fresh_system_view_table_inputs"}, {"status", "unresolved"},
                {"collection_selection", "not_evaluated"}, {"runtime_application", "not_evaluated"},
                {"tables", Json::array()}};
    try {
        require(list.at("status") == "resolved" &&
                    list.at("scope") == "empty_list_before_runtime_registration" &&
                    list.at("system_bootstrap_required") == true && ids.at("status") == "resolved" &&
                    ids.at("scope") == "first_system_input_with_empty_id_registry",
                "complete_fresh_system_input_and_id_assignments_required");
        const auto &roots = list.at("roots");
        require(roots.size() == ids.at("roots").size(), "view_table_root_assignment_mismatch");
        for (std::size_t ri = 0; ri < roots.size(); ++ri) {
            const auto &root = roots.at(ri);
            const auto ni = root.at("native_record_index").get<std::size_t>();
            const auto &record = records.at(ni);
            if (record.at("element_type") != 14) continue;
            const auto &assigned = ids.at("roots").at(ri);
            const auto &items = assigned.at("records");
            const auto &headers = root.at("headers");
            require(assigned.at("native_record_index") == ni &&
                        assigned.at("block_number") == root.at("block_number") &&
                        !items.empty() && items.size() == headers.size(), "view_table_assignment_mismatch");
            auto identity = [&](std::size_t i) {
                const auto &item = items.at(i);
                const auto &header = headers.at(i);
                require(header.at("status") == "resolved" &&
                            item.at("native_record_index") == header.at("native_record_index") &&
                            item.at("parent_record_index") == header.at("parent_record_index"),
                        "view_table_member_assignment_mismatch");
                return Json{{"native_record_index", item.at("native_record_index")},
                            {"input_occurrence_index", item.at("input_occurrence_index")},
                            {"source_id", item.at("source_id")}, {"assigned_id", item.at("assigned_id")}};
            };
            require(items.front().at("native_record_index") == ni &&
                        items.front().at("parent_record_index").is_null(), "view_table_root_identity_mismatch");
            Json table = {{"source", identity(0)}, {"root_input_index", ri},
                          {"block_number", root.at("block_number")}, {"status", "unresolved"}};
            try {
                const auto bytes = bytesof(record.at("data"));
                require(bytes.size() >= 0x40, "truncated_type14_constructor_prefix");
                const auto subtype = Reader(bytes, 16).u32();
                table["source_subtype"] = subtype;
                if (subtype != 1 && subtype != 1000) {
                    table["status"] = "not_a_view_table_input";
                    out["tables"].push_back(std::move(table));
                    continue;
                }
                table["table_kind"] = subtype == 1 ? "ordinary" : "lite";
                table["conditions"] = Json::array({"R1.18_builtin_type14_handler_unchanged",
                    "prepared_direct_children_preserve_native_sibling_order",
                    "view_construction_succeeds_without_intervening_mutation"});
                table["child_scan"] = Json::array();
                Json slots = Json::array();
                for (unsigned slot = 0; slot < 8; ++slot)
                    slots.push_back({{"slot_index", slot}, {"origin", "empty"}});
                for (std::size_t i = 1; i < items.size(); ++i) {
                    // 393980 / 393840 walks entity+18 followed by entity+0.
                    // It does not recursively enumerate descendants.
                    auto source = identity(i);
                    if (items.at(i).at("parent_record_index") != ni) continue;
                    const auto ci = source.at("native_record_index").get<std::size_t>();
                    const auto &child = records.at(ci);
                    Json scan = {{"source", source}, {"outcome", "skip_element_type"}};
                    if (child.at("element_type") == 11) {
                        const auto data = bytesof(child.at("data"));
                        require(data.size() >= 20, "truncated_type11_subtype");
                        scan["outcome"] = "skip_subtype";
                        if (Reader(data, 16).u32() == 1) {
                            require(data.size() >= 0x10e, "truncated_type11_slot_index");
                            const auto word = Reader(data, 0x10c).u16();
                            const int slot = word >= 0x8000 ? int(word) - 0x10000 : int(word);
                            scan["source_slot_index"] = slot;
                            scan["outcome"] = "skip_slot_out_of_range";
                            if (slot >= 0 && slot < 8) {
                                auto &selected = slots.at(static_cast<std::size_t>(slot));
                                scan["outcome"] = "skip_occupied_slot";
                                if (selected.at("origin") == "empty") {
                                    const auto background = project_initial_view_background(child);
                                    require(background.at("status") == "conditional", "incomplete_selected_view_constructor_input");
                                    const auto first = Reader(data, 0xfc).u32();
                                    const auto normalized = (first & 0x1f800000u) > 0x03800000u
                                                                ? first & ~0x1f800000u : first;
                                    selected.update({{"origin", "direct_child"}, {"source", source},
                                        {"constructor_rgb", background.at("constructor_rgb")},
                                        {"constructor_words", Json::array({normalized, Reader(data, 0x100).u32()})},
                                        {"auxiliary_bytes", rawbytes(slice(data, 0x74, 32))}});
                                    scan["outcome"] = "selected";
                                }
                            }
                        }
                    }
                    table["child_scan"].push_back(std::move(scan));
                }
                std::size_t first = 0;
                while (first < 8 && slots.at(first).at("origin") == "empty") ++first;
                table["slot_projection_status"] = first == 8 ? "unresolved" : "conditional";
                if (first == 8) {
                    table["slot_projection_reason"] = "all_empty_native_default_view_creation_not_established";
                } else {
                    // 4acc80 fills missing slots from the LOWEST occupied
                    // slot, not from the first record in input order.
                    for (std::size_t slot = 0; slot < 8; ++slot) {
                        if (slots.at(slot).at("origin") != "empty") continue;
                        slots.at(slot) = slots.at(first);
                        slots.at(slot)["slot_index"] = slot;
                        slots.at(slot)["origin"] = "copy_lowest_occupied_slot";
                        slots.at(slot)["copied_from_slot"] = first;
                        slots.at(slot)["constructor_words"][0] =
                            slots.at(slot).at("constructor_words")[0].get<std::uint32_t>() & ~0x84u;
                    }
                }
                table["slots"] = std::move(slots);
                table["status"] = "conditional";
                table["unmodeled_steps"] = Json::array({"name_based_common_and_lite_collection_merge",
                    "active_view_selection", "model_resolution_and_remaining_view_properties",
                    "display_style_application", "viewport_refresh"});
            } catch (const std::exception &e) {
                table["reason"] = e.what();
            }
            out["tables"].push_back(std::move(table));
        }
        out["status"] = "resolved";
        for (const auto &table : out.at("tables"))
            if (table.at("status") == "unresolved") out["status"] = "partial";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json Document::initial_view_table_inputs() const {
    Json out = Json::array();
    for (const auto &container : native_input_containers()) {
        if (container.at("kind") != "P3D-SSYS" || container.at("container").size() != 2) continue;
        auto tables = initial_native_view_table_inputs(container.at("list_preparation"), native_records(),
                                                       container.at("system_id_assignments"));
        tables["system_container"] = container.at("container");
        out.push_back(std::move(tables));
    }
    return out;
}
} // namespace p3d
