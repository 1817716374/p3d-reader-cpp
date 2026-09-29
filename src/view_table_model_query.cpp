#include "internal.hpp"
#include <p3d/view_model.hpp>

namespace p3d {
Json project_file_view_model_query(const Json &tables, const Json &selection) {
    Json out = {{"status", "unresolved"}, {"scope", "R1.18_initial_file_model_query"},
                {"runtime_application", "not_evaluated"}, {"model_object", "not_evaluated"}};
    try {
        require(selection.at("status") == "conditional" &&
                selection.at("scope") == "R1.18_initial_file_table_query",
                "file_view_table_selection_not_established");
        out["query_source"] = selection.at("query_source");
        out["requested_table_id"] = selection.at("requested_id");
        out["conditions"] = selection.at("conditions");
        out["conditions"].push_back("R1.18_12ef50_with_same_resident_file_and_no_intervening_mutation");
        if (!selection.contains("selected_entry")) {
            require(selection.value("native_return_code", Json()) == 0x1c010,
                    "file_query_table_absence_not_established");
            out.update({{"status", "conditional"}, {"decision", "no_table"},
                        {"model_id", -2}, {"marked_slot_mask", 0}, {"slot_scan_complete", true}});
            return out;
        }
        const auto &entry = selection.at("selected_entry");
        const auto &table = tables.at(entry.at("table_input_index").get<std::size_t>());
        require(table.at("source") == entry.at("source"), "selected_table_identity_mismatch");
        out["selected_entry"] = entry;
        require(table.at("slot_projection_status") == "conditional" && table.at("slots").size() == 8,
                "selected_table_constructor_slots_not_established");
        NativeViewModelSlots views;
        for (std::size_t i = 0; i < 8; ++i) {
            const auto &slot = table.at("slots")[i];
            require(slot.at("slot_index") == i &&
                    (slot.at("origin") == "direct_child" || slot.at("origin") == "copy_lowest_occupied_slot"),
                    "selected_table_slot_identity_not_established");
            views[i].presence = NativeViewSlotPresence::Present;
            views[i].flags_low = static_cast<std::uint8_t>(slot.at("constructor_words")[0].get<std::uint32_t>());
            const auto &model = slot.at("model_id_input");
            if (model.at("status") == "conditional")
                views[i].model_id = model.at("constructor_model_id").get<std::int32_t>();
        }
        const auto query = query_native_view_table_model(views);
        out["marked_slot_mask"] = query.marked_slot_mask;
        out["slot_scan_complete"] = query.scan_complete;
        if (query.slot_index) {
            out["selected_slot_index"] = *query.slot_index;
            out["selected_view_source"] = table.at("slots")[*query.slot_index].at("source");
        }
        if (query.model_id) out.update({{"status", "conditional"}, {"model_id", *query.model_id}});
        else out["reason"] = query.reason;
    } catch (const std::exception &e) { out["reason"] = e.what(); }
    return out;
}
} // namespace p3d
