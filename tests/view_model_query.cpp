#include "internal.hpp"
#include <p3d/view_model.hpp>
#include "view_model_query_oracle.hpp"
using namespace p3d;
unsigned view_model_query_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) { ++checks; require(value, message); };
    auto slots = [](const Json &row) {
        NativeViewModelSlots views;
        for (unsigned i = 0; i < 8; ++i) {
            views[i].presence = row.at("models")[i].is_null() ? NativeViewSlotPresence::Absent : NativeViewSlotPresence::Present;
            if (!row.at("models")[i].is_null()) views[i].model_id = row.at("models")[i].get<std::int32_t>();
            views[i].flags_low = row.at("flags_low")[i].get<std::uint8_t>();
        }
        return views;
    };
    const auto oracle = Json::parse(view_model_query_oracle);
    for (const auto &row : oracle.at("filter_cases")) {
        const auto views = slots(row);
        for (const auto &q : row.at("queries")) {
            const auto result = match_native_view_table_model(views, q.at("requested_id"), q.at("all_same"), q.at("slot"));
            check(result.matches && *result.matches == q.at("matches") && result.reason.empty(),
                  "model filter matches original signed IDs, sentinel order, null skipping and slot precedence");
        }
    }
    for (const auto &row : oracle.at("query_cases")) {
        const auto result = query_native_view_table_model(slots(row));
        check(result.scan_complete && result.model_id && *result.model_id == row.at("model_id") && result.reason.empty(),
              "file model result matches complete original getter for every flag mask");
        bool same = true;
        for (unsigned i = 0; i < 8; ++i)
            same &= row.at("dirty_after")[i] == ((result.marked_slot_mask & (1u << i)) ? Json(1) : row.at("dirty_before")[i]);
        check(same, "query describes only actual dirty writes and preserves arbitrary untouched byte values");
    }
    NativeViewModelSlots views;
    check(!match_native_view_table_model(views, 7, true, 0).matches, "unknown pointer presence does not mean absent");
    for (auto &v : views) v.presence = NativeViewSlotPresence::Absent;
    check(match_native_view_table_model(views, -2, true, 0).matches == true, "all absent views preserve the -2 accumulator");
    check(!match_native_view_table_model(views, -2, false, 0).matches, "direct null view is not a normal mismatch");
    check(!query_native_view_table_model(views).scan_complete, "file query cannot skip a null view");
    views[0] = {NativeViewSlotPresence::Present, 7, 0x80};
    check(match_native_view_table_model(views, 7, false, -1).matches == true,
          "first flag short circuit does not inspect later null views");
    auto q = query_native_view_table_model(views);
    check(q.model_id == 7 && q.slot_index == 0 && q.marked_slot_mask == 1, "query also short circuits before later nulls");
    views[0].flags_low.reset();
    check(match_native_view_table_model(views, 7, false, 0).matches == true, "explicit slot comparison does not read flags");
    check(!match_native_view_table_model(views, 7, false, -1).matches, "flag scan requires the flags it visits");
    check(query_native_view_table_model(views).marked_slot_mask == 1, "dirty write precedes an unresolved flag read");
    views[0] = {NativeViewSlotPresence::Present, 7, 0};
    views[1] = {NativeViewSlotPresence::Present, 8, 0};
    views[2].presence = NativeViewSlotPresence::Unknown;
    check(match_native_view_table_model(views, 7, true, 7).matches == false,
          "proven all-same mismatch short circuits before unknown suffix and ignores requested slot");
    for (auto &v : views) v = {NativeViewSlotPresence::Present, std::nullopt, 0};
    views[5].flags_low = 0x80;
    q = query_native_view_table_model(views);
    check(q.scan_complete && q.slot_index == 5 && q.marked_slot_mask == 63 && !q.model_id,
          "known slot and dirty effects remain available when its current model ID is unknown");
    check(match_native_view_table_model(views, 7, false, 9).reason == "constructor_model_id_not_established",
          "flagged view model uncertainty is not a false comparison");

    Json source = {{"native_record_index", 9}, {"input_occurrence_index", 3}, {"source_id", 99}, {"assigned_id", 42}};
    Json entry = {{"table_input_index", 0}, {"source", source}};
    Json selection = {{"status", "conditional"}, {"scope", "R1.18_initial_file_table_query"},
        {"query_source", source}, {"requested_id", 42}, {"conditions", Json::array()}, {"selected_entry", entry}};
    Json table = {{"source", source}, {"slot_projection_status", "conditional"}, {"slots", Json::array()}};
    for (unsigned i = 0; i < 8; ++i)
        table["slots"].push_back({{"slot_index", i}, {"origin", "direct_child"}, {"source", source},
            {"constructor_words", Json::array({i == 5 ? 0x80u : 0u, 0u})},
            {"model_id_input", {{"status", "unresolved"}}}});
    auto projected = project_file_view_model_query(Json::array({table}), selection);
    check(projected.at("status") == "unresolved" && projected.at("selected_slot_index") == 5 &&
          projected.at("marked_slot_mask") == 63 && !projected.contains("model_id"),
          "integrated query exposes proven slot identity without substituting source model IDs");
    table["slots"][5]["model_id_input"] = {{"status", "conditional"}, {"constructor_model_id", -1}};
    projected = project_file_view_model_query(Json::array({table}), selection);
    check(projected.at("status") == "conditional" && projected.at("model_id") == -1 &&
          projected.at("model_object") == "not_evaluated", "integrated known ID does not claim model binding");
    table["source"]["assigned_id"] = 43;
    check(!project_file_view_model_query(Json::array({table}), selection).contains("selected_slot_index"),
          "a stale selected table identity cannot supply model or slot data");
    selection.erase("selected_entry"); selection["native_return_code"] = 0x1c010;
    for (const auto &row : oracle.at("absent_cases")) {
        projected = project_file_view_model_query(Json::array(), selection);
        check(projected.at("model_id") == row.at("model_id") && projected.at("marked_slot_mask") == 0 && row.at("table_dirty_unchanged"),
              "proven no-table result matches full original file query without dirty writes");
    }
    selection["status"] = "unresolved";
    check(!project_file_view_model_query(Json::array(), selection).contains("model_id"),
          "unresolved table creation is not the proven no-table model sentinel");
    return checks;
}
