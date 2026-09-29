#include "display_style_xml.hpp"

namespace p3d {
namespace {
std::uint64_t count(const Json &table) {
    if (table.at("status") == "absent") return 0;
    require(table.at("status") == "selected", "style_table_selection_unresolved");
    const auto &load = table.at("table_load");
    if (load.at("status") == "empty") return 0;
    require(load.at("status") == "resolved", "style_table_slot_layout_unresolved");
    return load.at("slot_count").get<std::uint64_t>();
}

std::string name_key(std::string name) {
    // P3DDC ordinal 503 -> UCRT _wcsicmp, observed C locale. UTF-8
    // non-ASCII bytes remain exact; only ASCII uppercase folds here.
    name.resize(name.find('\0') == std::string::npos ? name.size() : name.find('\0'));
    for (auto &c : name)
        if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
    return name;
}
} // namespace

Json plan_initial_lite_style_list(const Json &common, const Json &lite) {
    Json out = {{"scope", "initial_lite_list_copy_plan"}, {"status", "unresolved"},
                {"object_construction", "not_evaluated"}, {"file_writeback", "not_evaluated"},
                {"operations", Json::array()}, {"entries", Json::array()},
                {"null_slot_ranges", Json::array()}};
    try {
        const auto lite_count = count(lite);
        out["initial_lite_slot_count"] = lite_count;
        // 0xdbaa0 tests begin != end, not the number of non-null objects.
        if (lite_count) {
            out.update({{"status", "layout_resolved"}, {"decision", "retain_nonempty_lite"},
                        {"slot_count", lite_count}, {"list_source", "lite"}});
            return out;
        }
        const auto common_count = count(common);
        out["common_slot_count"] = common_count;
        if (!common_count) {
            out.update({{"status", "layout_resolved"}, {"decision", "empty_lists"}, {"slot_count", 0}});
            return out;
        }
        out["decision"] = "copy_common";
        out["conditions"] = Json::array({"native_xml_imports_match_retained_trees",
            "same_file_context", "R1.18_UCRT_C_locale", "file_writeback_callbacks_do_not_mutate_style_lists"});
        // Sparse by source slot, never allocate the declared maximum. Every
        // built-in 22903 input stores its attribute index at object +0x10.
        std::map<std::uint64_t, const Json *> source;
        for (const auto &entry : common.at("table_load").at("entries")) {
            const auto i = entry.at("slot_index").get<std::uint64_t>();
            require(i < common_count && entry.at("attribute_index") == i,
                    "style_slot_attribute_index_mismatch");
            require(source.emplace(i, &entry).second, "duplicate_selected_style_slot");
        }
        std::map<std::uint64_t, Json> result;
        std::map<std::string, std::uint64_t> first_by_name;
        std::uint64_t next = 0;
        auto append_null = [&](std::uint64_t first, std::uint64_t last, const char *reason) {
            out["operations"].push_back({{"action", "append_null_slots"},
                {"first", first}, {"last", last}, {"reason", reason}});
        };
        for (const auto &[i, pointer] : source) {
            if (next < i) append_null(next, i - 1, "missing_attribute");
            next = i + 1;
            const auto &entry = *pointer;
            require(entry.contains("native_xml_import"), "style_xml_input_unresolved");
            const auto &imported = entry.at("native_xml_import");
            if (imported.at("status") == "rejected") {
                append_null(i, i, "rejected_xml_import");
                continue;
            }
            require(imported.at("status") == "partial", "style_xml_input_unresolved");
            require(imported.at("usages").at("status") == "decoded", "style_usage_import_unresolved");
            require(imported.at("fields").at("Overrides.DisplayHandler").at("value") == 0,
                    "custom_display_handler_import_unmodeled");
            const auto projection = project_display_style_lite_copy(entry.at("xml_tree"));
            require(projection.at("status") == "conditional", "style_copy_field_projection_unresolved");
            auto projected = decode_display_style_xml(projection.at("xml_tree"));
            require(projected.at("status") == "partial", "style_copy_field_projection_unresolved");
            const auto name = imported.at("fields").at("Name").at("value").get<std::string>();
            Json value = {{"slot_index", i}, {"stored_index", i}, {"state_at_38", 0},
                          {"source_common_slot", i}, {"source_ordinal", entry.at("source_ordinal")},
                          {"attribute_offset", entry.at("attribute_offset")},
                          {"name", name}, {"projected_xml_import", std::move(projected)}};
            result[i] = value;
            out["operations"].push_back({{"action", "append_common_copy"}, {"source_slot", i}, {"slot_index", i}});
            // 0xdbbb1 -> 0x5f1a0 -> 0x601f0: the already appended copy
            // participates in first-name lookup. A duplicate replaces the
            // first matching object's stored index, retaining that index/state.
            const auto target = first_by_name.emplace(name_key(name), i).first->second;
            value["slot_index"] = value["stored_index"] = target;
            result[target] = std::move(value);
            out["operations"].push_back({{"action", "replace_first_name_match"},
                {"source_slot", i}, {"target_slot", target}, {"retained_stored_index", target},
                {"file_writeback", "not_evaluated"}});
        }
        if (next < common_count) append_null(next, common_count - 1, "missing_attribute");
        next = 0;
        for (auto &[i, entry] : result) {
            if (next < i) out["null_slot_ranges"].push_back({{"first", next}, {"last", i - 1}});
            next = i + 1;
            out["entries"].push_back(std::move(entry));
        }
        if (next < common_count)
            out["null_slot_ranges"].push_back({{"first", next}, {"last", common_count - 1}});
        out["slot_count"] = common_count;
        out["status"] = "conditional";
    } catch (const std::exception &e) {
        // Operations are a proven prefix, not a completed list if a later
        // unresolved import might introduce another matching name.
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
