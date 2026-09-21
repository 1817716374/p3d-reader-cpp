#include "internal.hpp"
#include "text_bytes.hpp"
#include <set>

namespace p3d {
Json initial_native_font_catalog(const Json &list, const Json &records) {
    Json out = {{"status", "unresolved"},
                {"scope", "fresh_system_font_declarations_before_runtime_callbacks"},
                {"font_resolution_status", "not_evaluated"},
                {"selected_table", nullptr},
                {"table_candidates", Json::array()},
                {"entries", Json::array()},
                {"input", Json::array()}};
    try {
        require(list.at("scope") == "empty_list_before_runtime_registration" &&
                    list.at("status") == "resolved" &&
                    list.at("system_bootstrap_required") == true &&
                    list.at("system_bootstrap_found") == true,
                "complete_fresh_system_list_required");
        const Json *selected = nullptr;
        for (std::size_t ri = 0; ri < list.at("roots").size(); ++ri) {
            const auto &root = list.at("roots")[ri];
            const auto ni = root.at("native_record_index").get<std::size_t>();
            const auto &n = records.at(ni);
            const auto &headers = root.at("headers");
            require(!headers.empty() && headers[0].at("native_record_index") == ni &&
                        headers[0].at("parent_record_index").is_null() &&
                        headers[0].at("status") == "resolved",
                    "prepared_system_root_required");
            if (n.at("element_type") != 10)
                continue;
            const auto base = bytesof(n.at("data"));
            if (Reader(base, 16).u32() != 2)
                continue;
            selected = &root;
            Json source = {{"root_index", ri},
                           {"native_record_index", ni},
                           {"source_id", n.at("id")},
                           {"block_number", root.at("block_number")}};
            out["table_candidates"].push_back(source);
            out["selected_table"] = std::move(source);
        }
        if (!selected) {
            out["status"] = "resolved";
            out["table_selection"] = "absent";
            return out;
        }
        out["table_selection"] = "last_system_table";
        const auto table_index = selected->at("native_record_index").get<std::size_t>();
        std::map<std::uint32_t, Json> registry;
        std::set<std::size_t> members;
        for (const auto &header : selected->at("headers")) {
            const auto parent = header.at("parent_record_index");
            if (parent.is_null() || parent != table_index)
                continue; // The native iterator follows direct sibling records.
            require(header.at("status") == "resolved", "prepared_font_member_required");
            const auto ni = header.at("native_record_index").get<std::size_t>();
            require(members.insert(ni).second, "duplicate_font_member");
            const auto &record = records.at(ni);
            const auto base = bytesof(record.at("data"));
            require(record.at("element_type") == 49 && Reader(base, 16).u32() == 2,
                    "unsupported_font_table_member_layout");
            const auto definition = record.contains("font_definition")
                                        ? record.at("font_definition")
                                        : decode_native_font_record(base);
            require(definition.at("status") == "decoded", "invalid_font_definition");
            const auto id = definition.at("font_id").get<std::uint32_t>();
            Json source = {{"native_record_index", ni}, {"source_id", record.at("id")}};
            Json step = {{"font_id", id}, {"source", source}};
            if (id < 512) {
                step["action"] = "skipped_font_id";
            } else {
                auto previous = registry.find(id);
                step["action"] =
                    previous == registry.end() ? "registered_request" : "replaced_request";
                if (previous != registry.end())
                    step["previous_source"] = previous->second.at("source");
                registry[id] = {{"font_id", id},
                                {"name", definition.at("name")},
                                {"requested_font_family", definition.at("requested_font_family")},
                                {"source", std::move(source)}};
            }
            out["input"].push_back(std::move(step));
        }
        for (auto &[id, entry] : registry)
            out["entries"].push_back(std::move(entry));
        out["entry_order"] = "unsigned_font_id_ascending";
        out["status"] = "resolved";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json native_primary_font_request(const Json &catalog, std::uint32_t lookup_id) {
    Json out = {{"status", "unresolved"},
                {"lookup_id", lookup_id},
                {"scope", "primary_font_request_before_font_substitution_and_big_font_selection"},
                {"font_resolution_status", "not_evaluated"}};
    if (lookup_id <= 255) {
        out.update({{"status", "default_font_request"}, {"requested_font_family", "TrueType"}});
        return out; // This native branch never accesses the file's font map.
    }
    try {
        require(catalog.at("scope") == "fresh_system_font_declarations_before_runtime_callbacks" &&
                    catalog.at("status") == "resolved",
                "complete_initial_font_catalog_required");
        if (lookup_id > 255) {
            const Json *selected = nullptr;
            for (const auto &entry : catalog.at("entries")) {
                if (entry.at("font_id") != lookup_id)
                    continue;
                require(selected == nullptr, "ambiguous_font_catalog_key");
                selected = &entry;
            }
            if (selected) {
                out.update({{"status", "declaration_request"},
                            {"declaration", *selected},
                            {"requested_font_family", selected->at("requested_font_family")}});
                return out;
            }
        }
        out.update(
            {{"status", "default_font_request"},
             {"requested_font_family", lookup_id > 255 && lookup_id < 1024 ? "Shx" : "TrueType"}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json Document::native_font_catalogs() const {
    Json out = Json::array();
    const auto containers = p3d::native_input_containers(streams(), index(), native_records());
    for (const auto &container : containers) {
        if (container.at("kind") != "P3D-SSYS")
            continue;
        auto catalog =
            initial_native_font_catalog(container.at("list_preparation"), native_records());
        catalog["system_container"] = container.at("container");
        out.push_back(std::move(catalog));
    }
    return out;
}
} // namespace p3d
