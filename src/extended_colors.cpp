#include "extended_colors.hpp"
namespace p3d {
namespace {
Json scan_color(const std::string &text) {
    // 2667bb: initialize ints to 255; UCRT L"(%d,%d,%d)"; retain low bytes.
    std::array<unsigned, 3> rgb{255, 255, 255};
    std::size_t p = 0;
    unsigned assigned = 0;
    if (!text.empty() && text[p++] == '(') {
        for (unsigned channel = 0; channel < 3; ++channel) {
            while (p < text.size() && (text[p] == ' ' || (text[p] >= '\t' && text[p] <= '\r'))) ++p;
            bool negative = false;
            if (p < text.size() && (text[p] == '+' || text[p] == '-')) negative = text[p++] == '-';
            const auto start = p;
            const auto limit = negative ? 0x8000000000000000ull : 0x7fffffffffffffffull;
            std::uint64_t value = 0;
            while (p < text.size() && text[p] >= '0' && text[p] <= '9') {
                const auto digit = unsigned(text[p++] - '0');
                value = value > (limit - digit) / 10 ? limit : value * 10 + digit;
            }
            if (p == start) break;
            rgb[channel] = unsigned(negative ? std::uint64_t(0) - value : value) & 255u;
            ++assigned;
            // Literal commas do not skip whitespace. A missing closing ')'
            // cannot undo successful assignments; the native return is ignored.
            if (channel < 2 && (p == text.size() || text[p++] != ',')) break;
        }
    }
    return {{"rgb", Json::array({rgb[0], rgb[1], rgb[2]})}, {"assigned_channels", assigned}};
}
} // namespace

Json initial_native_extended_color_table(const Json &list, const Json &records,
                                         const Json &ids, const Json &input) {
    Json out = {{"status", "unresolved"}, {"scope", "fresh_file_extended_color_cache_input"},
                {"runtime_cache", "not_evaluated"}, {"system_slot", 0},
                {"skip_deleted", false}, {"selected_record", nullptr}, {"selected_attribute", nullptr},
                {"conditions", {"fresh_system_list_loaded_in_prepared_order",
                                "initial_attributes_attached_before_color_cache_construction",
                                "no_intervening_host_or_cache_mutation"}}};
    try {
        require(list.at("scope") == "empty_list_before_runtime_registration" &&
                    list.at("status") == "resolved" && list.at("system_bootstrap_required") == true &&
                    list.at("system_bootstrap_found") == true, "complete_fresh_system_list_required");
        require(ids.at("scope") == "first_system_input_with_empty_id_registry" &&
                    ids.at("status") == "resolved", "complete_system_id_assignments_required");
        require(!list.at("roots").empty() && list.at("roots").size() == ids.at("roots").size(),
                "system_input_order_mismatch");
        const auto &root = list.at("roots")[0];
        const auto &assigned = ids.at("roots")[0];
        require(!root.at("headers").empty() && !assigned.at("records").empty(), "missing_system_slot_zero");
        const auto &header = root.at("headers")[0];
        const auto &item = assigned.at("records")[0];
        const auto ni = root.at("native_record_index").get<std::size_t>();
        require(list.at("bootstrap_root_record_index") == ni && assigned.at("native_record_index") == ni &&
                    header.at("native_record_index") == ni && header.at("status") == "resolved" &&
                    header.at("parent_record_index").is_null() && item.at("native_record_index") == ni &&
                    item.at("input_occurrence_index") == 0, "system_slot_zero_identity_mismatch");
        const auto &record = records.at(ni);
        const auto bytes = bytesof(record.at("data"));
        require(record.at("element_type") == 46 && Reader(bytes, 16).u32() == 8 &&
                    !(record.at("element_flags").get<unsigned>() & 8), "system_bootstrap_record_required");
        out["selected_record"] = {{"native_record_index", ni}, {"input_occurrence_index", 0},
                                  {"source_id", item.at("source_id")}, {"assigned_id", item.at("assigned_id")}};
        const auto status = input.at("status");
        require(status == "resolved" || status == "absent" || status == "not_loaded",
                "complete_initial_attribute_input_required");
        const Json *attachment = nullptr;
        std::size_t attachment_index = 0;
        for (std::size_t i = 0; i < input.at("attachments").size(); ++i) {
            const auto &candidate = input.at("attachments")[i];
            if (candidate.at("target").at("input_occurrence_index") != 0) continue;
            require(!attachment, "ambiguous_initial_attribute_attachment");
            attachment = &candidate;
            attachment_index = i;
        }
        const Json *selected = nullptr;
        if (attachment) {
            const auto &lookup = attachment->at("lookup");
            require(lookup.at("status") == "resolved", "initial_attribute_lookup_required");
            for (const auto &key : lookup.at("keys")) {
                if (key.at("group") != 0 || key.at("key") != 22902 || key.at("index") != 0) continue;
                require(!selected, "ambiguous_extended_color_lookup_key");
                selected = &key;
            }
        }
        if (!selected) {
            out["outcome"] = "empty_cache_missing_attribute";
            out["cache_input"] = {{"entries", Json::array()}, {"rgb_lookup", Json::array()}, {"slot_count", 0}};
        } else {
            const auto ordinal = selected->at("selected_source_ordinal").get<std::size_t>();
            const auto &attribute = attachment->at("attributes").at(ordinal);
            require(attribute.at("group") == 0 && attribute.at("key") == 22902 && attribute.at("index") == 0,
                    "selected_color_attribute_key_mismatch");
            out["selected_attribute"] = {{"attachment_index", attachment_index}, {"source_ordinal", ordinal},
                {"attribute_offset", attribute.at("offset")}, {"stream", attachment->at("stream")},
                {"matching_source_ordinals", selected->at("matching_source_ordinals")}};
            const auto &decoded = attribute.at("decoded");
            require(decoded.contains("native_extended_color_import") &&
                        decoded.at("native_extended_color_import").at("status") == "resolved",
                    "selected_color_payload_import_unresolved");
            out["cache_input"] = decoded.at("native_extended_color_import");
            out["outcome"] = "selected_xml_input";
        }
        out["status"] = "resolved";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json Document::initial_extended_color_tables() const {
    Json out = Json::array();
    for (const auto &container : native_input_containers()) {
        if (container.at("kind") != "P3D-SSYS" || container.at("container").size() != 2) continue;
        auto table = initial_native_extended_color_table(container.at("list_preparation"), native_records(),
            container.at("system_id_assignments"), container.at("initial_attribute_input"));
        table["system_container"] = container.at("container");
        out.push_back(std::move(table));
    }
    return out;
}
Json decode_native_extended_colors(const Json &tree) {
    Json out = {{"status", "unresolved"}, {"scope", "R1.18_extended_color_XML_input"},
                {"runtime_table_selection", "not_evaluated"}, {"source_locale", "UCRT_C"},
                {"source_entries", Json::array()}, {"entries", Json::array()}, {"rgb_lookup", Json::array()}};
    try {
        // Native XPath //Entry includes the matching root and descendants,
        // excludes namespace-qualified nodes, and retains document order.
        struct Pending { const Json *node; Json path; };
        std::vector<Pending> pending{{&tree, Json::array()}};
        std::map<std::uint32_t, std::size_t> last_by_rgb;
        while (!pending.empty()) {
            auto current = std::move(pending.back()); pending.pop_back();
            const auto &node = *current.node;
            if (node.at("tag") == "Entry") {
                const auto &attributes = node.at("attributes");
                require(attributes.is_object(), "invalid_extended_color_attributes");
                Json source = {{"source_path", current.path}, {"source_attributes", attributes},
                    {"source_ordinal", out["source_entries"].size()}, {"action", "skip_missing_color"}};
                if (attributes.contains("Color")) {
                    require(attributes.at("Color").is_string(), "extended_color_text_required");
                    const auto &color = attributes.at("Color").get_ref<const std::string &>();
                    source["action"] = "skip_empty_color";
                    if (!color.empty()) {
                        auto entry = scan_color(color);
                        entry["ordinal"] = out["entries"].size();
                        entry["native_index"] = out["entries"].size() + 1;
                        entry["source_ordinal"] = source.at("source_ordinal");
                        entry["source_path"] = current.path;
                        entry["source_attributes"] = attributes;
                        entry["book_name"] = nullptr;
                        const auto book = attributes.value("Book", std::string());
                        const auto name = attributes.value("Name", std::string());
                        if (!book.empty() && !name.empty()) entry["book_name"] = {{"book", book}, {"name", name}};
                        const auto &rgb = entry.at("rgb");
                        const auto key = (rgb[0].get<unsigned>() << 16) | (rgb[1].get<unsigned>() << 8) | rgb[2].get<unsigned>();
                        // 266990 appends even duplicates and updates RGB->index
                        // to the most recently appended one-based slot.
                        last_by_rgb[key] = out["entries"].size();
                        source.update({{"action", "append"}, {"native_index", entry.at("native_index")}});
                        out["entries"].push_back(std::move(entry));
                    }
                }
                out["source_entries"].push_back(std::move(source));
            }
            const auto &children = node.at("children");
            require(children.is_array(), "invalid_extended_color_children");
            for (std::size_t i = children.size(); i; --i) {
                auto path = current.path; path.push_back(i - 1);
                pending.push_back({&children[i - 1], std::move(path)});
            }
        }
        for (const auto &[key, ordinal] : last_by_rgb)
            out["rgb_lookup"].push_back({{"rgb_key", key}, {"native_index", ordinal + 1}});
        out["slot_count"] = out["entries"].size();
        out["status"] = "resolved";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
}
