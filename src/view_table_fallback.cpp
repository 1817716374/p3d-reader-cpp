#include "internal.hpp"

namespace p3d {
Json project_file_view_table_fallback(const Json &records, const Json &collection, const Json &header) {
    Json out = {{"status", "unresolved"}, {"scope", "R1.18_file_table_fallback_without_model_filter"},
                {"name_profile", "R1.18_UCRT_C_locale"}, {"comparison_data_offset", 0x1c},
                {"scan", Json::array()}, {"default_table_creation", "not_evaluated"}};
    try {
        require(collection.at("status") == "conditional" &&
                    collection.at("scope") == "R1.18_initial_common_then_lite_view_collection",
                "complete_initial_view_collection_required");
        const auto &entries = collection.at("entries");
        const std::string name = "p3d_global_liteviewgourp";
        std::size_t special = entries.size();
        // 4b34a0 trims only the constant, then calls UCRT _wcsicmp on
        // each first field. Candidate whitespace and the second field remain.
        for (std::size_t i = 0; i < entries.size(); ++i) {
            const auto ni = entries[i].at("source").at("native_record_index").get<std::size_t>();
            const auto text = project_view_table_text_fields(records.at(ni));
            require(text.at("fields").size() >= 1 && text.at("fields")[0].at("status") == "resolved",
                    "default_table_name_unresolved");
            const auto &units = text.at("fields")[0].at("constructor_utf16_units");
            bool equal = units.size() == name.size();
            for (std::size_t j = 0; equal && j < units.size(); ++j) {
                auto unit = units[j].get<unsigned>();
                if (unit >= 'A' && unit <= 'Z') unit += 'a' - 'A';
                equal = unit == static_cast<unsigned char>(name[j]);
            }
            if (equal) { special = i; break; }
        }
        auto empty = [&]() {
            // The full file getter would otherwise try 4b1e00. Its model,
            // host and record creation dependencies are not inferred here.
            if (header.value("status", Json()) == "resolved" && header.contains("initial_probe") &&
                header.at("initial_probe").value("action", Json()) == "read_header_payload" &&
                header.at("initial_probe").value("default_model_id", Json()) == 0xfffffffeu) {
                out.update({{"status", "conditional"}, {"decision", "no_table"},
                    {"native_return_code", 0x1c010}, {"default_table_creation", "disabled_by_model_minus2"}});
            } else {
                out["reason"] = "default_table_creation_or_file_model_state_not_established";
            }
        };
        out["conditions"] = Json::array({"fresh_resident_collection_with_record_backed_tables",
            "R1.18_UCRT_C_locale", "no_intervening_table_or_record_mutation",
            "native_floating_point_exceptions_masked"});
        if (special == entries.size()) {
            out["default_name_lookup"] = "absent";
            empty();
            return out;
        }
        out["default_name_lookup"] = "first_case_insensitive_match";
        out["excluded_default_entry"] = entries[special];
        const Json *selected = nullptr;
        double score = 0;
        for (std::size_t i = 0; i < entries.size(); ++i) {
            const auto &entry = entries[i];
            if (entry.at("source").at("assigned_id").get<std::uint64_t>() == UINT64_MAX) {
                out["scan"].push_back({{"collection_index", i}, {"action", "preferred_all_ones_id"}});
                selected = &entry;
                break; // This bypasses the comparison and default exclusion.
            }
            const auto ni = entry.at("source").at("native_record_index").get<std::size_t>();
            const auto data = bytesof(records.at(ni).at("data"));
            require(data.size() >= 0x24, "truncated_table_fallback_comparison_field");
            const auto bits = Reader(data, 0x1c).u64();
            const auto value = Reader(data, 0x1c).f64();
            Json step = {{"collection_index", i}, {"comparison_bits", bits}};
            // COMISD/JBE admits unordered comparisons too. Do not sort:
            // ties pick the later entry and a NaN resets subsequent ordering.
            if (score > value) step["action"] = "skip_lower_value";
            else if (i == special) step["action"] = "skip_default_entry";
            else { selected = &entry; score = value; step["action"] = "replace_candidate"; }
            out["scan"].push_back(std::move(step));
        }
        if (selected) {
            out.update({{"status", "conditional"}, {"decision", "existing_table"},
                {"selected_entry", *selected}, {"native_return_code", 0},
                {"default_table_creation", "not_needed"}});
        } else empty();
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
