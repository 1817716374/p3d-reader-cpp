#include "internal.hpp"
#include <p3d/material_lookup.hpp>

namespace p3d {
Json lookup_native_material_name(const std::u16string &name,
                                 const NativeMaterialNameContext &context) {
    Json out = {{"status", "unresolved"},
                {"scope", "prepared_native_material_catalog_empty_result_list"},
                {"comparison", "native_case_insensitive_locale_dependent"},
                {"candidate_indices", Json::array()},
                {"comparisons", Json::array()},
                {"loads", Json::array()},
                {"material_indices", Json::array()},
                {"first_result_status", "unresolved"},
                {"native_return_code", nullptr},
                {"native_diagnostic_code", nullptr}};
    auto prefix = [](const std::u16string &s) { return s.substr(0, s.find(u'\0')); };
    const auto query = prefix(name);
    out["query_utf16_code_units"] = std::vector<std::uint16_t>(query.begin(), query.end());
    auto missing = [&](const char *status, unsigned diagnostic) {
        out["status"] = status;
        out["first_result_status"] = "not_found";
        out["native_return_code"] = 1;
        out["native_diagnostic_code"] = diagnostic;
    };
    const char *phase = "candidate_selection";
    std::size_t index = 0;
    try {
        // The native empty-name branch does not inspect the catalog.
        if (query.empty()) {
            missing("not_found", 3);
            return out;
        }
        require(context.catalog_complete, "complete_prepared_material_catalog_required");
        std::vector<std::size_t> candidates;
        for (; index < context.names.size(); ++index) {
            require(bool(context.compare), "native_material_name_comparator_required");
            const auto candidate = prefix(context.names[index]);
            const auto compared = context.compare(candidate, query);
            require(compared.has_value(), "native_material_name_comparison_unknown");
            const bool match = *compared == 0;
            out["comparisons"].push_back({{"catalog_index", index}, {"matched", match}});
            if (match) {
                candidates.push_back(index);
                out["candidate_indices"].push_back(index);
            }
        }
        if (candidates.empty()) {
            missing("not_found", 3);
            return out;
        }
        phase = "candidate_loading";
        for (auto candidate : candidates) {
            index = candidate;
            require(bool(context.load), "native_material_loader_required");
            const auto loaded = context.load(candidate);
            require(loaded.known || !loaded.material_index, "unknown_material_has_identity");
            require(loaded.known, "native_material_load_unknown");
            Json step = {{"catalog_index", candidate},
                         {"status", loaded.material_index ? "loaded" : "not_loaded"}};
            if (loaded.material_index) {
                step["material_index"] = *loaded.material_index;
                out["material_indices"].push_back(*loaded.material_index);
                if (out["first_result_status"] != "matched") {
                    out["first_result_status"] = "matched";
                    out["first_material_index"] = *loaded.material_index;
                    out["first_catalog_index"] = candidate;
                }
            }
            out["loads"].push_back(std::move(step));
        }
        if (out["material_indices"].empty()) {
            missing("load_failed", 4);
        } else {
            out["status"] = "loaded";
            out["native_return_code"] = 0;
            out["native_diagnostic_code"] = 0;
        }
    } catch (const std::exception &e) {
        out["reason"] = e.what();
        out["unresolved_phase"] = phase;
        out["unresolved_catalog_index"] = index;
    }
    return out;
}
Json lookup_native_graphics_material_name(const std::u16string &name,
                                          const NativeGraphicsMaterialNameContext &context) {
    Json out = {{"status", "unresolved"},
                {"scope", "prepared_graphics_project_material_name_lookup"},
                {"project_selection", "caller_supplied_current_project"},
                {"catalog_update", "disabled"},
                {"catalog_assumption", "stable_native_enumeration_order"},
                {"comparison", "native_case_insensitive_locale_dependent"},
                {"fallback_loads", Json::array()},
                {"display_comparisons", Json::array()}};
    const auto query = name.substr(0, name.find(u'\0'));
    out["query_utf16_code_units"] = std::vector<std::uint16_t>(query.begin(), query.end());
    auto primary = lookup_native_material_name(query, context.catalog);
    out["catalog_name_lookup"] = primary;
    if (primary.at("status") == "loaded") {
        out["status"] = "matched";
        out["matched_by"] = "catalog_name";
        out["material_index"] = primary.at("first_material_index");
        out["catalog_index"] = primary.at("first_catalog_index");
        return out;
    }
    if (primary.at("status") != "not_found" && primary.at("status") != "load_failed") {
        out["reason"] = "primary_name_lookup_unresolved";
        return out;
    }
    const char *phase = "full_catalog_loading";
    std::size_t index = 0;
    try {
        require(context.catalog.catalog_complete, "complete_prepared_material_catalog_required");
        std::vector<std::pair<std::size_t, std::size_t>> loaded;
        // Enumeration loads every descriptor before checking any display name.
        // Do not memoize failures from the first query or merge repeated identities.
        for (; index < context.catalog.names.size(); ++index) {
            require(bool(context.catalog.load), "native_material_loader_required");
            const auto result = context.catalog.load(index);
            require(result.known || !result.material_index, "unknown_material_has_identity");
            require(result.known, "native_material_load_unknown");
            Json step = {{"catalog_index", index},
                         {"status", result.material_index ? "loaded" : "not_loaded"}};
            if (result.material_index) {
                step["material_index"] = *result.material_index;
                loaded.emplace_back(index, *result.material_index);
            }
            out["fallback_loads"].push_back(std::move(step));
        }
        phase = "display_name_comparison";
        for (const auto &entry : loaded) {
            index = entry.first;
            require(bool(context.display_name), "loaded_material_display_name_required");
            const auto display = context.display_name(entry.second);
            require(display.has_value(), "loaded_material_display_name_unknown");
            const auto prefix = display->substr(0, display->find(u'\0'));
            require(bool(context.catalog.compare), "native_material_name_comparator_required");
            const auto compared = context.catalog.compare(query, prefix);
            require(compared.has_value(), "native_material_name_comparison_unknown");
            out["display_comparisons"].push_back(
                {{"catalog_index", index},
                 {"material_index", entry.second},
                 {"display_name_utf16_code_units",
                  std::vector<std::uint16_t>(prefix.begin(), prefix.end())},
                 {"matched", *compared == 0}});
            if (*compared == 0) {
                out["status"] = "matched";
                out["matched_by"] = "display_name";
                out["material_index"] = entry.second;
                out["catalog_index"] = index;
                return out;
            }
        }
        out["status"] = "not_found";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
        out["unresolved_phase"] = phase;
        out["unresolved_catalog_index"] = index;
    }
    return out;
}
} // namespace p3d
