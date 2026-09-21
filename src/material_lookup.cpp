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
} // namespace p3d
