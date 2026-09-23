#include "internal.hpp"
#include <p3d/material_lookup.hpp>
#include <codecvt>
#include <locale>

namespace p3d {
namespace {
std::uint64_t catalog_id(const Json &value) {
    require(value.is_number_integer() &&
                (value.is_number_unsigned() || value.get<std::int64_t>() >= 0),
            "material_catalog_id_must_be_uint64");
    return value.get<std::uint64_t>();
}
std::u16string context_key(const Json &value) {
    const auto text = value.get<std::string>();
    // Catalog registration rejects embedded NUL in context keys. Requiring the
    // same here prevents edited input from changing the already registered key.
    require(text.find('\0') == std::string::npos, "catalog_context_key_contains_nul");
    Json(text).dump();
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> codec;
    auto wide = codec.from_bytes(text);
    require(codec.converted() == text.size(), "catalog_context_key_utf8_incomplete");
    return wide;
}
} // namespace

Json lookup_native_material_id(std::uint64_t id, const Json &catalog,
                               const NativeMaterialIdContext &context) {
    Json out = {{"status", "unresolved"},
                {"scope", "prepared_native_material_id_index"},
                {"material_id", id},
                {"id_candidate_indices", Json::array()},
                {"matching_catalog_indices", Json::array()},
                {"comparisons", Json::array()},
                {"loads", Json::array()},
                {"native_diagnostic_code", nullptr}};
    auto missing = [&](const char *status, unsigned diagnostic) {
        out["status"] = status;
        out["native_diagnostic_code"] = diagnostic;
    };
    const char *phase = "catalog_selection";
    std::optional<std::size_t> index;
    try {
        // The ID-only native identifier has an empty name. This sentinel is
        // rejected before project/catalog access; ID zero remains valid.
        if (id == UINT64_MAX) {
            missing("invalid_identifier", 1);
            return out;
        }
        const auto status = catalog.at("status");
        if (status == "absent") {
            missing("not_found", 3);
            return out;
        }
        require(status == "resolved", "complete_prepared_material_catalog_required");
        const auto &entries = catalog.at("entries");
        require(entries.is_array(), "material_catalog_entries_must_be_array");
        std::vector<std::size_t> candidates;
        for (std::size_t i = 0; i < entries.size(); ++i) {
            index = i;
            if (catalog_id(entries[i].at("id")) == id) {
                candidates.push_back(i);
                out["id_candidate_indices"].push_back(i);
            }
        }
        index.reset();
        if (candidates.empty()) {
            missing("not_found", 3);
            return out;
        }

        phase = "resource_context_selection";
        bool current_read = false;
        std::optional<std::u16string> current;
        auto current_key = [&]() -> std::optional<std::u16string> {
            if (!current_read) {
                const auto &source = catalog.at("current_context");
                const auto kind = source.at("kind");
                require(kind == "current_resource_context" || kind == "explicit_context_keys",
                        "material_catalog_current_context_unresolved");
                if (kind == "explicit_context_keys")
                    current = context_key(source.at("index_key"));
                current_read = true;
            }
            return current;
        };
        auto resource_key = [&](const Json &resource) -> std::optional<std::u16string> {
            const auto &primary = resource.at("primary_context");
            const auto kind = primary.at("kind");
            if (kind == "current_resource_context")
                return current_key();
            require(kind == "explicit_context_keys", "material_resource_context_unresolved");
            return context_key(primary.at("index_key"));
        };
        auto query = context.resource_index_key;
        if (query)
            query->resize(query->find(u'\0') == std::u16string::npos ? query->size()
                                                                     : query->find(u'\0'));
        else
            query = current_key();
        out["query_context"] =
            query ? Json{{"kind", "explicit_index_key"},
                         {"utf16_code_units",
                          std::vector<std::uint16_t>(query->begin(), query->end())}}
                  : Json{{"kind", "current_resource_context"}};
        std::vector<std::size_t> matches;
        for (auto candidate : candidates) {
            index = candidate;
            const auto key = resource_key(entries[candidate].at("resource"));
            Json comparison = {{"catalog_index", candidate}};
            bool equal = false;
            if (!key && !query) {
                equal = true;
                comparison["method"] = "same_current_resource_context";
            } else {
                require(key && query,
                        "current_resource_index_key_required_for_cross_context_query");
                comparison["source_key_utf16_code_units"] =
                    std::vector<std::uint16_t>(key->begin(), key->end());
                if (*key == *query) {
                    equal = true;
                    comparison["method"] = "identical_index_key";
                } else {
                    require(bool(context.compare), "native_resource_key_comparator_required");
                    const auto compared = context.compare(*key, *query);
                    require(compared.has_value(), "native_resource_key_comparison_unknown");
                    equal = *compared == 0;
                    comparison["method"] = "caller_comparison";
                }
            }
            comparison["matched"] = equal;
            out["comparisons"].push_back(std::move(comparison));
            if (equal) {
                matches.push_back(candidate);
                out["matching_catalog_indices"].push_back(candidate);
            }
        }
        index.reset();
        if (matches.empty()) {
            missing("not_found", 3);
            return out;
        }
        // A registered native ID index contains only one value for this key.
        // Conflicting input/comparison contexts cannot be repaired by first/last
        // wins, by loading every duplicate, or by manufacturing shared materials.
        require(matches.size() == 1, "duplicate_registered_material_id_context");
        const auto selected = matches.front();
        out["selected_catalog_index"] = selected;
        index = selected;
        phase = "candidate_loading";
        require(bool(context.load), "native_material_loader_required");
        const auto loaded = context.load(selected);
        require(loaded.known || !loaded.material_index, "unknown_material_has_identity");
        Json load = {{"catalog_index", selected},
                     {"status", !loaded.known           ? "unresolved"
                                : loaded.material_index ? "loaded"
                                                        : "not_loaded"}};
        if (loaded.material_index)
            load["material_index"] = *loaded.material_index;
        out["loads"].push_back(std::move(load));
        require(loaded.known, "native_material_load_unknown");
        if (loaded.material_index) {
            out["status"] = "loaded";
            out["material_index"] = *loaded.material_index;
            out["native_diagnostic_code"] = 0;
        } else {
            missing("load_failed", 4);
        }
    } catch (const std::exception &e) {
        out["reason"] = e.what();
        out["unresolved_phase"] = phase;
        if (index)
            out["unresolved_catalog_index"] = *index;
    }
    return out;
}
} // namespace p3d
