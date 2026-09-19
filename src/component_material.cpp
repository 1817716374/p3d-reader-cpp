#include "internal.hpp"

namespace p3d {
Json resolve_component_material_overrides(const Json &instance,
                                          const ComponentMaterialContext &context) {
    Json out = {{"reader_profile", "bimbase_2025_component_material_overrides"},
                {"scope", "component_graphics_after_extraction_before_entity_part_overrides"},
                {"applies_to", "all_rebuilt_graphics_entries"},
                {"comparison", "case_sensitive_utf16_code_units"},
                {"duplicate_name_rule", "first_loaded_material_wins"},
                {"material_list_complete", context.material_list_complete},
                {"project_selection", "caller_supplied"},
                {"lookups", Json::array()},
                {"status", "preserve_entry_materials"}};
    // Native getMaterialList inserts loaded materials into a case-sensitive
    // PString map without replacing an existing name. Retain source indices.
    std::map<std::u16string, std::size_t> names;
    for (std::size_t i = 0; i < context.material_names.size(); ++i)
        names.emplace(context.material_names[i], i);

    auto lookup = [&](const Bytes &bytes) {
        Json step = {{"source_bytes", rawbytes(bytes)}, {"status", "unresolved"}};
        const auto end = std::find(bytes.begin(), bytes.end(), std::uint8_t(0));
        const Bytes prefix(bytes.begin(), end);
        step["lookup_name_bytes"] = rawbytes(prefix);
        std::u16string name;
        if (context.ansi_decoder) {
            name = context.ansi_decoder(prefix);
            step["text_decoder"] = "caller_supplied";
        } else if (std::all_of(prefix.begin(), prefix.end(), [](auto c) { return c < 128; })) {
            name.assign(prefix.begin(), prefix.end());
            step["text_decoder"] = "ascii_subset";
        } else {
            step["reason"] = "requires_ansi_decoder";
            return step;
        }
        step["lookup_name_utf16_code_units"] = std::vector<std::uint16_t>(name.begin(), name.end());
        // Empty names can match in this path. Do not reuse the native element
        // linkage reader's nonempty, case-insensitive lookup policy.
        auto it = names.find(name);
        if (it != names.end()) {
            step["status"] = "matched";
            step["material_index"] = it->second;
        } else if (context.material_list_complete) {
            step["status"] = "not_found";
        } else {
            step["reason"] = "material_list_incomplete";
        }
        return step;
    };
    auto apply = [&](Json step, const char *source) {
        step["source"] = source;
        const auto status = step.at("status");
        if (status == "matched") {
            out["status"] = "override_selected";
            out["material_index"] = step.at("material_index");
            out["lookup_index"] = out["lookups"].size();
        } else if (status == "unresolved") {
            out["status"] = "unresolved";
            out.erase("material_index");
            out.erase("lookup_index");
        }
        // Absent/missing overrides preserve the previous stage, including its
        // uncertainty. A confirmed later hit overrides even an unknown inner stage.
        out["lookups"].push_back(std::move(step));
    };
    Json inner;
    try {
        if (!context.noumenon_material_known)
            inner = {{"status", "unresolved"}, {"reason", "noumenon_material_context_unavailable"}};
        else if (!context.noumenon_material_name)
            inner = {{"status", "absent"}};
        else
            inner = lookup(*context.noumenon_material_name);
    } catch (const std::exception &e) {
        inner = {{"status", "unresolved"}, {"reason", e.what()}};
    }
    apply(std::move(inner), "noumenon_component_material");
    Json outer;
    try {
        const auto &reference = instance.at("footer").at("material_name_reference");
        // Re-read the preserved source bytes rather than trusting a display name
        // or a previously decoded name from a different Windows code page.
        outer = lookup(bytesof(reference.at("source_bytes")));
    } catch (const std::exception &e) {
        outer = {{"status", "unresolved"}, {"reason", e.what()}};
    }
    apply(std::move(outer), "component_data_footer");
    return out;
}
} // namespace p3d
