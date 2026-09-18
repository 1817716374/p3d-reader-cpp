#include "internal.hpp"

namespace p3d {
static Json extension_resources(const Bytes &b) {
    Json resources = Json::array();
    for (unsigned offset : {8u, 16u, 24u, 32u, 56u}) {
        const auto index = Reader(b, offset).i32();
        Json resource = {{"source_offset", offset},
                         {"index", index},
                         {"source_value", Reader(b, offset).u32()},
                         {"lookup_domain", "selected_file_resource_table"},
                         {"context_selection", "not_evaluated"},
                         {"negative_index", index < 0},
                         {"target_resolution", "not_evaluated"},
                         {"slot_role", "unresolved"}};
        if (offset != 56) {
            const auto flags = Reader(b, offset - 4).u32();
            resource["region_slot"] = (offset - 8) / 8;
            resource["source_flags"] = flags;
            resource["flags_source_offset"] = offset - 4;
            resource["included_in_region_mode_query"] = (flags & 1) != 0;
        }
        resources.push_back(std::move(resource));
    }
    return resources;
}

static Json extension_object_references(const Bytes &b) {
    Json references = Json::array();
    for (unsigned offset : {40u, 48u})
        references.push_back({{"source_offset", offset},
                              {"slot", (offset - 40) / 8},
                              {"id", Reader(b, offset).u64()},
                              {"lookup_domain", "connected_model_id_registry"},
                              {"owner_reference_path_indirection", true},
                              {"target_resolution", "not_evaluated"},
                              {"slot_role", "unresolved_clip_object"}});
    return references;
}

Json decode_reference_extension(unsigned group, unsigned key, const Bytes &b, unsigned index) {
    if (key != 20081 || (group != 0 && group != 20117))
        return nullptr;
    const auto expected = group == 0 ? 64u : 12u;
    Json out = {{"encoding", "native_reference_extension"},
                {"status", "partial"},
                {"group", group},
                {"lookup_index_matches", index == 0},
                {"payload_size", b.size()},
                {"expected_payload_size", expected},
                {"loader_status", b.size() == expected ? "accepted_length" : "ignored_length"}};
    if (b.size() != expected)
        return out;
    if (group == 0) {
        const auto id = Reader(b, 60).u32();
        out["scale_provider"] = {{"id", id}, {"source_offset", 60}, {"enabled", (id >> 16) != 0}};
        out["indexed_resources"] = extension_resources(b);
        out["object_references"] = extension_object_references(b);
        out["unresolved_ranges"] = Json::array({{{"offset", 0}, {"size", 8}},
                                                {{"offset", 12}, {"size", 4}},
                                                {{"offset", 20}, {"size", 4}},
                                                {{"offset", 28}, {"size", 4}}});
        out["remaining_semantics"] =
            Json::array({"header_and_region_flags", "resource_slot_roles_and_table_selection",
                         "clip_object_slot_roles_and_target_resolution"});
        // The writer emits zero here; the reader ignores all four bytes,
        // including nonzero values. Do not impose a reserved-zero constraint.
        out["ignored_word"] = {{"source_offset", 36}, {"value", Reader(b, 36).u32()}};
    } else {
        out["auxiliary_state_word"] = {
            {"value", Reader(b, 8).u32()}, {"source_offset", 8}, {"meaning", "unresolved"}};
        out["ignored_prefix"] = {
            {"source_offset", 0}, {"size", 8}, {"data", rawbytes(slice(b, 0, 8))}};
    }
    return out;
}

Json reference_extension_input(const Json &attributes) {
    Json out = {{"profile", "bimbase_2025_reference_extension_load"},
                {"scope", "initial_selected_persisted_attribute_collection"},
                {"status", "not_evaluated"},
                {"target_resolution", "not_evaluated"},
                {"parameter_semantics", "partial"}};
    try {
        require(attributes.is_array(), "complete attribute array required");
        require(attributes.size() <= 65535, "initial_attribute_count_exceeds_native_read_width");
        auto lookup = native_attribute_lookup(attributes);
        require(lookup.at("status") == "resolved",
                "reference extension attribute lookup unresolved");
        out["lookup"] = lookup;
        std::uint32_t provider = 0, auxiliary = 0xfffffffeu;
        bool loaded = false;
        // Only the persisted part of the native constructor defaults. These
        // are not a substitute for later view inheritance or host callbacks.
        Bytes parameter_bytes(64, 0);
        parameter_bytes[4] = parameter_bytes[12] = 1;
        parameter_bytes[28] = 6;
        for (unsigned offset : {8u, 16u, 24u, 32u, 56u})
            std::fill_n(parameter_bytes.begin() + offset, 4, 0xff);
        Json selections = Json::array();
        for (unsigned group : {0u, 20117u}) {
            Json selection = {{"group", group}, {"key", 20081}, {"index", 0}, {"status", "absent"}};
            for (const auto &key : lookup.at("keys")) {
                if (key.at("group") != group || key.at("key") != 20081 || key.at("index") != 0)
                    continue;
                const auto ordinal = key.at("selected_source_ordinal").get<std::size_t>();
                const auto &attribute = attributes.at(ordinal);
                const auto decoded =
                    decode_reference_extension(group, 20081, bytesof(attribute.at("payload")), 0);
                selection["source_ordinal"] = ordinal;
                selection["matching_source_ordinals"] = key.at("matching_source_ordinals");
                selection["decoded"] = decoded;
                const bool accepted = decoded.at("loader_status") == "accepted_length";
                selection["status"] = accepted ? "loaded" : "ignored_length";
                if (accepted && group == 0) {
                    parameter_bytes = bytesof(attribute.at("payload"));
                    provider = decoded.at("scale_provider").at("id").get<std::uint32_t>();
                    loaded = true;
                } else if (accepted) {
                    auxiliary = decoded.at("auxiliary_state_word").at("value").get<std::uint32_t>();
                }
                // Native reads only the selected entry. A wrong length does
                // not cause fallback to another duplicate with a valid length.
                break;
            }
            selections.push_back(std::move(selection));
        }
        out.update({{"status", "decoded"},
                    {"selections", selections},
                    {"parameter_block_loaded", loaded},
                    {"parameter_source", loaded ? "selected_attribute" : "initial_default"},
                    {"indexed_resources", extension_resources(parameter_bytes)},
                    {"object_references", extension_object_references(parameter_bytes)},
                    {"scale_provider",
                     {{"id", provider},
                      {"enabled", (provider >> 16) != 0},
                      {"source", loaded ? "selected_attribute" : "initial_default"}}},
                    {"auxiliary_state_word", auxiliary}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
