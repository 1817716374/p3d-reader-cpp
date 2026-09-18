#include "internal.hpp"

namespace p3d {
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
        out["unresolved_ranges"] =
            Json::array({{{"offset", 0}, {"size", 36}}, {{"offset", 40}, {"size", 20}}});
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
