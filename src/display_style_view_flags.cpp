#include "display_style_xml.hpp"

namespace p3d {
std::array<std::uint32_t, 2> apply_display_style_flag_words(
    std::uint32_t first, std::uint32_t second, std::uint32_t flags48,
    std::uint32_t flags50, std::uint64_t material) {
    // Original R1.18 P3DKJ 0xdaa40. All shifts are DWORD operations.
    first = (first & ~0x1f800000u) | ((flags48 << 23) & 0x1f800000u);
    second = (second & ~0x7a026010u) | ((flags48 << 7) & 0x6000u) |
             ((flags48 << 19) & 0x38000000u) | ((flags48 << 14) & 0x02000000u) |
             ((flags48 << 4) & 0x20000u) | ((~(flags48 >> 12)) & 0x10u);
    if ((flags50 & 0x40u) && material == 0) second |= 0x40000000u;
    return {first, second};
}

Json project_display_style_view_flags(const Json &record, const Json &imported) {
    Json out = {{"status", "unresolved"}, {"scope", "R1.18_type11_initial_view_flag_writes"},
                {"runtime_application", "not_evaluated"}};
    try {
        require(record.at("element_type") == 11, "type11_view_input_required");
        const auto bytes = bytesof(record.at("data"));
        require(bytes.size() >= 0x123, "truncated_type11_constructor_input");
        require(Reader(bytes, 16).u32() == 1, "type11_subtype1_input_required");
        require(imported.at("status") == "partial", "accepted_style_xml_input_required");
        const auto flags48 = imported.at("packed_flags_at_48").get<std::uint32_t>();
        const auto flags50 = imported.at("packed_flags_at_50").get<std::uint32_t>();
        const auto material = imported.at("fields").at("Overrides.Material").at("value").get<std::uint64_t>();
        // Native header starts after the stream's 4-byte prefix. 4b47c9
        // copies header+f8 to object+10, then 4b4887 clamps mode values >7.
        const auto first = Reader(bytes, 0xfc).u32(), second = Reader(bytes, 0x100).u32();
        const auto normalized = (first & 0x1f800000u) > 0x03800000u ? first & ~0x1f800000u : first;
        const auto projected = apply_display_style_flag_words(normalized, second, flags48, flags50, material);
        out.update({{"status", "conditional"},
                    {"conditions", Json::array({"fresh_type11_object_uses_R1.18_4b46b0_reader",
                        "selected_style_object_matches_imported_fields",
                        "R1.18_daa40_applied_after_constructor_without_intervening_flag_changes"})},
                    {"source_offsets_include_stream_prefix", true},
                    {"source_offsets", Json::array({0xfc, 0x100})},
                    {"source_words", Json::array({first, second})},
                    {"constructor_words", Json::array({normalized, second})},
                    {"style_application_words", Json::array({projected[0], projected[1]})},
                    {"style_write_masks", Json::array({0x1f800000u, 0x7a026010u})},
                    {"unmodeled_steps", Json::array({"view_table_selection_and_object_dispatch",
                        "background_color_resource_resolution", "remaining_display_override_application"})}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
