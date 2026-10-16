#include "internal.hpp"
#include <p3d/color_resources.hpp>

namespace p3d {
Json prepare_native_draw_colors(const Bytes &source, const NativeDrawColorContext &context) {
    Json out = {{"status", "unresolved"},
                {"scope", "ordinary_draw_color_conversion_after_inheritance"},
                {"source_state", rawbytes(source)},
                {"prepared_state", nullptr},
                {"channels", Json::array()},
                {"owner_selection", "caller_supplied_actual_loaded_context"},
                {"material_assignment", "not_evaluated"}};
    try {
        require(source.size() >= 56, "ordinary_draw_style_color_prefix_truncated");
        Bytes result = source;
        bool complete = true;
        auto packed_rgb = [](const NativeRgb &rgb) {
            return std::uint32_t(rgb[0]) | (std::uint32_t(rgb[1]) << 8) |
                   (std::uint32_t(rgb[2]) << 16);
        };
        for (unsigned channel = 0; channel < 2; ++channel) {
            const auto id_offset = channel ? 44u : 32u;
            const auto rgb_offset = channel ? 52u : 40u;
            const auto flag = channel ? 8u : 4u;
            const auto id = Reader(source).at<std::uint32_t>(id_offset);
            const auto saved_rgb = Reader(source).at<std::uint32_t>(rgb_offset);
            Json item = {{"channel", channel == 0 ? "primary" : "secondary"},
                         {"source_color_id", id},
                         {"color_id_offset", id_offset},
                         {"packed_rgb_offset", rgb_offset},
                         {"source_packed_rgb", saved_rgb},
                         {"status", "unresolved"}};
            try {
                std::optional<std::uint32_t> value;
                if (source[1] & flag) {
                    value = saved_rgb;
                    item["selection"] = "existing_resolved_color";
                } else if (id == 0xffffff00u) {
                    item.update({{"status", "unchanged"},
                                 {"selection", "native_color_conversion_bypass"},
                                 {"resolved_bit", false}});
                } else {
                    const auto low = id & 255u;
                    item["palette_index"] = low;
                    if (id >= 256 && id <= 0xfffffffdu) {
                        // The native outer range check precedes the twenty-bit
                        // mask. Masked zero is an extended-table miss, not the
                        // special marker and not an owner-palette lookup.
                        const auto number = (id >> 8) & 0xfffffu;
                        item["extended_color_id"] = number;
                        require(context.extended_colors.has_value(),
                                "actual_loaded_owner_extended_colors_required");
                        if (number && number <= context.extended_colors->size()) {
                            value = packed_rgb(context.extended_colors->at(number - 1));
                            item["selection"] = "owner_extended_color";
                            item["extended_ordinal"] = number - 1;
                        } else
                            item["extended_lookup"] = "not_found";
                    }
                    if (!value) {
                        require(!context.draw_palette || !context.draw_palette_confirmed_absent,
                                "contradictory_draw_palette_context");
                        require(context.draw_palette || context.draw_palette_confirmed_absent,
                                "actual_draw_context_palette_required");
                        value = context.draw_palette ? context.draw_palette->at(low) : 0;
                        item["selection"] = context.draw_palette ? "draw_context_palette"
                                                                 : "confirmed_null_draw_palette";
                    }
                    for (unsigned byte = 0; byte < 4; ++byte)
                        result[rgb_offset + byte] = std::uint8_t(*value >> (byte * 8));
                    result[1] |= std::uint8_t(flag);
                }
                if (value)
                    item.update(
                        {{"status", "resolved"},
                         {"packed_rgb", *value},
                         {"rgb", {*value & 255u, (*value >> 8) & 255u, (*value >> 16) & 255u}},
                         {"unassigned_high_byte", *value >> 24},
                         {"resolved_bit", true}});
            } catch (const std::exception &e) {
                complete = false;
                item["reason"] = e.what();
            }
            out["channels"].push_back(std::move(item));
        }
        out["partial_state"] = rawbytes(result);
        out["status"] = complete ? "prepared" : "partial";
        if (complete)
            out["prepared_state"] = rawbytes(result);
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
