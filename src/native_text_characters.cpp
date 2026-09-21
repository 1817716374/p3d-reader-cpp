#include "internal.hpp"

namespace p3d {
Json map_native_text_characters(const Json &native_text,
                                const NativeTextCharacterContext &context) {
    Json out = {
        {"status", "unresolved"},          {"scope", "native_truetype_shx_character_mapping"},
        {"mapping_status", "unresolved"},  {"text_status", "not_evaluated"},
        {"glyph_status", "not_evaluated"}, {"units", Json::array()},
        {"steps", Json::array()},          {"text", nullptr}};
    std::size_t index = 0;
    try {
        const auto &conversion = native_text.at("text_conversion");
        const auto &units_status = conversion.contains("units_status")
                                       ? conversion.at("units_status")
                                       : conversion.at("status");
        require(units_status == "decoded", "decoded_native_text_units_required");
        const auto &units = conversion.at("source_units");
        require(units.is_array(), "native_text_source_units_array_required");
        out["source_unit_count"] = units.size();
        bool terminated = false;
        Bytes unicode;
        for (; index < units.size(); ++index) {
            const auto &source = units[index];
            require(source.is_number_integer() &&
                        (!source.is_number_unsigned() ? source.get<std::int64_t>() >= 0 : true) &&
                        source.get<std::uint64_t>() <= 0xffffu,
                    "native_text_source_unit_out_of_range");
            const auto unit = source.get<std::uint16_t>();
            if (unit == 0) {
                terminated = true;
                break; // Font metadata and trailing source data are not consulted.
            }
            Json step = {{"source_index", index}, {"source_unit", unit}};
            std::uint16_t mapped = unit;
            if (context.mode == NativeFontCharacterMode::TrueType ||
                context.mode == NativeFontCharacterMode::ShxUnicode) {
                step["rule"] = "unicode_identity";
            } else {
                require(context.mode == NativeFontCharacterMode::ShxLegacy,
                        "native_font_character_mode_unknown");
                require(context.symbol_codes.has_value(), "native_font_symbol_codes_unknown");
                const auto &symbols = *context.symbol_codes;
                // This order matters if a font configuration reuses a code.
                if (unit == symbols[0]) {
                    mapped = 0xb0;
                    step["rule"] = "degree_symbol";
                } else if (unit == symbols[2]) {
                    mapped = 0xb1;
                    step["rule"] = "plus_minus_symbol";
                } else if (unit == symbols[1]) {
                    mapped = 0x2205;
                    step["rule"] = "diameter_symbol";
                } else if (unit < 255) {
                    step["rule"] = "shx_byte_identity";
                } else {
                    require(context.codepage.has_value(), "native_font_codepage_unknown");
                    require(bool(context.decode), "native_codepage_decoder_required");
                    Bytes bytes;
                    if (unit > 255)
                        bytes.push_back(std::uint8_t(unit >> 8));
                    if (unit & 255)
                        bytes.push_back(std::uint8_t(unit));
                    // Legacy font objects in the supported native class set do
                    // not enable the base class's fractional-character branch.
                    const auto decoded = context.decode(bytes, *context.codepage);
                    require(decoded.has_value(), "native_codepage_conversion_unknown");
                    step["codepage"] = *context.codepage;
                    step["codepage_input_hex"] = hex(bytes);
                    step["decoder_unit_count"] = decoded->size();
                    if (!decoded->empty() && decoded->front() != 0 && decoded->front() != 0xffff) {
                        mapped = decoded->front();
                        step["rule"] = "codepage_first_unit";
                    } else {
                        step["rule"] = "codepage_preserve_source";
                    }
                }
            }
            step["mapped_unit"] = mapped;
            out["steps"].push_back(std::move(step));
            out["units"].push_back(mapped);
            unicode.push_back(std::uint8_t(mapped));
            unicode.push_back(std::uint8_t(mapped >> 8));
        }
        out["consumed_units"] = index;
        out["terminated_by_zero"] = terminated;
        out["ignored_source_units"] = units.size() - index - (terminated ? 1 : 0);
        out["mapping_status"] = "resolved";
        try {
            auto text = utf16(unicode);
            // Validate the UTF-8 view as well, rejecting lone low surrogates.
            out["text"] = utf8(Bytes(text.begin(), text.end()));
            out["text_status"] = "decoded";
            out["status"] = "resolved";
        } catch (const std::exception &e) {
            out["text_status"] = "invalid_utf16";
            out["text_error"] = e.what();
            out["status"] = "partial"; // Exact native units remain available.
        }
    } catch (const std::exception &e) {
        out["reason"] = e.what();
        out["unresolved_source_index"] = index;
    }
    return out;
}
} // namespace p3d
