#include "internal.hpp"
#include "text_bytes.hpp"

unsigned native_text_character_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto input = [](const Json &units) {
        return Json{{"text_conversion", {{"status", "decoded"}, {"source_units", units}}}};
    };
    NativeTextCharacterContext context;
    auto convert = [&](const Json &units) {
        return map_native_text_characters(input(units), context);
    };
    auto result = convert(Json::array());
    check(result.at("status") == "resolved" && result.at("text") == "" &&
              result.at("consumed_units") == 0,
          "empty native string does not require font metadata");
    result = convert({0, 65536, "unread"});
    check(result.at("status") == "resolved" && result.at("terminated_by_zero") == true &&
              result.at("ignored_source_units") == 2,
          "source terminator stops before font query and before unused trailing units");
    check(convert({65}).at("reason") == "native_font_character_mode_unknown",
          "unknown mode is not silently treated as Unicode");
    context.mode = NativeFontCharacterMode::TrueType;
    context.symbol_codes = std::array<std::uint16_t, 3>{65, 66, 67};
    context.decode = [](const Bytes &, std::uint32_t) -> std::optional<std::u16string> {
        throw std::runtime_error("Unicode branch must not query codepage");
    };
    result = convert({65, 66, 67, 0x4e2d, 0xd83d, 0xde00, 0, 88});
    check(result.at("status") == "resolved" && result.at("text") == u8"ABC中\U0001f600" &&
              result.at("consumed_units") == 6 && result.at("ignored_source_units") == 1 &&
              result.at("glyph_status") == "not_evaluated",
          "TrueType passes all units through before strict Unicode rendering");
    context.mode = NativeFontCharacterMode::ShxUnicode;
    check(convert({0xd6d0}).at("units") == Json::array({0xd6d0}),
          "Unicode SHX bypasses legacy symbol and codepage interpretation");
    context.mode = NativeFontCharacterMode::ShxLegacy;
    result = convert({65, 66, 67});
    check(result.at("units") == Json::array({0xb0, 0x2205, 0xb1}) && result.at("text") == u8"°∅±",
          "configured symbols override even ASCII and use native diameter codepoint");
    context.symbol_codes = std::array<std::uint16_t, 3>{65, 65, 65};
    check(convert({65}).at("units") == Json::array({0xb0}),
          "degree match has priority when configured symbols collide");
    context.symbol_codes = std::array<std::uint16_t, 3>{0, 65, 65};
    check(convert({65}).at("units") == Json::array({0xb1}),
          "plus-minus match precedes diameter for a shared configured code");
    context.symbol_codes.reset();
    check(convert({65}).at("reason") == "native_font_symbol_codes_unknown",
          "ASCII cannot bypass an unknown font symbol configuration");
    context.symbol_codes = std::array<std::uint16_t, 3>{0, 0, 0};
    Json byte_units = Json::array();
    for (unsigned unit = 1; unit < 255; ++unit)
        byte_units.push_back(unit);
    check(convert(byte_units).at("units") == byte_units,
          "legacy SHX preserves every non-symbol unit strictly below 255");
    check(convert({255}).at("reason") == "native_font_codepage_unknown",
          "255 itself requires codepage conversion rather than SHX byte identity");
    context.codepage = 936;
    context.decode = {};
    check(convert({255}).at("reason") == "native_codepage_decoder_required",
          "known codepage does not invent a decoder or use GB18030 implicitly");
    std::vector<Bytes> requests;
    context.decode = [&](const Bytes &bytes, std::uint32_t cp) -> std::optional<std::u16string> {
        require(cp == 936, "source codepage passed unchanged");
        requests.push_back(bytes);
        if (bytes == Bytes{0xd6, 0xd0})
            return std::u16string(u"中");
        if (bytes == Bytes{0xff})
            return std::u16string(u"\ufffd");
        if (bytes == Bytes{0x01})
            return std::u16string(u"Qextra");
        return std::u16string{};
    };
    result = convert({0xd6d0, 255, 0x100});
    check(result.at("units") == Json::array({0x4e2d, 0xfffd, 81}) &&
              requests == std::vector<Bytes>({{0xd6, 0xd0}, {0xff}, {1}}),
          "conversion sends high byte first and stops at embedded NUL before decoding");
    check(result.at("steps")[2].at("decoder_unit_count") == 6 &&
              result.at("steps")[1].at("rule") == "codepage_first_unit",
          "only first decoded unit is used and replacement character is accepted");
    for (const auto &decoded :
         {std::u16string{}, std::u16string(1, 0), std::u16string(1, 0xffff)}) {
        context.decode = [decoded](const Bytes &, std::uint32_t) { return std::optional(decoded); };
        result = convert({0xd6d0});
        check(result.at("mapping_status") == "resolved" &&
                  result.at("units") == Json::array({0xd6d0}) &&
                  result.at("steps")[0].at("rule") == "codepage_preserve_source",
              "known empty zero or FFFF decode result preserves original native code");
    }
    context.decode = [](const Bytes &, std::uint32_t) { return std::optional<std::u16string>{}; };
    result = convert({65, 0xd6d0});
    check(result.at("status") == "unresolved" && result.at("text").is_null() &&
              result.at("units") == Json::array({65}) && result.at("unresolved_source_index") == 1,
          "unknown conversion retains known prefix without publishing a complete string");
    context.decode = [](const Bytes &, std::uint32_t) -> std::optional<std::u16string> {
        throw std::runtime_error("decoder failure");
    };
    check(convert({0xd6d0}).at("reason") == "decoder failure",
          "decoder exception is not treated as a known conversion failure");
    context.decode = [](const Bytes &, std::uint32_t) {
        return std::optional<std::u16string>(u"\U0001f600");
    };
    result = convert({0xd6d0});
    check(result.at("mapping_status") == "resolved" && result.at("status") == "partial" &&
              result.at("text_status") == "invalid_utf16" &&
              result.at("units") == Json::array({0xd83d}) && result.at("text").is_null(),
          "native first-unit behavior is retained even when it leaves a lone surrogate");
    context.mode = NativeFontCharacterMode::TrueType;
    for (const Json &unit : std::vector<Json>{-1, 65536, 1.5, true, "65"})
        check(convert(Json::array({unit})).at("reason") == "native_text_source_unit_out_of_range",
              "malformed external source units cannot wrap or narrow silently");
    for (unsigned unit : {0xd800u, 0xdc00u}) {
        result = convert({unit});
        check(result.at("status") == "partial" && result.at("text_status") == "invalid_utf16" &&
                  result.at("units") == Json::array({unit}),
              "both lone surrogate halves retain native units without invalid UTF8 JSON");
    }
    auto bad = input({65});
    bad["text_conversion"]["units_status"] = "invalid";
    check(map_native_text_characters(bad, context).at("status") == "unresolved",
          "invalid source extraction cannot be overridden by an old preview status");
    // A failed compatibility preview must not block valid source-unit mapping.
    Bytes native(210);
    auto write = [&](std::size_t offset, unsigned value, unsigned size) {
        for (unsigned i = 0; i < size; ++i)
            native[offset + i] = std::uint8_t(value >> (8 * i));
    };
    write(4, 54, 2);
    write(114, 4, 2);
    native[206] = 0xff;
    native[207] = 0xfd;
    native[208] = 0xff;
    native[209] = 0;
    auto text = decode_native_text_record(native);
    check(text.at("text_conversion").at("units_status") == "decoded" &&
              text.at("text_conversion").at("source_units") == Json::array({255}),
          "packed font code extraction is independent of compatibility preview decoding");
    context.mode = NativeFontCharacterMode::ShxLegacy;
    context.decode = [](const Bytes &, std::uint32_t) {
        return std::optional<std::u16string>(u"ÿ");
    };
    result = map_native_text_characters(text, context);
    check(result.at("status") == "resolved" && result.at("text") == u8"ÿ",
          "font context can decode a code rejected by the older GB18030 preview");
    native[207] = 0xfe;
    native[208] = 0;
    native[209] = 0xdc;
    text = decode_native_text_record(native);
    check(text.at("text_conversion").at("units_status") == "decoded" && text.at("text").is_null(),
          "native preview cannot emit a lone low surrogate as invalid UTF8");
    static_cast<void>(text.dump());
    return checks;
}
