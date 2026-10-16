#include "internal.hpp"
#include <p3d/color_resources.hpp>

unsigned color_palette_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        require(value, why);
        ++checks;
    };
    const auto &module = native_module_default_palette();
    const std::array<NativeRgb, 15> prefix{{{255, 255, 255},
                                            {0, 0, 255},
                                            {0, 255, 0},
                                            {255, 0, 0},
                                            {255, 255, 0},
                                            {255, 0, 255},
                                            {255, 127, 0},
                                            {0, 255, 255},
                                            {64, 64, 64},
                                            {192, 192, 192},
                                            {254, 0, 96},
                                            {160, 224, 0},
                                            {0, 254, 160},
                                            {128, 0, 160},
                                            {176, 176, 176}}};
    const std::array<NativeRgb, 8> channels{
        {{0, 1, 1}, {1, 1, 1}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}, {1, 0, 1}, {1, 1, 0}}};
    for (unsigned i = 0; i < 256; ++i) {
        NativeRgb expected;
        if (i < prefix.size())
            expected = prefix[i];
        else if (i == 254)
            expected = {192, 192, 192};
        else if (i == 255)
            expected = {0, 0, 0};
        else {
            const auto hue = (i + 1) % 8, tier = (i + 1) / 16;
            for (unsigned c = 0; c < 3; ++c)
                expected[c] = std::uint8_t(channels[hue][c] * (255 - 15 * tier));
            if (hue == 7)
                expected[1] = std::uint8_t(127 - 5 * tier);
        }
        check(module[i] == expected,
              "module palette agrees with independent closed-form color bands and exceptions");
    }
    const auto &fixed = native_fixed_color_palette();
    const std::array<NativeRgb, 10> fixed_prefix{{{255, 255, 255},
                                                  {255, 0, 0},
                                                  {255, 255, 0},
                                                  {0, 255, 0},
                                                  {0, 255, 255},
                                                  {0, 0, 255},
                                                  {255, 0, 255},
                                                  {255, 255, 255},
                                                  {128, 128, 128},
                                                  {192, 192, 192}}};
    const std::array<unsigned, 5> levels{{255, 204, 153, 127, 76}};
    for (unsigned i = 0; i < 256; ++i) {
        NativeRgb expected;
        if (i < 10)
            expected = fixed_prefix[i];
        else if (i >= 250) {
            const std::array<std::uint8_t, 6> gray{{51, 91, 132, 173, 214, 255}};
            expected = {gray[i - 250], gray[i - 250], gray[i - 250]};
        } else {
            const auto n = i - 10, hue = n / 10, level = (n % 10) / 2;
            const double value = double(levels[level]) / 255.0, saturation = n % 2 ? 0.5 : 1.0;
            const double h = double(hue) / 4.0, chroma = value * saturation,
                         cross = chroma * (1 - std::abs(std::fmod(h, 2.0) - 1)),
                         offset = value - chroma;
            std::array<double, 3> color;
            switch (unsigned(h)) {
            case 0:
                color = {chroma, cross, 0};
                break;
            case 1:
                color = {cross, chroma, 0};
                break;
            case 2:
                color = {0, chroma, cross};
                break;
            case 3:
                color = {0, cross, chroma};
                break;
            case 4:
                color = {cross, 0, chroma};
                break;
            default:
                color = {chroma, 0, cross};
                break;
            }
            for (unsigned c = 0; c < 3; ++c)
                expected[c] = std::uint8_t(std::floor((color[c] + offset) * 255.0 + 1e-9));
            if (i == 68)
                expected = {51, 76, 0};
        }
        check(fixed[i] == expected,
              "fixed palette agrees with independent HSV-ramp oracle and stored exception");
    }
    check(module[1] != fixed[1] && module[255] != fixed[255],
          "module and selector-3/4 tables have independent color identities");
    check(default_palette().at("rgb").at(255) == Json::array({0, 0, 0}),
          "legacy summary default uses confirmed native black at index 255");
    auto set = [](Bytes &b, std::size_t offset, std::uint32_t value) {
        std::memcpy(b.data() + offset, &value, 4);
    };
    auto header = [&](std::uint32_t fallback, std::uint32_t override, std::uint16_t flags = 0) {
        Bytes payload(0x610);
        set(payload, 0x11c, fallback);
        set(payload, 0x314, override);
        Bytes index(32);
        std::uint8_t previous = 0x26;
        for (unsigned i = 0; i < 20; ++i) {
            const auto value = i < 2 ? std::uint8_t(flags >> (8 * i)) : 0;
            previous = std::uint8_t(value ^ previous);
            index[12 + i] = previous;
        }
        return native_file_header(index, payload);
    };
    NativePaletteSelectionContext current;
    NativeColorPalette custom;
    custom.fill({7, 8, 9});
    current.current_module_palette = custom;
    current.allow_unmodified_module_default = true;
    const std::array<std::uint32_t, 7> selectors{{0, 1, 2, 3, 4, 5, UINT32_MAX}};
    for (auto fallback : selectors)
        for (auto override : selectors) {
            auto input = header(fallback, override);
            const auto original = input;
            const auto chosen = override ? override : fallback;
            const auto selection = input.at("initial_palette_selection");
            check(selection.at("status") == "resolved" && selection.at("selector") == chosen &&
                      selection.at("fallback") == fallback && selection.at("override") == override,
                  "initial header selection honors nonzero override before fallback for full "
                  "uint32 values");
            auto prepared = prepare_native_file_palette(input, current);
            const auto expected = chosen == 3 || chosen == 4 ? fixed : custom;
            check(prepared.at("status") == "resolved" && prepared.at("rgb") == Json(expected) &&
                      input == original,
                  "fixed selection ignores module replacements; other kinds use supplied current "
                  "module table without source mutation");
            const auto unknown = prepare_native_file_palette(input);
            check(chosen == 3 || chosen == 4
                      ? unknown.at("status") == "resolved"
                      : unknown.at("status") == "unresolved" && unknown.at("rgb").is_null(),
                  "ordinary module branch needs actual context while fixed selectors are "
                  "independently known");
        }
    NativePaletteSelectionContext initial;
    initial.allow_unmodified_module_default = true;
    check(prepare_native_file_palette(header(2, 0), initial).at("rgb") == Json(module),
          "explicit unmodified module profile connects the initial native table");
    auto blank = header(3, 4, 2);
    const auto blank_selection = blank.at("initial_palette_selection");
    check(
        blank_selection.at("source_fallback") == 3 && blank_selection.at("source_override") == 4 &&
            blank_selection.at("selector") == 0 &&
            blank_selection.at("selection") == "cleared_initial_header" &&
            prepare_native_file_palette(blank).at("status") == "unresolved",
        "initial blank-header branch keeps source values but clears both active palette selectors");
    auto forged = header(2, 0);
    forged["initial_palette_selection"] = {{"selector", 3}, {"palette_source", "fixed_palette"}};
    forged["control_flags"] = 2;
    check(prepare_native_file_palette(forged, current).at("rgb") == Json(custom),
          "palette preparation reads header/control bytes rather than replaceable annotations");
    auto short_header = header(3, 4);
    auto payload = bytesof(short_header.at("header_payload"));
    payload.resize(0x318);
    short_header["header_payload"] = rawbytes(payload);
    check(prepare_native_file_palette(short_header, current).at("status") == "unresolved",
          "incomplete full-header load is not accepted merely because selector fields fit");
    auto bad_control = header(2, 0);
    bad_control["decoded_control_header"] = rawbytes(Bytes(19));
    check(prepare_native_file_palette(bad_control, initial).at("status") == "unresolved",
          "palette preparation requires the complete decoded control header");
    const auto prepared = prepare_native_file_palette(header(3, 0));
    NativeExtendedColorCache cache;
    const auto color =
        cache.resolve({{{255, 0, 0}, {}, {}}}, prepared.at("rgb").get<NativeColorPalette>());
    check(color.metadata().at("operations")[0].at("packed_color") == 0x101,
          "prepared fixed palette directly connects to native extended-color resolution");
    check(&native_module_default_palette() == &module && &native_fixed_color_palette() == &fixed,
          "immutable native tables are prepared once and shared without altering source object "
          "reuse");
    return checks;
}
