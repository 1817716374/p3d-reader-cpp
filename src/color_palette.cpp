#include "color_palette_source.hpp"
#include <p3d/color_resources.hpp>

namespace p3d {
namespace {
NativeColorPalette rotate_input(const NativeColorPalette &input) {
    NativeColorPalette result;
    std::copy(input.begin() + 1, input.end(), result.begin());
    result.back() = input.front();
    return result;
}
} // namespace
const NativeColorPalette &native_module_default_palette() {
    static const auto palette = [] {
        const std::array<NativeRgb, 8> base{{{0, 255, 255},
                                             {255, 255, 255},
                                             {0, 0, 255},
                                             {0, 255, 0},
                                             {255, 0, 0},
                                             {255, 255, 0},
                                             {255, 0, 255},
                                             {255, 127, 0}}};
        const std::array<NativeRgb, 8> increment{{{0, 241, 241},
                                                  {241, 241, 241},
                                                  {0, 0, 241},
                                                  {0, 241, 0},
                                                  {241, 0, 0},
                                                  {241, 241, 0},
                                                  {241, 0, 241},
                                                  {241, 251, 0}}};
        NativeColorPalette input;
        for (std::size_t i = 0; i < 16; ++i)
            input[i] = base[i % 8];
        for (std::size_t i = 16; i < input.size(); ++i)
            for (unsigned component = 0; component < 3; ++component)
                input[i][component] =
                    std::uint8_t(unsigned(input[i - 16][component]) + increment[i % 8][component]);
        input[0] = {0, 0, 0};
        input[9] = {64, 64, 64};
        input[10] = {192, 192, 192};
        input[11] = {254, 0, 96};
        input[12] = {160, 224, 0};
        input[13] = {0, 254, 160};
        input[14] = {128, 0, 160};
        input[15] = {176, 176, 176};
        input[255] = {192, 192, 192};
        return rotate_input(input);
    }();
    return palette;
}
const NativeColorPalette &native_fixed_color_palette() {
    static const auto palette = [] {
        NativeColorPalette result;
        const std::array<NativeRgb, 10> first{{{255, 255, 255},
                                               {255, 0, 0},
                                               {255, 255, 0},
                                               {0, 255, 0},
                                               {0, 255, 255},
                                               {0, 0, 255},
                                               {255, 0, 255},
                                               {255, 255, 255},
                                               {128, 128, 128},
                                               {192, 192, 192}}};
        std::copy(first.begin(), first.end(), result.begin());
        const std::array<unsigned, 5> levels{{255, 204, 153, 127, 76}};
        for (unsigned hue = 0; hue < 24; ++hue) {
            const unsigned sector = hue / 4, q = hue % 4;
            const std::array<std::array<unsigned, 3>, 6> quarters{
                {{4, q, 0}, {4 - q, 4, 0}, {0, 4, q}, {0, 4 - q, 4}, {q, 0, 4}, {4, 0, 4 - q}}};
            for (unsigned level = 0; level < levels.size(); ++level)
                for (unsigned component = 0; component < 3; ++component) {
                    const auto amount = quarters[sector][component], value = levels[level];
                    result[10 + hue * 10 + level * 2][component] = std::uint8_t(value * amount / 4);
                    result[11 + hue * 10 + level * 2][component] =
                        std::uint8_t(value * (4 + amount) / 8);
                }
        }
        // Preserve the stored table's exceptional value, not an idealized hue ramp.
        result[68] = {51, 76, 0};
        const std::array<std::uint8_t, 6> gray{{51, 91, 132, 173, 214, 255}};
        for (unsigned i = 0; i < gray.size(); ++i)
            result[250 + i] = {gray[i], gray[i], gray[i]};
        return result;
    }();
    return palette;
}
Json native_initial_palette_selection(const Bytes &payload, std::uint16_t flags) {
    Json result = {{"status", "unresolved"},
                   {"scope", "initial_header_empty_owner_palette_cache"},
                   {"fallback_source_offset", 0x11c},
                   {"override_source_offset", 0x314},
                   {"source_fallback", nullptr},
                   {"source_override", nullptr},
                   {"selector", nullptr}};
    Reader reader(payload);
    if (payload.size() >= 0x120)
        result["source_fallback"] = reader.at<std::uint32_t>(0x11c);
    if (payload.size() >= 0x318)
        result["source_override"] = reader.at<std::uint32_t>(0x314);
    if (flags & 2) {
        result.update({{"status", "resolved"},
                       {"fallback", 0},
                       {"override", 0},
                       {"selector", 0},
                       {"selection", "cleared_initial_header"},
                       {"palette_source", "module_palette"}});
    } else if (payload.size() >= 0x610) {
        const auto override = reader.at<std::uint32_t>(0x314),
                   fallback = reader.at<std::uint32_t>(0x11c);
        const auto selector = override ? override : fallback;
        result.update({{"status", "resolved"},
                       {"fallback", fallback},
                       {"override", override},
                       {"selector", selector},
                       {"selection", override ? "nonzero_header_override" : "header_fallback"},
                       {"palette_source",
                        selector == 3 || selector == 4 ? "fixed_palette" : "module_palette"}});
    } else
        result["reason"] = "incomplete_initial_file_header";
    return result;
}
Json prepare_native_file_palette(const Json &header, const NativePaletteSelectionContext &context) {
    Json out = {{"status", "unresolved"},
                {"scope", "initial_header_empty_owner_palette_cache"},
                {"rgb", nullptr},
                {"runtime_callbacks", "not_performed"}};
    try {
        const auto control = bytesof(header.at("decoded_control_header"));
        require(control.size() == 20, "native_file_control_header_required");
        const auto flags = Reader(control).u16();
        const auto selection =
            native_initial_palette_selection(bytesof(header.at("header_payload")), flags);
        out["selection"] = selection;
        require(selection.at("status") == "resolved", "initial_palette_selection_unresolved");
        if (selection.at("palette_source") == "fixed_palette") {
            out.update({{"status", "resolved"},
                        {"source", "fixed_palette"},
                        {"rgb", native_fixed_color_palette()}});
        } else if (context.current_module_palette) {
            out.update({{"status", "resolved"},
                        {"source", "caller_supplied_current_module_palette"},
                        {"rgb", *context.current_module_palette}});
        } else if (context.allow_unmodified_module_default) {
            out.update({{"status", "resolved"},
                        {"source", "unmodified_module_default_palette"},
                        {"rgb", native_module_default_palette()}});
        } else
            out["reason"] = "current_module_palette_or_explicit_initial_default_required";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
