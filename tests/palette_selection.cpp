#include "internal.hpp"
#include "palette_selection_oracle.hpp"
#include <p3d/color_registration.hpp>

unsigned palette_selection_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) { ++checks; require(ok, message); };
    std::string fixture;
    for (const auto *part : palette_selection_oracle_parts) fixture += part;
    const auto oracle = Json::parse(fixture);
    const auto &palettes = oracle.at("palettes");
    for (const auto &row : oracle.at("queries")) {
        NativePaletteSelectionContext context;
        context.file_cache_known = context.global_cache_known = true;
        const auto mode = row.at("mode").get<std::string>();
        if (mode == "fresh") context.host_profile = NativePaletteHostProfile::BuiltinDefaultService;
        else if (mode == "global_custom") context.global_cache = palettes.at(mode).get<NativeColorPalette>();
        else context.file_cache = palettes.at(mode).get<NativeColorPalette>();
        auto selected = select_native_color_palette(row.at("base").get<std::uint32_t>(),
                                                     row.at("override").get<std::uint32_t>(), context);
        check(selected.palette && Json(*selected.palette) == palettes.at(row.at("palette").get<std::string>()) &&
                  selected.file_cache_created == row.at("file_created").get<bool>() &&
                  selected.global_cache_created == row.at("global_created").get<bool>(),
              "palette RGB and cache creation match original DLL across selectors and populated caches");
        const auto expected_source = mode == "file_custom" ? NativePaletteSource::FileCache :
            row.at("palette") == "builtin" ? NativePaletteSource::Builtin :
            mode == "global_custom" ? NativePaletteSource::GlobalCache : NativePaletteSource::DefaultService;
        check(selected.source == expected_source &&
                  (mode == "file_custom" ? !selected.selector : selected.selector.has_value()),
              "native precedence distinguishes cached file, builtin, global and default service sources");
        context.file_cache = selected.palette;
        if (selected.global_cache_created) context.global_cache = selected.palette;
        const auto cached = select_native_color_palette(2, 4, context);
        check(cached.palette == selected.palette && cached.source == NativePaletteSource::FileCache &&
                  !cached.file_cache_created && !cached.global_cache_created && !cached.selector,
              "cached native file palette survives header changes without consulting selector or host");
        const auto previous_global = context.global_cache;
        (*selected.palette)[0][0] ^= 0xff;
        check(context.global_cache == previous_global && cached.palette == context.file_cache,
              "projected file palette is independent of supplied global and file caches");
    }
    NativePaletteSelectionContext context;
    context.host_profile = NativePaletteHostProfile::BuiltinDefaultService;
    check(!select_native_color_palette(2, 3, context).palette,
          "unknown file cache prevents inferring builtin from header alone");
    context.file_cache_known = true;
    check(!select_native_color_palette(2, 0, context).palette,
          "known default service cannot replace an unknown existing global cache");
    check(select_native_color_palette(std::nullopt, 3, context).source == NativePaletteSource::Builtin,
          "nonzero builtin override needs neither base selector nor global cache nor host");
    check(!select_native_color_palette(3, std::nullopt, context).palette &&
              !select_native_color_palette(std::nullopt, 0, context).palette,
          "missing relevant header fields stay unresolved");
    context.global_cache_known = true;
    context.host_profile = NativePaletteHostProfile::UnspecifiedHost;
    check(!select_native_color_palette(2, 0, context).palette,
          "fresh general selector needs established host palette callback");
    context.global_cache = palettes.at("global_custom").get<NativeColorPalette>();
    check(select_native_color_palette(2, 0, context).palette == context.global_cache,
          "existing global palette bypasses unknown callback");
    context.file_cache = palettes.at("file_custom").get<NativeColorPalette>();
    context.global_cache_known = false;
    check(select_native_color_palette(std::nullopt, std::nullopt, context).palette == context.file_cache,
          "known populated file cache does not require any header or global state");
    for (auto profile : {NativePaletteHostProfile::UnspecifiedHost, NativePaletteHostProfile::BuiltinDefaultService}) {
        for (std::uint32_t selector : {0U,2U,3U,4U,0xffffffffU}) {
            Json header = {{"status", "resolved"}, {"initial_probe", {{"action", "read_header_payload"}}},
                           {"source_palette_base_selector", selector}, {"source_palette_override_selector", 0}};
            const auto result = initial_native_color_palette(header, profile);
            const bool builtin = selector == 3 || selector == 4;
            const bool known = builtin || profile == NativePaletteHostProfile::BuiltinDefaultService;
            check(known ? result.at("status") == "resolved" && result.at("rgb") == palettes.at(builtin ? "builtin" : "pre_host")
                        : result.at("status") == "unresolved" && !result.contains("rgb"),
                  "initial header projection only publishes palette under established callback profile");
            header["initial_probe"]["action"] = "initialize_blank_header";
            check(initial_native_color_palette(header, profile).at("status") == "unresolved",
                  "blank header never reuses ignored source palette fields");
        }
    }
    return checks;
}
