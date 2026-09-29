#pragma once
#include <p3d/color_registration.hpp>

namespace p3d {
// The fourth byte is compared by remapping even though HSV only uses RGB.
using NativePackedPalette = std::array<std::array<std::uint8_t, 4>, 256>;
struct NativeSystemColorContext {
    // Identity of the actual system context, not its file name or source ID.
    // Nonzero identities must be stable and unique for distinct native objects.
    std::uint64_t identity = 0;
    const NativePackedPalette *palette = nullptr; // nullptr means unknown
    NativeExtendedColorCache *extended_colors = nullptr; // nullptr means unknown
};
struct NativeColorRemapCache {
    bool initialized = false;
    std::uint64_t source_identity = 0, target_identity = 0;
    std::array<std::uint8_t, 256> indices{};
    bool palettes_differ = false;
    // Values used to derive the native cached HSV table. Refreshed only when
    // the target context identity changes, not when its palette is edited.
    NativeColorPalette target_hsv_input{};
};
struct NativeColorRemapResult {
    std::optional<std::uint32_t> color_id;
    std::optional<NativeColorRegistration> registration;
    std::string reason;
};
// 1e3380 for resident system contexts (virtual +0x58 returns no model).
// Does not load caches, mutate palettes, persist XML or implement model-context
// filters. nullptr mapping_cache means native caller disables low-byte mapping.
// Unknown dependencies return no color_id, without changing supplied state.
NativeColorRemapResult remap_native_system_color(
    std::uint32_t color_id, const NativeSystemColorContext &source,
    const NativeSystemColorContext &target, NativeColorRemapCache *mapping_cache = nullptr,
    bool bypass_palette_mapping = false);

enum class NativeBackgroundColorSource { Unresolved, Reserved, Palette, Extended, InvalidExtendedSlot };
struct NativeBackgroundColor {
    NativeBackgroundColorSource source = NativeBackgroundColorSource::Unresolved;
    std::optional<NativeRgb> rgb;
    std::string reason;
};
// RGB lookup after remapping in daaf0. Reserved/invalid IDs leave the view
// background unchanged; invalid extended slots never fall back to a palette.
NativeBackgroundColor lookup_native_background_color(
    std::uint32_t color_id, const NativeSystemColorContext &target);
struct NativeBackgroundProjection {
    std::optional<NativeRgb> rgb;
    std::optional<std::uint32_t> remapped_id;
    bool changed = false;
    std::string reason;
};
// daaf0 background branch with a fresh per-call mapping cache. Does not apply
// the separate view-flag projection, refresh a model or update native viewports.
NativeBackgroundProjection project_native_system_background(
    std::uint32_t color_id, bool override_enabled, bool view_present,
    const NativeRgb &initial_rgb, const NativeSystemColorContext &source,
    const NativeSystemColorContext &target);
} // namespace p3d
