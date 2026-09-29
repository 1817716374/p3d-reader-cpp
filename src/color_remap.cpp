#include <p3d/color_remap.hpp>
#include <algorithm>

namespace p3d {
namespace {
bool reserved(std::uint32_t id) {
    return id == 0xffffff00U || id >= 0xfffffffeU;
}
NativeColorPalette rgb_palette(const NativePackedPalette &packed) {
    NativeColorPalette result;
    for (std::size_t i = 0; i < result.size(); ++i)
        result[i] = {packed[i][0], packed[i][1], packed[i][2]};
    return result;
}
}

NativeColorRemapResult remap_native_system_color(
    std::uint32_t color_id, const NativeSystemColorContext &source,
    const NativeSystemColorContext &target, NativeColorRemapCache *mapping_cache,
    bool bypass_palette_mapping) {
    NativeColorRemapResult result;
    result.color_id = color_id;
    if (reserved(color_id)) return result;
    const bool extended = (color_id >> 8) != 0;
    const auto index = color_id & 255;
    NativeExtendedColor entry;
    auto unresolved = [&](const char *reason) {
        result.color_id.reset(); result.reason = reason; return result;
    };
    if (extended) {
        if (!source.extended_colors) return unresolved("source_extended_color_cache_unknown");
        const auto slot = (color_id >> 8) & 0xfffff;
        if (!slot || slot > source.extended_colors->entries.size()) {
            result.reason = "source_extended_slot_out_of_range";
            return result;
        }
        // Copy before any target registration: the contexts may share a cache.
        entry = source.extended_colors->entries[slot - 1];
        if (!target.extended_colors) return unresolved("target_extended_color_cache_unknown");
        if (!target.palette) return unresolved("target_palette_unknown");
    } else if (bypass_palette_mapping) return result;
    bool map = mapping_cache && index < 255;
    if (map) {
        if (!source.identity || !target.identity) return unresolved("system_context_identity_unknown");
        map = source.identity != target.identity;
    }
    if (map) {
        if (!source.palette) return unresolved("source_palette_unknown");
        if (!target.palette) return unresolved("target_palette_unknown");
        auto &cache = *mapping_cache;
        const bool target_changed = !cache.initialized || cache.target_identity != target.identity;
        if (target_changed) cache.target_hsv_input = rgb_palette(*target.palette);
        if (!cache.initialized || cache.source_identity != source.identity || target_changed) {
            cache.indices.fill(255);
            cache.palettes_differ = !std::equal(source.palette->begin(), source.palette->begin()+255,
                                               target.palette->begin());
            cache.source_identity = source.identity;
            cache.target_identity = target.identity;
            cache.initialized = true;
        }
        if (cache.palettes_differ) {
            auto &mapped = cache.indices[index];
            if (mapped == 255) {
                const auto &color = (*source.palette)[index];
                mapped = color == (*target.palette)[index] ? std::uint8_t(index) :
                    nearest_native_palette_color(cache.target_hsv_input, {color[0],color[1],color[2]});
            }
            result.color_id = mapped;
        }
    }
    if (extended) {
        const auto palette = rgb_palette(*target.palette);
        // Native passes two nonnull strings even when the source has no pair.
        const auto book = entry.book_name ? entry.book_name->book : std::u16string{};
        const auto name = entry.book_name ? entry.book_name->name : std::u16string{};
        result.registration = register_native_color(*target.extended_colors, entry.rgb, &palette, book, name, true);
        result.color_id = result.registration->color_id;
    }
    return result;
}

NativeBackgroundColor lookup_native_background_color(
    std::uint32_t color_id, const NativeSystemColorContext &target) {
    if (reserved(color_id)) return {NativeBackgroundColorSource::Reserved, {}, {}};
    if (!(color_id >> 8)) {
        if (!target.palette) return {NativeBackgroundColorSource::Unresolved, {}, "target_palette_unknown"};
        const auto &c = (*target.palette)[color_id & 255];
        return {NativeBackgroundColorSource::Palette, NativeRgb{c[0],c[1],c[2]}, {}};
    }
    if (!target.extended_colors)
        return {NativeBackgroundColorSource::Unresolved, {}, "target_extended_color_cache_unknown"};
    const auto slot = (color_id >> 8) & 0xfffff;
    if (!slot || slot > target.extended_colors->entries.size())
        return {NativeBackgroundColorSource::InvalidExtendedSlot, {}, {}};
    return {NativeBackgroundColorSource::Extended, target.extended_colors->entries[slot-1].rgb, {}};
}
NativeBackgroundProjection project_native_system_background(
    std::uint32_t color_id, bool override_enabled, bool view_present,
    const NativeRgb &initial_rgb, const NativeSystemColorContext &source,
    const NativeSystemColorContext &target) {
    NativeBackgroundProjection result;
    result.rgb = initial_rgb;
    if (!override_enabled || !view_present) return result;
    NativeColorRemapCache cache;
    auto mapped = remap_native_system_color(color_id, source, target, &cache);
    result.remapped_id = mapped.color_id;
    if (!mapped.color_id) {
        result.rgb.reset(); result.reason = mapped.reason; return result;
    }
    auto color = lookup_native_background_color(*mapped.color_id, target);
    if (color.source == NativeBackgroundColorSource::Unresolved) {
        result.rgb.reset(); result.reason = color.reason; return result;
    }
    if (color.rgb) {
        result.rgb = color.rgb;
        result.changed = *color.rgb != initial_rgb;
    }
    return result;
}
} // namespace p3d
