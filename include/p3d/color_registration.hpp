#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace p3d {
using NativeRgb = std::array<std::uint8_t, 3>;
using NativeColorPalette = std::array<NativeRgb, 256>;
enum class NativePaletteProfile { PreHostDefault, BuiltinSelectors3And4 };
// Immutable R1.18 palette values. PreHostDefault is the input to the host's
// palette callback, not evidence that the callback leaves it unchanged.
const NativeColorPalette &native_color_palette(NativePaletteProfile profile);

enum class NativePaletteHostProfile { UnspecifiedHost, BuiltinDefaultService };
struct NativePaletteSelectionContext {
    // known=true + nullopt means confirmed empty; known=false is unknown.
    bool file_cache_known = false;
    std::optional<NativeColorPalette> file_cache;
    bool global_cache_known = false;
    std::optional<NativeColorPalette> global_cache;
    NativePaletteHostProfile host_profile = NativePaletteHostProfile::UnspecifiedHost;
};
enum class NativePaletteSource { Unresolved, FileCache, Builtin, GlobalCache, DefaultService };
struct NativePaletteSelection {
    std::optional<NativeColorPalette> palette;
    std::optional<std::uint32_t> selector; // not read when file cache wins
    NativePaletteSource source = NativePaletteSource::Unresolved;
    std::string reason;
    bool file_cache_created = false;
    bool global_cache_created = false;
};
// Pure projection of 12e680 + 1e4dd0. Does not mutate supplied caches or invoke
// callbacks. Returned RGB is a value copy; callers own runtime object identity.
NativePaletteSelection select_native_color_palette(
    std::optional<std::uint32_t> base_selector,
    std::optional<std::uint32_t> override_selector,
    const NativePaletteSelectionContext &context);
// R1.18 integer H/S/V: hue 0..359, saturation/value 0..100.
std::array<int, 3> native_color_hsv(const NativeRgb &rgb);
// Matches 1e4500 with no external HSV cache. Index 255 is not searched.
std::uint8_t nearest_native_palette_color(const NativeColorPalette &palette, const NativeRgb &rgb);

struct NativeColorBookName {
    std::u16string book;
    std::u16string name;
};
struct NativeExtendedColor {
    NativeRgb rgb{};
    std::optional<NativeColorBookName> book_name;
};
struct NativeExtendedColorCache {
    // Complete, ordered slots of ONE already selected cache. Duplicates remain;
    // the native reverse map must point to the last slot for each RGB.
    std::vector<NativeExtendedColor> entries;
    bool dirty = false;
};
enum class NativeColorRegistrationAction { Existing, AttachedBookName, Appended, Missing };
struct NativeColorRegistration {
    std::uint32_t native_index = 0; // one-based extended slot, or zero when missing
    std::uint32_t color_id = 0;     // (native_index << 8) | nearest palette index
    NativeColorRegistrationAction action = NativeColorRegistrationAction::Missing;
};
// Pure cache-state simulation of 266c60 + the encoding in 1e41c0. Modifies only
// this caller-owned cache; does not select a file cache, edit XML, persist data,
// or invoke host callbacks. nullptr palette explicitly means the native palette
// lookup returned null (fallback zero), NOT that the palette is unknown.
// Book and Name are independent optional UTF-16 pointers, truncated at NUL.
// allow_append=false still permits attaching Book/Name to an existing entry.
NativeColorRegistration register_native_color(
    NativeExtendedColorCache &cache, const NativeRgb &rgb, const NativeColorPalette *palette,
    const std::optional<std::u16string> &book = std::nullopt,
    const std::optional<std::u16string> &name = std::nullopt, bool allow_append = true);
} // namespace p3d
