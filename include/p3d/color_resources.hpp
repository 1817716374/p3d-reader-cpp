#pragma once
#include <p3d/reader.hpp>

namespace p3d {
using NativeRgb = std::array<std::uint8_t, 3>;
using NativeColorPalette = std::array<NativeRgb, 256>;
// Native rounded hue/saturation/value, in 0..359 / 0..100 / 0..100.
std::array<std::uint32_t, 3> native_color_hsv(const NativeRgb &);
// Uses entries 0..254, source HSV distance, first ties and black fallback.
// Entry 255 is not a candidate; no default palette is inferred.
Json nearest_native_palette_color(const NativeRgb &, const NativeColorPalette &);
struct NativeColorRequest {
    NativeRgb rgb;
    // A present empty name differs from a null name pointer in the native cache.
    std::optional<std::u16string> book, name;
};
// Fresh extended RGB cache, optionally loaded from saved XML. Keeps duplicate
// loaded entries; its RGB index points to the last such entry. Requests reuse
// actual native RGB entries, never content-deduplicate arbitrary source objects.
class NativeExtendedColorCache {
  public:
    NativeExtendedColorCache();
    explicit NativeExtendedColorCache(const std::string &saved_xml);
    const Json &metadata() const noexcept;
    Json lookup(const NativeRgb &) const;
    // Owns a derived immutable state; input state is unchanged. A supplied
    // palette is the actual owner's table. nullopt means unknown unless
    // palette_confirmed_absent is true, in which case its low index is zero.
    // Unknown palette leaves packed results unknown but the independent RGB
    // cache transition remains available. Does not execute host services.
    NativeExtendedColorCache resolve(const std::vector<NativeColorRequest> &,
                                     const std::optional<NativeColorPalette> &palette = {},
                                     bool palette_confirmed_absent = false) const;

  private:
    struct Impl;
    explicit NativeExtendedColorCache(std::shared_ptr<const Impl>);
    std::shared_ptr<const Impl> impl_;
};
} // namespace p3d
