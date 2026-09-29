#include <p3d/color_registration.hpp>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace p3d {
std::array<int, 3> native_color_hsv(const NativeRgb &rgb) {
    const double r = rgb[0], g = rgb[1], b = rgb[2];
    const double maximum = std::max({r, g, b}), minimum = std::min({r, g, b});
    const double delta = maximum - minimum;
    std::array<int, 3> hsv{0, 0, int((maximum / 255.0) * 100.0 + 0.5)};
    if (maximum == 0) return hsv;
    hsv[1] = int((delta / maximum) * 100.0 + 0.5);
    if (!hsv[1]) return hsv;
    const double rc = (maximum - r) / delta;
    const double gc = (maximum - g) / delta;
    const double bc = (maximum - b) / delta;
    double hue = r == maximum ? bc - gc : g == maximum ? (rc + 2.0) - bc : (gc + 4.0) - rc;
    hue *= 60.0;
    if (hue < 0) hue += 360.0;
    hsv[0] = int(hue + 0.5);
    if (hsv[0] >= 360) hsv[0] = 0;
    return hsv;
}

std::uint8_t nearest_native_palette_color(const NativeColorPalette &palette, const NativeRgb &rgb) {
    const auto query = native_color_hsv(rgb);
    int best_distance = std::numeric_limits<int>::max();
    unsigned best_index = 0;
    int best_value = 0, darkest_grey = -1, darkest_value = 255;
    for (unsigned i = 0; i < 255; ++i) {
        const auto entry = native_color_hsv(palette[i]);
        int dh = query[1] > 2 ? entry[0] - query[0] : 0;
        if (dh > 180) dh = 360 - dh;
        else if (dh < -180) dh += 360;
        const int ds = entry[1] - query[1], dv = entry[2] - query[2];
        const int distance = 2 * dh * dh + ds * ds + dv * dv;
        if (distance < best_distance) {
            best_distance = distance;
            best_index = i;
            best_value = entry[2];
            // Native early exit precedes grey tracking, even for a non-black
            // query quantized to V=0. Do not scan the rest for a grey fallback.
            if (!distance) break;
        }
        if (!entry[1] && entry[2] && entry[2] < darkest_value) {
            darkest_grey = int(i);
            darkest_value = entry[2];
        }
    }
    if (!best_value && rgb != NativeRgb{} && darkest_grey >= 0) best_index = unsigned(darkest_grey);
    return std::uint8_t(best_index);
}

NativeColorRegistration register_native_color(
    NativeExtendedColorCache &cache, const NativeRgb &rgb, const NativeColorPalette *palette,
    const std::optional<std::u16string> &book, const std::optional<std::u16string> &name,
    bool allow_append) {
    if (cache.entries.size() > std::numeric_limits<std::uint32_t>::max())
        throw std::length_error("native_extended_color_slot_overflow");
    NativeColorRegistration result;
    std::optional<NativeColorBookName> pair;
    if (book && name) pair = NativeColorBookName{book->substr(0, book->find(u'\0')),
                                               name->substr(0, name->find(u'\0'))};
    for (std::size_t i = cache.entries.size(); i; --i) {
        auto &entry = cache.entries[i - 1];
        if (entry.rgb != rgb) continue;
        result.native_index = std::uint32_t(i);
        result.action = NativeColorRegistrationAction::Existing;
        if (pair && !pair->book.empty() && !pair->name.empty() && !entry.book_name) {
            entry.book_name = std::move(pair);
            cache.dirty = true;
            result.action = NativeColorRegistrationAction::AttachedBookName;
        }
        break;
    }
    if (!result.native_index && allow_append) {
        if (cache.entries.size() == std::numeric_limits<std::uint32_t>::max())
            throw std::length_error("native_extended_color_slot_overflow");
        // Missing RGB accepts an allocated pair of empty strings; an existing
        // RGB requires both strings nonempty before attaching a missing pair.
        cache.entries.push_back({rgb, std::move(pair)});
        cache.dirty = true;
        result.native_index = std::uint32_t(cache.entries.size());
        result.action = NativeColorRegistrationAction::Appended;
    }
    const auto fallback = palette ? nearest_native_palette_color(*palette, rgb) : 0;
    result.color_id = (result.native_index << 8) | std::uint32_t(fallback);
    return result;
}
} // namespace p3d
