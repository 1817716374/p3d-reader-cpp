#pragma once
#include "internal.hpp"
#include <p3d/layer_lookup.hpp>

namespace p3d::detail {
using LayerNameKey = std::pair<std::uint32_t, std::uint32_t>;
std::u16string layer_name_wide(const std::string &);
inline std::u16string layer_name_prefix(std::u16string s) {
    s.resize(std::min(s.size(), s.find(u'\0')));
    return s;
}
inline LayerNameKey layer_name_key(const std::u16string &s, const NativeLayerNameContext &context) {
    require(bool(context.lowercase), "layer_name_source_lowercase_required");
    std::uint32_t hash = 0x1505;
    for (auto c : s) {
        const auto lower = context.lowercase(c);
        require(lower.has_value(), "layer_name_source_lowercase_unknown");
        hash = hash * 33u + std::uint16_t(*lower);
    }
    require(s.size() <= UINT32_MAX, "layer_name_length_out_of_range");
    return {hash, std::uint32_t(s.size())};
}
inline bool layer_name_equal(const std::u16string &a, const std::u16string &b,
                             const NativeLayerNameContext &context) {
    if (a == b)
        return true;
    require(bool(context.compare), "layer_name_source_comparison_required");
    const auto result = context.compare(a, b);
    require(result.has_value(), "layer_name_source_comparison_unknown");
    return *result == 0;
}
} // namespace p3d::detail
