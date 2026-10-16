#include <p3d/display_priority.hpp>
#include <limits>

namespace p3d {
namespace {
std::int32_t signed_word(std::uint32_t word) {
    return word <= 0x7fffffffu ? std::int32_t(word)
                               : std::int32_t(std::int64_t(word) - 0x100000000ll);
}
std::int32_t native_limit(std::uint32_t word) {
    const auto value = signed_word(word);
    return value < -0x77ffe0 ? -0x77ffe0 : value > 0x77ffe0 ? 0x77ffe0 : value;
}
} // namespace
std::int32_t combine_native_model_display_priorities(const std::vector<std::int32_t> &values) {
    std::uint32_t sum = 0;
    for (auto value : values)
        sum += std::uint32_t(value);
    return native_limit(sum);
}
std::int32_t calculate_native_inherited_display_priority(std::int32_t parent, std::int32_t layer,
                                                         std::int32_t model) {
    const auto base =
        parent == std::numeric_limits<std::int32_t>::min() ? 0u : std::uint32_t(parent);
    return native_limit(base + std::uint32_t(layer) + std::uint32_t(model));
}
std::int32_t apply_native_display_priority_range(std::int32_t value, std::int32_t minimum,
                                                 std::int32_t maximum) {
    if (value < minimum)
        return minimum;
    return value > maximum ? maximum : value;
}
} // namespace p3d
