#include "internal.hpp"
#include <p3d/display_priority.hpp>
#include <limits>
using namespace p3d;
unsigned display_priority_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) {
        require(value, message);
        ++checks;
    };
    const auto lo = std::numeric_limits<std::int32_t>::min();
    const auto hi = std::numeric_limits<std::int32_t>::max();
    constexpr std::int32_t limit = 0x77ffe0;
    check(combine_native_model_display_priorities({}) == 0 &&
              combine_native_model_display_priorities({17, -8, 3}) == 12,
          "known model contributions are added in native integer units");
    check(combine_native_model_display_priorities({limit, 10}) == limit &&
              combine_native_model_display_priorities({-limit, -10}) == -limit,
          "model chain clamps at both native display-priority limits");
    check(combine_native_model_display_priorities({hi, 1}) == -limit &&
              combine_native_model_display_priorities({lo, -1}) == limit,
          "native model sum wraps before signed clamping without C++ signed overflow");
    check(combine_native_model_display_priorities({hi, hi, 2}) == 0 &&
              combine_native_model_display_priorities({lo, lo}) == 0,
          "model contributions are not individually clamped or treated as parent sentinels");
    check(calculate_native_inherited_display_priority(3, 7, 11) == 21 &&
              calculate_native_inherited_display_priority(lo, 7, 11) == 18,
          "inherited-priority branch replaces only its parent INT32_MIN with zero");
    check(calculate_native_inherited_display_priority(0, lo, 0) == -limit &&
              calculate_native_inherited_display_priority(hi, 1, 0) == -limit,
          "layer contribution retains all bits and combined arithmetic uses native wrap");
    check(calculate_native_inherited_display_priority(limit, 1, 0) == limit &&
              calculate_native_inherited_display_priority(-limit, -1, 0) == -limit,
          "inherited priority clamps after all three inputs have been added");
    check(apply_native_display_priority_range(-50, -10, 10) == -10 &&
              apply_native_display_priority_range(50, -10, 10) == 10 &&
              apply_native_display_priority_range(3, -10, 10) == 3,
          "enabled view range performs its separate subsequent comparisons");
    check(apply_native_display_priority_range(0, 10, -10) == 10 &&
              apply_native_display_priority_range(20, 10, -10) == -10,
          "inverted view bounds retain ordered native comparisons instead of std::clamp");
    auto set = [](Bytes &b, std::size_t at, auto value) {
        std::memcpy(b.data() + at, &value, sizeof(value));
    };
    for (unsigned version = 1; version <= 7; ++version) {
        Bytes b(108, 0);
        set(b, 4, std::uint16_t(49));
        set(b, 36, std::uint32_t(64));
        set(b, 44, std::uint16_t(version));
        set(b, 88, std::int32_t(-700 - version));
        const auto d = decode_native_layer(b, Json::array());
        check(d.at("display_priority") == -700 - std::int32_t(version) &&
                  d.at("unassigned_ranges").size() == 2 &&
                  d.at("unassigned_ranges")[1].at("source_offset") == 100,
              "all supported layer versions read signed display priority and remove its unknown "
              "source span");
    }
    for (unsigned type : {19u, 20u, 21u, 37u, 40u, 44u, 52u, 54u, 62u, 97u}) {
        Bytes b(116, 0);
        set(b, 4, std::uint16_t(type));
        set(b, 6, std::uint16_t(0x20));
        set(b, 8, std::uint32_t(56));
        set(b, 12, std::uint32_t(56));
        set(b, 44, lo);
        auto p = parse_native(b).at(0).at("native_graphic_header").at("display_priority");
        check(p.at("status") == "decoded" && p.at("value") == lo && p.at("source_offset") == 44,
              "extended graphic headers retain signed source priority including the sentinel");
        set(b, 6, std::uint16_t(0));
        check(!parse_native(b).at(0).at("native_graphic_header").contains("display_priority"),
              "nonextended data is not assigned an extended-header priority");
    }
    return checks;
}
