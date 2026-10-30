#include "internal.hpp"
#include <cstring>
#include <stdexcept>

namespace {
void append_u8(p3d::Bytes &bytes, std::uint8_t value) { bytes.push_back(value); }

void append_u16(p3d::Bytes &bytes, std::uint16_t value) {
    append_u8(bytes, static_cast<std::uint8_t>(value));
    append_u8(bytes, static_cast<std::uint8_t>(value >> 8));
}

void append_u32(p3d::Bytes &bytes, std::uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8)
        append_u8(bytes, static_cast<std::uint8_t>(value >> shift));
}

void append_f64(p3d::Bytes &bytes, double value) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    for (unsigned shift = 0; shift < 64; shift += 8)
        append_u8(bytes, static_cast<std::uint8_t>(bits >> shift));
}

p3d::Json decode_contour(bool is_major, double distance_mm) {
    p3d::Bytes bytes;
    append_u16(bytes, 402); // DTMEntityContoursDisplayProcessor

    // Base display frame: length, version, layer/symbology, transparency,
    // visibility and the three native padding bytes.
    append_u16(bytes, 30);
    append_u16(bytes, 2);
    append_u32(bytes, 64);
    append_u32(bytes, 3);
    append_u32(bytes, 0);
    append_u32(bytes, 0);
    append_f64(bytes, 0.0);
    append_u8(bytes, 0);
    append_u8(bytes, 0);
    append_u8(bytes, 0);
    append_u8(bytes, 0);

    // ContoursProcessor::Read version 4 scalar sequence.
    append_u16(bytes, 4);
    append_u8(bytes, is_major ? 1 : 0);
    append_f64(bytes, distance_mm);
    append_u32(bytes, 0);
    append_f64(bytes, 5.0);
    append_u32(bytes, 5);
    append_u8(bytes, 0);
    append_u16(bytes, 0);
    append_u16(bytes, 0);
    append_f64(bytes, 50.0);
    append_u8(bytes, 0);
    append_f64(bytes, 0.1);
    append_u32(bytes, 0);
    append_u8(bytes, 0);
    append_f64(bytes, 1.0);
    append_u32(bytes, 0x7ffffffe);
    append_u32(bytes, 0xfffffffe);
    append_u32(bytes, 0xfffffffe);
    append_u32(bytes, 2);
    return p3d::decode_attribute(307, 23223, bytes, 0);
}
} // namespace

unsigned terrain_display_tests() {
    unsigned checks = 0;
    auto check = [&](bool condition, const char *message) {
        ++checks;
        if (!condition)
            throw std::runtime_error(message);
    };

    const auto main = decode_contour(true, 10.0);
    const auto &main_state = main.at("decoded_state").at("contours");
    check(main_state.at("is_major_contour") == true &&
              main_state.at("contour_distance_mm") == 10.0,
          "major contour role and millimeter distance are projected");
    check(main_state.at("member_0x38_u8") == 1 &&
              main_state.at("member_0x40_f64") == 10.0 &&
              main_state.at("unmapped_suffix").at("bytes") == 0,
          "semantic contour fields preserve the complete raw members");

    const auto secondary = decode_contour(false, 2.0);
    const auto &secondary_state = secondary.at("decoded_state").at("contours");
    check(secondary_state.at("is_major_contour") == false &&
              secondary_state.at("contour_distance_mm") == 2.0,
          "secondary contour role and distance are projected");
    check(secondary.at("status") == "processor_state_decoded",
          "semantic projection preserves the complete processor status");
    return checks;
}
