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

p3d::Json decode_flow_arrows(std::uint16_t version, std::int32_t arrow_type,
                             double scale) {
    p3d::Bytes bytes;
    append_u16(bytes, 407); // DTMEntityFlowArrowsDisplayProcessor

    // Base display frame.
    append_u16(bytes, 30);
    append_u16(bytes, 2);
    append_u32(bytes, 64);
    append_u32(bytes, 0xfffffffe);
    append_u32(bytes, 0x7ffffffe);
    append_u32(bytes, 0xfffffffe);
    append_f64(bytes, 0.0);
    append_u8(bytes, 0);
    append_u8(bytes, 0);
    append_u8(bytes, 0);
    append_u8(bytes, 0);

    append_u16(bytes, version);
    append_u16(bytes, 64);
    append_f64(bytes, 1.0);
    append_f64(bytes, 1.0);
    append_f64(bytes, 1.0);
    append_u32(bytes, 1);
    if (version > 1) {
        append_u32(bytes, static_cast<std::uint32_t>(arrow_type));
        append_f64(bytes, scale);
    }
    return p3d::decode_attribute(307, 23223, bytes, 0);
}

p3d::Json decode_low_points() {
    p3d::Bytes bytes;
    append_u16(bytes, 409); // DTMEntityLowPointsDisplayProcessor
    append_u16(bytes, 30);
    append_u16(bytes, 2);
    append_u32(bytes, 64);
    append_u32(bytes, 0xfffffffe);
    append_u32(bytes, 0x7ffffffe);
    append_u32(bytes, 0xfffffffe);
    append_f64(bytes, 0.0);
    append_u8(bytes, 0);
    append_u8(bytes, 0);
    append_u8(bytes, 0);
    append_u8(bytes, 0);

    append_u16(bytes, 2);
    append_u8(bytes, 1); // visible
    append_u16(bytes, 0); // +0x40 string
    append_f64(bytes, 1.0);
    append_f64(bytes, 1.0);
    append_f64(bytes, 1.0);
    append_u32(bytes, 0x00112233); // color
    append_f64(bytes, 0.0); // +0x110
    append_u32(bytes, 17); // layer id
    append_u16(bytes, 0); // +0x80 string
    append_u16(bytes, 0); // +0xa0 string
    append_f64(bytes, 0.0);
    append_f64(bytes, 0.0);
    append_f64(bytes, 0.0);
    append_u16(bytes, 0); // +0xd8
    append_u32(bytes, 1); // circular point type
    append_f64(bytes, 25.0); // size in millimeters
    append_u32(bytes, 3); // decimal places
    for (const auto c : std::u16string(u"Arial"))
        append_u16(bytes, c);
    append_u16(bytes, 0);
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

    const auto flow_v1 = decode_flow_arrows(1, 0, 0.0);
    const auto &flow_v1_state = flow_v1.at("decoded_state").at("flow_arrows");
    check(flow_v1_state.at("flow_arrows_scale") == 10000.0 &&
              flow_v1_state.at("scale") == flow_v1_state.at("flow_arrows_scale"),
          "version 1 exposes the SDK flow-arrow scale native default");
    const auto flow_v2 = decode_flow_arrows(2, 3, 250.0);
    const auto &flow_v2_state = flow_v2.at("decoded_state").at("flow_arrows");
    check(flow_v2_state.at("arrow_type_name") == "double" &&
              flow_v2_state.at("flow_arrows_scale") == 250.0 &&
              flow_v2_state.at("unmapped_suffix").at("bytes") == 0,
          "version 2 exposes the SDK arrow type and scale semantics");

    const auto low_points = decode_low_points();
    const auto &low_style =
        low_points.at("decoded_state").at("low_points").at("display_style");
    check(low_style.at("visible") == true && low_style.at("layer_id") == 17 &&
              low_style.at("color") == 0x00112233,
          "low-point visibility, layer and color map to SDK properties");
    check(low_style.at("point_type_name") == "circle" &&
              low_style.at("size_mm") == 25.0 &&
              low_style.at("decimal_places") == 3 &&
              low_style.at("font_name") == "Arial",
          "low-point type, millimeter size, precision and font are named");
    check(low_points.at("status") == "processor_state_decoded" &&
              low_points.at("decoded_state").at("status") == "known_fields_decoded" &&
              low_points.at("decoded_state").at("low_points")
                      .at("unmapped_suffix").at("bytes") == 0,
          "low-point semantic aliases preserve complete source consumption");
    return checks;
}
