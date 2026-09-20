#include "internal.hpp"
#include "text_bytes.hpp"

unsigned text_bytes_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto write = [](Bytes &b, std::size_t offset, std::uint64_t value, unsigned size) {
        for (unsigned i = 0; i < size; ++i)
            b.at(offset + i) = std::uint8_t(value >> (i * 8));
    };
    auto fixture = [&](std::uint32_t mask, const Bytes &extra, std::u16string text = u"A\u4e2d") {
        const auto header = 97 + extra.size();
        Bytes b(header);
        b[0] = 1;
        write(b, 1, header, 4);
        write(b, 5, (text.size() + 1) * 2, 4);
        write(b, 81, 0xff123456, 4);
        write(b, 85, 0x12340021, 4);
        write(b, 89, (1u << 1) | (1u << 3) | (1u << 5) | (1u << 6) | (1u << 10) | (1u << 14), 4);
        write(b, 93, mask, 4);
        std::copy(extra.begin(), extra.end(), b.begin() + 97);
        text.push_back(0);
        for (auto c : text) {
            b.push_back(std::uint8_t(c));
            b.push_back(std::uint8_t(c >> 8));
        }
        return b;
    };
    auto b = fixture(0, {});
    auto j = decode_text_bytes(b);
    check(j.at("text") == u8"A中" && j.at("extra_style_hex") == "" && j.at("header_bytes") == 97,
          "text common source fields retain Chinese and declared boundaries");
    auto style = j.at("text_style");
    check(style.at("color") == 0xff123456u && style.at("font_id") == 0x12340021u &&
              style.at("font_lookup_id") == 33,
          "font lookup truncation does not change source font ID");
    for (const auto *key : {"underline", "overline", "bold", "italic", "vertical", "is_3d"})
        check(style.at("flags_decoded").at(key) == true, "confirmed native text flag getter");
    auto aligned = b;
    write(aligned, 89, (8u << 20) | (1u << 14), 4);
    auto alignment = decode_text_bytes(aligned).at("text_style");
    check(alignment.at("justification").at("name") == "CenterBaseline" &&
              alignment.at("flags_decoded").at("is_3d") == true,
          "text alignment and 3D flag occupy independent bits");
    write(aligned, 89, 0xf1800000u, 4);
    alignment = decode_text_bytes(aligned).at("text_style");
    check(alignment.at("justification").at("value") == 24 &&
              alignment.at("justification").at("name") == "RightDescender",
          "native alignment is eight bits and retains non-grid descender enum values");
    write(aligned, 89, 0x0ff00000u, 4);
    const auto unknown_alignment = decode_text_bytes(aligned).at("text_style").at("justification");
    write(aligned, 89, 127u << 20, 4);
    check(unknown_alignment.at("value") == 255 && unknown_alignment.at("name").is_null() &&
              decode_text_bytes(aligned).at("text_style").at("justification").at("name") ==
                  "Invalid",
          "unknown alignment is not silently replaced by the native invalid sentinel");
    const unsigned sizes[] = {4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, 4, 16, 1, 8, 4, 8, 4, 16, 4, 8};
    Bytes all;
    for (unsigned bit = 0; bit < 21; ++bit) {
        const Bytes value(sizes[bit], 0);
        const auto single = decode_text_bytes(fixture(1u << bit, value)).at("text_style");
        check(single.at("status") == "decoded" && single.at("fields").size() == 1 &&
                  single.at("fields")[0].at("source_offset") == 97 &&
                  single.at("fields")[0].at("source_bytes") == sizes[bit],
              "each optional field consumes its independent native width");
        auto short_value = value;
        short_value.pop_back();
        const auto short_j = decode_text_bytes(fixture(1u << bit, short_value));
        check(short_j.at("text") == u8"A中" && short_j.at("text_style").at("status") == "invalid",
              "optional field cannot consume text bytes when its header is truncated");
        all.insert(all.end(), value.begin(), value.end());
    }
    auto all_j = decode_text_bytes(fixture((1u << 21) - 1, all)).at("text_style");
    check(all_j.at("fields").size() == 21 && all_j.at("status") == "decoded" &&
              all_j.at("fields")[17].at("native_usage") == "skipped",
          "all optional fields coexist without losing the explicitly skipped word");
    for (unsigned id : {0u, 511u, 512u, 1023u, 1024u, 65535u}) {
        Bytes font(4);
        write(font, 0, id, 4);
        auto f = decode_text_bytes(fixture(1u << 15, font)).at("text_style").at("fields")[0];
        check(f.at("value") == id && f.at("name") == "big_font_id" &&
                  f.at("native_font_id_in_lookup_range") == (id >= 512 && id <= 1023),
              "big font source ID is retained even when native lookup is gated out");
    }
    Bytes angle(8);
    double slant = -0.25;
    std::memcpy(angle.data(), &slant, 8);
    auto f = decode_text_bytes(fixture(1u << 16, angle)).at("text_style").at("fields")[0];
    check(f.at("name") == "custom_slant_angle" && f.at("value") == slant,
          "optional slant angle is decoded as double without unit conversion");
    Bytes mask{0x81, 0x80, 0x01, 0, 0, 0, 0, 0, 0};
    auto bitmap = decode_text_bytes(fixture(1u << 21, mask, u"abcdefghijklmnopq"))
                      .at("text_style")
                      .at("character_mask");
    check(bitmap.at("bit_count") == 17 && bitmap.at("packed_bits_hex") == "818001" &&
              bitmap.at("remaining_header_hex") == "818001000000000000",
          "UTF16 character mask excludes the terminal code unit and preserves unused bytes");
    check(decode_text_bytes(fixture(1u << 21, Bytes{1}, u"abcdefghijk"))
                  .at("text_style")
                  .at("status") == "invalid",
          "short character mask never borrows bytes from the text");
    auto unknown = decode_text_bytes(fixture(1u << 31, Bytes{7, 8})).at("text_style");
    check(unknown.at("status") == "partial" &&
              unknown.at("unassigned_presence_bits") == 0x80000000u &&
              unknown.at("unassigned_suffix_hex") == "0708",
          "unknown presence bits and suffix are retained");
    auto future = b;
    future[0] = 2;
    check(decode_text_bytes(future).at("text_style").at("status") == "not_evaluated",
          "future text version does not reuse current extension semantics");
    auto cmd = command_fields(37, b);
    check(cmd.at("text_style") == j.at("text_style") && cmd.at("text") == j.at("text"),
          "command stream uses the same text payload decoder");
    auto malformed = b;
    write(malformed, 1, UINT32_MAX, 4);
    check(command_fields(37, malformed).contains("field_decode_error"),
          "overflowing text boundary reports a field error");

    // A graphics container, one text Entry and an empty material footer.
    Bytes entry(36);
    write(entry, 4, 7, 4);
    write(entry, 28, b.size(), 8);
    entry.insert(entry.end(), b.begin(), b.end());
    Bytes container(142);
    write(container, 0, 1, 8);
    write(container, 134, 1, 8);
    container.resize(150);
    write(container, 142, entry.size(), 8);
    container.insert(container.end(), entry.begin(), entry.end());
    auto packets = decode_graphics_bytes(container).at("geometry_packets");
    check(packets[0].at("geometry_status") == "decoded" &&
              packets[0].at("geometry").at("_type") == "TextEntity" &&
              packets[0].at("geometry").at("text_style") == j.at("text_style"),
          "serialized text Entry exposes text and style instead of opaque bytes");
    return checks;
}
