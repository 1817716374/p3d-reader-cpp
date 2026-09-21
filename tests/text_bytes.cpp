#include "internal.hpp"
#include "text_bytes.hpp"
#include "geometry.hpp"

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
    // Distinct values distinguish line color/style/weight and background fill
    // from its border. Signed styles must keep the same original source bits.
    Bytes lines(40);
    write(lines, 0, 0x12345678, 4);
    write(lines, 4, 0xffffffff, 4);
    write(lines, 8, 7, 4);
    double under_offset = -0.125, over_offset = 2.5;
    std::memcpy(lines.data() + 12, &under_offset, 8);
    write(lines, 20, 0x87654321, 4);
    write(lines, 24, 0x80000000, 4);
    write(lines, 28, 9, 4);
    std::memcpy(lines.data() + 32, &over_offset, 8);
    auto line_fields = decode_text_bytes(fixture(0xff, lines)).at("text_style").at("fields");
    check(line_fields[0].at("name") == "underline_color" &&
              line_fields[0].at("value") == 0x12345678u &&
              line_fields[2].at("name") == "underline_weight" && line_fields[2].at("value") == 7,
          "underline color and weight retain distinct native positions");
    check(line_fields[1].at("name") == "underline_line_style" &&
              line_fields[1].at("signed_value") == -1 && line_fields[1].at("value") == UINT32_MAX &&
              line_fields[1].at("raw_hex") == "ffffffff",
          "negative underline style preserves signed meaning and original bits");
    check(line_fields[3].at("name") == "underline_offset" &&
              line_fields[3].at("value") == under_offset &&
              line_fields[7].at("name") == "overline_offset" &&
              line_fields[7].at("value") == over_offset,
          "line offsets are source doubles, not rescaled style distances");
    check(line_fields[4].at("name") == "overline_color" &&
              line_fields[4].at("value") == 0x87654321u &&
              line_fields[5].at("name") == "overline_line_style" &&
              line_fields[5].at("signed_value") == INT32_MIN &&
              line_fields[6].at("name") == "overline_weight" && line_fields[6].at("value") == 9,
          "overline color style and weight do not borrow the underline fields");
    Bytes background(32);
    write(background, 0, 12, 4);
    write(background, 4, 34, 4);
    write(background, 8, 0xfffffffe, 4);
    write(background, 12, 56, 4);
    double border[2] = {-1.25, 3.75};
    std::memcpy(background.data() + 16, border, 16);
    auto bg = decode_text_bytes(fixture(0x1f00, background)).at("text_style");
    const auto &bf = bg.at("fields");
    check(bf[0].at("name") == "background_fill_color" && bf[0].at("value") == 12 &&
              bf[0].at("text_style_property_id") == 18 && bf[1].at("name") == "background_color" &&
              bf[1].at("value") == 34 && bf[1].at("text_style_property_id") == 17,
          "background source fill precedes border color despite SDK enum order");
    check(bf[2].at("name") == "background_line_style" && bf[2].at("signed_value") == -2 &&
              bf[3].at("name") == "background_weight" && bf[3].at("value") == 56 &&
              bf[4].at("name") == "background_border" &&
              bf[4].at("value") == Json::array({-1.25, 3.75}) &&
              bg.at("flags_decoded").at("background_style") == false,
          "disabled background retains every stored field without enabling itself");
    Bytes offset(16);
    std::memcpy(offset.data(), border, 16);
    auto lf = decode_text_bytes(fixture(1u << 18, offset)).at("text_style").at("fields")[0];
    check(lf.at("name") == "line_offset" && lf.at("text_style_property_id") == 32 &&
              lf.at("value") == Json::array({-1.25, 3.75}),
          "line offset is a two component property independent of background border");
    Bytes spacing(9);
    spacing[0] = 2;
    double gap = -0.75;
    std::memcpy(spacing.data() + 1, &gap, 8);
    auto spaced = fixture((1u << 13) | (1u << 14), spacing);
    auto sf = decode_text_bytes(spaced).at("text_style").at("fields");
    check(sf[0].at("name") == "inter_character_spacing_mode" &&
              sf[0].at("mode_name") == "AcadInterCharSpacing" &&
              sf[1].at("name") == "inter_character_spacing" && sf[1].at("value") == gap &&
              sf[0].at("native_value_used") == false && sf[1].at("native_value_used") == false,
          "spacing flag disables use without discarding serialized mode or distance");
    write(spaced, 89, 1u << 7, 4);
    sf = decode_text_bytes(spaced).at("text_style").at("fields");
    check(sf[0].at("native_value_used") == true && sf[1].at("native_value_used") == true &&
              sf[1].at("value") == gap && sf[1].at("text_style_property_id") == 33,
          "spacing use follows its source flag independently of optional presence");
    spaced[97] = 1;
    check(decode_text_bytes(spaced).at("text_style").at("fields")[0].at("mode_name") ==
              "FixedSpacing",
          "fixed spacing is distinct from Acad spacing");
    spaced[97] = 255;
    sf = decode_text_bytes(spaced).at("text_style").at("fields");
    check(sf[0].at("mode_name").is_null() && sf[0].at("value") == 255,
          "unknown spacing mode is preserved without an invented default");
    const std::pair<unsigned, const char *> new_flags[] = {
        {0, "background_style"}, {2, "underline_style"},
        {4, "overline_style"},   {7, "use_inter_character_spacing"},
        {8, "subscript"},        {9, "superscript"}};
    for (const auto &selected : new_flags) {
        auto flagged = fixture(0, {});
        write(flagged, 89, 1u << selected.first, 4);
        const auto fs = decode_text_bytes(flagged).at("text_style");
        unsigned enabled = 0;
        for (const auto &flag : fs.at("flags_decoded"))
            enabled += flag.get<bool>();
        check(fs.at("flags_decoded").at(selected.second) == true && enabled == 1 &&
                  fs.at("fields").empty(),
              "a style flag neither enables another flag nor creates optional source data");
    }
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
    Bytes weight(4);
    write(weight, 0, 0xfffffffeu, 4);
    auto wf = decode_text_bytes(fixture(1u << 19, weight)).at("text_style").at("fields")[0];
    check(wf.at("name") == "line_weight" && wf.at("value") == 0xfffffffeu,
          "default text line weight preserves native sentinel bits");

    auto native = [&](bool planar, Bytes source, const Bytes &ranges = Bytes{}) {
        const std::size_t start = planar ? 174 : 206;
        Bytes data(start + source.size() + ranges.size());
        write(data, 4, 54, 2);
        write(data, 6, 0x20, 2);
        write(data, 36, planar ? 0 : 0x1000, 2);
        write(data, 108, 1024, 4);
        write(data, 112, 8, 2);
        write(data, 114, source.size(), 2);
        auto real = [&](std::size_t p, double value) { std::memcpy(data.data() + p, &value, 8); };
        real(116, 500.);
        real(124, 0.);
        real(132, 12.);
        real(140, 6.);
        if (planar) {
            real(148, 1.5707963267948966);
            real(156, 7.);
            real(164, -3.);
        } else {
            real(148, -1.);
            real(180, 7.);
            real(188, -3.);
            real(196, 5.);
        }
        write(data, start - 2, ranges.size() / 3, 2);
        std::copy(source.begin(), source.end(), data.begin() + start);
        std::copy(ranges.begin(), ranges.end(), data.begin() + start + source.size());
        if (data.size() % 2)
            data.push_back(0);
        write(data, 8, (data.size() - 4) / 2, 4);
        write(data, 12, (data.size() - 4) / 2, 4);
        return data;
    };
    auto spatial = native(false, {0xff, 0xfe, 0x41, 0, 0x2d, 0x4e});
    auto nt = decode_native_text_record(spatial);
    check(nt.at("status") == "decoded" && nt.at("text") == u8"A中" &&
              nt.at("origin") == Json::array({7., -3., 5.}) &&
              nt.at("quaternion") == Json::array({-1., 0., 0., 0.}) && nt.at("font_id") == 1024 &&
              nt.at("justification_value") == 8,
          "spatial native text retains source origin quaternion and full font reference");
    check(nt.at("font_scale") == Json::array({500., 0.}) &&
              nt.at("native_font_size") == Json::array({3., 0.006}) && nt.at("width") == 12. &&
              nt.at("height") == 6.,
          "native text font size uses the native scale rule independently of stored extents");
    auto planar = native(true, {0xff, 0xfe, 0x41, 0});
    auto pt = decode_native_text_record(planar);
    check(pt.at("text_offset") == 174 && pt.at("source_layout") == "planar" &&
              pt.at("origin") == Json::array({7., -3., 0.}) && pt.at("text") == "A",
          "planar native text is read within its smaller header with source zero Z");
    const auto pq = pt.at("quaternion").get<std::array<double, 4>>();
    check(std::abs(pq[0] - std::sqrt(0.5)) < 1e-14 && std::abs(pq[3] - std::sqrt(0.5)) < 1e-14 &&
              pq[1] == 0 && pq[2] == 0,
          "planar source rotation is a positive Z rotation in radians");
    auto no_extended = spatial;
    write(no_extended, 6, 0, 2);
    write(no_extended, 36, 0, 2);
    check(decode_native_text_record(no_extended).at("source_layout") == "spatial",
          "missing extended flag selects spatial layout regardless of dimension bit");
    auto nr = native(true, {0xff, 0xfe, 0x41, 0}, {1, 1, 0xff, 0, 9, 2});
    auto ranges = decode_native_text_record(nr);
    check(ranges.at("declared_text_range_count") == 2 &&
              ranges.at("text_ranges")[0].at("source_offset") == 178 &&
              ranges.at("text_ranges")[0].at("raw_hex") == "0101ff" &&
              ranges.at("text_ranges")[1].at("start_1_based") == 0 &&
              ranges.at("text_ranges")[1].at("control_byte") == 2,
          "native text range triples retain invalid source starts and control bytes");
    auto capped = native(true, {0xff, 0xfe, 0x41, 0}, Bytes(60, 0));
    write(capped, 172, 21, 2);
    auto cap = decode_native_text_record(capped);
    check(cap.at("declared_text_range_count") == 21 && cap.at("native_text_range_count") == 20 &&
              cap.at("text_ranges").size() == 20 && cap.at("text_range_status") == "decoded",
          "native range reader clamps count to twenty without borrowing a twenty-first triple");
    auto short_ranges = native(true, {0xff, 0xfe, 0x41, 0}, {1, 1, 0});
    write(short_ranges, 172, 2, 2);
    auto sr = decode_native_text_record(short_ranges);
    check(sr.at("status") == "partial" && sr.at("text") == "A" &&
              sr.at("text_range_status") == "invalid" && sr.at("text_ranges").size() == 1,
          "truncated native range tail does not discard text and decoded source coordinates");
    auto encoded = decode_native_text_record(native(false, {0xff, 0xfd, 0xd0, 0xd6}));
    check(encoded.at("text") == u8"中" &&
              encoded.at("text_conversion").at("source_units") == Json::array({0xd6d0}) &&
              encoded.at("text_conversion").at("font_mapping_status") == "not_evaluated" &&
              encoded.at("text_conversion").at("text_interpretation") ==
                  "gb18030_compatibility_preview",
          "packed font character codes keep the old preview but never claim native font mapping");
    auto unsigned_text = decode_native_text_record(native(true, {0xff, 0xfe, 1, 0, 0x41, 0xe9}));
    check(unsigned_text.at("text") == u8"Aé" &&
              unsigned_text.at("text_conversion").at("prefix_bytes") == 4 &&
              unsigned_text.at("text_conversion").at("source_units") == Json::array({65, 233}),
          "byte marker widens unsigned values instead of guessing UTF8 or system codepage");
    auto bad_marker = decode_native_text_record(native(true, {0xfe, 0xff, 0x41, 0}));
    check(bad_marker.at("status") == "partial" && bad_marker.at("text").is_null() &&
              bad_marker.at("text_source").at("bytes") == 4 && bad_marker.contains("origin"),
          "unsupported native marker retains the source and independently decoded geometry");
    auto parsed_native = parse_native(planar);
    check(parsed_native.size() == 1 && parsed_native[0].at("native_text") == pt,
          "native record traversal exposes the same text decoder result");
    auto geometry = reconstruct_native(parsed_native[0], {});
    auto scene_text = geometry.texts.at(0);
    check(scene_text.at("source_origin") == pt.at("origin") &&
              scene_text.at("placement_matrix") == Json(identity()),
          "native text scene placement retains the decoded source position");
    scene_text.erase("source_origin");
    scene_text.erase("placement_matrix");
    check(geometry.texts.size() == 1 && scene_text == pt && geometry.unknown.empty(),
          "scene fallback reuses parsed planar native text without a second fixed-header parser");
    auto broken = planar;
    write(broken, 114, 65535, 2);
    auto broken_record = parse_native(broken)[0];
    check(broken_record.at("native_text").at("status") == "invalid" &&
              bytesof(broken_record.at("data")) == broken,
          "native text declared size cannot escape its base record");
    auto broken_geometry = reconstruct_native(broken_record, {});
    check(broken_geometry.texts.empty() && !broken_geometry.unknown.empty(),
          "invalid cached text metadata cannot create a positionless scene primitive");
    auto font_record = [&](std::uint32_t id, const Bytes &name, const Bytes &tail = Bytes()) {
        Bytes data(50);
        data.insert(data.end(), name.begin(), name.end());
        data.insert(data.end(), tail.begin(), tail.end());
        if (data.size() % 2)
            data.push_back(0);
        write(data, 4, 49, 2);
        write(data, 6, 0x10, 2);
        write(data, 8, (data.size() - 4) / 2, 4);
        write(data, 12, (data.size() - 4) / 2, 4);
        write(data, 16, 2, 4);
        write(data, 20, 37, 8);
        write(data, 44, id, 4);
        write(data, 48, name.size(), 2);
        return data;
    };
    const Bytes chinese_name{0x8b, 0x5b, 0x53, 0x4f};
    auto fd = decode_native_font_record(font_record(1024, chinese_name));
    check(fd.at("name") == u8"宋体" && fd.at("font_id") == 1024 && fd.at("status") == "decoded" &&
              fd.at("font_resolution_status") == "not_evaluated",
          "font declaration exposes name without claiming installed font");
    check(fd.at("name_source_units") == Json::array({0x5b8b, 0x4f53}) &&
              bytesof(fd.at("name_source")) == chinese_name && fd.at("name_source_offset") == 50,
          "font name offsets include stream prefix and preserve UTF16 units");
    for (unsigned id : {0u, 255u, 256u, 511u, 512u, 1023u, 1024u, 0xffffffffu}) {
        const auto f = decode_native_font_record(font_record(id, {65, 0}));
        check(f.at("font_id") == id &&
                  f.at("initial_loader_action") == (id < 512 ? "skip_font_id" : "request_font") &&
                  f.at("requested_font_family") ==
                      (id < 512 ? Json() : Json(id < 1024 ? "Shx" : "TrueType")),
              "font loading ID boundaries do not truncate stored catalog key");
    }
    const auto null_name = decode_native_font_record(font_record(512, {65, 0, 0, 0, 66, 0}));
    check(null_name.at("name") == "A" &&
              null_name.at("name_source_units") == Json::array({65, 0, 66}),
          "font name stops at first zero while source retains full span");
    const auto odd = decode_native_font_record(font_record(512, {65, 0, 0xfe}, {0xab}));
    check(odd.at("name") == "A" && odd.at("ignored_name_suffix_hex") == "fe" &&
              bytesof(odd.at("unassigned_suffix")) == Bytes{0xab},
          "font name floors odd byte count without borrowing following byte");
    check(decode_native_font_record(font_record(1024, {})).at("name") == "",
          "empty font name does not invent a default face");
    check(decode_native_font_record(font_record(1024, Bytes(1022, 0))).at("status") == "decoded",
          "native name accepts 511 wchar units plus terminator");
    const auto long_name = decode_native_font_record(font_record(1024, Bytes(1024, 0)));
    check(long_name.at("status") == "partial" && long_name.at("name").is_null() &&
              long_name.at("name_source_units").size() == 512,
          "oversized font name preserves source and reports native buffer limit");
    check(decode_native_font_record(font_record(1024, {0x3d, 0xd8, 0, 0xde})).at("name") ==
              "\xf0\x9f\x98\x80",
          "font name converts valid surrogate pair");
    for (const Bytes name : {Bytes{0, 0xdc}, Bytes{0, 0xd8}, Bytes{0, 0xd8, 65, 0}}) {
        const auto bad = decode_native_font_record(font_record(1024, name));
        check(bad.at("status") == "partial" && bad.at("name").is_null() &&
                  bytesof(bad.at("name_source")) == name,
              "malformed font UTF16 keeps source without invalid JSON UTF8");
    }
    auto truncated_font = font_record(513, chinese_name);
    write(truncated_font, 48, 0xffff, 2);
    auto invalid_font = parse_native(truncated_font)[0];
    check(invalid_font.at("font_definition").at("status") == "invalid" &&
              invalid_font.at("font_definition").at("font_id") == 513 &&
              bytesof(invalid_font.at("data")) == truncated_font,
          "font extent failure keeps independent ID and base record");
    auto parsed_font = parse_native(font_record(1024, chinese_name));
    check(parsed_font[0].at("font_definition") == fd && parsed_font[0].at("id") == 37,
          "font lookup ID differs from native record object ID");
    Bytes font_table(44);
    write(font_table, 4, 10, 2);
    write(font_table, 8, 20, 4);
    write(font_table, 12, 20, 4);
    write(font_table, 16, 2, 4);
    write(font_table, 36, 7, 4);
    write(font_table, 40, 0x1234abcd, 4);
    const auto table = parse_native(font_table)[0].at("font_table");
    check(table.at("declared_descendant_count") == 7 && table.at("status") == "decoded" &&
              bytesof(table.at("unassigned_suffix")) == Bytes({0xcd, 0xab, 0x34, 0x12}) &&
              table.at("font_resolution_status") == "not_evaluated",
          "font table count does not claim validated membership or runtime loading");
    auto high_font_text = planar;
    auto extended_table = font_table;
    extended_table.resize(116);
    write(extended_table, 6, 0x20, 2);
    write(extended_table, 108, 9, 4);
    const auto ext = decode_native_font_record(extended_table);
    check(ext.at("declared_descendant_count") == 9 &&
              ext.at("descendant_count_source_offset") == 108 &&
              ext.at("unassigned_header_extension").at("bytes") == 72,
          "extended font table reads generic compound count after extended header");
    extended_table.resize(110);
    check(decode_native_font_record(extended_table).at("status") == "invalid",
          "truncated extended font table does not use the normal header count");
    auto wrong_class = font_record(1024, chinese_name);
    write(wrong_class, 16, 3, 4);
    check(decode_native_font_record(wrong_class).at("status") == "invalid" &&
              !parse_native(wrong_class)[0].contains("font_definition"),
          "font decoder is restricted to the native font class");
    write(high_font_text, 108, 0x12340401, 4);
    const auto ft = decode_native_text_record(high_font_text);
    check(ft.at("font_id") == 0x12340401 && ft.at("font_lookup_id") == 1025,
          "native text lookup uses low word without changing stored reference");
    return checks;
}
