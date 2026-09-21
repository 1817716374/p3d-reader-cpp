#include "internal.hpp"
#include "text_bytes.hpp"

namespace {
using namespace p3d;
template <class T> void put(Bytes &b, T value) {
    const auto start = b.size();
    b.resize(start + sizeof(value));
    std::memcpy(b.data() + start, &value, sizeof(value));
}
template <class T> void set(Bytes &b, std::size_t offset, T value) {
    std::memcpy(b.data() + offset, &value, sizeof(value));
}
Json link(unsigned app, const Bytes &payload, std::size_t offset = 200) {
    return {{"app", app}, {"header", 0x1000}, {"offset", offset}, {"payload", rawbytes(payload)}};
}
} // namespace

unsigned native_text_style_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    Bytes base(206);
    set<std::uint16_t>(base, 4, 54);
    set<std::uint32_t>(base, 56, 123);
    set<std::uint32_t>(base, 108, 0x12340400);
    set<std::uint16_t>(base, 112, 14);
    auto decode = [&](const Json &links) { return decode_native_text_style(base, links); };
    auto empty = decode(Json::array());
    check(empty.at("status") == "decoded" && empty.at("font_reference_status") == "resolved" &&
              empty.at("initial_values").at("big_font_id") == 0 &&
              empty.at("initial_values").at("color") == 123 &&
              empty.at("initial_values").at("font_id") == 0x12340400 &&
              empty.at("initial_values").at("font_lookup_id") == 1024 &&
              empty.at("unassigned_state").at("auxiliary_scalar") == 1.,
          "native style starts with source header values and confirmed initial defaults");
    const std::uint16_t flags = 0xff0f;
    const std::uint32_t extended =
        1u | 0x8000u | 0x10000u | 0x20000u | 0x40000u | 0x80000u | 0x200000u | 0x400000u;
    Bytes full;
    put(full, flags);
    put<std::uint16_t>(full, 0x5678);
    put(full, extended);
    put<double>(full, 2.5);                   // spacing
    put<double>(full, 3.141592653589793 * 3); // one native subtraction
    put<double>(full, 3.5);                   // underline offset
    put<double>(full, 4.5);
    put<double>(full, 5.5);               // line offset
    put<std::uint32_t>(full, 6);          // unnamed bit9
    put<std::uint32_t>(full, 0xabc00201); // full big key
    put<std::uint32_t>(full, 7);
    put<std::uint32_t>(full, 0xfffffffd); // background color/style
    put<std::uint32_t>(full, 9);          // weight
    put<double>(full, 10.5);
    put<double>(full, 11.5);      // border
    put<std::uint32_t>(full, 12); // fill color
    put<double>(full, 13.5);      // overline offset
    put<std::uint32_t>(full, 14); // unnamed bit15
    for (unsigned value : {15, 16, 17, 18, 19, 20})
        put(full, value);
    put<double>(full, 21.5); // unnamed extended bit19
    for (std::uint16_t value : {0xffff, 0xffff, 0xffff, 0xffff})
        put(full, value);
    put<std::uint32_t>(full, 22); // color override
    for (unsigned value : {23, 24, 25})
        put(full, value);
    put<double>(full, 26.5);
    put<double>(full, 27.5);
    check(full.size() == 172, "complete style fixture has the expected source extent");
    auto result = decode(Json::array({link(0x80d4, full)}));
    const auto &v = result.at("initial_values");
    check(result.at("status") == "decoded" && result.at("semantic_status") == "partial" &&
              v.at("big_font_id") == 0xabc00201 && v.at("big_font_lookup_id") == 513 &&
              v.at("color") == 22 && v.at("justification_value") == 14,
          "source font ID stays full width while character mapping uses low-word query");
    check(v.at("line_offset") == Json::array({4.5, 5.5}) &&
              v.at("background_border") == Json::array({10.5, 11.5}) &&
              v.at("background_color") == 7 && v.at("background_fill_color") == 12 &&
              v.at("underline_color") == 15 && v.at("overline_color") == 18 &&
              v.at("underline_offset") == 3.5 && v.at("overline_offset") == 13.5,
          "native linkage order maps independent background underline and overline parameters");
    check(v.at("inter_character_spacing_mode") == 1 && v.at("inter_character_spacing") == 2.5 &&
              std::abs(v.at("custom_slant_angle").get<double>() - 3.141592653589793) < 1e-14,
          "fixed spacing wins over Acad spacing and slant performs exactly one subtraction");
    const auto &fields = result.at("links")[0].at("fields");
    check(fields[2].at("source_offset") == 4 && fields[9].at("source_offset") == 52 &&
              fields[9].at("name") == "big_font_id" && fields[11].at("signed_value") == -3 &&
              fields.back().at("source_offset") == 156,
          "source offsets refer to linkage payload and signed line style retains original bits");
    for (unsigned bit : {1u, 2u, 0u}) {
        Bytes spacing;
        put<std::uint16_t>(spacing, bit);
        put<std::uint16_t>(spacing, 0);
        put<std::uint32_t>(spacing, 0x400000);
        put<double>(spacing, 4.);
        auto value = decode(Json::array({link(0x80d4, spacing)})).at("initial_values");
        check(value.at("inter_character_spacing_mode") == (bit == 2 ? 1 : 2) &&
                  value.at("inter_character_spacing") == 4.,
              "spacing gates use native bit precedence");
    }
    Bytes angle(8);
    set<std::uint16_t>(angle, 0, 8);
    put<double>(angle, -3.141592653589793 * 3);
    check(
        decode(Json::array({link(0x80d4, angle)})).at("initial_values").at("custom_slant_angle") ==
            -3.141592653589793 * 3,
        "negative slant is not normalized with modulo arithmetic");
    Bytes override_flags;
    put<std::uint16_t>(override_flags, 7);
    put<std::uint16_t>(override_flags, 8);
    put<double>(override_flags, 1.75);
    put<std::uint16_t>(override_flags, 0);
    Bytes auxiliary;
    put<std::uint32_t>(auxiliary, 0x87654321);
    auto non_user = link(0x80d4, full);
    non_user["header"] = 0;
    result =
        decode(Json::array({link(0x56f3, override_flags, 10), non_user, link(0x5817, auxiliary, 20),
                            link(0x80d4, full, 30), link(0x80d4, Bytes{}, 40)}));
    check(result.at("links")[0].at("link_index") == 3 &&
              result.at("links")[1].at("reader_selection") == "shadowed" &&
              result.at("links")[2].at("app") == 0x56f3 &&
              result.at("unassigned_state").at("control_words") ==
                  Json::array({0xfe13, 0xfdff, 0xfffe, 0xffef}) &&
              result.at("unassigned_state").at("auxiliary_scalar") == 1.75 &&
              result.at("unassigned_state").at("auxiliary_flags") == 0x87654321 &&
              (result.at("effective_flags32").get<unsigned>() & 0x10000000),
          "native getter order overrides control bits independently of physical linkage order");
    for (unsigned bit = 0; bit < 16; ++bit) {
        set<std::uint16_t>(override_flags, 12, 1u << bit);
        result = decode(Json::array({link(0x56f3, override_flags)}));
        Json expected = Json::array({0, 0, 0, 0});
        if (bit < 2)
            expected[0] = 1u << (bit + 2);
        else if (bit < 6)
            expected[0] = 1u << (bit + 3);
        else if (bit == 6)
            expected[2] = 1;
        else if (bit == 7)
            expected[1] = 0x200;
        else if (bit == 8)
            expected[3] = 0x10;
        check(result.at("unassigned_state").at("control_words") == expected,
              "packed extension flags have the native sparse destination bit mapping");
    }
    for (std::size_t size = 1; size < full.size(); ++size) {
        result = decode(Json::array({link(0x80d4, slice(full, 0, size)), link(0x80d4, full)}));
        std::uint32_t keep = size < 16 ? 0xff90fffeu : 0xffd0fffeu;
        if (size >= 96)
            keep |= 1;
        if (size >= 112)
            keep |= 0x10000;
        if (size >= 124)
            keep |= 0x20000;
        if (size >= 132)
            keep |= 0x80000;
        if (size >= 140)
            keep |= 0x40000;
        if (size >= 144)
            keep |= 0x200000;
        check(result.at("status") == "partial" &&
                  result.at("effective_flags16") == (size < 2 ? 0u : flags & keep & 0xffffu) &&
                  result.at("effective_flags32") == (size < 8 ? 0u : extended & keep) &&
                  result.at("font_reference_status") == (size < 56 ? "unresolved" : "resolved") &&
                  result.at("links")[1].at("status") == "not_selected",
              "truncated first linkage retains native prefix state and never selects the second");
    }
    result = decode(Json::array({link(0x80d4, Bytes{}), link(0x80d4, full)}));
    check(result.at("status") == "decoded" && result.at("initial_values").at("big_font_id") == 0,
          "empty first style linkage is selected without falling through");
    auto suffix = full;
    suffix.insert(suffix.end(), {0xab, 0xcd});
    check(decode(Json::array({link(0x80d4, suffix)})).at("links")[0].at("ignored_suffix_hex") ==
              "abcd",
          "unconsumed linkage tail is retained rather than parsed as another optional field");
    result = decode(Json::array({link(0x80d4, full), link(0x56f3, slice(override_flags, 0, 12))}));
    check(result.at("status") == "partial" && result.at("font_reference_status") == "resolved" &&
              result.at("unassigned_state").at("auxiliary_scalar") == 1.75 &&
              result.at("unassigned_state").at("control_words") ==
                  Json::array({0xffff, 0xffff, 0xffff, 0xffff}),
          "later extension truncation preserves earlier scalar update and known font references");
    check(decode_native_text_style(Bytes(8), Json::array()).at("status") == "invalid",
          "short base cannot produce synthetic source style values");
    // Exercise the public source-record path, including real linkage framing.
    Bytes record = base;
    set<std::uint32_t>(record, 8, (base.size() - 4 + 4 + full.size()) / 2);
    set<std::uint32_t>(record, 12, (base.size() - 4) / 2);
    put<std::uint16_t>(record, 0x1000 | ((4 + full.size()) / 2 - 1));
    put<std::uint16_t>(record, 0x80d4);
    record.insert(record.end(), full.begin(), full.end());
    const auto records = parse_native(record);
    check(records.size() == 1 &&
              records[0].at("native_text_style").at("initial_values").at("big_font_lookup_id") ==
                  513 &&
              records[0].at("links")[0].at("payload") == rawbytes(full),
          "native record parsing connects the style without altering source linkage data");
    return checks;
}
