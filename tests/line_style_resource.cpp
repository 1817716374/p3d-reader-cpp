#include "internal.hpp"
#include "line_style_source.hpp"
#include <p3d/line_style.hpp>

unsigned line_style_resource_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool v, const char *why) {
        require(v, why);
        ++checks;
    };
    auto put = [](Bytes &b, std::size_t p, auto value) {
        std::memcpy(b.data() + p, &value, sizeof(value));
    };
    auto record = [&](unsigned type, unsigned subtype, std::size_t size) {
        Bytes b(size, 0);
        put(b, 4, std::uint16_t(type));
        put(b, 8, std::uint32_t((size - 4) / 2));
        put(b, 12, std::uint32_t((size - 4) / 2));
        put(b, 16, std::uint32_t(subtype));
        put(b, 20, std::uint64_t(0x123456789abcdef0ull));
        return b;
    };
    auto query = [&](const Bytes &b) {
        return read_native_line_style_resource({{"data", rawbytes(b)}});
    };
    auto b = record(49, 7, 228);
    put(b, 48, std::uint16_t(1));
    put(b, 52, std::int32_t(-123));
    put(b, 56, std::int32_t(-37500));
    put(b, 68, std::uint32_t(0x87654321));
    put(b, 84, std::uint64_t(0xfedcba9876543210ull));
    put(b, 92, std::uint16_t(3));
    put(b, 96, std::int32_t(-2));
    put(b, 100, std::uint16_t(0x7ebf));
    auto j = query(b);
    check(j.at("status") == "decoded" && j.at("name").at("value") == u8"线" &&
              j.at("line_style_number") == -2 && j.at("creates_local_definition") == true &&
              j.at("root_component").at("source_id") == 0xfedcba9876543210ull &&
              j.at("root_component").at("resource_type") == 2002 &&
              j.at("definition_flags") == 0x87654321u &&
              j.at("native_numeric_parameters")[0].at("value") == -123 &&
              j.at("native_numeric_parameters")[1].at("value") == -37500 &&
              j.at("native_parameter_ratio") == -0.375,
          "saved line-style name preserves number, full ID, flags and source parameters");
    for (auto v : {INT32_MIN, -100001, -100000, -1, 0, 1, 100000, INT32_MAX}) {
        put(b, 56, std::int32_t(v));
        const auto result = query(b);
        check(result.at("native_numeric_parameters")[1].at("value") == v &&
                  result.at("native_parameter_ratio") == double(v) / 100000.0,
              "definition parameter uses signed 32-bit conversion before its native ratio");
    }
    for (unsigned kind : {0u, 1u, 2u, 3u, 4u, 5u, 6u, 2005u, 65535u}) {
        put(b, 92, std::uint16_t(kind));
        const auto expected = kind == 1   ? 2004u
                              : kind == 2 ? 2001u
                              : kind == 3 ? 2002u
                              : kind == 4 ? 2003u
                              : kind == 6 ? 0u
                                          : kind;
        check(query(b).at("root_component").at("resource_type") == expected,
              "native resource type conversion preserves unmatched kinds");
    }
    put(b, 48, std::uint16_t(2));
    put(b, 96, std::int32_t(0));
    check(query(b).at("creates_local_definition") == false && query(b).at("line_style_number") == 0,
          "definition eligibility uses only low flag bit and does not discard zero mapping");
    for (unsigned i = 0; i < 64; ++i)
        put(b, 100 + i * 2, std::uint16_t('a'));
    check(query(b).at("status") == "partial" && query(b).at("name").at("value").is_null(),
          "unterminated name does not cross saved field into adjacent data");
    put(b, 226, std::uint16_t(0));
    check(query(b).at("name").at("value").get<std::string>().size() == 63,
          "63 UTF-16 units and a terminator fit the saved name field");
    b = record(49, 7, 228);
    put(b, 100, std::uint16_t(0xd83d));
    put(b, 102, std::uint16_t(0xde00));
    check(query(b).at("name").at("value") == u8"😀", "valid supplementary character is decoded");
    put(b, 102, std::uint16_t(0));
    check(query(b).at("name").at("value").is_null() && !query(b).dump().empty(),
          "malformed UTF-16 remains dumpable with its complete source units");
    for (unsigned subtype : {7u, 8u})
        for (bool extended : {false, true}) {
            b = record(10, subtype, extended ? 116 : 44);
            put(b, 6, std::uint16_t(extended ? 0x20 : 0));
            put(b, extended ? 108 : 36, std::uint32_t(11));
            j = query(b);
            check(j.at("status") == "decoded" && j.at("declared_descendant_count") == 11 &&
                      j.at("descendant_count_source_offset") == (extended ? 108 : 36),
                  "both resource tables honor the saved extended descendant count position");
        }
    // Independent expected values deliberately differ between start/end widths,
    // saved payload size and count, and each bit field.
    for (unsigned mode = 0; mode < 256; ++mode)
        for (unsigned width = 0; width < 8; ++width) {
            b = record(49, 8, 492);
            put(b, 70, std::uint16_t(3));
            put(b, 200, std::uint16_t(65535)); // not a line-code stroke count
            put(b, 204, std::uint16_t(1));     // not the native consumed extent
            put(b, 352, 0.125);
            put(b, 360, -4.0);
            put(b, 368, 5.0);
            put(b, 400, std::uint32_t(mode));
            put(b, 404, std::uint32_t(37));
            put(b, 412, std::uint32_t(1));
            put(b, 416, 1.5);
            put(b, 424, 2.5);
            put(b, 432, 3.5);
            put(b, 460, std::uint8_t(mode));
            put(b, 461, std::uint8_t(width));
            put(b, 462, std::uint8_t(251));
            j = query(b);
            const auto &c = j.at("line_code"), &s = c.at("strokes")[0];
            check(j.at("status") == "decoded" && c.at("stroke_count") == 1 &&
                      c.at("native_consumed_bytes") == 256 && c.at("iteration_limit") == 37 &&
                      c.at("native_phase_mode") == (mode % 2 ? 2 : 1) &&
                      c.at("phase").at("value") == 0.125 && s.at("length").at("value") == 1.5 &&
                      s.at("is_dash") == bool(mode % 2) && s.at("start_width").at("value") == 2.5 &&
                      s.at("saved_end_width").at("value") == 3.5 &&
                      s.at("effective_end_width").at("value") == (width / 4 ? 3.5 : 2.5) &&
                      s.at("caps_type") == 251 && s.at("stroke_mode") == mode &&
                      s.at("width_mode") == width &&
                      c.at("unconsumed_suffix").at("data").at("bytes") == 28,
                  "line-code native count, independent widths, dash selection and suffix oracle");
        }
    b = record(49, 8, 416);
    put(b, 70, std::uint16_t(3));
    for (const double phase : {0.0, -0.0, 0.5, -0.25, std::numeric_limits<double>::quiet_NaN()})
        for (unsigned options : {0u, 1u, 8u, 16u, 32u, 33u, 0xffffffffu}) {
            put(b, 352, phase);
            put(b, 400, std::uint32_t(options));
            const auto c = query(b).at("line_code");
            const auto expected = options % 2 ? 2 : phase != 0.0 ? 1 : options & 32 ? 3 : 0;
            check(c.at("status") == "decoded" && c.at("native_phase_mode") == expected &&
                      c.at("native_option_bits").at("bit_3") == bool(options & 8) &&
                      c.at("native_option_bits").at("bit_4") == bool(options & 16) &&
                      !c.dump().empty(),
                  "phase mode priority handles signed zero, NaN and option bit combinations");
        }
    put(b, 412, std::uint32_t(0xffffffff));
    check(query(b).at("line_code").at("status") == "unresolved",
          "oversize stroke count is bounded");
    put(b, 70, std::uint16_t(6));
    check(query(b).at("status") == "partial" &&
              query(b).at("component_payload").at("status") == "unresolved",
          "internal component is not silently interpreted as a line code");
    for (std::size_t size : {0u, 35u, 36u, 70u, 100u, 207u, 227u}) {
        auto cut = record(49, 7, 228);
        cut.resize(size);
        check(query(cut).at("status") == "unresolved", "truncated name record remains unresolved");
    }
    b = record(49, 7, 228);
    auto parsed = parse_native(b);
    check(parsed.size() == 1 && parsed[0].at("line_style_name") == query(b),
          "automatic native records expose the same raw-source name decode");
    b = record(49, 8, 416);
    put(b, 70, std::uint16_t(3));
    parsed = parse_native(b);
    check(parsed[0].at("line_style_component") == query(b), "automatic component decode agrees");
    auto forged = Json{{"data", rawbytes(b)},
                       {"element_type", 10},
                       {"line_style_component", {{"status", "wrong"}}}};
    check(read_native_line_style_resource(forged) == query(b),
          "query re-reads raw source instead of annotations");
    return checks;
}
