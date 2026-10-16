#include "internal.hpp"
#include <p3d/line_style.hpp>

unsigned line_style_point_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool v, const char *why) {
        require(v, why);
        ++checks;
    };
    auto put = [](Bytes &b, std::size_t p, auto value) {
        std::memcpy(b.data() + p, &value, sizeof(value));
    };
    auto make = [&](unsigned count, unsigned refs, std::size_t size = 0) {
        const auto extent = count ? 168 + count * 88 : 256;
        Bytes b(size ? size : 208 + extent + 8 * std::max(refs, count + 1) + 28, 0);
        put(b, 4, std::uint16_t(49));
        put(b, 8, std::uint32_t((b.size() - 4) / 2));
        put(b, 12, std::uint32_t((b.size() - 4) / 2));
        put(b, 16, std::uint32_t(8));
        put(b, 70, std::uint16_t(4));
        put(b, 200, std::uint16_t(refs));
        put(b, 202, std::uint16_t(refs * 8));
        put(b, 204, std::uint16_t(extent));
        put(b, 360, std::uint32_t(2002));
        put(b, 364, std::uint32_t(777));
        put(b, 372, std::uint32_t(count));
        for (unsigned i = 0; i < count; ++i) {
            put(b, 376 + i * 88, std::uint32_t(2005));
            put(b, 380 + i * 88, std::uint32_t(901 + i));
            put(b, 384 + i * 88, std::uint16_t(65535));
            put(b, 386 + i * 88, std::uint16_t(1));
            put(b, 392 + i * 88, -1.25);
            put(b, 400 + i * 88, 4.5);
            put(b, 432 + i * 88, 90.0);
        }
        return b;
    };
    auto read = [](const Bytes &b) {
        return read_native_line_style_resource({{"data", rawbytes(b)}});
    };
    const std::array<unsigned, 32> roles{{4, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 0, 1, 0, 0, 0,
                                          3, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 0, 1, 0, 0, 0}};
    const std::array<unsigned, 4> modes{{0, 2, 1, 2}};
    const std::array<const char *, 5> role_names{
        {"stroke", "element_origin", "element_end", "each_vertex", "not_retained"}};
    auto b = make(1, 2);
    put(b, 464, std::uint64_t(0xfedcba9800000033ull));
    put(b, 472, std::uint64_t(0x1234567800000066ull));
    unsigned iteration = 0;
    for (unsigned low = 0; low < 32; ++low)
        for (unsigned a = 0; a < 4; ++a)
            for (unsigned c = 0; c < 4; ++c)
                for (unsigned booleans = 0; booleans < 16; ++booleans) {
                    const auto ignored = iteration++ % 8;
                    const auto flags = low | (c << 5) | (a << 10) | ((booleans & 3) << 8) |
                                       ((booleans & 12) << 12) | ((ignored & 1) << 7) |
                                       ((ignored & 6) << 11);
                    put(b, 386, std::uint16_t(flags));
                    const auto result = read(b), &p = result.at("line_point"),
                               &s = p.at("symbols")[0];
                    const auto &state = s.at("initial_placement"),
                               &obj = p.at("initial_object_state");
                    const auto role = roles[low];
                    const Json options = {
                        {"stroke_position", low % 4},
                        {"at_element_origin", bool(low & 4)},
                        {"at_element_end", bool(low & 8)},
                        {"at_each_vertex", bool(low & 16)},
                        {"mirror_on_reversed_line", bool(c & 1)},
                        {"absolute_rotation", bool(c & 2)},
                        {"do_not_scale", bool(booleans & 1)},
                        {"do_not_clip", bool(booleans & 2)},
                        {"no_partial_strokes", bool(a & 1)},
                        {"partial_origin_beyond_end", bool(a & 2)},
                        {"use_symbol_color", bool(booleans & 4)},
                        {"use_symbol_weight", bool(booleans & 8)},
                        {"unassigned_modifier_bits", (ignored & 1) * 128 + (ignored >> 1) * 4096}};
                    bool valid =
                        p.at("source_status") == "decoded" && s.at("modifiers") == flags &&
                        s.at("source_options") == options &&
                        state.at("placement") == role_names[role] &&
                        state.at("native_placement_kind") == role &&
                        state.at("stroke_position") == (low % 4 ? low % 4 : 3) &&
                        state.at("native_partial_stroke_mode") == modes[a] &&
                        state.at("native_rotation_mode") == modes[c] &&
                        state.at("scale_by_line_style") == !(booleans & 1) &&
                        state.at("clip_at_element_end") == !(booleans & 2) &&
                        state.at("use_symbol_color") == bool(booleans & 4) &&
                        state.at("use_symbol_weight") == bool(booleans & 8) &&
                        std::abs(state.at("rotation_radians").get<double>() - std::acos(-1.0) / 2) <
                            2e-15 &&
                        s.at("x_offset").at("value") == -1.25 &&
                        s.at("y_offset").at("value") == 4.5 && s.at("stroke_number") == 65535 &&
                        p.at("copy_reader_fields").at("status") == "resolved";
                    if (role == 4)
                        valid = valid && obj.at("status") == "unresolved" &&
                                obj.at("retained_symbol_indices").empty() &&
                                obj.at("final_placements").is_null() &&
                                obj.at("assignments")[0].at("action") ==
                                    "rejected_compacted_vector_access" &&
                                result.at("status") == "partial";
                    else
                        valid = valid && obj.at("status") == "resolved" &&
                                result.at("status") == "decoded" &&
                                obj.at("final_placements")[0].at("source_symbol_index") == 0 &&
                                obj.at("final_placements")[0].at("resource_id") ==
                                    0x1234567800000066ull;
                    check(valid, "independent placement priority/mode tables and all consumed flag "
                                 "combinations");
                }
    const std::array<std::pair<std::uint32_t, std::uint32_t>, 13> kinds{{{0, 6},
                                                                         {1, 1},
                                                                         {2, 2},
                                                                         {3, 3},
                                                                         {4, 4},
                                                                         {6, 6},
                                                                         {2001, 2},
                                                                         {2002, 3},
                                                                         {2003, 4},
                                                                         {2004, 1},
                                                                         {2005, 1},
                                                                         {0x80000000u, 0x80000000u},
                                                                         {UINT32_MAX, UINT32_MAX}}};
    for (const auto &[base, base_mapped] : kinds)
        for (const auto &[sym, sym_mapped] : kinds)
            for (unsigned mask = 0; mask < 4; ++mask)
                for (unsigned used = 0; used < 4; ++used) {
                    b = make(1, used);
                    put(b, 360, base);
                    put(b, 376, sym);
                    const auto first = mask & 1 ? 0xffffffff00000000ull : 0ull;
                    const auto second = mask & 2 ? 0x123456780000002aull : 0ull;
                    put(b, 464, first);
                    put(b, 472, second);
                    const auto before = b;
                    const auto result = read(b);
                    const auto &p = result.at("line_point"), &copy = p.at("copy_reader_fields"),
                               &obj = p.at("initial_object_state");
                    bool valid =
                        b == before && obj.at("status") == "resolved" &&
                        obj.at("base_pattern").at("kind") == base_mapped &&
                        obj.at("base_pattern").at("resource_id") == (first ? first : 777ull) &&
                        obj.at("final_placements")[0].at("resource_id") == second &&
                        obj.at("symbol_type_selection") == "not_consumed_by_fresh_symbol_reader";
                    if (used == 3)
                        valid = valid && copy.at("status") == "unresolved" &&
                                copy.at("fixups").back().at("action") == "rejected_slot_read" &&
                                copy.at("final_symbols").is_null();
                    else
                        valid = valid && copy.at("status") == "resolved" &&
                                copy.at("final_base_pattern").at("type_value") ==
                                    (base ? base_mapped : 0u) &&
                                copy.at("final_base_pattern").at("local_resource_id") ==
                                    (base ? std::uint32_t(first) : 777u) &&
                                copy.at("final_symbols")[0].at("type_value") ==
                                    (used >= 2 ? sym_mapped : sym) &&
                                copy.at("final_symbols")[0].at("local_resource_id") ==
                                    (used >= 2 ? std::uint32_t(second) : 901u);
                    check(valid, "line-point zero/full IDs, type aliases, independent counts and "
                                 "no zero-reference skip");
                }
    b = make(2, 3);
    put(b, 386, std::uint16_t(0)); // First source suppressed; second source retained.
    put(b, 552, std::uint64_t(10));
    put(b, 560, std::uint64_t(20));
    put(b, 568, std::uint64_t(30));
    auto j = read(b).at("line_point"), obj = j.at("initial_object_state");
    check(j.at("source_status") == "decoded" && obj.at("status") == "unresolved" &&
              obj.at("retained_symbol_indices") == Json::array({1}) &&
              obj.at("assignments")[0].at("source_symbol_index") == 1 &&
              obj.at("assignments")[0].at("resource_id") == 20 &&
              obj.at("assignments")[1].at("action") == "rejected_compacted_vector_access" &&
              obj.at("final_placements").is_null(),
          "native compacted vector keeps original-count reference assignment and exposes its "
          "failed suffix");
    b = make(0, 0);
    j = read(b).at("line_point");
    check(j.at("status") == "decoded" && j.at("symbols").empty() &&
              j.at("copy_reader_fields").at("unused_slot_storage").at("bytes") == 88 &&
              j.at("initial_object_state").at("final_placements").empty(),
          "empty line-point still copies one physical symbol slot and reads its base reference");
    b = make(0, 2);
    put(b, 376, std::uint32_t(0));
    put(b, 380, std::uint32_t(7));
    j = read(b).at("line_point");
    const auto storage = bytesof(j.at("copy_reader_fields").at("unused_slot_storage"));
    check(j.at("status") == "decoded" && Reader(storage).at<std::uint32_t>(0) == 6 &&
              Reader(storage).at<std::uint32_t>(4) == 0,
          "unused physical symbol slot is fixed even for a zero reference without adding logical "
          "symbols");
    b = make(0, 0, 464);
    put(b, 360, std::uint32_t(0));
    j = read(b).at("line_point");
    check(j.at("copy_reader_fields").at("status") == "resolved" &&
              j.at("initial_object_state").at("status") == "unresolved",
          "zero base type skips copy reference but object entry still reads it");
    b = make(0, 0, 464);
    j = read(b).at("line_point");
    check(j.at("copy_reader_fields").at("fixups")[0].at("action") ==
                  "type_converted_reference_unread" &&
              j.at("copy_reader_fields").at("fixups")[0].at("type_value") == 3,
          "base type conversion precedes an unsafe reference read and its known effect remains "
          "visible");
    b = make(1, 2, 472);
    j = read(b).at("line_point");
    check(j.at("copy_reader_fields").at("status") == "unresolved" &&
              j.at("copy_reader_fields").at("fixups").back().at("action") ==
                  "type_converted_reference_unread" &&
              j.at("copy_reader_fields").at("fixups").back().at("type_value") == 1 &&
              j.at("initial_object_state").at("assignments").back().at("action") ==
                  "reference_unread",
          "symbol type writes precede missing reference reads and preserve independent route "
          "failures");
    b = make(1, 2);
    put(b, 204, std::uint16_t(168));
    j = read(b).at("line_point");
    check(j.at("status") == "decoded" &&
              j.at("initial_object_state").at("base_pattern").at("resource_id") ==
                  (901ull << 32 | 2005ull) &&
              j.at("initial_object_state").at("final_placements")[0].at("resource_id") == 131071 &&
              j.at("copy_reader_fields").at("final_symbols")[0].at("local_resource_id") == 131071,
          "overlapping references use immutable source fields instead of previously converted copy "
          "fields");
    b = make(1, 2);
    put(b, 202, std::uint16_t(0));
    check(read(b).at("line_point").at("status") == "decoded",
          "saved capacity does not clamp either reader");
    b = make(1, 2);
    put(b, 392, std::numeric_limits<double>::infinity());
    put(b, 432, std::numeric_limits<double>::quiet_NaN());
    j = read(b).at("line_point");
    check(j.at("symbols")[0].at("x_offset").at("value").is_null() &&
              j.at("symbols")[0].at("initial_placement").at("rotation_radians").is_null() &&
              !j.dump().empty(),
          "nonfinite placement values retain source and dumpable diagnostics");
    put(b, 372, UINT32_MAX);
    check(read(b).at("line_point").at("status") == "unresolved",
          "oversize symbol count is bounded");
    b = make(1, 2);
    auto parsed = parse_native(b);
    check(parsed.size() == 1 && parsed[0].at("line_style_component") == read(b),
          "automatic line-point source decode agrees");
    return checks;
}
