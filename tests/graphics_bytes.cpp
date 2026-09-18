#include "blob_internal.hpp"
#include <cstring>
#include <future>
namespace {
using namespace p3d;
template <class T> void put(Bytes &b, T v) {
    auto p = reinterpret_cast<const std::uint8_t *>(&v);
    b.insert(b.end(), p, p + sizeof(v));
}
Bytes material_fixture() {
    Bytes b{1};
    auto text = [&](std::u16string s) {
        put<std::uint64_t>(b, s.size() * 2);
        for (auto c : s)
            put<std::uint16_t>(b, c);
    };
    text(u"part material");
    b.push_back(1);
    for (double x : {.1, .2, .3})
        put(b, x);
    b.push_back(1);
    put(b, .4);
    b.push_back(1);
    put<std::int32_t>(b, 3);
    put<std::int32_t>(b, 2);
    text(u"relative/texture.jpg");
    for (double x : {1., 2., 3., 4., 5.})
        put(b, x);
    text(u"");
    put(b, 0.);
    for (unsigned i = 0; i < 9; ++i) {
        b.push_back(1);
        for (unsigned j = 0; j < (i == 0 || i == 2 ? 3u : 1u); ++j)
            put(b, double(i + j));
    }
    return b;
}
void footer(Bytes &b, const Bytes &m, bool entry) {
    b.push_back(3);
    put<std::uint32_t>(b, m.size());
    b.insert(b.end(), m.begin(), m.end());
    put(b, 2.5);
    if (entry) {
        put(b, 3.5);
        put(b, 4.5);
        put<std::uint32_t>(b, 0xfffffff1u);
    }
}
Bytes entry(const Bytes &m = {}, const Bytes &geometry = {}) {
    Bytes b;
    put<std::int32_t>(b, 0);
    put<std::int32_t>(b, 7);
    put<std::int32_t>(b, -3);
    put<std::uint32_t>(b, 19);
    put<std::uint32_t>(b, 0x00332211);
    put(b, .25);
    put<std::uint64_t>(b, geometry.size());
    b.insert(b.end(), geometry.begin(), geometry.end());
    footer(b, m, true);
    return b;
}
Bytes graphics(const std::vector<Bytes> &entries, bool model = false, const Bytes &m = {}) {
    Bytes b;
    put<std::uint64_t>(b, 1);
    b.push_back(model);
    if (model)
        put<std::uint32_t>(b, 123);
    put<std::int32_t>(b, 5);
    put<std::uint32_t>(b, 11);
    put<std::uint32_t>(b, 0x00665544);
    put<std::int32_t>(b, 1);
    put(b, .75);
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 4; ++j)
            put(b, j == 3 ? double(10 + i) : double(i == j));
    b.push_back(1);
    put<std::uint32_t>(b, 12);
    put<std::uint64_t>(b, entries.size());
    for (const auto &e : entries) {
        put<std::uint64_t>(b, e.size());
        b.insert(b.end(), e.begin(), e.end());
    }
    footer(b, m, false);
    return b;
}
} // namespace
unsigned graphics_bytes_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto rejects = [&](const Bytes &b) {
        bool threw = false;
        try {
            decode_graphics_bytes(b);
        } catch (const std::exception &) {
            threw = true;
        }
        check(threw, "invalid graphics framing rejected");
    };
    const auto mat = material_fixture();
    const auto e = entry(mat, Bytes{1, 2, 3});
    const auto data = graphics({e, entry()}, true, mat);
    const auto j = decode_graphics_bytes(data);
    const auto &p = j["geometry_packets"][0];
    check(j["model_id"] == 123 && j["includes_model_id"] == true,
          "optional model shifts complete header");
    check(j["symbology"]["line_style"] == 5 && j["symbology"]["line_weight"] == 11 &&
              j["symbology"]["resolved_color"] == 0x00665544,
          "graphics symbology fields in native order");
    check(j["symbology_source_name"] == "entity" && j["transparency"] == .75,
          "graphics display context");
    check(j["transform_enabled"] == true && j["transform_3x4_rows"][2][3] == 12 &&
              j["layer_id"] == 12,
          "transform and layer identified");
    check(p["entry_index"] == 0 && j["geometry_packets"][1]["entry_index"] == 1 &&
              p["source_offset"] == 154 &&
              j["geometry_packets"][1]["source_offset"] == 162 + e.size(),
          "64-bit lengths delimit ordered entries");
    check(p["raw_base64"] == base64(e) && p["header_hex"].get<std::string>().size() == 56,
          "entry bytes exclude length high word and include layer ID");
    check(p["symbology_source_name"] == "part" && p["geometry_type_name"] == "Text" &&
              p["transparency"] == .25,
          "entry type and display context");
    check(p["symbology"]["line_style"] == -3 && p["symbology"]["line_weight"] == 19 &&
              p["symbology"]["resolved_color"] == 0x00332211,
          "entry style separated from graphics style");
    check(p["geometry_offset"] == 36 && p["geometry_bytes"] == 3 &&
              p["geometry_status"] == "unsupported_encoding",
          "unsupported geometry retained without guessed text decoding");
    check(p["inline_material"]["name"] == "part material" &&
              p["inline_material"]["texture_references"][0]["filename"] == "relative/texture.jpg",
          "entry material and external texture reference decoded");
    check(j["inline_material"]["parameters"] == p["inline_material"]["parameters"],
          "graphics and entry material independently decoded");
    check(p["line_style_scale"] == 2.5 && p["start_width"] == 3.5 && p["end_width"] == 4.5 &&
              p["layer_id"] == 0xfffffff1u,
          "entry footer fields retain unsigned layer ID");
    check(j["line_style_scale"] == 2.5 && p["view_flag_status"] == "not_serialized" &&
              j["view_flag_status"] == "not_serialized",
          "runtime view flags are not invented");
    check(j["owning_element_part_mapping_status"] == "not_established",
          "entry index does not invent an owning element material association");
    const auto boundary = decode_attribute(106, 10001, data, 0);
    check(boundary["role"] == "view_link_boundary_geometry" &&
              boundary["geometry_packets"] == j["geometry_packets"],
          "view boundary attribute routed to same decoder");
    auto without = decode_graphics_bytes(graphics({}));
    check(without["model_id"].is_null() && without["geometry_packets"].empty(),
          "zero entry graphics without model ID");
    auto damaged = data;
    damaged[0] = 2;
    rejects(damaged);
    damaged = data;
    damaged[150] = 1;
    rejects(damaged); // High DWORD of the first entry length.
    damaged = data;
    damaged[142] = 1;
    rejects(damaged); // High DWORD of entry count.
    for (auto n : {0u, 7u, 8u, 12u, 36u, 132u, 137u, 145u, 153u})
        rejects(slice(data, 0, n));
    auto short_entry = e;
    short_entry.resize(20);
    auto short_j = decode_graphics_bytes(graphics({short_entry, entry()}));
    check(short_j["geometry_packets"][0].contains("decode_error") &&
              short_j["geometry_packets"][1]["entry_index"] == 1,
          "damaged entry does not shift later boundaries");
    auto bad_geo = e;
    bad_geo[32] = 1;
    check(
        decode_graphics_bytes(graphics({bad_geo}))["geometry_packets"][0].contains("decode_error"),
        "geometry length high DWORD validated");
    auto bad_mat = entry(Bytes{1});
    const auto invalid_mat = decode_graphics_bytes(graphics({bad_mat}))["geometry_packets"][0];
    check(invalid_mat["inline_material"].contains("decode_error") &&
              invalid_mat["layer_id"] == 0xfffffff1u,
          "invalid bounded material does not hide trailing layer");
    auto legacy = entry();
    legacy.resize(36);
    legacy.insert(legacy.end(), mat.begin(), mat.end());
    const auto lj = decode_graphics_bytes(graphics({legacy}))["geometry_packets"][0];
    check(lj["footer_format"] == "legacy_inline_material" &&
              lj["inline_material"]["name"] == "part material" && !lj.contains("layer_id"),
          "legacy material has no invented new footer fields");
    auto future = entry();
    future.push_back(0x5a);
    check(decode_graphics_bytes(
              graphics({future}))["geometry_packets"][0]["unassigned_suffix_hex"] == "5a",
          "future entry suffix preserved");
    auto missing = entry();
    missing.resize(36);
    check(decode_graphics_bytes(graphics({missing}))["geometry_packets"][0]["footer_status"] ==
              "not_present",
          "absent legacy footer explicitly reported");
    auto f = std::async(std::launch::async, [&] { return decode_graphics_bytes(data); });
    check(f.get() == j && data == graphics({e, entry()}, true, mat),
          "decoder is reentrant and preserves source");

    // The enclosing instance list has a variable-length typed suffix of its own.
    auto component = [](const Bytes &tail) {
        Bytes b(15); // null parent and no instances
        b.insert(b.end(), tail.begin(), tail.end());
        return b;
    };
    auto text = [](Bytes &b, const Bytes &v) {
        put<std::uint32_t>(b, v.size());
        b.insert(b.end(), v.begin(), v.end());
    };
    Bytes tail;
    text(tail, Bytes{'M', 'a', 't', 0, 'X'});
    text(tail, Bytes{0xc4, 0xe3});
    text(tail, Bytes{0, 1, 2});
    tail.push_back(9);
    put<std::uint32_t>(tail, 7);
    auto value = [&](std::uint64_t key, unsigned type, const Bytes &data) {
        put(tail, key);
        tail.push_back(std::uint8_t(type));
        text(tail, data);
    };
    Bytes v;
    put<std::int64_t>(v, -9000000000000000000LL);
    value(123, 1, v);
    v.clear();
    put(v, 1.25);
    value(124, 2, v);
    value(125, 3, Bytes{1});
    value(126, 4, Bytes{'a', 0, 'b'});
    value(127, 5, Bytes{0, 0xff});
    value(128, 6, {});
    value(129, 31, Bytes{7, 8});
    put<std::uint32_t>(tail, 2);
    put<std::uint64_t>(tail, 0xf123456789abcdefULL);
    put<std::uint64_t>(tail, 3);
    put<std::uint32_t>(tail, 1);
    put<std::uint64_t>(tail, 44);
    const auto cf = complex_blob("ParaCmptInstance", component(tail));
    const auto &ft = cf["footer"];
    check(cf["footer_hex"] == hex(tail) && ft["source_offset"] == 15,
          "variable component footer retains all source bytes");
    check(ft["material_name_reference"]["text"] == std::string("Mat\0X", 5) &&
              ft["material_name_reference"]["lookup_name"] == "Mat",
          "material lookup uses terminated name within declared field");
    check(ft["material_name_reference"]["comparison"] == "case_sensitive_utf16_code_units" &&
              ft["material_name_reference"]["applies_to"] == "all_rebuilt_graphics_entries",
          "container material override has separate native scope and comparison");
    check(ft["remark"]["text"].is_null() && ft["remark"]["status"] == "requires_text_decoder" &&
              ft["remark"]["encoding"] == "not_established",
          "remark text encoding is not guessed from a narrow string");
    check(ft["hollow_byte"] == 9 && ft["hollow"].is_null() &&
              ft["hollow_status"] == "invalid_boolean" && ft["values"].size() == 7,
          "noncanonical hollow flag is preserved without inventing a boolean");
    check(ft["values"][0]["value"] == -9000000000000000000LL && ft["values"][1]["value"] == 1.25 &&
              ft["values"][2]["value"] == true,
          "typed integral floating and boolean values decoded");
    check(ft["values"][3]["value"]["text"] == std::string("a\0b", 3) &&
              ft["values"][4]["kind"] == "binary" && ft["values"][5]["value"].is_null(),
          "string binary and null values distinguished");
    check(ft["values"][6]["kind"] == "unassigned" && ft["values"][6]["key"] == 129,
          "unknown bounded value preserves key and payload");
    check(ft["unassigned_id_sets"][0] == Json({0xf123456789abcdefULL, 3}) &&
              ft["unassigned_id_sets"][1] == Json({44}),
          "trailing sets keep full 64-bit values and source order");
    auto extended = tail;
    extended.push_back(0x9a);
    check(complex_blob("ParaCmptInstance",
                       component(extended))["footer"]["unassigned_suffix_hex"] == "9a",
          "future component suffix retained");
    auto empty = complex_blob("ParaCmptInstance", component(Bytes(25)));
    check(empty["footer"]["material_name_reference"]["lookup_name"] == "" &&
              empty["footer"]["values"].empty(),
          "empty 25-byte form is an instance of general layout");
    check(empty["footer"]["hollow"] == false &&
              ft["attached_data_block"] == rawbytes(Bytes{0, 1, 2}),
          "solid default flag and independent attached block decoded");
    auto semantic_tail = tail;
    semantic_tail[4 + 5 + 4 + 2 + 4 + 3] = 1;
    semantic_tail[4 + 5 + 4] = 'A';
    semantic_tail[4 + 5 + 4 + 1] = 0;
    const auto semantic = complex_blob("ParaCmptInstance", component(semantic_tail))["footer"];
    check(semantic["hollow"] == true && semantic["hollow_status"] == "decoded" &&
              semantic["remark"]["text"] == std::string("A\0", 2) &&
              semantic["remark"]["native_value"]["text"] == "A",
          "hollow flag and terminated native remark keep full source bytes");
    check(ft["value_key_semantics"] == "component_property_id" &&
              ft["values"][0]["property_id"] == 123,
          "value identifiers are generic component property IDs");
    auto bad_footer = [&](const Bytes &b) {
        bool threw = false;
        try {
            complex_blob("ParaCmptInstance", component(b));
        } catch (const std::exception &) {
            threw = true;
        }
        check(threw, "truncated component suffix rejected");
    };
    for (auto n : {0u, 3u, 8u, 14u, 21u, 25u})
        bad_footer(slice(tail, 0, n));
    auto invalid = tail;
    invalid[3] = 0x80;
    bad_footer(invalid);
    invalid = tail;
    // First value has 8+1+4 framing after the three fields, flag and count.
    const std::size_t first_value = 4 + 5 + 4 + 2 + 4 + 3 + 1 + 4;
    std::fill(invalid.begin() + first_value, invalid.begin() + first_value + 8, 0xff);
    const auto signed_id = complex_blob("ParaCmptInstance", component(invalid))["footer"];
    check(signed_id["values"][0]["property_id"] == -1 &&
              signed_id["values"][0]["key"] == std::numeric_limits<std::uint64_t>::max(),
          "signed component property identifier preserves complete source bits");
    invalid[first_value + 8] = 6; // Bounded eight-byte payload is invalid for null.
    auto bad_value = complex_blob("ParaCmptInstance", component(invalid))["footer"];
    check(bad_value["values"][0].contains("decode_error") &&
              bad_value["unassigned_id_sets"] == ft["unassigned_id_sets"],
          "bad bounded typed value does not lose following sets");
    check(ft["stored_value_selection"]["selected_entry_indices"] == Json({0, 1, 2, 3, 4, 5, 6}) &&
              ft["stored_value_selection"]["duplicate_key_rule"] == "first_entry_wins" &&
              ft["stored_value_selection"]["index_order"] == "source_order" &&
              ft["stored_value_selection"]["scope"] == "component_data_footer" &&
              ft["stored_value_selection"]["fallback_to_type_values"] == false,
          "component data getter selects source-first values within its own stored footer");
    check(ft["stored_value_selection"]["missing_key_action"] == "leave_destination_unchanged" &&
              empty["footer"]["stored_value_selection"]["selected_entry_indices"].empty(),
          "missing stored values do not invent a type fallback or a new output value");
    check(ft["values"][3]["value"]["native_value"]["text"] == "a" &&
              ft["values"][3]["value"]["text"] == std::string("a\0b", 3) &&
              ft["values"][3]["native_read_action"] == "set_value",
          "stored string getter uses a bounded NUL prefix while retaining every source byte");
    check(ft["values"][5]["native_read_action"] == "leave_destination_unchanged" &&
              ft["values"][6]["native_read_action"] == "leave_destination_unchanged" &&
              bad_value["values"][0]["native_read_action"] == "leave_destination_unchanged",
          "null and unknown getter branches do not overwrite the caller value or emulate invalid "
          "reads");
    Bytes duplicate_tail(13, 0); // Three empty fields and the hollow byte.
    put<std::uint32_t>(duplicate_tail, 10);
    auto duplicate_value = [&](std::int64_t id, unsigned kind, const Bytes &bytes) {
        put(duplicate_tail, id);
        duplicate_tail.push_back(std::uint8_t(kind));
        text(duplicate_tail, bytes);
    };
    Bytes integer;
    put<std::int64_t>(integer, 73);
    duplicate_value(3, 6, {});
    duplicate_value(-1, 4, Bytes{'a', 0, 'b'});
    duplicate_value(0, 31, Bytes{0xaa});
    duplicate_value(7, 1, Bytes{0xbb});
    duplicate_value(3, 1, integer);
    duplicate_value(-1, 1, integer);
    duplicate_value(0, 1, integer);
    duplicate_value(7, 1, integer);
    duplicate_value(9, 1, integer);
    duplicate_value(9, 6, {});
    put<std::uint32_t>(duplicate_tail, 0);
    put<std::uint32_t>(duplicate_tail, 0);
    const auto duplicate_footer =
        complex_blob("ParaCmptInstance", component(duplicate_tail))["footer"];
    check(
        duplicate_footer["stored_value_selection"]["selected_entry_indices"] ==
                Json({0, 1, 2, 3, 8}) &&
            duplicate_footer["values"].size() == 10 &&
            duplicate_footer["values"][1]["property_id"] == -1 &&
            duplicate_footer["values"][8]["value"] == 73,
        "duplicate selection preserves source ordering and signed ID bit identity without sorting");
    check(duplicate_footer["values"][0]["native_read_action"] == "leave_destination_unchanged" &&
              duplicate_footer["values"][2]["native_read_action"] ==
                  "leave_destination_unchanged" &&
              duplicate_footer["values"][3]["native_read_action"] ==
                  "not_evaluated_malformed_payload" &&
              duplicate_footer["values"][4]["value"] == 73 &&
              duplicate_footer["values"][7]["value"] == 73,
          "null unknown and truncated first matches prevent substituting later same-ID values");
    return checks;
}
