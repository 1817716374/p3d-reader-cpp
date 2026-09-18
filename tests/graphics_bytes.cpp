#include "internal.hpp"
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
    return checks;
}
