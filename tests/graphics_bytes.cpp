#include "blob_internal.hpp"
#include <p3d/swept_mesh.hpp>
#include <cstring>
#include <future>
unsigned graphics_native_tests();
unsigned text_bytes_tests();
p3d::Bytes swept_binary_fixture(bool capped);
unsigned swept_binary_tests();
namespace {
using namespace p3d;
template <class T> void put(Bytes &b, T v) {
    auto p = reinterpret_cast<const std::uint8_t *>(&v);
    b.insert(b.end(), p, p + sizeof(v));
}
Bytes material_fixture(std::u16string pattern = u"relative/texture.jpg",
                       std::u16string bump = u"") {
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
    text(pattern);
    for (double x : {1., 2., 3., 4., 5.})
        put(b, x);
    text(bump);
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
              p["geometry_status"] == "invalid" && p.contains("geometry_decode_error"),
          "truncated text payload remains bounded and reports the decoding failure");
    check(p["inline_material"]["name"] == "part material" &&
              p["inline_material"]["texture_references"][0]["filename"] == "relative/texture.jpg",
          "entry material and external texture reference decoded");
    check(j["inline_material"]["parameters"] == p["inline_material"]["parameters"],
          "graphics and entry material independently decoded");
    const auto &core_transfer = p.at("inline_material").at("project_core_parameter_transfer");
    check(core_transfer.at("status") == "decoded" && core_transfer.at("parameters").size() == 11 &&
              core_transfer.at("project_registration") == "not_performed" &&
              core_transfer.at("current_project_material_selection") == "not_evaluated",
          "inline core transfer is conditional and does not claim actual project registration");
    auto parameter_named = [](const Json &parameters, const char *name) -> const Json & {
        for (const auto &parameter : parameters)
            if (parameter.at("kind") == name)
                return parameter;
        throw std::runtime_error("missing fixture material parameter");
    };
    auto changed_flags = mat;
    changed_flags.at(mat.size() - 54) = 0; // glow_factor flag
    changed_flags.at(mat.size() - 45) = 0; // ambient_factor flag
    changed_flags.at(mat.size() - 27) = 0; // roughness_factor flag
    const auto flag_source = decode_inline_material(changed_flags);
    for (const auto *name : {"glow_factor", "ambient_factor"}) {
        check(
            !parameter_named(flag_source.at("parameters"), name).at("enabled").get<bool>() &&
                parameter_named(flag_source.at("project_core_parameter_transfer").at("parameters"),
                                name)
                        .at("enabled") == true,
            "source-disabled glow and ambient flags are kept but forced on in conditional reload");
    }
    check(parameter_named(flag_source.at("project_core_parameter_transfer").at("parameters"),
                          "roughness_factor")
                  .at("enabled") == false,
          "other disabled flags survive registration and reload");
    for (const auto &test : std::vector<std::pair<double, double>>{{-5.0, 0.0},
                                                                   {0.0, 0.0},
                                                                   {-0.0, -0.0},
                                                                   {0.5, 0.5},
                                                                   {6.0, 6.0},
                                                                   {100.0, 100.0},
                                                                   {101.0, 100.0}}) {
        auto bytes = changed_flags;
        std::memcpy(bytes.data() + bytes.size() - 26, &test.first, sizeof(double));
        const auto material = decode_inline_material(bytes);
        const auto source_value = parameter_named(material.at("parameters"), "roughness_factor")
                                      .at("value")
                                      .get<double>();
        const auto effective_value =
            parameter_named(material.at("project_core_parameter_transfer").at("parameters"),
                            "roughness_factor")
                .at("value")
                .get<double>();
        check(source_value == test.first && effective_value == test.second &&
                  std::signbit(effective_value) == std::signbit(test.second),
              "native roughness limits are 0..100 even when disabled; source and signed zero "
              "survive");
    }
    for (const auto &parameter : core_transfer.at("parameters")) {
        const auto name = parameter.at("kind").get<std::string>();
        if (name == "roughness_factor")
            continue;
        check(
            parameter.at("value") ==
                parameter_named(p.at("inline_material").at("parameters"), name.c_str()).at("value"),
            "other core values, including values over one, are transferred without clamping");
    }
    auto invalid_source = mat;
    invalid_source[0] = 0;
    const auto invalid_transfer =
        decode_inline_material(invalid_source).at("project_core_parameter_transfer");
    check(invalid_transfer.at("status") == "not_applied" &&
              invalid_transfer.at("parameters").empty(),
          "invalid source does not claim new registration, independently of later existing lookup");
    for (double nonfinite :
         {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        auto bytes = mat;
        std::memcpy(bytes.data() + bytes.size() - 26, &nonfinite, sizeof(double));
        const auto transfer = decode_inline_material(bytes).at("project_core_parameter_transfer");
        check(transfer.at("status") == "not_evaluated" && transfer.at("parameters").empty() &&
                  transfer.at("stopping_parameter") == "roughness_factor",
              "nonfinite parameters do not silently become a complete JSON null-valued transfer");
    }
    auto texture_input = [&](const Bytes &bytes) {
        return decode_inline_material(bytes).at("project_texture_registration_input");
    };
    auto texture_source = material_fixture(u"pattern.jpg", u"bump.jpg");
    Reader texture_offsets(texture_source);
    texture_offsets.u8();
    texture_offsets.take(texture_offsets.u64());
    texture_offsets.take(25);
    const auto transparency_offset = texture_offsets.p;
    texture_offsets.take(9);
    const auto has_map_offset = texture_offsets.p++;
    const auto unit_offset = texture_offsets.p;
    texture_offsets.take(8);
    texture_offsets.take(texture_offsets.u64());
    const auto scale_offset = texture_offsets.p;
    texture_offsets.take(40);
    texture_offsets.take(texture_offsets.u64());
    const auto bump_factor_offset = texture_offsets.p;
    const auto initial_texture_input = texture_input(texture_source);
    check(initial_texture_input.at("status") == "decoded" &&
              initial_texture_input.at("maps").size() == 2 &&
              initial_texture_input.at("resource_resolution") == "not_performed" &&
              initial_texture_input.at("readback") == "not_evaluated",
          "texture inputs do not claim resource resolution or material readback");
    const auto &pattern_map = initial_texture_input.at("maps")[0];
    const auto &bump_map = initial_texture_input.at("maps")[1];
    check(pattern_map.at("map_type") == 1 && pattern_map.at("weight") == 1.0 &&
              bump_map.at("map_type") == 2 && bump_map.at("weight") == 0.0 &&
              bump_map.at("enabled") == true,
          "pattern weight is one while zero-strength bump creates an enabled map");
    check(pattern_map.at("layer").at("scale") == Json::array({1., 2., 1.}) &&
              pattern_map.at("layer").at("offset") == Json::array({3., 4., 1.}) &&
              pattern_map.at("layer").at("rotation_degrees") == 5.0 &&
              pattern_map.at("layer").at("use_image_alpha_channel") == false,
          "fresh inline layer receives source XY, Z one, degree rotation and default alpha");
    check(pattern_map.at("layer").at("opacity_input").at("value") == 1.0 - 0.4 &&
              bump_map.at("layer").at("opacity_input").at("value") == 1.0 - 0.4,
          "enabled transparency sets both layer opacities");
    texture_source.at(has_map_offset) = 0;
    auto texture_plan = texture_input(texture_source);
    check(texture_plan.at("maps").size() == 2 &&
              texture_plan.at("maps")[0].at("enabled") == false &&
              texture_plan.at("maps")[1].at("enabled") == true,
          "disabled pattern remains a map request and does not suppress bump");
    texture_source.at(transparency_offset) = 0;
    texture_plan = texture_input(texture_source);
    check(texture_plan.at("maps")[0].at("layer").at("opacity_input").at("status") == "not_set" &&
              !texture_plan.at("maps")[1].at("layer").at("opacity_input").contains("value"),
          "disabled transparency does not invent a layer opacity default");
    const double high_transparency = 2.0;
    texture_source.at(transparency_offset) = 1;
    std::memcpy(texture_source.data() + transparency_offset + 1, &high_transparency,
                sizeof(double));
    texture_plan = texture_input(texture_source);
    check(texture_plan.at("maps")[0].at("layer").at("opacity_input").at("value") == -1.0,
          "native opacity input is not clamped to a generic rendering range");
    for (const auto &mode : std::vector<std::pair<std::int32_t, int>>{{INT32_MIN, 0},
                                                                      {-1, 0},
                                                                      {0, 0},
                                                                      {1, 1},
                                                                      {2, 2},
                                                                      {3, 0},
                                                                      {4, 4},
                                                                      {5, 5},
                                                                      {6, 6},
                                                                      {7, 0},
                                                                      {INT32_MAX, 0}}) {
        auto bytes = texture_source;
        std::memcpy(bytes.data() + unit_offset + 4, &mode.first, sizeof(std::int32_t));
        const auto decoded = decode_inline_material(bytes);
        check(decoded.at("map_mode") == mode.first &&
                  decoded.at("project_texture_registration_input")
                          .at("maps")[0]
                          .at("layer")
                          .at("map_mode") == mode.second,
              "native mode switch preserves six named modes without altering source enum");
    }
    for (std::int32_t unit : {-1, 0, 3, 7}) {
        auto bytes = texture_source;
        std::memcpy(bytes.data() + unit_offset, &unit, sizeof(unit));
        const auto decoded = decode_inline_material(bytes);
        check(decoded.at("map_unit") == unit && decoded.at("project_texture_registration_input")
                                                        .at("maps")[1]
                                                        .at("layer")
                                                        .at("map_unit") == (unit == 0 ? 0 : 3),
              "any nonzero unit becomes absolute while the saved enum remains intact");
    }
    texture_plan = texture_input(material_fixture(u"", u"bump.jpg"));
    check(texture_plan.at("maps").size() == 1 &&
              texture_plan.at("maps")[0].at("source_reference_index") == 1,
          "empty pattern does not lose bump source identity");
    check(texture_input(material_fixture(u"", u"")).at("maps").empty(),
          "empty filenames produce no map requests even with hasMap true");
    const auto nul_texture_source = decode_inline_material(
        material_fixture(std::u16string(u"a.jpg\0hidden", 12), std::u16string(u"\0hidden", 7)));
    check(nul_texture_source.at("texture_references")[0].at("filename").get<std::string>().size() ==
                  12 &&
              nul_texture_source.at("project_texture_registration_input").at("maps").size() == 1 &&
              nul_texture_source.at("project_texture_registration_input")
                      .at("maps")[0]
                      .at("layer")
                      .at("filename_request") == "a.jpg",
          "registration consumes C-string prefixes while complete source filenames survive");
    texture_plan = texture_input(material_fixture(u"same.jpg", u"same.jpg"));
    check(texture_plan.at("maps").size() == 2 && texture_plan.at("maps")[0].at("map_type") == 1 &&
              texture_plan.at("maps")[1].at("map_type") == 2,
          "same texture filename does not merge map roles");
    auto invalid_texture_source = texture_source;
    invalid_texture_source[0] = 0;
    check(texture_input(invalid_texture_source).at("status") == "not_applied",
          "invalid inline material does not claim new texture registration");
    for (const auto offset : {scale_offset, bump_factor_offset}) {
        auto bytes = texture_source;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        std::memcpy(bytes.data() + offset, &nan, sizeof(nan));
        texture_plan = texture_input(bytes);
        check(texture_plan.at("status") == "not_evaluated" && texture_plan.at("maps").empty(),
              "unknown numeric input does not publish a partial complete texture sequence");
    }
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
    check(short_j.at("native_geometry_input").at("status") == "rejected" &&
              short_j.at("native_geometry_input").at("rejecting_entry_index") == 0 &&
              short_j.at("geometry_packets").size() == 2,
          "native container rejection does not discard inspectable later source packets");
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
    Bytes csg_archive(32);
    csg_archive[0] = 22;
    csg_archive[16] = 36;
    auto csg_entry = entry({}, csg_archive);
    csg_entry[4] = 10;
    auto csg = decode_graphics_bytes(graphics({csg_entry})).at("geometry_packets").at(0);
    check(csg.at("geometry_status") == "decoded_archive" &&
              csg.at("geometry").at("tree").at("nodes").empty() &&
              csg.at("geometry").at("boolean_evaluation_status") == "not_evaluated" &&
              csg.at("footer_status") == "decoded",
          "CSG graphics entry uses its own archive and keeps material footer boundaries");
    csg_entry[36] = 99;
    csg = decode_graphics_bytes(graphics({csg_entry})).at("geometry_packets").at(0);
    check(csg.at("geometry_status") == "invalid" && csg.at("footer_status") == "decoded",
          "invalid CSG archive marker does not suppress independent graphics footer");
    auto sweep_entry = entry(mat, swept_binary_fixture(true));
    sweep_entry[4] = 6;
    const auto sweep_container = decode_graphics_bytes(graphics({sweep_entry}, true, mat));
    const auto &sweep_packet = sweep_container.at("geometry_packets").at(0);
    check(sweep_packet.at("geometry_status") == "decoded" &&
              sweep_packet.at("geometry_type") == 6 &&
              sweep_packet.at("inline_material") ==
                  j.at("geometry_packets").at(0).at("inline_material") &&
              sweep_packet.at("layer_id") == 0xfffffff1u,
          "binary sweep geometry leaves material and layer footer independently intact");
    const auto sweep_mesh = mesh_bgfb_swept_body(sweep_packet.at("geometry").at("geometry"));
    check(sweep_mesh.status == "meshed" && sweep_mesh.point_indices.size() == 48 &&
              sweep_container.at("transform_3x4_rows") == j.at("transform_3x4_rows") &&
              sweep_container.at("owning_element_part_mapping_status") == "not_established",
          "binary graphics sweep exposes local mesh without guessing container placement or "
          "material parts");
    check(sweep_mesh.report.at("material_part_mapping_status") == "not_evaluated" &&
              sweep_mesh.points.front()[0] < 10,
          "serialized graphics placement is not silently applied to standalone source geometry");
    const auto restored = [](const Json &value) -> const Json & {
        return value.at("native_entry_restore").at("with_project");
    };
    check(
        restored(sweep_container).at("status") == "restored" &&
            restored(sweep_container).at("entry_indices") == Json::array({0}) &&
            restored(sweep_container).at("finish_result") == "ignored_by_reader",
        "container restore exposes actual retained source sequence independently of finish status");
    check(restored(short_j).at("status") == "rejected" &&
              restored(short_j).at("entry_indices").empty(),
          "null source Entry rejects the container rather than filtering one packet");
    check(restored(decode_graphics_bytes(graphics({entry()}))).at("status") == "not_evaluated",
          "retained Entry with unread geometry is not proof of successful container finish");
    check(restored(decode_graphics_bytes(graphics({e, short_entry}))).at("status") ==
              "not_evaluated",
          "an unknown earlier read prevents claiming execution reached a later null Entry");
    auto cache = [&](const std::vector<Bytes> &containers) {
        Bytes b(11); // null type reference
        put<std::uint32_t>(b, containers.size());
        for (const auto &container : containers) {
            b.insert(b.end(), {0x7c, 0x23, 0x40});
            put<std::uint64_t>(b, 42); // intentionally repeated object references
            put<std::uint32_t>(b, container.size());
            b.insert(b.end(), container.begin(), container.end());
        }
        b.insert(b.end(), 25, 0); // empty component footer
        return complex_blob("ParaCmptInstance", b);
    };
    auto ignored = entry();
    ignored[4] = 8;
    const auto repeated_source = graphics({sweep_entry, ignored, sweep_entry}, true, mat);
    const auto mapped = cache(
        {repeated_source, graphics({short_entry, sweep_entry}), graphics({}), repeated_source});
    auto cache_variant = [](const Json &field, const char *mode = "disabled") -> const Json & {
        return field.at("cache_rebuild").at("with_project").at("destination_variants").at(mode);
    };
    const auto &rebuild = cache_variant(mapped);
    check(rebuild.at("status") == "rebuilt" && rebuild.at("entries").size() == 4,
          "cache reconstruction skips rejected containers and unhandled types in source order");
    for (std::size_t i = 0; i < 4; ++i) {
        const auto &item = rebuild.at("entries").at(i);
        check(item.at("rebuilt_entry_index") == i &&
                  item.at("source_instance_index") == (i < 2 ? 0 : 3) &&
                  item.at("source_entry_index") == (i % 2 ? 2 : 0) &&
                  item.at("source_object_id") == 42 && item.at("entry_geometry_type") == 6,
              "cache mapping preserves repeated payloads and references without fabricated reuse");
    }
    check(rebuild.at("skipped_containers").size() == 2 &&
              rebuild.at("skipped_containers").at(0).at("source_instance_index") == 1 &&
              rebuild.at("skipped_containers").at(1).at("reason") == "empty_source_container" &&
              rebuild.at("skipped_entries").size() == 2,
          "cache skip reasons preserve provenance for omitted sources");
    check(rebuild.at("destination_transform_enabled") == false &&
              rebuild.at("source_container_transform") == "not_applied" &&
              mapped.at("instances").at(0).at("transform_enabled") == true &&
              mapped.at("instances").at(0).at("geometry_packets").size() == 3 &&
              mapped.at("cache_rebuild").at("active_graphics_selection") == "not_established" &&
              mapped.at("cache_rebuild").at("owning_element_part_mapping_status") ==
                  "not_established",
          "cache branch provenance does not choose visible geometry or silently apply source "
          "placement");
    const auto uncertain = cache({repeated_source, graphics({entry()}), repeated_source});
    const auto &unknown = cache_variant(uncertain);
    check(unknown.at("status") == "not_evaluated" && unknown.at("entries").empty() &&
              unknown.at("stopping_instance_index") == 1,
          "unknown source restore never publishes a partial or guessed rebuilt index list");
    const auto empty_cache = cache({graphics({ignored}), graphics({short_entry}), graphics({})});
    check(cache_variant(empty_cache).at("status") == "rebuilt" &&
              cache_variant(empty_cache).at("entries").empty(),
          "known empty merged output differs from unknown reconstruction");
    // Valid empty curve-vector object: a nonempty Entry vector must not be
    // filtered by whether the emitted geometry contains points or triangles.
    const Bytes empty_group = {'b', 'g', '0', '0', '0', '1', 'f', 'b', 12, 0, 0, 0, 8,
                               0,   12,  0,   4,   0,   8,   0,   8,   0,  0, 0, 5, 0,
                               0,   0,   12,  0,   0,   0,   8,   0,   8,  0, 0, 0, 4,
                               0,   8,   0,   0,   0,   4,   0,   0,   0,  0, 0, 0, 0};
    auto empty_group_entry = entry({}, empty_group);
    empty_group_entry[4] = 2;
    const auto with_empty_group = cache({graphics({empty_group_entry, sweep_entry})});
    const auto &empty_group_order = cache_variant(with_empty_group);
    check(empty_group_order.at("status") == "rebuilt" &&
              empty_group_order.at("entries").size() == 2 &&
              empty_group_order.at("entries").at(1).at("rebuilt_entry_index") == 1 &&
              empty_group_order.at("skipped_containers").empty(),
          "an empty but constructed curve group occupies its native rebuilt Entry slot");
    check(mapped.at("cache_rebuild").at("with_project").at("status") == "not_evaluated" &&
              !mapped.at("cache_rebuild").at("with_project").contains("entries") &&
              mapped.at("cache_rebuild").at("destination_transform_selection") == "not_established",
          "cache source alone cannot establish the target transform inherited from getGraphics");
    const auto &transformed_rebuild = cache_variant(mapped, "identity_enabled");
    check(
        transformed_rebuild.at("status") == "rebuilt" &&
            transformed_rebuild.at("entries") == rebuild.at("entries") &&
            transformed_rebuild.at("destination_transform_enabled") == true &&
            rebuild.at("destination_transform_enabled") == false &&
            transformed_rebuild.at("material_override_path") == "component_data_footer_only",
        "identity-enabled cache append preserves repeated entries under its separate assumptions");
    check(cache_variant(uncertain, "identity_enabled").at("status") == "not_evaluated" &&
              cache_variant(uncertain, "identity_enabled").at("entries").empty(),
          "enabled transform does not make unknown source restoration successful");
    check(cache_variant(with_empty_group, "identity_enabled").at("entries").size() == 2 &&
              cache_variant(empty_cache, "identity_enabled").at("entries").empty(),
          "transformed cache still counts constructed empty curve groups and skips unhandled "
          "entries");
    return checks + graphics_native_tests() + text_bytes_tests() + swept_binary_tests();
}
