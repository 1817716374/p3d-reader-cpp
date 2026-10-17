#include "internal.hpp"
#include <p3d/property_blocks.hpp>
#include <thread>

unsigned property_binary_value_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        require(ok, why);
        ++checks;
    };
    auto append = [](Bytes &b, auto v) {
        const auto n = b.size();
        b.resize(n + sizeof(v));
        std::memcpy(b.data() + n, &v, sizeof(v));
    };
    auto text = [&](Bytes &b, const std::string &s) {
        append(b, std::uint32_t(4));
        append(b, std::uint32_t(s.size()));
        b.insert(b.end(), s.begin(), s.end());
    };
    auto typed = [&](Bytes &b, unsigned type, auto v) {
        append(b, std::uint32_t(type));
        append(b, v);
    };
    auto binary = [&](Bytes &b, const Bytes &value) {
        append(b, std::uint32_t(5));
        append(b, std::uint32_t(value.size()));
        b.insert(b.end(), value.begin(), value.end());
    };
    auto header = [&] {
        Bytes b;
        text(b, "UserSchema");
        text(b, "CustomClass");
        return b;
    };
    auto field = [&](Bytes &b, const std::string &space, const std::string &name,
                     const Bytes &value) {
        text(b, name);
        text(b, space);
        typed(b, 0, std::uint8_t(0));
        text(b, "binary");
        binary(b, value);
    };
    const Bytes empty_map(8, 0);
    auto source = header();
    field(source, "PBM_CommonData", "ExtendProperty", empty_map);
    field(source, "PBM_CommonData", "ExtendProperty", empty_map);
    field(source, "UserAttributes", "ExtendProperty", empty_map);
    field(source, "PBM_CommonData", "extendproperty", empty_map);
    field(source, "PBM_CommonData", "ParaCmptProperty", {});
    const auto block = decode_property_data_block(source);
    const auto &views = block.at("binary_value_views"), &entries = views.at("entries"),
               &tokens = block.at("tokens");
    check(block.at("property_structure").at("status") == "structured" &&
              views.at("status") == "partial" && entries.size() == 5,
          "outer wire completion is distinct from nested binary interpretation");
    for (unsigned i = 0; i < 2; ++i) {
        const auto &v = entries[i];
        const auto ti = v.at("value_token_index").get<std::size_t>();
        check(v.at("field_index") == i && v.at("status") == "parsed" &&
                  v.at("encoding") == "extension_variant_map" && v.at("decoded").empty() &&
                  v.at("source_offset") == tokens[ti].at("offset").get<std::size_t>() + 8 &&
                  v.at("source_byte_count") == 8,
              "recognized inherited declarations keep their exact source token and payload range");
    }
    check(entries[0].at("value_token_index") != entries[1].at("value_token_index") &&
              entries[0].at("source_offset") != entries[1].at("source_offset"),
          "equal independent binary values are not deduplicated");
    check(entries[2].at("status") == "not_interpreted" && !entries[2].contains("decoded") &&
              entries[3].at("status") == "not_interpreted",
          "names are not dispatched across namespaces or by case-insensitive guesses");
    check(entries[4].at("status") == "empty" && !entries[4].contains("decoded"),
          "empty source bytes do not fabricate a component or a default object");
    check(views.at("active_geometry_selection") == "not_performed" &&
              bytesof(block.at("source_storage")) == source,
          "embedded views do not select a graphics cache or modify source bytes");
    auto arrays = header();
    text(arrays, "ExtendProperty");
    text(arrays, "PBM_CommonData");
    typed(arrays, 0, std::uint8_t(1));
    text(arrays, "binary");
    typed(arrays, 3, std::int64_t(3));
    binary(arrays, empty_map);
    binary(arrays, {});
    binary(arrays, empty_map);
    const auto a = decode_property_data_block(arrays);
    const auto &av = a.at("binary_value_views").at("entries");
    check(a.at("binary_value_views").at("status") == "parsed" && av.size() == 3 &&
              av[0].at("array_index") == 0 && av[1].at("array_index") == 1 &&
              av[2].at("array_index") == 2,
          "binary arrays preserve value occurrence order including an empty middle entry");
    auto broken = header();
    field(broken, "PBM_CommonData", "ExtendProperty", Bytes{1, 0, 0});
    field(broken, "PBM_CommonData", "ExtendProperty", empty_map);
    const auto b = decode_property_data_block(broken);
    check(b.at("property_structure").at("status") == "structured" &&
              b.at("binary_value_views").at("entries")[0].at("status") == "not_parsed" &&
              b.at("binary_value_views").at("entries")[0].contains("reason") &&
              b.at("binary_value_views").at("entries")[1].at("status") == "parsed",
          "one malformed binary value does not erase a later independent valid declaration");
    auto spoof = header();
    const std::string json = "{\"value\":1}";
    field(spoof, "PBM_BimBase", "GraphicsGeom", Bytes(json.begin(), json.end()));
    const auto s = decode_property_data_block(spoof).at("binary_value_views").at("entries")[0];
    check(s.at("status") == "not_parsed" && !s.contains("decoded") &&
              s.at("encoding") == "cached_geometry_3_1",
          "strict native contracts do not fall back to an unrelated JSON heuristic");
    PropertyBlockLimits limits;
    limits.max_decoded_bytes = arrays.size() + 8;
    const auto limited = decode_property_data_block(arrays, limits);
    check(limited.at("property_structure").at("status") == "structured" &&
              limited.at("binary_value_views").at("entries")[0].at("status") == "parsed" &&
              limited.at("binary_value_views").at("entries")[2].at("status") == "not_parsed",
          "recognized binary decoding shares the containing archive byte budget");
    auto child = header();
    field(child, "PBM_CommonData", "ExtendProperty", empty_map);
    auto nested = header();
    text(nested, "Children");
    text(nested, "UserSchema");
    typed(nested, 0, std::uint8_t(1));
    text(nested, "UserClass");
    typed(nested, 3, std::int64_t(2));
    for (unsigned i = 0; i < 2; ++i) {
        typed(nested, 3, std::int64_t(nested.size()));
        typed(nested, 3, std::int64_t(child.size()));
        binary(nested, child);
    }
    limits.max_decoded_bytes = nested.size() + 2 * child.size() + 8;
    const auto n = decode_property_data_block(nested, limits);
    const auto &children = n.at("property_structure").at("fields")[0].at("elements");
    check(children[0].at("data_block").at("binary_value_views").at("status") == "parsed" &&
              children[1].at("data_block").at("binary_value_views").at("status") == "partial",
          "sibling struct blocks cannot reset the shared binary byte budget");
    auto mesh = header();
    Bytes mesh_data;
    append(mesh_data, 1.f);
    append(mesh_data, std::uint32_t(0));
    append(mesh_data, std::uint32_t(0));
    field(mesh, "PBM_BimBase", "GraphicsVecExtend", mesh_data);
    const auto m = decode_property_data_block(mesh).at("binary_value_views").at("entries")[0];
    check(m.at("status") == "parsed" && m.at("decoded").at("vertices").empty() &&
              m.at("decoded").at("faces").empty(),
          "directed mesh values use their own native decoder without adding geometry to a scene");
    auto stored_style = [&](Bytes &b, bool entry) {
        for (auto c :
             entry ? std::array<unsigned, 3>{{11, 22, 33}} : std::array<unsigned, 3>{{44, 55, 66}})
            append(b, std::uint32_t(c));
        append(b, std::int32_t(entry ? -3 : 5));
        append(b, std::uint32_t(entry ? 0xfffffff0u : 6u));
        if (!entry)
            append(b, std::int32_t(1));
        append(b, entry ? 0.25 : 0.5);
        append(b, entry ? 2.0 : 0.75);
        if (entry) {
            append(b, 3.0);
            append(b, 4.0);
            append(b, std::uint32_t(0xfffffffeu));
        }
    };
    auto legacy_header = [&](unsigned type) {
        Bytes b;
        append(b, 3.1);
        append(b, std::uint32_t(1));
        append(b, std::uint32_t(type));
        stored_style(b, true);
        return b;
    };
    auto legacy = legacy_header(3);
    append(legacy, std::uint32_t(1));
    append(legacy, std::uint32_t(0));
    append(legacy, std::uint32_t(0));
    append(legacy, std::uint8_t(1));
    append(legacy, std::uint32_t(3));
    for (const auto &p :
         std::array<std::array<double, 3>, 3>{{{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}}})
        for (auto v : p)
            append(legacy, v);
    for (unsigned i = 0; i < 6; ++i)
        append(legacy, std::uint32_t(0));
    append(legacy, std::uint32_t(4));
    for (int i : {1, 2, 3, 0})
        append(legacy, std::int32_t(i));
    for (unsigned i = 0; i < 3; ++i)
        append(legacy, std::uint32_t(0));
    const auto tag_offset = legacy.size();
    legacy.insert(legacy.end(), {0xff, 0xfe, 0xff, 0});
    for (unsigned i = 0; i < 3; ++i)
        append(legacy, std::uint32_t(0));
    stored_style(legacy, false);
    auto legacy_block = header();
    field(legacy_block, "PBM_BimBase", "GraphicsGeom", legacy);
    const auto legacy_view =
        decode_property_data_block(legacy_block).at("binary_value_views").at("entries")[0];
    check(legacy_view.at("status") == "parsed",
          "legacy native graphics remain attached to their source property");
    const auto &g = legacy_view.at("decoded"), &es = g.at("entry_style"),
               &cs = g.at("container_style");
    check(es.at("serialized_rgb") == Json::array({11, 22, 33}) && es.at("line_style") == -3 &&
              es.at("line_weight") == 0xfffffff0u && es.at("transparency") == 0.25 &&
              es.at("line_style_scale") == 2.0 && es.at("widths") == Json::array({3.0, 4.0}) &&
              es.at("layer_id") == 0xfffffffeu,
          "legacy Entry RGB, line settings, widths, transparency and full layer sentinel are "
          "distinct fields");
    check(cs.at("serialized_rgb") == Json::array({44, 55, 66}) && cs.at("line_style") == 5 &&
              cs.at("line_weight") == 6 && cs.at("symbology_source") == 1 &&
              cs.at("transparency") == 0.5 && cs.at("line_style_scale") == 0.75,
          "container symbology is decoded independently rather than applied over each Entry");
    check(es.at("source_offset") == 16 && es.at("source_byte_count") == 56 &&
              cs.at("source_offset") == legacy.size() - 40 && cs.at("source_byte_count") == 40 &&
              g.at("mesh_tail").at("source_offset") == tag_offset,
          "legacy metadata source ranges distinguish the display header, mesh tail and container "
          "footer");
    check(g.at("mesh_layout").at("mesh_style") == 1 && g.at("mesh_layout").at("num_per_row") == 0 &&
              g.at("mesh_layout").at("num_per_face") == 0 &&
              g.at("mesh_layout").at("two_sided") == true &&
              g.at("mesh_tail").at("tag_name") == "" && g.at("mesh_tail").at("face_data").empty() &&
              g.at("unassigned_regions").empty(),
          "mesh layout and supported empty native name/face/edge channels have explicit semantics");
    bool zero_channels = g.at("empty_mesh_channels").size() == 9;
    for (const auto &channel : g.at("empty_mesh_channels"))
        zero_channels =
            zero_channels && channel.at("count") == 0 &&
            Reader(legacy).at<std::uint32_t>(channel.at("source_offset").get<std::size_t>()) == 0;
    check(
        zero_channels && g.at("empty_mesh_channels")[2].at("name") == "rgb_factors" &&
            g.at("empty_mesh_channels")[8].at("name") == "rgb_indices",
        "empty color, parameter and normal pool/index counts retain their distinct wire positions");
    check(g.at("points").size() == 3 && g.at("indices") == Json::array({1, 2, 3, 0}) &&
              g.at("polygons") == Json::array({Json::array({1, 2, 3})}) &&
              g.at("display_header_hex") == hex(slice(legacy, 16, 56)) &&
              g.at("footer_hex") == hex(slice(legacy, tag_offset, 56)),
          "legacy geometry and original header/footer byte views remain unchanged");
    auto curves = legacy_header(2);
    append(curves, std::uint32_t(1));
    append(curves, std::uint32_t(0));
    stored_style(curves, false);
    auto curve_block = header();
    field(curve_block, "PBM_BimBase", "GraphicsGeom", curves);
    const auto curve = decode_property_data_block(curve_block)
                           .at("binary_value_views")
                           .at("entries")[0]
                           .at("decoded");
    check(curve.at("kind") == "curve_collection" &&
              curve.at("footer_rgb") == Json::array({44, 55, 66}) &&
              curve.at("container_style").at("symbology_source") == 1 &&
              curve.at("unassigned_regions") == Json::array({"boundary_code"}),
          "legacy curve footer metadata does not hide its still-unassigned boundary enum");
    legacy[tag_offset] = 0;
    auto invalid_tag = header();
    field(invalid_tag, "PBM_BimBase", "GraphicsGeom", legacy);
    check(decode_property_data_block(invalid_tag)
                  .at("binary_value_views")
                  .at("entries")[0]
                  .at("status") == "not_parsed",
          "a malformed mesh tag marker cannot be silently labeled as an empty name");
    std::array<Json, 4> parallel;
    std::array<std::thread, 4> threads;
    for (unsigned i = 0; i < 4; ++i)
        threads[i] = std::thread([&, i] { parallel[i] = decode_property_data_block(source); });
    for (auto &t : threads)
        t.join();
    check(std::all_of(parallel.begin(), parallel.end(), [&](const Json &j) { return j == block; }),
          "parallel nested decoders share no mutable selection or source state");
    return checks;
}
