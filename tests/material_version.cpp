#include "internal.hpp"

unsigned material_version_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto settings = [](const Json &attributes, const Json &map_attributes, int version) {
        Json children = Json::array();
        for (const auto &a : map_attributes)
            children.push_back(
                {{"tag", "Map"}, {"attributes", a}, {"children", Json::array()}, {"text", ""}});
        auto a = attributes;
        a["material_version"] = std::to_string(version);
        return material_settings(
            {{"tag", "Material"}, {"attributes", a}, {"children", children}, {"text", ""}});
    };
    auto convert = [&](const Json &attributes, const Json &map_attributes, int version) {
        // Isolate conversion boundaries from the separate legacy name reader.
        const auto s = settings(attributes, map_attributes, 9);
        return material_version_conversion(s.at("initial_parameters"), s.at("maps"),
                                           s.at("map_bindings"), version);
    };
    auto active = [](const Json &c, int type) -> Json {
        const auto &t = c.at("map_topology");
        for (const auto &id : t.at("active_object_ids")) {
            const auto &o = t.at("objects").at(id.get<std::size_t>());
            if (o.at("native_type_key") == type)
                return o;
        }
        return nullptr;
    };
    Json a = {
        {"Flags", "4294967295"},   {"finish", ".11"},          {"specular", ".22"},
        {"reflect", ".33"},        {"reflect_fresnel", ".44"}, {"refraction_roughness", ".55"},
        {"color.r", "2"},          {"color.g", "3"},           {"color.b", "4"},
        {"specular_color.r", "5"}, {"specular_color.g", "6"},  {"specular_color.b", "7"}};
    for (const int version : {-2147483647 - 1, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 2147483647}) {
        const auto c = convert(a, Json::array(), version);
        const auto &r = c.at("root_parameters");
        const auto &p = r.at("parameters");
        check(c.at("status") == "resolved" && p.size() == 37 &&
                  p.at("reflect_fresnel").at("value") == (version <= 6 ? .33 : .44) &&
                  p.at("refraction_roughness").at("value") == (version <= 4 ? .11 : .55),
              "version thresholds and signed extreme versions preserve final scalar precedence");
        const auto expected_flags = version <= 4   ? 0xffdffbffu
                                    : version == 5 ? 0xfffffbffu
                                                   : 0xffffffffu;
        check(r.at("flags") == expected_flags &&
                  p.at("transparent_color").at("value") ==
                      (version <= 2 ? Json::array({5., 6., 7.}) : Json::array({1., 1., 1.})) &&
                  p.at("reflect_color").at("value") ==
                      (version <= 5 ? Json::array({5., 6., 7.}) : Json::array({1., 1., 1.})),
              "color copies and flag clear masks use their own version boundaries");
    }
    auto c = convert(a, Json::array(), 2);
    check(c["root_parameters"]["parameters"]["reflect_fresnel"]["source_parameter"] == "reflect" &&
              c["root_parameters"]["operations"].size() == 9,
          "conversion keeps sequential assignments including the superseded specular copy");
    a = {{"Flags", "4096"}, {"displacement_distance", "1"}};
    Json maps = Json::array({{{"Type", "1"}},
                             {{"Type", "2"}},
                             {{"Type", "14"}},
                             {{"Type", "15"}},
                             {{"Type", "4"}},
                             {{"Type", "26"}}});
    c = convert(a, maps, 2);
    check(c["status"] == "resolved" && active(c, 2).is_null() &&
              c["map_topology"]["active_object_ids"].size() == 7 &&
              c["map_topology"]["objects"].size() == 12,
          "legacy conversion replaces maps and removes the original bump object");
    const auto pattern = active(c, 1), displacement = active(c, 15);
    check(active(c, 14)["link_type"] == 1 && active(c, 12)["link_type"] == 1 &&
              active(c, 13)["link_type"] == 12 && active(c, 4)["link_type"] == 12 &&
              active(c, 26)["link_type"] == 1,
          "legacy linked maps follow the native creation order");
    check(active(c, 13)["object_id"] != pattern["object_id"] &&
              active(c, 13)["layer_container_owner_object_id"] == pattern["object_id"] &&
              displacement["local_layers"]["origin"] == "native_layer_copy" &&
              displacement["local_layers"]["source_object_id"] == 1 &&
              displacement["layer_container_owner_object_id"] == displacement["object_id"],
          "new linked maps share effective layers while copied layers have distinct ownership");
    check(displacement["matrix_axes_status"] == "not_written_by_constructor_or_copy" &&
              displacement["copied_settings_from_object_id"] == 1 &&
              c["map_topology"]["layer_flag_updates"].size() == 2,
          "map copy scope excludes matrix axes and effective layer flag owners are deduplicated");
    check(c["map_topology"]["objects"][2]["replaced_by_object_id"] == active(c, 14)["object_id"] &&
              c["map_topology"]["objects"][1]["removed_by_version_conversion"] == true,
          "replaced and removed source objects retain separate provenance");
    c = convert({{"Flags", "4096"}},
                Json::array({{{"Type", "1"}, {"pattern_off", "-2"}},
                             {{"Type", "12"}, {"pattern_off", "1"}}}),
                2);
    check(active(c, 14).is_null() && active(c, 13).is_null() && active(c, 4)["enabled"] == false &&
              active(c, 26)["enabled"] == false,
          "disabled pattern skips early creation but later conversions copy disabled activation");
    c = convert({{"Flags", "0"}}, Json::array({{{"Type", "1"}}}), 2);
    check(!active(c, 14).is_null() && active(c, 12).is_null() && active(c, 13).is_null() &&
              !active(c, 26).is_null(),
          "legacy bit 12 gates only the additional two linked maps");
    for (const int version : {3, 4, 5, 6, 7, 8}) {
        c = convert(a, maps, version);
        check(
            (active(c, 2).is_null()) == (version == 3) && active(c, 14)["origin"] == "xml_map" &&
                active(c, 26)["origin"] == (version <= 5 ? "version_constructor" : "xml_map") &&
                c["map_topology"]["layer_flag_updates"].empty() == (version >= 8),
            "map conversion and effective layer flag updates have independent version thresholds");
    }
    c = convert({{"displacement_distance", "-0"}}, Json::array({{{"Type", "2"}}}), 3);
    check(!active(c, 2).is_null() && active(c, 15).is_null(),
          "signed zero does not migrate bump map to displacement");
    c = convert(a, Json::array({{{"Type", "2"}}, {{"Type", "5"}, {"map_link", "2"}}}), 3);
    check(active(c, 5)["link_type"] == 2 &&
              active(c, 5)["layer_container_resolution"] == "dangling_uses_local",
          "deleting a map does not rewrite references and the later getter keeps dangling links");
    c = convert(a, Json::array({{{"Type", "2"}, {"map_link", "15"}}, {{"Type", "15"}}}), 3);
    check(active(c, 15)["link_type"] == 0, "copy clears link equal to the destination type");
    c = convert(a,
                Json::array({{{"Type", "2"}, {"map_link", "3"}},
                             {{"Type", "3"}, {"map_link", "15"}},
                             {{"Type", "15"}}}),
                3);
    check(c["status"] == "partial" && c["map_topology"]["active_object_ids"].empty() &&
              c["map_topology"]["reason"] == "cyclic_effective_layer_container" &&
              c["root_parameters"]["parameters"]["reflect"]["value"].is_null(),
          "copy-created cycles stop traversal and do not publish fabricated completed parameters");
    c = convert(a, Json::array({{{"Type", "1"}, {"map_link", "14"}}, {{"Type", "14"}}}), 2);
    check(active(c, 14)["link_type"] == 0 &&
              c["map_topology"]["operations"][1]["result"] == "not_written",
          "setter rejects a chain reaching the newly created destination");
    for (const int version : {7, 8}) {
        c = convert(Json::object(),
                    Json::array({{{"Type", "1"}}, {{"Type", "30"}, {"map_link", "1"}}}), version);
        const auto special = active(c, 30);
        check(special["layer_container_owner_object_id"] == special["object_id"] &&
                  c["map_topology"]["operations"].size() == (version == 7 ? 1u : 0u),
              "type 30 retains local layers with a separate pre-v8 linked mapping copy");
    }
    c = convert({{"finish", "uncertain"}}, Json::array(), 5);
    check(c["status"] == "resolved" &&
              c["root_parameters"]["parameters"]["refraction_roughness"]["value"] == .05,
          "failed floating input retains the constructor value through parameter copies");
    c = convert({{"Flags", 123}}, Json::array({{{"Type", "1"}}}), 2);
    check(c["status"] == "partial" &&
              c["map_topology"]["reason"] == "unknown_flags_for_legacy_map_creation" &&
              c["map_topology"]["objects"].size() == 2 &&
              c["map_topology"]["active_object_ids"].empty(),
          "unknown branch flags retain a confirmed prefix without a fabricated final table");
    c = convert({{"displacement_distance", "1suffix"}}, Json::array({{{"Type", "2"}}}), 3);
    check(c["status"] == "resolved" && active(c, 15)["origin"] == "version_constructor",
          "accepted nonzero floating prefix selects the native displacement migration branch");
    const auto s = settings({{"reflect", ".25"}}, Json::array(), 6);
    check(
        s["initial_parameters"]["parameters"]["reflect_fresnel"]["value"] == .05 &&
            s["version_conversion"]["root_parameters"]["parameters"]["reflect_fresnel"]["value"] ==
                .25,
        "integrated output preserves initial parameters separately from version-normalized values");
    c = material_version_conversion(s.at("initial_parameters"), s.at("maps"), s.at("map_bindings"),
                                    nullptr);
    check(c["status"] == "partial" && c["map_topology"]["reason"] == "unknown_version" &&
              c["root_parameters"]["flags"].is_null(),
          "unknown version does not select a conversion branch");
    Json chain = Json::array();
    for (int i = 1; i <= 2048; ++i)
        chain.push_back(
            {{"native_table_member", true},
             {"semantics",
              {{"type", {{"status", "decoded"}, {"value", i}}},
               {"map_link", {{"status", "decoded"}, {"value", i == 2048 ? 0 : i + 1}}}}},
             {"numeric_reader",
              {{"parameters", {{"pattern_off", {{"write_status", "not_written"}}}}}}}});
    const auto binding = material_map_bindings(chain);
    c = material_version_conversion(s.at("initial_parameters"), chain, binding, 8);
    check(c["status"] == "resolved" && active(c, 1)["layer_container_owner_object_id"] == 2047 &&
              active(c, 30)["layer_container_owner_object_id"] == 29 &&
              c["map_topology"]["active_object_ids"].size() == 2048,
          "long linked tables resolve iteratively while preserving type 30 local ownership");
    auto readback = [&](const Json &maps) {
        return settings(Json::object(), maps, 9)
            .at("version_conversion")
            .at("bpmaterial_pbr_readback");
    };
    auto pbr = readback(Json::array());
    check(pbr.at("status") == "resolved" && pbr.at("entry_count") == 0 &&
              pbr.at("shared_parameters").at("status") == "not_written_by_pbr_enumeration",
          "empty PBR enumeration does not invent constructor or earlier bump state");
    pbr = readback(Json::array({{{"Type", "1"}, {"pattern_off", "1"}, {"PatternFlags", "1"}}}));
    check(pbr.at("entry_count") == 1 && pbr.at("entries")[0].at("native_type_key") == 1 &&
              pbr.at("entries")[0].at("bpmaterial_map_type") == 31 &&
              pbr.at("entries")[0].at("filename") == "" &&
              pbr.at("shared_parameters").at("has_map") == false &&
              pbr.at("shared_parameters").at("use_image_alpha_channel") == true,
          "disabled empty pattern still appends albedo and writes its local flags");
    pbr = readback(Json::array({{{"Type", "35"},
                                 {"pattern_off", "1"},
                                 {"pattern_mapping", "6"},
                                 {"pattern_scalemode", "-7"},
                                 {"pattern_scale.x", "7"},
                                 {"pattern_scale.y", "8"},
                                 {"pattern_offset.x", "9"},
                                 {"pattern_offset.y", "10"},
                                 {"pattern_offset.z", "11"},
                                 {"pattern_angle", "12"},
                                 {"PatternFlags", "1"}},
                                {{"Type", "1"}, {"pattern_mapping", "2"}, {"PatternFlags", "1"}},
                                {{"Type", "31"}},
                                {{"Type", "2"}}}));
    check(pbr.at("entry_count") == 3 && pbr.at("entries")[0].at("native_type_key") == 1 &&
              pbr.at("entries")[1].at("native_type_key") == 31 &&
              pbr.at("entries")[2].at("native_type_key") == 35,
          "readback uses signed native map order rather than XML order and excludes bump");
    check(pbr.at("entries")[0].at("bpmaterial_map_type") == 31 &&
              pbr.at("entries")[1].at("bpmaterial_map_type") == 31,
          "pattern and native albedo both survive without invented channel deduplication");
    const auto &last = pbr.at("shared_parameters");
    check(last.at("status") == "resolved" && last.at("source_entry_index") == 2 &&
              last.at("has_map") == false && last.at("use_image_alpha_channel") == false &&
              last.at("map_mode") == 6 && last.at("map_unit") == 3 &&
              last.at("uv_scale") == Json::array({7., 8.}) &&
              last.at("uv_offset") == Json::array({9., 10.}) && last.at("rotation_degrees") == 12.,
          "last PBR map overwrites shared mapping and flags even with empty filename and disabled "
          "map");
    check(pbr.at("entries")[0].at("shared_parameters_write").at("map_mode") == 2 &&
              pbr.at("entries")[0].at("shared_parameters_write").at("use_image_alpha_channel") ==
                  true,
          "earlier shared writes remain available separately from the last writer");
    pbr = readback(Json::array({{{"Type", "32"}, {"Filename", "normal.jpg"}},
                                {{"Type", "34"}, {"Filename", "metal.jpg"}}}));
    check(pbr.at("status") == "partial" && pbr.at("sequence_status") == "resolved" &&
              pbr.at("entry_count") == 2 &&
              pbr.at("entries")[0].at("filename_status") == "not_evaluated" &&
              !pbr.at("entries")[0].contains("filename") &&
              pbr.at("entries")[0].at("first_resource_slot_index") == 0 &&
              pbr.at("shared_parameters").at("status") == "resolved",
          "resource requests establish slots and mapping but never fake a provider-returned "
          "filename");
    pbr = readback(Json::array({{{"Type", "31"}, {"Filename", ""}}}));
    check(pbr.at("entries")[0].at("filename_status") == "not_evaluated",
          "empty resource request is different from no resource slots");
    pbr = readback(Json::array({{{"Type", "1"}, {"M556", "a.jpg,b.jpg"}}}));
    check(pbr.at("entry_count") == 1 && pbr.at("entries")[0].at("first_resource_slot_index") == 0,
          "PBR filename getter uses only the first slot rather than expanding appended resources");
    pbr = readback(Json::array({{{"Type", "31"}}, {{"Type", "31"}, {"pattern_off", "1"}}}));
    check(pbr.at("entry_count") == 1,
          "native replacement of a duplicate type occurs before PBR readback enumeration");
    pbr = readback(
        Json::array({{{"Type", "31"}, {"map_link", "35"}, {"pattern_off", "1"}},
                     {{"Type", "35"}, {"pattern_scale.x", "4"}, {"pattern_scale.y", "5"}}}));
    check(pbr.at("entries")[0].at("layer_container_owner_object_id") ==
                  pbr.at("entries")[1].at("layer_container_owner_object_id") &&
              pbr.at("entries")[0].at("shared_parameters_write").at("uv_scale") ==
                  Json::array({4., 5.}) &&
              pbr.at("entries")[0].at("shared_parameters_write").at("has_map") == false &&
              pbr.at("shared_parameters").at("has_map") == true,
          "linked maps reuse native layers but preserve per-map enabled state and separate PBR "
          "entries");
    pbr = readback(Json::array({{{"Type", "2"}}, {{"Type", "30"}}, {{"Type", "36"}}}));
    check(pbr.at("entry_count") == 0,
          "non-PBR map types do not become PBR entries or shared-field writers");
    pbr = readback(
        Json::array({{{"Type", "31"}, {"map_link", "32"}}, {{"Type", "32"}, {"map_link", "31"}}}));
    check(pbr.at("status") == "not_evaluated" && pbr.at("entry_count").is_null() &&
              pbr.at("entries").empty(),
          "unresolved cyclic topology cannot be presented as an empty or complete PBR sequence");
    for (const auto &mode : std::vector<std::pair<int, int>>{{-1, 0}, {3, 0}, {7, 0}, {5, 5}}) {
        pbr = readback(
            Json::array({{{"Type", "33"}, {"pattern_mapping", std::to_string(mode.first)}}}));
        check(pbr.at("shared_parameters").at("map_mode") == mode.second,
              "BPMaterial readback uses native mapping-mode switch rather than raw enum cast");
    }
    pbr = readback(Json::array(
        {{{"Type", "31"}, {"scale_z", 5}, {"pattern_scale.x", "4"}, {"pattern_scale.y", "5"}}}));
    check(pbr.at("status") == "resolved" &&
              pbr.at("shared_parameters").at("uv_scale") == Json::array({4., 5.}),
          "unknown unused scale Z does not invalidate copied XY");
    pbr = readback(Json::array({{{"Type", "31"},
                                 {"pattern_angle", "nan(snan)"},
                                 {"pattern_scale.x", "inf"},
                                 {"pattern_scale.y", "-0"}}}));
    check(pbr.at("status") == "resolved" &&
              pbr.at("shared_parameters").at("rotation_degrees").at("ieee754_hex") ==
                  "7ff0000000000001" &&
              pbr.at("shared_parameters").at("uv_scale")[0].at("floating_point") == "infinity" &&
              std::signbit(pbr.at("shared_parameters").at("uv_scale")[1].get<double>()),
          "direct parameter copies preserve nonfinite representations and signed zero");
    pbr = readback(Json::array({{{"Type", "31"}, {"pattern_mapping", 1}}}));
    check(pbr.at("status") == "partial" && pbr.at("sequence_status") == "resolved" &&
              pbr.at("entry_count") == 1 &&
              pbr.at("shared_parameters").at("status") == "not_evaluated",
          "unknown used mapping value does not invent a mode or erase known entry sequence");
    pbr = readback(Json::array({{{"Type", "31"}, {"pattern_mapping", 1}}, {{"Type", "35"}}}));
    check(
        pbr.at("status") == "partial" && pbr.at("entry_count") == 2 &&
            pbr.at("shared_parameters").at("status") == "resolved" &&
            pbr.at("shared_parameters").at("source_entry_index") == 1,
        "later known writer replaces unknown shared fields while preserving partial earlier trace");
    auto textures = [&](const Json &maps) {
        return settings(Json::object(), maps, 9)
            .at("version_conversion")
            .at("bpmaterial_texture_readback");
    };
    auto tex = textures(Json::array());
    check(
        tex.at("status") == "resolved" && !tex.at("classic_pattern").at("present").get<bool>() &&
            !tex.at("classic_bump").at("present").get<bool>() &&
            tex.at("bump_factor").at("value") == 0. &&
            tex.at("shared_parameters").at("has_map") == false &&
            tex.at("shared_parameters").at("map_unit") == 3 &&
            tex.at("shared_parameters").at("map_mode") == 0 &&
            tex.at("shared_parameters").at("uv_scale") == Json::array({1., 1.}) &&
            tex.at("shared_parameters").at("uv_offset") == Json::array({0., 0.}) &&
            tex.at("shared_parameters").at("rotation_degrees") == 0.,
        "no maps preserves texture constructor values except the explicit no-pattern enable reset");
    tex = textures(Json::array({{{"Type", "2"}, {"bump_map_scale", "7"}, {"BumpFlags", "1"}}}));
    check(tex.at("status") == "resolved" && tex.at("classic_bump").at("present") == true &&
              tex.at("bump_factor").at("value") == 0. &&
              tex.at("shared_parameters").at("use_image_alpha_channel") == false,
          "empty bump filename skips weight assignment and BumpFlags does not set image alpha");
    tex = textures(Json::array({{{"Type", "2"},
                                 {"bump_map_scale", "-7"},
                                 {"Filename", "bump.jpg"},
                                 {"pattern_off", "1"}}}));
    check(tex.at("status") == "partial" && tex.at("classic_bump").at("map_enabled") == false &&
              tex.at("bump_factor").at("status") == "conditional_on_returned_filename" &&
              tex.at("bump_factor").at("if_true").at("value") == -7. &&
              tex.at("bump_factor").at("if_false").at("value") == 0. &&
              tex.at("shared_parameters").at("has_map") == false,
          "disabled bump still reads local weight when the provider returns a nonempty filename");
    tex = textures(Json::array({{{"Type", "2"}, {"Filename", ""}}}));
    check(tex.at("bump_factor").at("status") == "conditional_on_returned_filename" &&
              tex.at("bump_factor").at("if_true").at("value") == 1.,
          "empty bump source request cannot decide returned filename or constructor map weight");
    tex = textures(Json::array({{{"Type", "2"}, {"Filename", "bump.jpg"}, {"bump_map_scale", 7}}}));
    check(tex.at("bump_factor").at("if_true").at("status") == "not_evaluated" &&
              tex.at("bump_factor").at("if_false").at("status") == "resolved",
          "unknown map weight does not erase the known empty-filename branch");
    tex = textures(Json::array({{{"Type", "2"}, {"map_link", "3"}, {"bump_map_scale", "8"}},
                                {{"Type", "3"}, {"Filename", "shared.jpg"}}}));
    check(tex.at("classic_bump").at("layer_container_owner_object_id") == 1 &&
              tex.at("bump_factor").at("source_object_id") == 0 &&
              tex.at("bump_factor").at("if_true").at("value") == 8.,
          "linked bump reads shared resource but its own weight");
    tex = textures(Json::array({{{"Type", "1"},
                                 {"PatternFlags", "1"},
                                 {"pattern_off", "1"},
                                 {"pattern_mapping", "5"},
                                 {"pattern_scale.x", "6"},
                                 {"pattern_scale.y", "7"}}}));
    check(tex.at("status") == "resolved" && tex.at("classic_pattern").at("filename") == "" &&
              tex.at("shared_parameters").at("source_stage") == "pbr_enumeration" &&
              tex.at("shared_parameters").at("uv_scale") == Json::array({6., 7.}) &&
              tex.at("shared_parameters").at("map_mode") == 5 &&
              tex.at("shared_parameters").at("has_map") == false &&
              tex.at("shared_parameters").at("use_image_alpha_channel") == true,
          "PBR pattern write overrides the classic empty-filename mapping skip");
    tex = textures(Json::array({{{"Type", "1"}, {"PatternFlags", "1"}}, {{"Type", "2"}}}));
    check(tex.at("classic_bump").at("use_image_alpha_channel") == false &&
              tex.at("shared_parameters").at("use_image_alpha_channel") == true,
          "PBR enumeration restores pattern alpha after the intervening bump alpha write");
    tex = textures(Json::array({{{"Type", "1"}, {"PatternFlags", "1"}}, {{"Type", "35"}}}));
    check(tex.at("shared_parameters").at("source_object_id") == 1 &&
              tex.at("shared_parameters").at("use_image_alpha_channel") == false,
          "last accepted PBR map wins over pattern and bump stages");
    tex = textures(Json::array({{{"Type", "1"}, {"Filename", "pattern.jpg"}}}));
    check(tex.at("status") == "partial" &&
              tex.at("classic_pattern").at("filename_status") == "not_evaluated" &&
              !tex.at("classic_pattern").contains("filename") &&
              tex.at("shared_parameters").at("status") == "resolved",
          "unknown classic filename does not hide unconditional final PBR mapping");
    tex = textures(
        Json::array({{{"Type", "2"}, {"map_link", "3"}}, {{"Type", "3"}, {"map_link", "2"}}}));
    check(tex.at("status") == "not_evaluated" && !tex.contains("shared_parameters"),
          "cyclic texture graph does not publish constructor values as final readback");
    tex = convert({{"displacement_distance", "2"}},
                  Json::array({{{"Type", "2"}, {"Filename", "bump.jpg"}}}), 3)
              .at("bpmaterial_texture_readback");
    check(tex.at("status") == "resolved" && tex.at("classic_bump").at("present") == false,
          "readback uses the converted table after legacy bump-to-displacement migration");
    auto read_settings = [&](const Json &attributes, int version = 9) {
        return settings(attributes, Json::array(), version).at("bpmaterial_readback");
    };
    auto material = read_settings(Json::object());
    check(material.at("status") == "resolved" && material.at("parameters").size() == 11 &&
              material.at("parameters").at("color").at("value") == Json::array({1., 1., 1.}) &&
              material.at("parameters").at("color").at("enabled") == false &&
              material.at("parameters").at("roughness_factor").at("value") == .05 &&
              material.at("parameters").at("refract_factor").at("value") == 1.,
          "BPMaterial settings readback uses native root defaults rather than only explicit XML");
    material = read_settings({{"Flags", "0"},
                              {"color.r", "2"},
                              {"color.g", "-3"},
                              {"color.b", "4"},
                              {"transmit", "1.5"},
                              {"glow", "8"},
                              {"ambient", "-2"}});
    check(material.at("parameters").at("color").at("value") == Json::array({2., -3., 4.}) &&
              material.at("parameters").at("color").at("enabled") == false &&
              material.at("parameters").at("transparency").at("value") == 1.5 &&
              material.at("parameters").at("glow_factor").at("enabled") == true &&
              material.at("parameters").at("glow_factor").at("value") == 8. &&
              material.at("parameters").at("ambient_factor").at("enabled") == true &&
              material.at("parameters").at("ambient_factor").at("value") == -2.,
          "disabled fields retain native values while glow and ambient enable are unconditional");
    const std::vector<std::pair<const char *, unsigned>> enabled_bits = {{"color", 2},
                                                                         {"transparency", 6},
                                                                         {"specular_color", 3},
                                                                         {"specular_factor", 9},
                                                                         {"glow_color", 21},
                                                                         {"diffuse_factor", 7},
                                                                         {"roughness_factor", 4},
                                                                         {"reflect_factor", 5},
                                                                         {"refract_factor", 8}};
    for (const auto &selected : enabled_bits) {
        material = read_settings({{"Flags", std::to_string(1u << selected.second)}});
        unsigned enabled = 0;
        for (const auto &entry : enabled_bits)
            enabled += material.at("parameters").at(entry.first).at("enabled").get<bool>();
        check(enabled == 1 && material.at("parameters").at(selected.first).at("enabled") == true,
              "each converted enable bit selects its own BPMaterial field");
    }
    for (const auto &value : {"-7", "0.25", "101", "1000000"}) {
        material = read_settings({{"finish", value}});
        check(material.at("parameters").at("roughness_factor").at("value") == std::stod(value),
              "reading stored roughness does not repeat the registration setter clamp");
    }
    material = read_settings({{"finish", "-0"}, {"transmit", "nan(snan)"}});
    check(
        std::signbit(material.at("parameters").at("roughness_factor").at("value").get<double>()) &&
            material.at("parameters").at("transparency").at("value").at("ieee754_hex") ==
                "7ff0000000000001",
        "direct getter readback preserves negative zero and nonfinite bit representations");
    for (int version : {0, 4, 5, 9}) {
        material = read_settings({{"Flags", "2097152"},
                                  {"color.r", "2"},
                                  {"color.g", "3"},
                                  {"color.b", "4"},
                                  {"glow_color.r", "5"},
                                  {"glow_color.g", "6"},
                                  {"glow_color.b", "7"}},
                                 version);
        const auto &glow = material.at("parameters").at("glow_color");
        check(glow.at("value") ==
                      (version <= 4 ? Json::array({2., 3., 4.}) : Json::array({5., 6., 7.})) &&
                  glow.at("source_parameter") == (version <= 4 ? "color" : "glow_color") &&
                  glow.at("enabled") == (version > 4),
              "version conversion changes both glow source and enable before readback");
    }
    material = read_settings({{"Flags", 1}});
    check(material.at("parameters_status") == "partial" &&
              material.at("parameters").at("color").at("value_status") == "resolved" &&
              material.at("parameters").at("color").at("enabled_status") == "not_evaluated" &&
              material.at("parameters").at("glow_factor").at("status") == "resolved",
          "unknown flags do not erase values or unconditional enables");
    material = read_settings({{"finish", 1}, {"scattering_distance", 4}});
    check(material.at("parameters_status") == "partial" &&
              material.at("parameters").at("roughness_factor").at("value").is_null() &&
              material.at("parameters").at("color").at("status") == "resolved",
          "unknown used parameter remains local to that field");
    material = read_settings({{"scattering_distance", 4}});
    check(material.at("parameters_status") == "resolved",
          "unknown root parameter not read by BPMaterial does not invalidate confirmed getters");
    material = read_settings({{"M311", "different-name"}, {"Name", "not-display-name"}});
    check(
        material.at("display_name").at("value") == "" &&
            material.at("display_name").at("read_status") == "not_written" &&
            material.at("extended_data_json_string").at("value") == "" &&
            material.at("project_identity_status") == "not_evaluated" && !material.contains("name"),
        "missing display and extended strings use constructors without inventing project identity");
    for (int version : {0, 9}) {
        material = read_settings({{"M319", u8"材质显示名"}, {"M320", "  not-json {  "}}, version);
        check(material.at("display_name").at("value") == u8"材质显示名" &&
                  material.at("display_name").at("source_attribute") == "M319" &&
                  material.at("extended_data_json_string").at("value") == "  not-json {  " &&
                  material.at("extended_data_json_string").at("status") == "resolved",
              "hard-coded text keys do not shift with legacy numeric tables or require valid JSON");
    }
    const auto nul_text = std::string("Name\0after", 10);
    const auto source_settings =
        settings({{"M319", nul_text}, {"M320", nul_text}}, Json::array(), 9);
    material = source_settings.at("bpmaterial_readback");
    check(material.at("display_name").at("value") == "Name" &&
              material.at("extended_data_json_string").at("value") == "Name" &&
              source_settings.at("source_parameters").at("M320") == nul_text,
          "native C-string text input truncates at NUL while original attributes stay intact");
    material = read_settings({{"M319", 4}, {"M320", Json::object()}});
    check(material.at("status") == "partial" &&
              material.at("display_name").at("status") == "not_evaluated" &&
              material.at("extended_data_json_string").at("value").is_null() &&
              material.at("parameters_status") == "resolved",
          "nontext synthetic XML input does not fabricate display or extra strings");
    material =
        settings({{"M319", "kept"}}, Json::array({{{"Type", "1"}, {"Filename", "a.jpg"}}}), 9)
            .at("bpmaterial_readback");
    check(material.at("status") == "partial" && material.at("parameters_status") == "resolved" &&
              material.at("display_name").at("value") == "kept",
          "unresolved resource return leaves settings fields usable without claiming complete "
          "material");
    return checks;
}
