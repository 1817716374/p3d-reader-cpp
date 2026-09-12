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
    return checks;
}
