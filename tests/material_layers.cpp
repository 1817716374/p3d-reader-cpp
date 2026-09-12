#include "internal.hpp"

unsigned material_layers_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto node = [](const char *tag, Json a = Json::object(), Json children = Json::array()) {
        return Json{{"tag", tag}, {"attributes", a}, {"children", children}, {"text", ""}};
    };
    auto settings = [&](Json maps, int version = 9, Json attributes = Json::object()) {
        attributes["material_version"] = std::to_string(version);
        return material_settings(node("Material", attributes, maps));
    };
    auto one = [&](Json a, Json children = Json::array(), int version = 9) {
        return settings(Json::array({node("Map", a, children)}), version);
    };
    auto s = one({{"Type", "1"}});
    const auto &base = s["maps"][0]["initial_layer_state"];
    check(base["status"] == "resolved" && base["layer_count"] == 1 &&
              base["layers"][0]["data_flags"]["value"] == 0x800 &&
              base["layers"][0]["enabled"]["value"] == true &&
              base["layers"][0]["option_bit_1"]["value"] == false &&
              base["layers"][0]["option_bit_2"]["value"] == false,
          "Map starts with one layer and the confirmed data and option flag defaults");
    s = one({{"Type", "1"},
             {"Flags", "4095"},
             {"pattern_off", "1"},
             {"enable_antialiasing", "0"},
             {"PatternFlags", "3"}});
    auto layer = s["maps"][0]["initial_layer_state"]["layers"][0];
    check(layer["data_flags"]["value"] == 2047 && layer["enabled"]["value"] == false &&
              layer["option_bit_1"]["value"] == true && layer["option_bit_2"]["value"] == false,
          "single Map reads Flags before antialias bit replacement and keeps options separate");
    for (int version : {7, 8, 9}) {
        s = one({{"Type", "1"}, {"Flags", "0"}, {"enable_antialiasing", "0"}}, Json::array(),
                version);
        check(s["maps"][0]["initial_layer_state"]["layers"][0]["data_flags"]["value"] == 0 &&
                  s["version_conversion"]["layer_containers"]["containers"][0]["layers"][0]
                   ["data_flags"]["value"] == (version < 8 ? 0x800 : 0),
              "pre-v8 layer update restores bit 11 after explicit antialias disable");
    }
    const std::vector<std::pair<int, const char *>> flag_names = {{1, "PatternFlags"},
                                                                  {2, "BumpFlags"},
                                                                  {3, "SpecularFlags"},
                                                                  {4, "ReflectFlags"},
                                                                  {5, "TransparencyFlags"},
                                                                  {6, "TranslucencyFlags"},
                                                                  {7, "FinishFlags"},
                                                                  {8, "DiffuseFlags"},
                                                                  {9, "GlowAmountFlags"},
                                                                  {10, "ClearcoatAmountFlags"},
                                                                  {11, "AnisotropicDirectionFlags"},
                                                                  {15, "DisplacementFlags"},
                                                                  {16, "NormalFlags"},
                                                                  {17, "M611"},
                                                                  {18, "M612"},
                                                                  {19, "M613"},
                                                                  {20, "M614"},
                                                                  {21, "M615"},
                                                                  {22, "M616"},
                                                                  {23, "M617"},
                                                                  {24, "M622"},
                                                                  {25, "M623"},
                                                                  {28, "M624"},
                                                                  {29, "M625"}};
    for (const auto &entry : flag_names) {
        s = one({{"Type", std::to_string(entry.first)}, {entry.second, "-1suffix"}});
        const auto &l = s["maps"][0]["initial_layer_state"]["layers"][0];
        check(l[entry.first == 1 ? "option_bit_1" : "option_bit_2"]["value"] == true &&
                  l[entry.first == 1 ? "option_bit_2" : "option_bit_1"]["value"] == false,
              "native map-type table selects the exact unsigned flag source and destination bit");
    }
    for (int type : {12, 13, 14, 26, 27, 30, 31, 32, 33, 34, 35, -1}) {
        s = one({{"Type", std::to_string(type)}, {"PatternFlags", "1"}, {"BumpFlags", "1"}});
        const auto &l = s["maps"][0]["initial_layer_state"]["layers"][0];
        check(l["option_bit_1"]["value"] == false && l["option_bit_2"]["value"] == false,
              "types without native flag-source entries do not borrow similarly named attributes");
    }
    s = one({{"Type", "2"}, {"Flags", "bad"}, {"BumpFlags", "2"}, {"enable_antialiasing", "bad"}});
    layer = s["maps"][0]["initial_layer_state"]["layers"][0];
    check(layer["data_flags"]["value"] == 0x800 && layer["option_bit_2"]["value"] == false,
          "failed integer assignment preserves flags and map option uses bit zero rather than "
          "nonzero");
    const Json attrs = {
        {"Type", "2"}, {"Filename", "layers.pma"}, {"Flags", "0"}, {"pattern_off", "1"}};
    s = one(attrs);
    check(s["maps"][0]["initial_layer_state"]["layer_count"] == 1 &&
              s["maps"][0]["initial_layer_state"]["layers"][0]["origin"] == "native_constructor" &&
              s["maps"][0]["initial_layer_state"]["layers"][0]["enabled"]["value"] == true,
          "empty layer collection retains a default layer without reading single-provider flags");
    auto ignored = node("Ignored", {{"LayerType", "LAYER IMAGE"}});
    auto image = node(
        "Any",
        {{"LayerType", "layer IMAGE image.jpg"}, {"LayerDataFlags", "512"}, {"LayerFlags", "6"}});
    s = one(attrs, Json::array({node("M633"), ignored}));
    check(s["maps"][0]["initial_layer_state"]["layer_count"] == 1 &&
              s["maps"][0]["initial_layer_state"]["layers"][0]["origin"] == "native_constructor",
          "only ignored children leave the initial layer intact");
    s = one(attrs, Json::array({ignored, image, ignored}));
    auto input = s["maps"][0]["initial_layer_state"];
    check(input["layer_count"] == 2 && input["layers"][0]["source_layer_child_index"] == 1 &&
              input["layers"][1]["origin"] == "native_constructor",
          "successful layer followed by ignored siblings leaves an extra default layer");
    check(input["layers"][0]["data_flags"]["value"] == 0x800 &&
              input["layers"][0]["enabled"]["value"] == false &&
              input["layers"][0]["option_bit_1"]["value"] == true &&
              input["layers"][0]["option_bit_2"]["value"] == true,
          "layer data bit remapping and three option bits have distinct native destinations");
    s = one(attrs, Json::array({image, ignored, image}));
    check(s["maps"][0]["initial_layer_state"]["layer_count"] == 2 &&
              s["maps"][0]["initial_layer_state"]["layers"][1]["source_layer_child_index"] == 2,
          "skipped middle children reuse the pending layer without creating extra entries");
    s = one(attrs, Json::array({node("Any", {{"LayerType", "layer CELL"},
                                             {"LayerDataFlags", "3072"},
                                             {"LayerFlags", "0"}})}));
    layer = s["maps"][0]["initial_layer_state"]["layers"][0];
    check(layer["data_flags"]["value"] == 0 && layer["enabled"]["value"] == true,
          "CELL reads data flags but does not apply LayerFlags");
    s = settings(Json::array({node("Map", {{"Type", "1"}, {"map_link", "2"}, {"Flags", "0"}}),
                              node("Map", {{"Type", "2"}, {"Flags", "64"}})}),
                 7);
    const auto &containers = s["version_conversion"]["layer_containers"]["containers"];
    check(containers[0]["layers"][0]["data_flags"]["value"] == 0 &&
              containers[1]["layers"][0]["data_flags"]["value"] == 2112,
          "version updates apply to effective shared containers rather than every unused local "
          "layer");
    image["attributes"]["LayerDataFlags"] = "0";
    s = settings(Json::array({node("Map", attrs, Json::array({image, ignored}))}), 3,
                 {{"displacement_distance", "1"}});
    const auto &copy = s["version_conversion"]["layer_containers"]["containers"][1];
    check(copy["copied_from_object_id"] == 0 && copy["layer_count"] == 2 &&
              copy["layers"][0]["data_flags"]["value"] == 0x800 &&
              copy["layers"][0]["enabled"]["value"] == false &&
              copy["layers"][1]["enabled"]["value"] == true,
          "native map copy retains layer count, trailing default and option states before version "
          "flags");
    s = one({{"Type", "1"}, {"Flags", 17}});
    check(s["maps"][0]["initial_layer_state"]["status"] == "partial" &&
              s["version_conversion"]["layer_containers"]["status"] == "partial" &&
              s["version_conversion"]["layer_containers"]["containers"][0]["layers"][0]
               ["data_flags"]["value"]
                   .is_null(),
          "unresolved source flag reads remain unknown in constructed and converted states");
    s = one({{"Type", "1"}, {"Flags", "0"}, {"texture_filter_type", "1"}}, Json::array(), 0);
    check(s["maps"][0]["initial_layer_state"]["layers"][0]["data_flags"]["value"] == 0x800 &&
              s["maps"][0]["initial_layer_state"]["layers"][0]["data_flags"]["antialias_read"]
               ["source_attribute_base"] == "texture_filter_type",
          "legacy antialias uses the native shifted source key rather than its modern spelling");
    s = one(attrs, Json::array({image, node("Any", {{"LayerType", u8"layer\u00a0IMAGE"}})}));
    check(s["maps"][0]["initial_layer_state"]["status"] == "partial" &&
              s["maps"][0]["initial_layer_state"]["layer_count"].is_null() &&
              s["maps"][0]["initial_layer_state"]["layers"].size() == 2,
          "unknown layer-reader behavior preserves a known prefix without claiming a final count");
    return checks;
}
