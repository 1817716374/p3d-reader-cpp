#include "internal.hpp"
unsigned material_numeric_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto node = [](const std::string &tag, const Json &a, const Json &c = Json::array()) {
        return Json{{"tag", tag}, {"attributes", a}, {"children", c}, {"text", ""}};
    };
    auto parse = [&](const Json &a, const Json &mode = Json("9"),
                     const Json &children = Json::array()) {
        Json attrs = Json::object();
        if (!mode.is_null())
            attrs["material_version"] = mode;
        return material_settings(node("Material", attrs, Json::array({node("Map", a, children)})));
    };
    Json attrs = {{"Type", "1"},
                  {"pattern_proj_scale.x", "0"},
                  {"pattern_proj_scale.y", "-0"},
                  {"pattern_proj_scale.z", "-2"},
                  {"pattern_scale.x", "0"},
                  {"pattern_scale.y", "-3"},
                  {"scale_z", "0"},
                  {"pattern_offset.x", "2"},
                  {"pattern_offset.y", "4"},
                  {"offset_z", "7"},
                  {"pattern_off", "0"},
                  {"enable_antialiasing", "-3"},
                  {"origin_uv_pro_matrix_on", "0"},
                  {"pattern_weight", "2"},
                  {"bump_map_scale", "3"}};
    const auto original = attrs;
    auto s = parse(attrs);
    auto p = s["maps"][0]["numeric_reader"]["parameters"];
    check(attrs == original && p["pattern_proj_scale"]["value"] == Json::array({0, 0, -2}) &&
              p["pattern_proj_scale"]["reader_value"] == Json::array({1, 1, -2}) &&
              p["pattern_scale"]["reader_value"] == Json::array({0, -3}) &&
              p["scale_z"]["reader_value"] == 1,
          "projection zero normalization is distinct from unchanged two-dimensional texture scale");
    check(
        p["pattern_offset"]["value"] == Json::array({2, 4}) &&
            p["pattern_offset"]["reader_value"] == Json::array({2, 4, 7}) &&
            p["offset_z"]["depends_on"] == "pattern_offset",
        "offset write combines stored XY and conditionally read Z without changing source vector");
    check(p["pattern_off"]["reader_value"] == true &&
              p["enable_antialiasing"]["reader_value"] == true &&
              p["origin_uv_pro_matrix_on"]["reader_value"] == false &&
              p["pattern_weight"]["write_status"] == "written" &&
              p["bump_map_scale"]["write_status"] == "not_written",
          "native boolean and map-specific weight writes follow the actual reader branch");
    attrs.erase("offset_z");
    p = parse(attrs)["maps"][0]["numeric_reader"]["parameters"];
    check(p["offset_z"]["status"] == "missing" && p["offset_z"]["reader_value"] == 0 &&
              p["pattern_offset"]["reader_value"] == Json::array({2, 4, 0}),
          "missing Z gets the native conditional zero only after complete XY");
    attrs.erase("pattern_offset.y");
    attrs["offset_z"] = "8";
    p = parse(attrs)["maps"][0]["numeric_reader"]["parameters"];
    check(p["offset_z"]["value"] == 8 && p["offset_z"]["reader_value"].is_null() &&
              p["offset_z"]["write_status"] == "not_written" &&
              p["pattern_offset"]["status"] == "partial",
          "stored Z cannot write when XY is incomplete");
    attrs["pattern_offset.y"] = "4";
    attrs["offset_z"] = "bad";
    p = parse(attrs)["maps"][0]["numeric_reader"]["parameters"];
    check(p["pattern_offset"]["reader_value"] == Json::array({2, 4, 0}) &&
              p["pattern_offset"]["write_status"] == "written",
          "failed Z getter leaves its zero temporary and complete XY still writes XYZ");
    p = parse({{"Type", "1"}})["maps"][0]["numeric_reader"]["parameters"];
    check(p["scale_z"]["reader_value"].is_null() && p["scale_z"]["write_status"] == "not_written" &&
              p["pattern_proj_scale"]["reader_value"].is_null(),
          "missing values are not silently replaced by constructor state");
    attrs = {{"Type", "1"},
             {"pattern_proj_angles.x", "1"},
             {"pattern_proj_angles.y", "2"},
             {"pattern_proj_angles.z", "3"},
             {"pattern_u_flip.x", "0"},
             {"pattern_u_flip.y", "4"},
             {"pattern_u_flip.z", "0"},
             {"scale_z", "8"},
             {"lxo_noise_value1", "9"},
             {"reflect_color", ".6"},
             {"layer_blend", ".7"},
             {"low_value", "3"},
             {"texture_filter_type", "4"},
             {"pattern_offset.x", "2"},
             {"pattern_offset.y", "3"}};
    for (const auto &mode : {Json("0"), Json()}) {
        p = parse(attrs, mode)["maps"][0]["numeric_reader"]["parameters"];
        check(p["pattern_proj_offset"]["source_keys"] ==
                      Json::array({"pattern_proj_angles.x", "pattern_proj_angles.y",
                                   "pattern_proj_angles.z"}) &&
                  p["pattern_proj_offset"]["reader_value"] == Json::array({1, 2, 3}) &&
                  p["pattern_proj_scale"]["reader_value"] == Json::array({1, 4, 1}),
              "missing and zero material mode shift the native field table before vector suffixes");
        check(p["scale_z"]["reader_value"] == 9 &&
                  p["pattern_offset"]["reader_value"] == Json::array({2, 3, 8}) &&
                  p["image_gamma"]["reader_value"] == .6 &&
                  p["pattern_opacity"]["reader_value"] == .7 &&
                  p["texture_filter_type"]["reader_value"] == 3 &&
                  p["enable_antialiasing"]["reader_value"] == true,
              "legacy field selection reads original attributes without cascading renamed keys");
        check(p["origin_uv_pro_matrix_za"]["source_key_status"] ==
                      "unavailable_mode_zero_table_entry" &&
                  p["origin_uv_pro_matrix_za"]["write_status"] == "unresolved",
              "invalid native legacy table entry is not promoted to an invented attribute name");
    }
    p = parse(attrs, Json(9))["maps"][0]["numeric_reader"]["parameters"];
    check(p["scale_z"]["source_key_status"] == "unresolved_reader_mode" &&
              p["scale_z"]["reader_value"].is_null() &&
              p["pattern_offset"]["status"] == "decoded" &&
              p["pattern_offset"]["write_status"] == "unresolved",
          "unknown mode keeps stable source names but does not resolve mode-dependent Z");
    attrs["Filename"] = "layers.pma";
    p = parse(attrs, "0")["maps"][0]["numeric_reader"]["parameters"];
    check(p["pattern_proj_offset"]["write_status"] == "written" &&
              p["scale_z"]["write_status"] == "not_written",
          "common Map fields are read before layer-collection gate but single-provider fields are "
          "skipped");
    auto layer = node("Layer", {{"LayerType", "layer IMAGE texture.jpg"},
                                {"pattern_offset.x", "1"},
                                {"pattern_offset.y", "2"},
                                {"scale_z", "3"},
                                {"lxo_noise_value1", "0"},
                                {"high_value", "4"},
                                {"antialias_strength", "5"},
                                {"minimum_spot", "6"},
                                {"use_cell_colors", "7"}});
    auto gamma = node("Layer", {{"LayerType", "layer GAMMA"}, {"layer_color", "2.5"}});
    auto tint = node("Layer", {{"LayerType", "layer TINT"},
                               {"layer_brightness.r", ".1"},
                               {"layer_brightness.g", ".2"},
                               {"layer_brightness.b", ".3"}});
    s = parse({{"Type", "1"}, {"Filename", "layers.pma"}}, "0", Json::array({layer, gamma, tint}));
    p = s["maps"][0]["texture_layers"]["entries"][0]["numeric_reader"]["parameters"];
    check(p["pattern_offset"]["reader_value"] == Json::array({1, 2, 3}) &&
              p["scale_z"]["reader_value"] == 1 && p["low_value"]["reader_value"] == 4 &&
              p["high_value"]["reader_value"] == 5 &&
              p["antialias_strength"]["reader_value"] == 6 &&
              p["minimum_spot"]["reader_value"] == 7,
          "texture-layer reads preserve mode-zero aliases and offset dependency");
    check(s["maps"][0]["texture_layers"]["entries"][1]["numeric_reader"]["parameters"]
           ["layer_gamma"]["reader_value"] == 2.5 &&
              s["maps"][0]["texture_layers"]["entries"][2]["numeric_reader"]["parameters"]
               ["layer_color"]["reader_value"] == Json::array({.1, .2, .3}),
          "gamma and tint use their own mode-selected scalar and color field names");
    s = parse({{"Type", "1"}, {"Filename", "ordinary.jpg"}}, "0", Json::array({layer}));
    check(s["maps"][0]["texture_layers"]["entries"][0]["numeric_reader"]["parameters"]["scale_z"]
           ["write_status"] == "not_written",
          "inactive nested layer cannot produce an effective numeric write");
    s = parse({{"Type", "0"}, {"pattern_angle", "7"}});
    check(s["maps"][0]["numeric_reader"]["parameters"]["pattern_angle"]["write_status"] ==
              "not_written",
          "zero Map type does not execute numeric reads");
    p = parse({{"Type", "2"},
               {"pattern_weight", "4"},
               {"bump_map_scale", "5"}})["maps"][0]["numeric_reader"]["parameters"];
    check(p["pattern_weight"]["write_status"] == "not_written" &&
              p["bump_map_scale"]["reader_value"] == 5,
          "bump map selects bump scale at the shared native weight destination");
    p = parse({{"Type", "1"},
               {"origin_uv_pro_matrix_on", "0"},
               {"origin_uv_pro_matrix_xa.x", "-nan"},
               {"origin_uv_pro_matrix_xa.y", "0"},
               {"origin_uv_pro_matrix_xa.z", "0"}})["maps"][0]["numeric_reader"]["parameters"];
    check(p["origin_uv_pro_matrix_on"]["reader_value"] == false &&
              p["origin_uv_pro_matrix_xa"]["write_status"] == "written" &&
              p["origin_uv_pro_matrix_xa"]["reader_value"][0]["ieee754_hex"] == "ffffffffffffffff",
          "stored negative NaN matrix component is assigned even when projection is disabled");
    return checks;
}
