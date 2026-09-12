#include "internal.hpp"

unsigned material_root_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto r = material_root_input(Json::object(), 9);
    const auto &defaults = r["parameters"];
    check(r["status"] == "resolved" && defaults.size() == 37 && r["flags"]["value"] == 0x182001 &&
              r["flags"]["read_status"] == "not_written",
          "empty root exposes the confirmed parameter constructor and flags");
    check(defaults["color"]["value"] == Json::array({1., 1., 1.}) &&
              defaults["transparent_color"]["value"] == Json::array({1., 1., 1.}) &&
              defaults["exit_color"]["value"] == Json::array({0., 0., 0.}),
          "color defaults distinguish the zero exit color from the white channels");
    check(defaults["ambient"]["value"] == .5 && defaults["diffuse"]["value"] == .5 &&
              defaults["finish"]["value"] == .05 && defaults["specular"]["value"] == .05 &&
              defaults["reflect_fresnel"]["value"] == .05 &&
              defaults["refraction_roughness"]["value"] == .05 &&
              defaults["refract"]["value"] == 1 && defaults["front_weighting"]["value"] == 50,
          "constructor coefficients retain their native values without renderer conversion");
    check(defaults["reflection_rays"]["value"] == 64 &&
              defaults["refraction_rays"]["value"] == 64 &&
              defaults["subsurface_samples"]["value"] == 64 &&
              defaults["blur_reflections"]["value"] == false &&
              defaults["back_face_culling"]["value"] == 0,
          "sample counts, boolean fields and integer culling have distinct native types");
    Json a = {{"Flags", "0"},
              {"ambient", "-2.5"},
              {"color.r", ".2"},
              {"color.g", ".3"},
              {"color.b", ".4"},
              {"reflection_rays", "-3tail"},
              {"blur_reflections", "-5"},
              {"back_face_culling", "7"}};
    const auto original = a;
    r = material_root_input(a, 9);
    check(a == original && r["flags"]["value"] == 0 &&
              r["parameters"]["color"]["value"] == Json::array({.2, .3, .4}) &&
              r["parameters"]["ambient"]["value"] == -2.5,
          "root values are copied independently of activation flags and preserve source input");
    check(r["parameters"]["reflection_rays"]["value"] == -3 &&
              r["parameters"]["blur_reflections"]["value"] == true &&
              r["parameters"]["back_face_culling"]["value"] == 7,
          "signed root integers and nonzero boolean conversion follow the getter");
    a["Flags"] = "bad";
    a["reflection_rays"] = "bad";
    a.erase("color.b");
    r = material_root_input(a, 9);
    check(r["flags"]["value"] == 0x182001 && r["parameters"]["reflection_rays"]["value"] == 64 &&
              r["parameters"]["reflection_rays"]["source_read"]["status"] == "invalid",
          "failed integer getters leave constructor values while preserving failure provenance");
    check(r["parameters"]["color"]["value"] == Json::array({1., 1., 1.}) &&
              r["parameters"]["color"]["read_status"] == "not_written",
          "missing RGB component prevents the complete color assignment");
    a["color.r"] = "1suffix";
    r = material_root_input(a, 9);
    check(r["parameters"]["color"]["read_status"] == "not_written",
          "a missing color component proves no assignment even if another component is uncertain");
    a["color.b"] = "1";
    r = material_root_input(a, 9);
    check(r["status"] == "partial" && r["parameters"]["color"]["value"].is_null() &&
              r["parameters"]["color"]["constructor_value"] == Json::array({1., 1., 1.}),
          "unconfirmed floating lexical conversion does not fabricate a constructor fallback");
    a = {{"translucent_color.r", "2"},   {"translucent_color.g", "3"},
         {"translucent_color.b", "4"},   {"anisotropy.r", "5"},
         {"anisotropy.g", "6"},          {"anisotropy.b", "7"},
         {"glow_color_map.r", "8"},      {"glow_color_map.g", "9"},
         {"glow_color_map.b", "10"},     {"glow_color", ".75"},
         {"linked_to_lxp", ".25"},       {"shader_type", "13"},
         {"value_map_invert", "17"},     {"reflection_rays", "2"},
         {"rounded_edge_width", ".125"}, {"shader_flag_visible_to_occlusion_rays", ".375"}};
    r = material_root_input(a, 0);
    const auto &p = r["parameters"];
    check(p["transparent_color"]["value"] == Json::array({2., 3., 4.}) &&
              p["translucent_color"]["value"] == Json::array({5., 6., 7.}) &&
              p["glow_color"]["value"] == Json::array({8., 9., 10.}) &&
              p["refraction_roughness"]["value"] == .75,
          "mode zero reads legacy table names without cascading field renaming");
    check(p["displacement_distance"]["value"] == .25 && p["refraction_rays"]["value"] == 13 &&
              p["subsurface_samples"]["value"] == 17 && p["blur_reflections"]["value"] == true &&
              p["diffuse_roughness"]["value"] == .125 && p["rounded_edge_width"]["value"] == .375,
          "legacy field shifts retain getter types even when source names suggest another type");
    r = material_root_input(a, -1);
    check(r["parameters"]["transparent_color"]["source_key"] == "transparent_color" &&
              r["parameters"]["rounded_edge_width"]["value"] == .125,
          "all nonzero modes use the unshifted attribute-name table");
    r = material_root_input({{"ambient", "2"}}, nullptr);
    check(r["status"] == "partial" && r["parameters"]["ambient"]["value"] == 2 &&
              r["parameters"]["transparent_color"]["source_key"].is_null(),
          "unknown mode retains stable fields without guessing shifted source keys");
    Json root = {{"tag", "Material"},
                 {"attributes", {{"material_version", "9"}}},
                 {"children", Json::array()},
                 {"text", ""}};
    auto s = material_settings(root);
    check(s["initial_parameters"]["flags"]["value"] == 0x182001 &&
              s["semantics"]["flags"]["status"] == "missing" &&
              s["initial_parameters"]["scope"] ==
                  "constructor_and_root_numeric_attributes_before_version_conversion",
          "constructed parameter view remains separate from explicit source-only semantics");
    return checks;
}
