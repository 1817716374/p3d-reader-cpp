#include "internal.hpp"

unsigned material_projection_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto node = [](const char *tag, Json a, Json children = Json::array()) {
        return Json{{"tag", tag}, {"attributes", a}, {"children", children}, {"text", ""}};
    };
    auto settings = [&](const Json &a, int version = 9, Json root = Json::object()) {
        root["material_version"] = std::to_string(version);
        return material_settings(node("Material", root, Json::array({node("Map", a)})));
    };
    Json a = {{"Type", "2"},
              {"BumpFlags", "1"},
              {"map_link", "2"},
              {"origin_uv_pro_matrix_on", "0"},
              {"bump_map_scale", "2.5"},
              {"pattern_proj_offset.x", "10"},
              {"pattern_proj_offset.y", "20"},
              {"pattern_proj_offset.z", "30"},
              {"pattern_proj_angles.x", "40"},
              {"pattern_proj_angles.y", "50"},
              {"pattern_proj_angles.z", "60"},
              {"pattern_proj_scale.x", "0"},
              {"pattern_proj_scale.y", "2"},
              {"pattern_proj_scale.z", "-0"}};
    int n = 1;
    for (const auto axis : {"xa", "ya", "za"})
        for (const auto component : {"x", "y", "z"})
            a[std::string("origin_uv_pro_matrix_") + axis + "." + component] = std::to_string(n++);
    auto s = settings(a);
    auto state = s["maps"][0]["initial_projection_state"];
    const auto p = state["parameters"];
    check(state["status"] == "resolved" && p["origin_uv_pro_matrix_on"]["value"] == false &&
              p["linked_option"]["value"] == true &&
              s["map_bindings"]["entries"][0]["effective_link_type"] == 0,
          "separate linked option uses original self-link and cannot change the explicit matrix "
          "switch");
    check(p["pattern_proj_offset"]["value"] == Json::array({10., 20., 30.}) &&
              p["pattern_proj_angles"]["value"] == Json::array({40., 50., 60.}) &&
              p["pattern_proj_scale"]["value"] == Json::array({1., 2., 1.}) &&
              p["weight"]["value"] == 2.5,
          "local projection frame and weight preserve native assignments and zero-scale "
          "normalization");
    check(state["matrix"]["storage_values"] == Json::array({1., 2., 3., 4., 5., 6., 7., 8., 9.}) &&
              state["matrix"]["axes"]["ya"]["value"] == Json::array({4., 5., 6.}),
          "matrix builder stores each complete axis contiguously even when its switch is off");
    for (const auto text : {"0", "bad"}) {
        auto input = a;
        input["map_link"] = text;
        state = settings(input)["maps"][0]["initial_projection_state"];
        check(state["parameters"]["linked_option"]["value"] == false,
              "zero or failed raw link prevents linked-option assignment");
    }
    for (const auto text : {"0", "2", "bad"}) {
        auto input = a;
        input["BumpFlags"] = text;
        input["origin_uv_pro_matrix_on"] = "7";
        state = settings(input)["maps"][0]["initial_projection_state"];
        check(state["parameters"]["linked_option"]["value"] == false &&
                  state["parameters"]["origin_uv_pro_matrix_on"]["value"] == true,
              "map option uses low bit or failure fallback independently of nonzero matrix switch");
    }
    auto input = a;
    input["Type"] = "1";
    input["map_link"] = "1";
    input["PatternFlags"] = "1";
    input["pattern_weight"] = "3.5";
    state = settings(input)["maps"][0]["initial_projection_state"];
    check(state["parameters"]["linked_option"]["value"] == false &&
              state["parameters"]["weight"]["value"] == 3.5,
          "pattern flag writes its layer option and skips the separate linked-option assignment");
    input = a;
    input["Filename"] = "layers.pma";
    state = settings(input)["maps"][0]["initial_projection_state"];
    check(state["parameters"]["linked_option"]["value"] == false &&
              state["matrix"]["status"] == "decoded",
          "layer-collection branch reads common matrix fields but skips single-provider linked "
          "option");
    for (const auto axis : {"xa", "ya", "za"}) {
        input = a;
        input.erase(std::string("origin_uv_pro_matrix_") + axis + ".y");
        state = settings(input)["maps"][0]["initial_projection_state"];
        check(state["status"] == "partial" && state["matrix"]["storage_values"].is_null() &&
                  state["matrix"]["axes"][axis]["state"] == "indeterminate_native_temporary" &&
                  state["matrix"]["axes"][axis]["value"].is_null(),
              "incomplete axis leaves an indeterminate native temporary instead of a zero or unit "
              "axis");
    }
    input = a;
    input["pattern_proj_offset.x"] = "2suffix";
    input.erase("pattern_proj_offset.y");
    state = settings(input)["maps"][0]["initial_projection_state"];
    check(state["parameters"]["pattern_proj_offset"]["value"] == Json::array({0., 0., 0.}),
          "known missing frame component proves no assignment even if another conversion is "
          "uncertain");
    input["pattern_proj_offset.y"] = "3";
    input["origin_uv_pro_matrix_xa.x"] = "2suffix";
    state = settings(input)["maps"][0]["initial_projection_state"];
    check(state["parameters"]["pattern_proj_offset"]["value"].is_null() &&
              state["matrix"]["axes"]["xa"]["state"] == "unresolved_read",
          "unconfirmed floating lexical conversion is not mistaken for a failed getter");
    input = a;
    input["origin_uv_pro_matrix_on"] = "1";
    s = settings(input, 3, {{"displacement_distance", "1"}});
    const auto &topology = s["version_conversion"]["map_topology"];
    const auto &copy = s["version_conversion"]["local_projection_states"]["objects"][1];
    check(topology["objects"][1]["settings_copy_scope"] ==
                  "link_enabled_weight_projection_frame_linked_option" &&
              copy["parameters"]["linked_option"]["value"] == true &&
              copy["parameters"]["origin_uv_pro_matrix_on"]["value"] == false &&
              copy["parameters"]["weight"]["value"] == 2.5 &&
              copy["parameters"]["pattern_proj_offset"]["value"] == Json::array({10., 20., 30.}),
          "native map copy includes linked option and frame but leaves true matrix switch at "
          "constructor false");
    check(copy["matrix"]["status"] == "not_initialized_by_constructor" &&
              copy["matrix"]["storage_values"].is_null() && copy["copied_from_object_id"] == 0,
          "copy does not invent or transfer the source explicit matrix axes");
    s = settings({{"Type", "1"}}, 2);
    const auto &created = s["version_conversion"]["local_projection_states"]["objects"][1];
    check(created["parameters"]["pattern_proj_scale"]["value"] == Json::array({1., 1., 1.}) &&
              created["parameters"]["origin_uv_pro_matrix_on"]["value"] == false &&
              created["matrix"]["status"] == "not_initialized_by_constructor",
          "new linked maps keep local constructor projection state rather than cloning the target "
          "matrix");
    return checks;
}
