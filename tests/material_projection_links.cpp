#include "internal.hpp"

unsigned material_projection_link_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto map = [](int type, int link, int value) {
        Json a = {{"Type", std::to_string(type)},
                  {"map_link", std::to_string(link)},
                  {"origin_uv_pro_matrix_on", value % 2 ? "1" : "0"}};
        for (const auto field :
             {"pattern_proj_offset", "pattern_proj_angles", "pattern_proj_scale",
              "origin_uv_pro_matrix_xa", "origin_uv_pro_matrix_ya", "origin_uv_pro_matrix_za"})
            for (const auto axis : {"x", "y", "z"})
                a[std::string(field) + "." + axis] = std::to_string(value++);
        return a;
    };
    auto parse = [](const Json &maps, int version = 9, Json root = Json::object()) {
        Json children = Json::array();
        for (const auto &a : maps)
            children.push_back(
                {{"tag", "Map"}, {"attributes", a}, {"children", Json::array()}, {"text", ""}});
        root["material_version"] = std::to_string(version);
        return material_settings(
            {{"tag", "Material"}, {"attributes", root}, {"children", children}, {"text", ""}});
    };
    auto getter = [](const Json &s, int type) -> Json {
        for (const auto &e : s.at("version_conversion").at("projection_getters").at("entries"))
            if (e.at("native_type_key") == type)
                return e;
        return nullptr;
    };
    Json maps = Json::array({map(1, 0, 10), map(3, 1, 101), map(30, 3, 201)});
    auto s = parse(maps);
    auto g = getter(s, 30);
    check(g["frame_source_object_id"] == 0 && g["frame_resolution"] == "linked" &&
              g["parameters"]["pattern_proj_offset"]["value"] == Json::array({10., 11., 12.}) &&
              g["parameters"]["pattern_proj_angles"]["value"] == Json::array({13., 14., 15.}) &&
              g["parameters"]["pattern_proj_scale"]["value"] == Json::array({16., 17., 18.}),
          "all three frame getters traverse a multi-hop link including type 30");
    check(
        s["version_conversion"]["map_topology"]["objects"][2]["layer_container_owner_object_id"] ==
                2 &&
            g["parameters"]["origin_uv_pro_matrix_on"]["value"] == true &&
            g["parameters"]["origin_uv_pro_matrix_on"]["source_object_id"] == 2 &&
            g["matrix"]["source_object_id"] == 2 &&
            g["matrix"]["axes"]["xa"]["value"] == Json::array({210., 211., 212.}),
        "type 30 local layers and explicit matrix remain separate from its linked projection "
        "frame");
    check(getter(s, 1)["frame_resolution"] == "local" &&
              getter(s, 3)["frame_source_object_id"] == 0 &&
              s["version_conversion"]["projection_getters"]["status"] == "resolved",
          "local and linked getter provenance is explicit for each active object");
    maps[1]["SpecularFlags"] = "1";
    s = parse(maps);
    check(getter(s, 3)["parameters"]["linked_option"]["value"] == true &&
              getter(s, 30)["parameters"]["linked_option"]["value"] == false,
          "linked option getter reads the caller boolean rather than any linked map boolean");
    maps[2]["pattern_proj_offset.x"] = 7;
    s = parse(maps);
    check(s["maps"][2]["initial_projection_state"]["parameters"]["pattern_proj_offset"]["value"]
                  .is_null() &&
              getter(s, 30)["status"] == "resolved" &&
              getter(s, 30)["parameters"]["pattern_proj_offset"]["source_object_id"] == 0,
          "unused unknown local frame does not obscure the known linked getter result");
    maps[0]["pattern_proj_offset.x"] = 7;
    s = parse(maps);
    check(getter(s, 30)["status"] == "partial" &&
              getter(s, 30)["parameters"]["pattern_proj_offset"]["value"].is_null() &&
              getter(s, 30)["matrix"]["status"] == "decoded",
          "unknown target frame propagates without discarding the independent local matrix");
    maps[0]["pattern_proj_offset.x"] = "-nan";
    maps[0].erase("origin_uv_pro_matrix_xa.x");
    s = parse(maps);
    g = getter(s, 30);
    check(g["status"] == "resolved" &&
              g["parameters"]["pattern_proj_offset"]["value"][0]["ieee754_hex"] ==
                  "ffffffffffffffff" &&
              g["matrix"]["all_components_finite"] == true && getter(s, 1)["status"] == "partial",
          "known nonfinite linked frame survives while a missing target matrix does not affect the "
          "caller matrix");
    maps.push_back(map(1, 0, 401));
    s = parse(maps);
    g = getter(s, 30);
    check(g["parameters"]["pattern_proj_offset"]["value"] == Json::array({401., 402., 403.}) &&
              s["version_conversion"]["projection_getters"]["entries"].size() == 3,
          "projection getters use the surviving native type entry after duplicate replacement");
    s = parse(Json::array({map(2, 0, 20), map(4, 2, 40), map(8, 4, 80)}), 3,
              {{"displacement_distance", "1"}});
    for (const int type : {4, 8}) {
        g = getter(s, type);
        const auto id = g["object_id"];
        check(g["frame_resolution"] == "dangling_uses_local" && g["frame_source_object_id"] == id &&
                  g["parameters"]["pattern_proj_offset"]["value"][0] == type * 10 &&
                  s["version_conversion"]["map_topology"]["objects"][id.get<std::size_t>()]
                   ["link_type"] != 0,
              "every caller of a broken post-conversion chain falls back to itself without "
              "clearing its link");
    }
    g = getter(s, 15);
    check(getter(s, 2).is_null() &&
              g["parameters"]["pattern_proj_offset"]["value"] == Json::array({20., 21., 22.}) &&
              g["matrix"]["status"] == "not_initialized_by_constructor" &&
              g["parameters"]["origin_uv_pro_matrix_on"]["value"] == false,
          "copied replacement retains frame values but does not recover a deleted source matrix");
    s = parse(Json::array({map(1, 0, 51)}), 2);
    g = getter(s, 14);
    check(
        g["frame_source_object_id"] == 0 &&
            g["parameters"]["pattern_proj_offset"]["value"] == Json::array({51., 52., 53.}) &&
            g["parameters"]["origin_uv_pro_matrix_on"]["value"] == false &&
            g["matrix"]["status"] == "not_initialized_by_constructor",
        "new linked map obtains frame through getters without cloning local explicit matrix state");
    s = parse(Json::array({map(1, 2, 1), map(2, 1, 2)}));
    check(s["version_conversion"]["projection_getters"]["status"] == "partial" &&
              s["version_conversion"]["projection_getters"]["entries"].empty(),
          "cyclic native traversal does not publish fabricated terminal getter values");
    return checks;
}
