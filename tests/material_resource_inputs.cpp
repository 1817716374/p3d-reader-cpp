#include "internal.hpp"

unsigned material_resource_input_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto node = [](const char *tag, Json a = Json::object(), Json c = Json::array()) {
        return Json{{"tag", tag}, {"attributes", a}, {"children", c}, {"text", ""}};
    };
    auto settings = [&](Json maps, int version = 9) {
        return material_settings(
            node("Material", {{"material_version", std::to_string(version)}}, maps));
    };
    auto one = [&](Json a, Json c = Json::array()) {
        return settings(Json::array({node("Map", a, c)}));
    };
    auto resources = [](const Json &s, std::size_t index = 0) -> const Json & {
        return s.at("maps")[index].at("initial_layer_state").at("layers")[0].at("resource_inputs");
    };
    auto s = one({{"Type", "1"}});
    check(resources(s).at("count") == 0, "missing Filename does not create a primary slot");
    s = one({{"Type", "1"}, {"Filename", ""}});
    check(resources(s).at("count") == 1 && resources(s).at("entries")[0].at("source_value") == "",
          "present empty Filename still requests a resource slot");
    s = one({{"Type", "1"}, {"Filename", "main.jpg"}, {"M556", "same.jpg,same.jpg,,last.jpg"}});
    auto r = resources(s);
    check(r.at("count") == 5 && r.at("entries")[0].at("service_entry") == "primary" &&
              r.at("entries")[1].at("service_entry") == "additional",
          "primary replacement and append service entries remain distinct");
    check(r.at("entries")[1].at("source_value") == "same.jpg" &&
              r.at("entries")[2].at("source_value") == "same.jpg" &&
              r.at("entries")[3].at("source_value") == "",
          "duplicate and empty appended slots are retained without interning");
    check(r.at("entries")[4].at("slot_index") == 4 &&
              r.at("entries")[4].at("source_append_index") == 3,
          "runtime input slot and M556 token index are distinct");
    s = one({{"Type", "1"}, {"M556", "first.jpg,second.jpg"}});
    check(resources(s).at("entries")[0].at("source_attribute") == "M556" &&
              resources(s).at("count") == 2,
          "an appended resource becomes first when Filename is absent");
    s = one({{"Type", "1"}, {"Filename", "a<02>x.jpg"}});
    check(resources(s).at("entries")[0].at("source_value") == "a<02>x.jpg" &&
              resources(s).at("runtime_resource_status") == "not_evaluated",
          "source service request is not replaced by guessed runtime path");
    auto image = node("Any", {{"LayerType", "layer IMAGE image.jpg"}, {"M556", "extra.jpg"}});
    auto blend = node("Any", {{"LayerType", "layer ADD"}, {"M556", "ignored.jpg"}});
    s = one({{"Type", "1"}, {"Filename", "layers.pma"}, {"M556", "parent-ignored.jpg"}},
            {image, blend});
    const auto &layers = s.at("maps")[0].at("initial_layer_state").at("layers");
    check(layers[0].at("resource_inputs").at("count") == 2 &&
              layers[1].at("resource_inputs").at("count") == 0,
          "layer resources use accepted child arguments and ignore parent or blend M556");
    check(layers[0].at("resource_inputs").at("entries")[0].at("source_attribute") == "LayerType",
          "layer primary retains its distinct source attribute");
    s = one({{"Type", "1"}, {"Filename", "layers.pma"}}, {image, node("Ignored")});
    check(s.at("maps")[0]
                  .at("initial_layer_state")
                  .at("layers")[1]
                  .at("resource_inputs")
                  .at("count") == 0,
          "native extra default layer has an empty resource sequence");
    for (const auto token : {"PROCEDURE", "GRADIENT", "LXOPROCEDURE", "TEXTURE_REPLICATOR"}) {
        s = one({{"Type", "1"}, {"Filename", "layers.pma"}},
                {node("Any", {{"LayerType", std::string("layer ") + token + " "}})});
        check(resources(s).at("count") == 1,
              "all texture-provider layers create a primary request even for empty argument");
    }
    s = one({{"Type", "1"}, {"Filename", "layers.pma"}},
            {node("Any", {{"LayerType", "layer CELL"}, {"M556", "ignored.jpg"}})});
    check(resources(s).at("count") == 0, "CELL does not create texture service requests");
    s = settings(
        {node("Map", {{"Type", "1"}, {"Filename", "base.jpg"}}),
         node("Map", {{"Type", "2"}, {"map_link", "1"}, {"Filename", "ignored-local.jpg"}}),
         node("Map", {{"Type", "30"}, {"map_link", "1"}, {"Filename", "local-normal.jpg"}})});
    const auto &g = s.at("version_conversion").at("layer_resource_getters").at("entries");
    check(g.size() == 3 && g[0].at("layer_container_owner_object_id") ==
                               g[1].at("layer_container_owner_object_id"),
          "linked maps reference one native resource container rather than copied lists");
    check(g[2].at("layer_container_owner_object_id") != g[0].at("layer_container_owner_object_id"),
          "type30 retains local resources while sharing linked mapping");
    check(g[0].at("first_slot_index") == 0 && g[0].at("resource_count") == 1,
          "effective first resource is indexed into its source layer");
    auto copied = material_settings(
        node("Material", {{"material_version", "3"}, {"displacement_distance", "1"}},
             {node("Map", {{"Type", "2"}, {"Filename", "bump.jpg"}, {"M556", "extra.jpg"}})}));
    const auto &copy_version = copied.at("version_conversion");
    bool checked_copy = false;
    for (const auto &object : copy_version.at("map_topology").at("objects")) {
        if (object.at("local_layers").at("origin") != "native_layer_copy")
            continue;
        const auto id = object.at("object_id").get<std::size_t>();
        const auto source = object.at("local_layers").at("source_object_id").get<std::size_t>();
        const auto &cs = copy_version.at("layer_containers").at("containers");
        check(id != source && cs.at(id).at("copied_from_object_id") == source &&
                  cs.at(id).at("layers")[0].at("resource_inputs") ==
                      cs.at(source).at("layers")[0].at("resource_inputs"),
              "native version copy preserves resource request provenance in an independent "
              "container");
        checked_copy = true;
    }
    check(checked_copy, "legacy bump migration exercises a native layer copy");
    // A partially parsed sequence keeps the known primary without claiming a full count.
    auto map = s.at("maps")[0];
    map["semantics"]["additional_texture_references"]["status"] = "invalid";
    auto input = material_layer_input(map, 0);
    check(input.at("layers")[0].at("resource_inputs").at("status") == "unresolved" &&
              input.at("layers")[0].at("resource_inputs").at("count").is_null() &&
              input.at("layers")[0].at("resource_inputs").at("entries").size() == 1,
          "unknown appended tokens cannot erase known primary or imply a complete array");
    auto containers = s.at("version_conversion").at("layer_containers");
    containers["containers"][0] = input;
    auto partial =
        material_layer_resource_getters(containers, s.at("version_conversion").at("map_topology"));
    check(partial.at("status") == "partial" &&
              partial.at("entries")[0].at("first_request_status") == "known",
          "first request can remain known while the whole sequence is unresolved");
    s = one({{"Type", "1"}, {"Filename", "layers.pma"}},
            {node("Any", {{"LayerType", u8"layer\u00a0IMAGE"}})});
    check(s.at("version_conversion").at("layer_resource_getters").at("entries")[0].at("status") ==
              "unresolved",
          "unknown first child acceptance cannot masquerade as an empty resource list");
    s = one({{"Type", "1"}, {"Filename", "layers.pma"}},
            {image, node("Any", {{"LayerType", u8"layer\u00a0IMAGE"}})});
    check(s.at("version_conversion").at("layer_resource_getters").at("entries")[0].at("status") ==
              "resolved",
          "uncertain later layer does not erase an already determined first resource sequence");
    return checks;
}
