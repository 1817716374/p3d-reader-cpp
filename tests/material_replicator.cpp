#include "internal.hpp"

unsigned material_replicator_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto node = [](const char *tag, Json attributes, Json children = Json::array()) {
        return Json{{"tag", tag}, {"attributes", attributes}, {"children", children}, {"text", ""}};
    };
    auto package = [&](const Json &children, const char *gate = "read") {
        auto p = material_replicator_nodes(node("Owner", Json::object(), children));
        p["reader_applicability"] = {{"status", gate}};
        return material_replicator_input(p);
    };
    auto r = package(Json::array());
    check(r["status"] == "resolved" && r["object_state"] == "absent" && r["parameters"].empty(),
          "missing M541 does not manufacture a default replicator attachment");
    r = package(Json::array({node("M541", Json::object())}));
    check(r["status"] == "resolved" && r["object_state"] == "constructed" &&
              r["parameters"].size() == 14 && r["selected_child_index"] == 0,
          "present empty M541 constructs a replicator with all fourteen stored parameters");
    const Json expected = {{"M542", 25.},    {"M543", 50.},  {"M544", 80.},   {"M545", 0.},
                           {"M546", 0.},     {"M547", 0.},   {"M548", false}, {"M549", false},
                           {"M550", 2000.},  {"M551", 500.}, {"M552", 100.},  {"M553", 100.},
                           {"M554", 40000u}, {"M555", 100u}};
    for (auto it = expected.begin(); it != expected.end(); ++it) {
        const auto &p = r["parameters"][it.key()];
        check(p["value"] == it.value() && p["constructor_value"] == it.value() &&
                  p["read_status"] == "not_written" && p["source_read"]["status"] == "missing",
              "constructor defaults retain typed values and missing source provenance");
    }
    const auto policy = r["copy_policy"];
    check(policy["scope"] == "material_layer_copy_into_new_layer" &&
              policy["replicator_object"] == "new_object" &&
              policy["nested_settings"] == "new_object" &&
              policy["absent_source"] == "clear_destination_attachment",
          "layer-copy ownership distinguishes newly constructed nested settings from sharing");
    Json a = {{"M542", "-2.5"}, {"M548", "-2suffix"}, {"M549", "4294967296"},
              {"M550", "3.25"}, {"M554", "-1"},       {"M555", "2147483648"}};
    auto source = Json::array(
        {node("m541", {{"M542", "777"}}), node("M541", a), node("M541", {{"M543", "666"}})});
    const auto original = source;
    r = package(source);
    check(source == original && r["selected_child_index"] == 1 &&
              r["parameters"]["M542"]["value"] == -2.5 && r["parameters"]["M543"]["value"] == 50.,
          "only first exact node supplies parameters and later nodes cannot fill missing fields");
    check(r["parameters"]["M548"]["value"] == true &&
              r["parameters"]["M548"]["source_read"]["value"] == 4294967294u &&
              r["parameters"]["M549"]["value"] == false &&
              r["parameters"]["M554"]["value"] == 4294967295u &&
              r["parameters"]["M555"]["value"] == 2147483648u,
          "boolean conversion follows native uint32 assignment including sign and truncation");
    a["M554"] = "invalid";
    a["M548"] = "+";
    a["M550"] = "2suffix";
    r = package(Json::array({node("M541", a)}));
    check(r["status"] == "partial" && r["parameters"]["M554"]["value"] == 40000u &&
              r["parameters"]["M548"]["value"] == false &&
              r["parameters"]["M550"]["value"].is_null() &&
              r["parameters"]["M550"]["constructor_value"] == 2000.,
          "failed integers preserve defaults while unconfirmed floating syntax stays unresolved");
    r = package(source, "skipped");
    check(r["object_state"] == "absent" && r["parameters"].empty(),
          "unread packages cannot instantiate a replicator from source nodes");
    r = package(source, "unresolved");
    check(r["object_state"] == "unresolved" && r["status"] == "partial" && r["parameters"].empty(),
          "unknown provider dispatch does not assume the package reader ran");
    auto root = node("Material", {{"material_version", "3"}, {"displacement_distance", "1"}},
                     Json::array({node("Map", {{"Type", "2"}, {"layer", "7"}}, source)}));
    auto s = material_settings(root);
    const auto &initial = s["maps"][0]["replicators"]["initial_state"];
    check(initial["status"] == "resolved" && initial["parameters"]["M550"]["value"] == 3.25,
          "single-provider dispatch attaches constructed M541 state");
    const auto &topology = s["version_conversion"]["map_topology"];
    const auto id = topology["active_object_ids"][0].get<std::size_t>();
    const auto &copies = topology["objects"][id]["local_layers"]["replicator_copies"];
    check(copies["status"] == "resolved" && copies["entries"].size() == 1 &&
              copies["entries"][0]["source_layer_child_index"].is_null() &&
              copies["entries"][0]["state"] == initial &&
              copies["entries"][0]["ownership"] == "new_replicator_and_nested_settings",
          "bump-to-displacement copy materializes independent replicator parameter state");
    auto layers =
        Json::array({node("Any", {{"LayerType", "layer GAMMA"}}, source),
                     node("Any", {{"LayerType", "layer TEXTURE_REPLICATOR image.jpg"}}, source),
                     node("Any", {{"LayerType", "LAYER TEXTURE_REPLICATOR"}}, source)});
    root["children"][0] = node("Map", {{"Type", "2"}, {"Filename", "layers.pma"}}, layers);
    s = material_settings(root);
    const auto &copy =
        s["version_conversion"]["map_topology"]["objects"][1]["local_layers"]["replicator_copies"];
    check(copy["status"] == "resolved" && copy["entries"].size() == 1 &&
              copy["entries"][0]["source_layer_child_index"] == 1 &&
              copy["entries"][0]["state"]["parameters"]["M542"]["value"] == -2.5,
          "layer collection copy keeps source child identity and excludes unread M541 packages");
    root["children"][0]["children"][1]["children"][1]["attributes"]["M550"] = "uncertain";
    s = material_settings(root);
    check(s["version_conversion"]["map_topology"]["status"] == "resolved" &&
              s["version_conversion"]["map_topology"]["objects"][1]["local_layers"]
               ["replicator_copies"]["status"] == "partial",
          "known object topology does not turn uncertain copied parameter values into defaults");
    return checks;
}
