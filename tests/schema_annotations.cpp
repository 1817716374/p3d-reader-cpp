#include "internal.hpp"
#include <stdexcept>

unsigned schema_annotation_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        if (!ok)
            throw std::runtime_error(message);
    };
    auto property = [](const char *tag, const char *name, const char *type,
                       const char *description = "") {
        return Json{{"tag", std::string("{urn:test}") + tag},
                    {"attributes", {{"propertyname", name},
                                    {"propertytype", type},
                                    {"description", description}}},
                    {"children", Json::array()}};
    };
    auto entity = [](const char *name, Json children) {
        return Json{{"tag", "{urn:test}PKPMEntity"},
                    {"attributes", {{"entityname", name}, {"description", "entity " + std::string(name)}}},
                    {"children", std::move(children)}};
    };
    Json schema_tree = {
        {"tag", "{urn:test}PKPMSchema"},
        {"attributes", {{"schemaname", "Demo"}, {"version", "1.0"}}},
        {"children",
         Json::array({entity("Base", Json::array({property("PKPMProperty", "inherited", "string")})),
                      entity("Root", Json::array({
                                                Json{{"tag", "{urn:test}BaseClass"}, {"text", "D:Base"}},
                                                property("PKPMProperty", "value", "int", "value field"),
                                                property("PKPMStructProperty", "details", "DetailsType"),
                                                property("PKPMArrayProperty", "items", "ItemType")})),
                      entity("DetailsType", Json::array({property("PKPMProperty", "code", "int")})),
                      entity("ItemType", Json::array({property("PKPMProperty", "name", "string")}))})}};
    Json schemas = Json::array({{{"schema_attributes", {{"schemaname", "Demo"},
                                                          {"namespaceprefix", "D"},
                                                          {"version", "1.0"}}},
                                 {"tree", std::move(schema_tree)}}});
    Json objects = Json::array({{{"root",
                                 {{"attributes", {{"xmlns", "Demo.01.00"}}},
                                  {"name", "Root"},
                                  {"children",
                                   Json::array({{{"name", "value"}, {"value", 4}},
                                                {{"name", "inherited"}, {"value", "kept"}},
                                                {{"name", "details"},
                                                 {"children", Json::array({{{"name", "code"}, {"value", 9}}})}},
                                                {{"name", "items"},
                                                 {"children", Json::array({
                                                                  {{"name", "ItemType"},
                                                                   {"children", Json::array({{{"name", "name"}, {"value", "first"}}})}},
                                                                  {{"name", "ItemType"},
                                                                   {"children", Json::array({{{"name", "name"}, {"value", "second"}}})}}})}},
                                                {{"name", "unknown"}, {"value", 12}}})}}}}});

    annotate_objects_from_schemas(objects, schemas);
    const auto &root = objects[0]["root"];
    check(root["schema_entity"]["entity_name"] == "Root" &&
              root["schema_entity"]["schema_name"] == "Demo.01.00" &&
              root["schema_entity"]["instance_namespace"] == "Demo.01.00",
          "unpadded schema version resolves its padded instance namespace");
    check(root["children"][0]["schema_property"]["propertytype"] == "int" &&
              root["children"][0]["schema_property"]["description"] == "value field",
          "direct property declaration and its description attach without replacing the value");
    check(root["children"][1]["schema_property"]["owner_entity"] == "Base",
          "inherited property metadata resolves through a schema base class");
    check(root["children"][2]["schema_entity"]["entity_name"] == "DetailsType" &&
              root["children"][2]["children"][0]["schema_property"]["propertytype"] == "int",
          "nested structure properties recurse using their declared entity type");
    check(root["children"][3]["schema_property"]["declaration_tag"] == "PKPMArrayProperty" &&
              root["children"][3]["children"][0]["schema_entity"]["entity_name"] == "ItemType" &&
              root["children"][3]["children"][1]["children"][0]["value"] == "second",
          "array element class metadata is attached to each source entry in order");
    check(!root["children"][4].contains("schema_property") &&
              root["children"][0]["value"] == 4,
          "unknown fields remain untouched and source property values are preserved");
    return checks;
}
