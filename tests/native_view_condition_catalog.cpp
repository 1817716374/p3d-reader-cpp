#include "internal.hpp"

unsigned native_view_condition_catalog_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto attribute = [](std::string name) {
        return Json{{"group", 0},
                    {"key", 22840},
                    {"index", 0},
                    {"decoded",
                     {{"encoding", "compressed_utf16_xml"},
                      {"tree", {{"tag", "P3DShowConditionSet"},
                                 {"attributes", {{"ConditionSetName", std::move(name)}}},
                                 {"children", Json::array()}}}}}};
    };
    const Json graphics = Json::array({{{"id", 42}, {"attributes", Json::array({attribute(
                                                                    "bimbase_conditionset_000000009_view0"),
                                                                attribute("bimbase_conditionset_16_view2"),
                                                                attribute("user_defined_set")})}}});
    const Json models = {{"9", {{"name", "主模型"}, {"physical_storage", "P3D-S18"}}}};
    const auto catalog = build_native_view_condition_catalog(graphics, models);
    check(catalog.at("records").size() == 3, "all stored view condition records are retained");
    const auto &resolved = catalog["records"][0]["view_binding"];
    check(resolved.at("status") == "candidate_model_view" && resolved.at("model_id") == 9 &&
              resolved.at("view_index") == 0 &&
              resolved.at("model_reference").at("status") == "resolved" &&
              resolved.at("model_reference").at("name") == "主模型",
          "condition set model and view components resolve by the observed name convention");
    const auto &missing = catalog["records"][1]["view_binding"];
    check(missing.at("status") == "candidate_model_view" && missing.at("model_id") == 16 &&
              missing.at("view_index") == 2 &&
              missing.at("model_reference").at("status") == "missing",
          "condition set references to unloaded models remain explicit unresolved candidates");
    check(catalog["records"][2]["view_binding"].at("status") ==
              "unrecognized_condition_set_name",
          "unrecognized condition-set names are not assigned guessed model or view semantics");
    return checks;
}
