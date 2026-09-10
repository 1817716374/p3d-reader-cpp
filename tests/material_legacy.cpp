#include "internal.hpp"
unsigned material_legacy_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto node = [](const std::string &tag, const std::string &text) {
        return Json{{"tag", tag},
                    {"attributes", Json::object()},
                    {"text", text},
                    {"children", Json::array()}};
    };
    auto parse = [&](const std::string &name, const std::vector<std::string> &records) {
        auto owner = node("Layer", "");
        for (const auto &text : records)
            owner["children"].push_back(node("AnyTag", text));
        auto dispatch = material_layer_semantics(
            {{"LayerType", "layer IMAGE " + name + ".pma"}})["reader_path"];
        return material_legacy_parameters(owner, dispatch);
    };
    auto b = parse("boards", {"boards_crack_width .25tail", "boards_boards_per_row 8.75",
                              "boards_brightness_variation .2", "boards_unknown 7"});
    check(b["parameters"]["crack_width"]["value"] == 25 && b["entries"][0]["value"] == .25 &&
              b["entries"][0]["reader_scale"] == 100 &&
              b["parameters"]["boards_per_row"]["value"] == 8 &&
              b["parameters"]["brightness_variation"]["value"] == 20 &&
              b["entries"][3]["status"] == "ignored_unknown_key",
          "legacy content scalars use numeric prefixes and native scaling without renaming unknown "
          "fields");
    b = parse("boards", {"width .5", "x_crack_width\t.5", "x_crack_width .5", "x_crack_width",
                         "x_extra_crack_width .7"});
    check(b["entries"][0]["status"] == "ignored_missing_prefix_separator" &&
              b["entries"][1]["status"] == "ignored_unknown_key" &&
              b["parameters"]["crack_width"]["value"] == 0 &&
              b["parameters"]["crack_width"]["source_entry_indices"] == Json::array({3}) &&
              b["entries"][3]["source_value"] == "crack_width" &&
              b["entries"][3]["conversion"] == "no_numeric_prefix_uses_zero" &&
              b["entries"][4]["status"] == "ignored_unknown_key",
          "first underscore and literal space grammar preserves native missing-space wraparound");
    b = parse("boards", {"x_primary_color .1 .2 .3", "x_primary_color .9 bad .7"});
    check(b["entries"][1]["status"] == "partial" &&
              b["parameters"]["primary_color"]["value"] == Json::array({.9, .2, .3}) &&
              b["parameters"]["primary_color"]["source_entry_indices"] == Json::array({1, 0, 0}),
          "partial color scan overwrites successful components only, with independent sources");
    b = parse("boards",
              {"x_primary_color .1 .2 .3", "x_primary_color 1e .6 .7", "x_crack_width 1e"});
    check(b["parameters"]["primary_color"]["value"] == Json::array({.1, .2, .3}) &&
              b["entries"][1]["components"][0]["conversion"] == "scan_failed" &&
              b["parameters"]["crack_width"]["value"] == 100,
          "unfinished exponent fails native color scan but scalar atof retains preceding numeric "
          "prefix");
    b = parse("brick", {"x_flemish_headers -2", "x_header_course_interval 2147483648",
                        "x_pattern_id -2147483649", "x_morter_width .04", "x_mortar_width 99"});
    check(b["parameters"]["flemish_headers"]["value"] == true && b["entries"][0]["value"] == -2 &&
              b["parameters"]["header_course_interval"]["value"] == INT32_MAX &&
              b["parameters"]["pattern_id"]["value"] == INT32_MIN &&
              b["entries"][1]["conversion"] == "integer_clamped" &&
              b["parameters"]["morter_width"]["value"] == 4 &&
              b["entries"][4]["status"] == "ignored_unknown_key",
          "brick preserves native misspelling, signed clamp and boolean conversion");
    b = parse("brick", {"x_bond_type One-third", "x_bond_type Running ", "x_bond_type english",
                        "x_bond_type Dutch", "x_bond_type unknown"});
    check(b["entries"][0]["reader_value"] == 2 && b["entries"][1]["reader_applies"] == false &&
              b["entries"][2]["reader_applies"] == false &&
              b["parameters"]["bond_type"]["value"] == 5 &&
              b["parameters"]["bond_type"]["source_entry_indices"] == Json::array({3}),
          "bond names are exact and unknown values retain the previous confirmed assignment");
    b = parse("clouds", {"x_thickness 2", "x_secondary_color 1 2 3", "x_only 0"});
    check(b["parameters"]["thickness"]["value"] == 40 &&
              b["parameters"]["secondary_color"]["value"] == Json::array({1, 2, 3}) &&
              b["parameters"]["only"]["value"] == false,
          "cloud thickness has its own scale and its second color is not omitted");
    b = parse("marble", {"x_level_of_detail .3", "x_vein_tightness 0x1p2"});
    check(b["parameters"]["level_of_detail"]["value"] == 30 &&
              b["parameters"]["vein_tightness"]["value"] == 4,
          "legacy hex float prefixes and per-preset scale differences are retained");
    b = parse("boards", {"x_primary_color 1 2 3", "x_primary_color --2 4 5", "x_crack_width +-2"});
    check(b["parameters"]["crack_width"]["value"] == 0 &&
              b["parameters"]["primary_color"]["value"] == Json::array({1, 2, 3}),
          "repeated signs fail scanning and do not become a valid signed scalar");
    b = parse("checkr3d", {"x_scale_factor 3.5", "x_secondary_color 7 8 9"});
    check(b["parameters"]["scale_factor"]["value"] == 3.5 &&
              b["parameters"]["secondary_color"]["value"] == Json::array({7, 8, 9}),
          "checkr3d uses the native shared checker reader");
    b = parse("wood01", {"x_secondary_color 1 2 3", "x_wood_grain_scale_factor 8"});
    check(b["parameters"].size() == 2 && b["entries"][1]["status"] == "ignored_unknown_key",
          "wood01 uses the two-color reader, not similarly named boards fields");
    b = parse("bwnoise", {"x_primary_color 1 2 3"});
    check(b["parameters"].empty() && b["entries"][0]["status"] == "ignored_no_content_reader",
          "presets with no native parameter callback do not borrow a modern schema");
    auto mixed = xml_tree("<Layer><Any>x_primary_color 1 <Inner>2</Inner> 3</Any><Empty/></Layer>");
    auto dispatch =
        material_layer_semantics({{"LayerType", "layer IMAGE wood.pma"}})["reader_path"];
    const auto original = mixed;
    b = material_legacy_parameters(mixed, dispatch);
    check(b["parameters"]["primary_color"]["value"] == Json::array({1, 2, 3}) &&
              b["entries"][0]["source_content"] == "x_primary_color 1 2 3" && mixed == original,
          "legacy getContent concatenates descendant text and tails without changing the XML tree");
    dispatch = material_layer_semantics({{"LayerType", "layer IMAGE custom.pma"}})["reader_path"];
    b = material_legacy_parameters(mixed, dispatch);
    check(b["entries"].size() == 2 && b["entries"][0]["status"] == "user_data_string" &&
              b["entries"][1]["source_content"] == "" && b["parameters"].empty(),
          "custom PMA user data is an ordered string list including empty entries");
    auto g = parse("grad1d", {});
    const auto &initial = g["gradient_controls"]["channels"];
    check(g["gradient_controls"]["status"] == "decoded" && initial.size() == 5 &&
              initial[0]["points"][0]["y"] == 1 && initial[0]["points"][1]["y"] == 0 &&
              initial[4]["points"][1]["y"] == 1 &&
              g["gradient_controls"]["input_selector_source"] == "preset_constructor",
          "grad1d starts with confirmed independent constructor control lists");
    g = parse("grad1d", {"x_u_x_offsets 0;.5;1", "x_x_colors .1 .2 .3 .4 .5 .6", "x_input 3"});
    const auto &channels = g["gradient_controls"]["channels"];
    check(g["status"] == "decoded" && channels[0]["points"].size() == 3 &&
              channels[0]["points"][0]["y"] == .1 && channels[1]["points"][0]["y"] == .1 &&
              channels[2]["points"][0]["y"] == .2 && channels[3]["points"][0]["y"] == .3 &&
              channels[2]["points"][1]["x"] == .5 && channels[2]["points"][1]["y"] == .5 &&
              channels[0]["points"][2]["y"] == 0 && channels[4]["points"][2]["y"] == 1 &&
              g["gradient_controls"]["input_selector"] == 3,
          "gradient offsets rebuild five independent lists; color triples map to four channels in "
          "order");
    g = parse("grad1d", {"x_x_colors .2 .3 .4", "x_u_x_offsets 0 1", "x_x_colors .7 .8 .9"});
    check(g["gradient_controls"]["channels"][0]["points"][1]["y"] == 0 &&
              g["gradient_controls"]["channels"][2]["points"][0]["y"] == .8 &&
              g["gradient_controls"]["channels"][2]["points"][0]["position_source_entry_index"] ==
                  1 &&
              g["gradient_controls"]["channels"][2]["points"][0]["value_source_entry_index"] == 2,
          "later gradient offsets discard earlier colors before later color assignment");
    g = parse("grad1d", {"x_u_x_offsets ", "x_x_colors 1 2 3"});
    check(g["gradient_controls"]["channels"][0]["points"].empty() &&
              g["entries"][1]["status"] == "color_count_exceeds_control_count" &&
              g["gradient_controls"]["status"] == "invalid_controls",
          "empty offsets clear controls and excess colors do not fabricate or resize points");
    g = parse("grad1d", {"x_u_x_offsets 0;;1"});
    check(g["status"] == "partial" && g["entries"][0]["array"]["values"] == Json::array({0}) &&
              g["gradient_controls"]["status"] == "invalid_controls",
          "gradient parser skips exactly one separator instead of collapsing delimiters");
    g = parse("grad1d", {"x_x_colors 1 2"});
    check(g["entries"][0]["status"] == "incomplete_color_triplets" && g["status"] == "partial",
          "incomplete gradient triples retain source and fail deterministically");
    b = parse("boards", {"x_crack_width 1e999", "x_primary_color nan 2 3"});
    check(b["status"] == "partial" && b["entries"][0]["value"].is_null() &&
              b["entries"][0]["source_value"] == "1e999" &&
              b["entries"][1]["components"][0]["conversion"] == "non_finite",
          "non-finite or out-of-range conversions keep source text and explicit incomplete status");
    const auto tree =
        xml_tree("<Material material_version='9'><Map Type='1' Filename='layers.pma'><L "
                 "LayerType='layer IMAGE boards.pma'><P>x_crack_width .5</P></L></Map></Material>");
    auto settings = material_settings(tree);
    const auto &layer = settings["maps"][0]["texture_layers"]["entries"][0];
    check(layer["legacy_parameters"]["parameters"]["crack_width"]["value"] == 50 &&
              layer["legacy_parameters"]["reader_applicability"]["status"] == "read" &&
              layer["semantics"]["reader_path"]["preset"]["parameter_status"] == "decoded" &&
              settings["maps"][0]["legacy_parameters"]["reader_applicability"]["status"] ==
                  "skipped",
          "legacy parameters attach to the correct source layer and inherit native reader gating");
    auto single = tree;
    single["children"][0]["attributes"]["Filename"] = "boards.pma";
    settings = material_settings(single);
    check(settings["maps"][0]["legacy_parameters"]["status"] == "unresolved_dispatch" &&
              settings["maps"][0]["texture_layers"]["entries"][0]["legacy_parameters"]
                      ["reader_applicability"]["status"] == "skipped",
          "single Map resource indirection is not bypassed by the legacy content parser");
    return checks;
}
