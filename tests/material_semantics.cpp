#include "internal.hpp"
namespace {
unsigned checks = 0;
void check(bool b, const char *message) {
    ++checks;
    p3d::require(b, message);
}
} // namespace
unsigned material_semantics_tests() {
    using namespace p3d;
    checks = 0;
    Json a = {{"Flags", "2098172"},
              {"color.r", ".2"},
              {"color.g", ".4"},
              {"color.b", ".8"},
              {"transmit", ".3"},
              {"specular_color.r", ".1"},
              {"specular_color.g", ".5"},
              {"specular_color.b", ".9"},
              {"specular", ".45"},
              {"glow_color.r", ".7"},
              {"glow_color.g", ".6"},
              {"glow_color.b", ".5"},
              {"glow", "24"},
              {"ambient", ".5"},
              {"diffuse", ".8"},
              {"finish", ".05"},
              {"reflect", ".25"},
              {"refract", "1.4"},
              {"custom", "preserved"}};
    auto original = a;
    const std::map<std::string, unsigned> bits = {{"color", 2},
                                                  {"transparency", 6},
                                                  {"specular_color", 3},
                                                  {"specular_factor", 9},
                                                  {"glow_color", 21},
                                                  {"diffuse_factor", 7},
                                                  {"roughness_factor", 4},
                                                  {"reflect_factor", 5},
                                                  {"refract_factor", 8}};
    for (unsigned bit = 0; bit != 32; ++bit) {
        a["Flags"] = std::to_string(std::uint32_t(1) << bit);
        auto s = material_parameter_semantics(a);
        for (const auto &entry : bits)
            check(s["parameters"][entry.first]["enabled"] == (entry.second == bit),
                  "material activation bit isolated from other parameter flags");
        check(s["parameters"]["ambient_factor"]["enabled"] == true &&
                  s["parameters"]["glow_factor"]["enabled"] == true,
              "native unconditional factors independent of flags");
    }
    a = original;
    auto s = material_parameter_semantics(a);
    check(s["parameters"]["color"]["value"] == Json({.2, .4, .8}) &&
              s["parameters"]["transparency"]["value"] == .3 &&
              s["parameters"]["roughness_factor"]["value"] == .05,
          "source shader values identified without renderer conversion");
    for (const auto &item : s["parameters"])
        check(item["status"] == "decoded", "all eleven parameters decoded");
    a["Flags"] = "0";
    s = material_parameter_semantics(a);
    check(s["parameters"]["color"]["enabled"] == false &&
              s["parameters"]["color"]["value"] == Json({.2, .4, .8}),
          "disabled colors retain values");
    a.erase("Flags");
    a.erase("color.g");
    s = material_parameter_semantics(a);
    check(s["parameters"]["color"]["enabled"].is_null() &&
              s["parameters"]["color"]["status"] == "partial" &&
              s["parameters"]["color"]["components"][0]["value"] == .2,
          "missing flags and partial color do not get defaults");
    for (auto bad : {"", "+", "NaN"}) {
        a["Flags"] = bad;
        auto v = material_parameter_semantics(a);
        check(v["flags"]["status"] == "invalid" && v["parameters"]["color"]["enabled"].is_null(),
              "invalid uint flag cannot become an activation mask");
    }
    a["Flags"] = " +4294967295 ";
    s = material_parameter_semantics(a);
    check(s["flags"]["unassigned_bits"].get<std::uint32_t>() == (UINT32_MAX & ~0x2003fcu),
          "unknown flag bits retained including highest bit");
    a["finish"] = "inf";
    check(material_parameter_semantics(
              a)["parameters"]["roughness_factor"]["value"]["floating_point"] == "infinity",
          "nonfinite source coefficient retains a tagged value instead of JSON null");

    Json map = {{"Type", "1"},
                {"Filename", "relative/纹理.jpg"},
                {"pattern_off", "0"},
                {"pattern_mapping", "6"},
                {"pattern_scalemode", "3"},
                {"pattern_scale.x", "2"},
                {"pattern_scale.y", "3"},
                {"pattern_offset.x", "4"},
                {"pattern_offset.y", "5"},
                {"pattern_angle", "90"},
                {"PatternFlags", "3"},
                {"BumpFlags", "0"},
                {"origin_uv_pro_matrix_on", "0"},
                {"pattern_weight", ".4"},
                {"bump_map_scale", "8"}};
    unsigned n = 0;
    for (auto row : {"xa", "ya", "za"})
        for (auto axis : {"x", "y", "z"})
            map[std::string("origin_uv_pro_matrix_") + row + "." + axis] = std::to_string(++n);
    auto m = material_map_semantics(map);
    check(m["type"]["role"] == "pattern" && m["enabled"]["value"] == true &&
              m["mapping_mode"]["name"] == "cylindrical" &&
              m["mapping_unit"]["sdk_size_mode"] == "absolute",
          "pattern mapping identity");
    check(m["uv_scale"]["value"] == Json({2, 3}) && m["uv_offset"]["value"] == Json({4, 5}) &&
              m["rotation_degrees"]["value"] == 90,
          "native UV field association");
    check(m["projection"]["matrix_rows"] == Json({{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}) &&
              m["projection"]["enabled"]["value"] == false,
          "projection rows not transposed or dropped when off");
    check(m["use_image_alpha_channel"]["value"] == true && m["pattern_weight"]["value"] == .4,
          "pattern alpha bit independent from bump flags");
    map["PatternFlags"] = "2";
    check(material_map_semantics(map)["use_image_alpha_channel"]["value"] == false,
          "pattern alpha uses low bit rather than nonzero");
    for (auto off : {"2", "-1", "-2147483648"}) {
        map["pattern_off"] = off;
        check(material_map_semantics(map)["enabled"]["value"] == false,
              "native signed off value uses zero comparison");
    }
    map["pattern_off"] = "2147483648";
    check(material_map_semantics(map)["enabled"]["source_value"] == INT32_MIN &&
              material_map_semantics(map)["enabled"]["value"] == false,
          "signed off narrows native integer input before its zero comparison");
    for (int mode : {0, 1, 2, 4, 5, 6, 3, -1, 77}) {
        map["pattern_mapping"] = std::to_string(mode);
        m = material_map_semantics(map);
        bool known = mode == 0 || mode == 1 || mode == 2 || mode == 4 || mode == 5 || mode == 6;
        check((m["mapping_mode"]["name_status"] == "identified") == known &&
                  m["mapping_mode"]["value"] == mode,
              "unrecognized mapping modes not coerced to parametric");
    }
    const std::map<unsigned, const char *> types = {
        {0, "none"},        {1, "pattern"},        {2, "bump"},          {31, "pbr_albedo"},
        {32, "pbr_normal"}, {33, "pbr_roughness"}, {34, "pbr_metallic"}, {35, "pbr_ao"}};
    for (auto entry : types) {
        map["Type"] = std::to_string(entry.first);
        m = material_map_semantics(map);
        check(m["type"]["role"] == entry.second, "native map role identity");
        check(m.contains("use_image_alpha_channel") == (entry.first == 1),
              "pattern flag not applied to other map types");
        check(m.contains("bump_factor") == (entry.first == 2), "bump factor scoped to bump map");
    }
    map["Type"] = "4294967295";
    check(material_map_semantics(map)["type"]["role_status"] == "unknown_value",
          "unknown map preserved");
    map.erase("origin_uv_pro_matrix_ya.z");
    m = material_map_semantics(map);
    check(m["projection"]["matrix_rows"].is_null() &&
              m["projection"]["source_rows"][1]["status"] == "partial",
          "missing matrix entry not synthesized");
    map["pattern_scale.x"] = "4junk";
    check(material_map_semantics(map)["uv_scale"]["value"][0] == 4,
          "scale component accepts the native floating prefix");
    Json tree = {
        {"tag", "Material"},
        {"attributes", original},
        {"children",
         Json::array({{{"tag", "Map"}, {"attributes", map}, {"children", Json::array()}},
                      {{"tag", "Map"}, {"attributes", map}, {"children", Json::array()}}})}};
    auto copy = tree;
    auto settings = material_settings(tree);
    check(tree == copy && settings["source_parameters"] == original &&
              settings["maps"].size() == 2 && settings["maps"][0]["source_parameters"] == map &&
              settings["maps"][0]["xml_path"] != settings["maps"][1]["xml_path"],
          "semantic view preserves source strings and duplicate map occurrences");
    check(settings["semantics"]["parameters"]["glow_factor"]["value"] == 24 &&
              settings["maps"][0].contains("semantics"),
          "material settings integrates semantic views");
    auto node = [](std::uint32_t type, std::int32_t link) {
        return Json{{"native_table_member", true},
                    {"semantics",
                     {{"type", {{"status", "decoded"}, {"value", type}}},
                      {"map_link", {{"status", "decoded"}, {"value", link}}}}}};
    };
    auto frame = material_map_semantics({{"pattern_proj_offset.x", "1"},
                                         {"pattern_proj_offset.y", "2"},
                                         {"pattern_proj_offset.z", "3"},
                                         {"pattern_proj_angles.x", "4"},
                                         {"pattern_proj_angles.y", "5"},
                                         {"pattern_proj_angles.z", "6"},
                                         {"pattern_proj_scale.x", "7"},
                                         {"pattern_proj_scale.y", "8"},
                                         {"pattern_proj_scale.z", "9"}});
    check(frame["projection_frame"]["offset"]["value"] == Json({1, 2, 3}) &&
              frame["projection_frame"]["angles"]["value"] == Json({4, 5, 6}) &&
              frame["projection_frame"]["scale"]["value"] == Json({7, 8, 9}),
          "projection frame components retain source order and scale");
    Json maps = Json::array({node(1, 2), node(2, 31), node(31, 0), node(30, 1)});
    auto binding = material_map_bindings(maps);
    check(binding["status"] == "resolved" && binding["entries"][0]["terminal_map_index"] == 2 &&
              binding["entries"][0]["texture_layers_mode"] == "native_shared",
          "transitive native texture layer sharing");
    check(binding["entries"][3]["texture_layers_source_map_index"] == 3 &&
              binding["entries"][3]["texture_mapping_source_map_index"] == 2 &&
              binding["entries"][3]["projection_frame_source_map_index"] == 2 &&
              binding["entries"][3]["map_settings_source_map_index"] == 3,
          "type30 retains local layers and map settings while inheriting mapping");
    maps.push_back(node(31, 0));
    binding = material_map_bindings(maps);
    check(binding["entries"][2]["selection"] == "superseded" &&
              binding["entries"][2]["replaced_by_map_index"] == 4 &&
              binding["entries"][0]["terminal_map_index"] == 4,
          "duplicate native type is replaced by later occurrence without deleting source");
    maps = Json::array({node(1, 2), node(2, 7), node(3, 1)});
    binding = material_map_bindings(maps);
    check(binding["entries"][0]["status"] == "dangling_link_cleared" &&
              binding["entries"][1]["status"] == "dangling_link_cleared" &&
              binding["entries"][2]["terminal_map_index"] == 0,
          "dangling chain cleanup follows signed type order rather than simultaneous clearing");
    maps = Json::array({node(3, 2), node(2, 7), node(1, 3)});
    binding = material_map_bindings(maps);
    check(binding["entries"][2]["status"] == "dangling_link_cleared" &&
              binding["entries"][1]["status"] == "dangling_link_cleared" &&
              binding["entries"][0]["terminal_map_index"] == 1,
          "cleanup independent of XML order and can resolve predecessor to newly cleared node");
    maps = Json::array({node(UINT32_MAX, 9), node(1, -1)});
    binding = material_map_bindings(maps);
    check(binding["entries"][0]["native_type_key"] == -1 &&
              binding["entries"][1]["terminal_map_index"] == 0 &&
              binding["entries"][1]["status"] == "linked",
          "uint XML types use signed native key order");
    maps = Json::array({node(1, 1), node(2, 0), node(0, 1), node(3, 4), node(4, 3), node(5, 3)});
    binding = material_map_bindings(maps);
    check(binding["entries"][0]["status"] == "local" &&
              binding["entries"][0]["normalized_link_type"] == 0 &&
              binding["entries"][2]["selection"] == "ignored_zero_type",
          "self references normalize and zero type is ignored");
    for (unsigned i : {3, 4, 5})
        check(binding["entries"][i]["status"] == "cyclic_link_chain" &&
                  !binding["entries"][i].contains("terminal_map_index"),
              "cycles and incoming chains cannot invent fallback");
    maps = Json::array({node(1, 2), node(2, 0), node(3, 0)});
    maps[1]["semantics"]["map_link"] = {{"status", "invalid"}, {"value", nullptr}};
    maps[2]["semantics"]["map_link"] = {{"status", "missing"}, {"value", nullptr}};
    binding = material_map_bindings(maps);
    check(binding["status"] == "incomplete" &&
              binding["entries"][0]["status"] == "invalid_link_chain" &&
              binding["entries"][2]["status"] == "local",
          "invalid links propagate while missing links use native zero");
    Json nested = {
        {"tag", "Material"},
        {"attributes", Json::object()},
        {"children",
         Json::array(
             {{{"tag", "mAp"},
               {"attributes", {{"Type", "1"}, {"map_link", "2"}}},
               {"children", Json::array()}},
              {{"tag", "Container"},
               {"attributes", Json::object()},
               {"children", Json::array({{{"tag", "Map"},
                                          {"attributes", {{"Type", "2"}, {"map_link", "0"}}},
                                          {"children", Json::array()}}})}},
              {{"tag", "Map"}, {"attributes", {{"Type", "3"}}}, {"children", Json::array()}}})}};
    auto ns = material_settings(nested);
    check(ns["maps"].size() == 3 &&
              ns["map_bindings"]["entries"][0]["status"] == "dangling_link_cleared" &&
              ns["map_bindings"]["entries"][1]["selection"] == "outside_native_table",
          "ASCII case-insensitive native child tags do not flatten nested maps into the table");
    // A separate traversal oracle reproduces the reader's sequential loop.
    // This checks the linear reverse-propagation implementation against both
    // chain directions, missing targets, cycles and several clearing orders.
    std::uint32_t random = 0x97b43210u;
    for (unsigned trial = 0; trial < 80; ++trial) {
        std::vector<int> links(12);
        maps = Json::array();
        for (int i = 0; i < 12; ++i) {
            random = random * 1664525u + 1013904223u;
            links[i] = int(random % 16);
            maps.push_back(node(i + 1, links[i]));
            if (links[i] == i + 1)
                links[i] = 0;
        }
        auto find = [&](int start) {
            std::set<int> visited;
            int at = start;
            while (links[at]) {
                if (!visited.insert(at).second)
                    return -2;
                at = links[at] - 1;
                if (at >= 12)
                    return -1;
            }
            return at;
        };
        for (int i = 0; i < 12; ++i)
            if (find(i) == -1)
                links[i] = 0;
        binding = material_map_bindings(maps);
        for (int i = 0; i < 12; ++i) {
            auto terminal = find(i);
            check(terminal == -2 ? binding["entries"][i]["status"] == "cyclic_link_chain"
                                 : binding["entries"][i]["terminal_map_index"] == terminal,
                  "native sequential lookup oracle matches binding graph");
        }
    }
    maps = Json::array();
    for (unsigned i = 1; i <= 4000; ++i)
        maps.push_back(node(i, i + 1));
    binding = material_map_bindings(maps);
    check(binding["entries"][0]["terminal_map_index"] == 0 &&
              binding["entries"][3999]["terminal_map_index"] == 3999,
          "long ascending missing chain has bounded iterative cleanup");
    Json layer = {{"LayerType", u8"layer IMAGE E:\\材质\\红桦木.jpg"},
                  {"LayerFlags", "4294967295"},
                  {"LayerDataFlags", "4294967295"},
                  {"pattern_scale.x", "2"},
                  {"pattern_scale.y", "3"},
                  {"pattern_mapping", "-1"},
                  {"texture_filter_type", "7"},
                  {"scale_z", "0"},
                  {"offset_z", "4"},
                  {"pattern_offset.x", "5"},
                  {"image_gamma", "2.2"},
                  {"pattern_opacity", ".4"},
                  {"layer_gamma", "9"},
                  {"custom", "untouched"}};
    const auto layer_original = layer;
    auto ls = material_layer_semantics(layer);
    check(ls["type"]["reader_type"] == 1 && ls["type"]["argument"] == u8"E:\\材质\\红桦木.jpg" &&
              ls["type"]["argument_role"] == "texture_reference",
          "texture layer keeps opaque Unicode reference and source type");
    check(ls["parameters"]["pattern_scale"]["value"] == Json::array({2, 3}) &&
              ls["parameters"]["scale_z"]["value"] == 0 &&
              ls["parameters"]["pattern_offset"]["status"] == "partial" &&
              ls["parameters"]["offset_z"]["value"] == 4 &&
              !ls["parameters"].contains("layer_gamma") && layer == layer_original,
          "layer parameters retain source zeros, partial vectors and do not read gamma operator "
          "field");
    check(ls["data_flags"]["reader_value"] == 0xfffff9ffu &&
              ls["data_flags"]["discarded_source_bits"] == 0xc00u &&
              ls["flags"]["unassigned_bits"] == 0xfffffff8u &&
              ls["flags"]["bits"][2]["value"] == true,
          "layer data flags and object flags use distinct bit layouts");
    for (unsigned i = 0; i < 32; ++i) {
        layer["LayerDataFlags"] = std::to_string(1u << i);
        ls = material_layer_semantics(layer);
        const auto expected = i == 9 ? 0x800u : i == 10 || i == 11 ? 0u : 1u << i;
        check(ls["data_flags"]["reader_value"] == expected,
              "each serialized layer data flag follows native relocation");
    }
    const std::vector<std::string> tokens = {"IMAGE",
                                             "PROCEDURE",
                                             "GRADIENT",
                                             "NORMAL",
                                             "ADD",
                                             "SUBTRACT",
                                             "ALPHA",
                                             "DISSOLVE",
                                             "ATOP",
                                             "IN",
                                             "OUT",
                                             "GAMMA",
                                             "TINT",
                                             "BRIGHTNESS",
                                             "CONTRAST",
                                             "GROUP_START",
                                             "GROUP_END",
                                             "ALPHABACKGROUND_START",
                                             "ALPHABACKGROUND_END",
                                             "LXOPROCEDURE",
                                             "DIFFERENCE",
                                             "NORMALMULTIPLY",
                                             "DIVIDE",
                                             "MULTIPLY",
                                             "SCREEN",
                                             "OVERLAY",
                                             "SOFTLIGHT",
                                             "HARDLIGHT",
                                             "DARKEN",
                                             "LIGHTEN",
                                             "COLORDODGE",
                                             "COLORBURN",
                                             "8119LXOPROCEDURE",
                                             "CELL",
                                             "TEXTURE_REPLICATOR"};
    const std::vector<unsigned> codes = {1,      2,      3,      0xf000, 0xf001, 0xf002, 0xf003,
                                         1,      1,      1,      1,      0xf00c, 0xf00d, 0xf00e,
                                         0xf00f, 0xf012, 0xf013, 0xf014, 0xf015, 4,      0xf016,
                                         0xf017, 0xf018, 0xf019, 0xf01a, 0xf01b, 0xf01c, 0xf01d,
                                         0xf01e, 0xf01f, 0xf020, 0xf021, 4,      5,      7};
    for (std::size_t i = 0; i < tokens.size(); ++i) {
        layer["LayerType"] = "layer " + tokens[i] + " argument";
        ls = material_layer_semantics(layer);
        check(ls["type"]["token_id"] == i && ls["type"]["reader_type"] == codes[i],
              "layer type token and reader code are different namespaces");
        check(ls["flags"]["reader_applies"] == (i != 13 && i != 14 && i != 33),
              "brightness contrast and cell reader skip layer flags");
    }
    layer["LayerType"] = "prefixlayer \tGAMMA  unused";
    ls = material_layer_semantics(layer);
    check(ls["parameters"].size() == 1 && ls["parameters"]["layer_gamma"]["value"] == 9 &&
              ls["type"]["argument_role"] == "not_read",
          "gamma reader finds the marker anywhere but only reads layer_gamma");
    layer["LayerType"] = "layer TINT";
    layer["layer_color.r"] = ".2";
    layer["layer_color.g"] = "NaN";
    ls = material_layer_semantics(layer);
    check(ls["parameters"]["layer_color"]["status"] == "partial" &&
              ls["parameters"]["layer_color"]["value"].is_null(),
          "tint preserves incomplete color independently of nonfinite component decoding");
    layer["LayerType"] = "layer GROUP_START \tmy group  ";
    ls = material_layer_semantics(layer);
    check(ls["type"]["argument"] == "my group" && ls["type"]["argument_role"] == "group_name" &&
              ls["parameters"].empty(),
          "group name uses full trimmed remainder, not one filename token");
    for (const auto text :
         {"layer image name", "layer IMAGE\tname", "layer Unknown name", "layer"}) {
        ls = material_layer_semantics({{"LayerType", text}});
        check(ls["type"]["status"] == "unknown_token_image_fallback" &&
                  ls["type"]["reader_type"] == 1 && ls["flags"]["value"].is_null(),
              "case and literal space delimiter reproduce native fallback without inventing flags");
    }
    for (const auto &text : {std::string("Layer IMAGE name"), std::string("IMAGE")}) {
        ls = material_layer_semantics({{"LayerType", text}});
        check(ls["type"]["status"] == "ignored_missing_layer_marker" && !ls.contains("parameters"),
              "missing lowercase marker is ignored");
    }
    for (const auto text : {u8"layer \u3000IMAGE name", u8"layer IMAGE \u00a0name"}) {
        ls = material_layer_semantics({{"LayerType", text}});
        check(ls["type"]["status"] == "locale_dependent_whitespace" &&
                  ls["type"]["reader_type"].is_null(),
              "non-ASCII boundary spaces are not silently normalized across locales");
    }
    check(material_layer_semantics(Json::object())["type"]["status"] == "missing" &&
              material_layer_semantics({{"LayerType", 7}})["type"]["status"] == "invalid" &&
              material_layer_semantics(
                  {{"LayerType", std::string("layer IMAGE\0x", 13)}})["type"]["status"] ==
                  "invalid",
          "absent, non-string and embedded NUL layer identifiers remain distinguishable");
    layer["LayerType"] = "layer IMAGE a";
    layer["LayerFlags"] = "4294967296";
    layer["LayerDataFlags"] = "-1";
    ls = material_layer_semantics(layer);
    check(ls["flags"]["value"] == 0 && ls["data_flags"]["value"] == UINT32_MAX,
          "unsigned layer flags follow native scanf sign and narrowing rules");
    Json children = Json::array();
    for (const auto &tag : {"Layer", "Other", "Container"})
        children.push_back({{"tag", tag},
                            {"attributes", {{"LayerType", "layer ADD"}, {"custom", "keep"}}},
                            {"children", Json::array()}});
    children[0]["children"].push_back(children[1]);
    children[1]["attributes"].erase("LayerType");
    nested["children"][0]["children"] = children;
    auto source_tree = nested;
    ns = material_settings(nested);
    const auto &entries = ns["maps"][0]["texture_layers"]["entries"];
    check(entries.size() == 2 && entries[0]["child_index"] == 0 && entries[1]["child_index"] == 2 &&
              entries[1]["source_parameters"]["custom"] == "keep" && nested == source_tree,
          "layer candidates preserve direct XML child order without flattening subtrees");
    check(ns["maps"][0]["texture_layers"]["activation_status"] == "not_evaluated" &&
              ns["maps"][0]["texture_layers"]["composition_status"] == "not_evaluated",
          "source layer decoding does not claim map activation or evaluated compositing");
    auto extra = [](const Json &source) {
        return material_map_semantics({{"M556", source}})["additional_texture_references"];
    };
    auto values = [&](const std::string &source) {
        Json result = Json::array();
        const auto parsed = extra(source);
        for (const auto &entry : parsed["entries"])
            result.push_back(entry["value"]);
        return result;
    };
    check(values(u8R"("E:\材质\含 空格.jpg","纹理,颜色.jpg",relative.jpg,)") ==
              Json::array({u8"E:\\材质\\含 空格.jpg", u8"纹理,颜色.jpg", "relative.jpg"}),
          "M556 quoted references preserve Unicode paths and embedded commas");
    check(values("a,,b,") == Json::array({"a", "", "b"}) &&
              values(", ,") == Json::array({"", ""}) &&
              values("a ,b") == Json::array({"a", "", "b"}) &&
              values("a\t b") == Json::array({"a", "b"}) && values("a\nb") == Json::array({"a\nb"}),
          "M556 skips spaces and tabs but preserves empty comma tokens and newlines");
    check(values(R"("" "a""b c")") == Json::array({"", "a\"b", "c"}),
          "M556 native doubled quote toggles quote state, unlike CSV");
    auto unterminated = extra(R"("a,b)");
    check(unterminated["status"] == "decoded" && unterminated["entries"][0]["value"] == "a,b" &&
              unterminated["entries"][0]["unterminated_quote"] == true,
          "M556 reports an accepted unterminated quote without losing the native token");
    for (unsigned n = 0; n < 14; ++n) {
        const std::string slashes(n, '\\');
        const auto input = slashes + "\"a b" + (n % 2 == 0 ? "\"" : "");
        Json expected = n % 2 == 0 ? Json::array({std::string(n / 2, '\\') + "a b"})
                                   : Json::array({std::string(n / 2, '\\') + "\"a", "b"});
        check(values(input) == expected, "M556 backslash parity controls quote escaping");
        check(values(slashes + "x,") == Json::array({slashes + "x"}),
              "M556 backslashes before ordinary characters remain literal");
    }
    const std::string extra_source = u8"  \"纹理,颜色.jpg\",same,same,";
    auto refs = extra(extra_source);
    check(refs["entries"].size() == 3 && refs["entries"][1]["value"] == "same" &&
              refs["entries"][2]["value"] == "same" && refs["entries"][2]["append_index"] == 2,
          "M556 preserves native reference order and duplicate references");
    for (const auto &entry : refs["entries"]) {
        const auto offset = entry["source_byte_offset"].get<std::size_t>();
        const auto count = entry["source_byte_count"].get<std::size_t>();
        check(extra_source.substr(offset, count) == entry["source_fragment"],
              "M556 source spans use UTF-8 bytes and retain original quoting");
    }
    check(extra(5)["status"] == "invalid" && extra(nullptr)["status"] == "invalid" &&
              extra(std::string("a\0b", 3))["status"] == "invalid" &&
              extra(" \t")["entries"].empty() && extra("")["status"] == "decoded" &&
              material_map_semantics(Json::object())["additional_texture_references"]["status"] ==
                  "missing",
          "invalid, empty and missing additional reference lists stay distinct");
    const auto ignored_extra =
        material_layer_semantics({{"LayerType", "layer GAMMA"}, {"M556", "ignored.jpg"}});
    const auto image_extra =
        material_layer_semantics({{"LayerType", "layer IMAGE primary.jpg"}, {"M556", "extra.jpg"}});
    check(!ignored_extra.contains("additional_texture_references") &&
              image_extra["additional_texture_references"]["entries"][0]["value"] == "extra.jpg",
          "only native texture provider layer branches consume M556");
    Json ref_tree = {
        {"tag", "Material"},
        {"attributes", Json::object()},
        {"children",
         Json::array(
             {{{"tag", "Map"},
               {"attributes", {{"Type", "1"}, {"Filename", "primary.jpg"}, {"M556", "same,same,"}}},
               {"children",
                Json::array(
                    {{{"tag", "Layer"},
                      {"attributes",
                       {{"LayerType", "layer IMAGE layer.jpg"}, {"M556", "detail.jpg"}}},
                      {"children", Json::array()}},
                     {{"tag", "Layer"},
                      {"attributes",
                       {{"LayerType", "layer GROUP_START named group"}, {"M556", "not_read"}}},
                      {"children", Json::array()}},
                     {{"tag", "Unrelated"},
                      {"attributes", {{"M556", "unrelated"}}},
                      {"children", Json::array()}}})}}})}};
    const auto saved_ref_tree = ref_tree;
    auto ref_settings = material_settings(ref_tree);
    auto inventory = material_texture_references(ref_tree, ref_settings);
    check(inventory.size() == 5 && inventory[0]["filename"] == "primary.jpg" &&
              !inventory[0].contains("source_attribute") && inventory[1]["filename"] == "same" &&
              inventory[2]["append_index"] == 1 && inventory[1]["layer_child_index"].is_null() &&
              inventory[3]["filename"] == "layer.jpg" &&
              inventory[3]["source_attribute"] == "LayerType" &&
              inventory[4]["filename"] == "detail.jpg" && inventory[4]["layer_child_index"] == 0 &&
              inventory[4]["map_index"] == 0 && ref_tree == saved_ref_tree,
          "material resource inventory includes ordered additional and layer primary references");
    auto xml_node = [](const std::string &tag, const Json &attributes,
                       const Json &children = Json::array()) {
        return Json{{"tag", tag}, {"attributes", attributes}, {"children", children}};
    };
    auto procedure_owner = [&](const Json &attributes) {
        return xml_node("Map", Json::object(), Json::array({xml_node("M633", attributes)}));
    };
    Json procedure = {{"M634", "1"},          {"Color1.R", ".2"}, {"Color1.G", ".3"},
                      {"Color1.B", ".4"},     {"Color2.R", "2"},  {"Alpha1", ".5"},
                      {"M711", "4294967295"}, {"M1", "7"},        {"unknown", "kept"}};
    auto procedural = material_procedure_nodes(procedure_owner(procedure));
    auto first = procedural["entries"][0];
    check(first["parameters"]["Color1"]["value"] == Json::array({.2, .3, .4}) &&
              first["parameters"]["Color2"]["status"] == "partial" &&
              first["parameters"]["Alpha1"]["value"] == .5 &&
              first["parameters"]["M711"]["value"] == 4294967295u &&
              first["unmapped_source_parameters"] == Json({{"M1", "7"}, {"unknown", "kept"}}),
          "procedure schemas decode only attributes read for the selected source type");
    procedure = {{"M634", "32"},       {"M24.R", "1"},    {"M24.G", "2"},   {"M24.B", "3"},
                 {"NoiseSeed", "7.5"}, {"Absolute", "2"}, {"Color1.R", "4"}};
    first = material_procedure_nodes(procedure_owner(procedure))["entries"][0];
    check(first["parameters"]["M24"]["value"] == Json::array({1, 2, 3}) &&
              first["parameters"]["NoiseSeed"]["value"] == 7.5 &&
              first["parameters"]["Absolute"]["source_type"] == "uint32" &&
              first["unmapped_source_parameters"]["Color1.R"] == "4",
          "procedure noise seed is stored as floating point, not guessed from its name");
    first = material_procedure_nodes(procedure_owner(
        {{"M634", "54"}, {"M81", "2.5"}, {"M82.R", "1"}, {"M82.G", "NaN"}}))["entries"][0];
    check(first["parameters"]["M81"]["value"] == 2.5 &&
              first["parameters"]["M82"]["status"] == "partial" &&
              first["parameters"]["M82"]["components"][1]["value"]["floating_point"] == "nan" &&
              !first["parameters"].contains("M24"),
          "procedure color retains nonfinite components while reporting the missing blue value");
    for (const auto &id : {"0", "71", "4294967295", "-1", "4294967296", "invalid"}) {
        first =
            material_procedure_nodes(procedure_owner({{"M634", id}, {"M1", "3"}}))["entries"][0];
        check(first["reader_schema_status"] == "unavailable" && first["parameters"].empty() &&
                  first["source_parameters"]["M634"] == id &&
                  first["unmapped_source_parameters"]["M1"] == "3",
              "unknown or invalid procedure types retain source without selecting another schema");
    }
    auto no_fallback = xml_node(
        "Map", Json::object(),
        Json::array({xml_node("m633", {{"M634", "32"}}), xml_node("M633", {{"M634", "invalid"}}),
                     xml_node("M633", {{"M634", "1"}})}));
    procedural = material_procedure_nodes(no_fallback);
    check(procedural["entries"].size() == 2 && procedural["entries"][0]["child_index"] == 1 &&
              procedural["entries"][0]["selection"] == "selected" &&
              procedural["entries"][0]["type"]["status"] == "invalid" &&
              procedural["entries"][1]["selection"] == "later_matching_node",
          "native procedure lookup selects first exact child even if its type is invalid");
    Json controls = Json::array({xml_node("AnyTag", {{"M175", ".25"},
                                                     {"M176", ".75"},
                                                     {"M211", "0"},
                                                     {"M177", "3"},
                                                     {"unknown", "value"}}),
                                 xml_node("Second", {{"M211", "4"}})});
    auto channel = xml_node(
        "ArbitraryChannel", Json::object(),
        Json::array({xml_node("M635", {{"M171", "999"}, {"M173", "3"}, {"M174", "5"}}, controls),
                     xml_node("M635", {{"M173", "9"}})}));
    Json channels = Json::array();
    for (unsigned i = 0; i < 6; ++i)
        channels.push_back(channel);
    auto type10 =
        xml_node("M633", {{"M634", "10"}, {"M218", "3"}, {"M175", "unrelated"}}, channels);
    auto type10_owner = xml_node("Map", Json::object(), Json::array({type10}));
    first = material_procedure_nodes(type10_owner)["entries"][0];
    const auto &c0 = first["channels"][0]["channel"]["entries"][0];
    check(first["parameters"].size() == 1 && first["parameters"]["M218"]["value"] == 3 &&
              first["unmapped_source_parameters"]["M175"] == "unrelated" &&
              first["channels"][4]["selection"] == "selected" &&
              first["channels"][5]["selection"] == "outside_native_channel_limit",
          "type ten consumes five source channels without flattening control fields");
    check(c0["parameters"]["M171"]["reader_applies"] == false &&
              c0["reader_selector"]["source_keys"] == Json::array({"M174"}) &&
              c0["reader_selector"]["value"] == 5 && c0["control_entries"].size() == 2 &&
              first["channels"][0]["channel"]["entries"][1]["selection"] == "later_matching_node",
          "channel count marker is discarded, later selector replaces earlier value, first M635 "
          "wins");
    check(c0["control_entries"][0]["parameters"]["M175"]["value"] == .25 &&
              c0["control_entries"][0]["parameters"]["M211"]["value"] == 0 &&
              c0["control_entries"][0]["parameters"]["M211"]["reader_value"] == 1 &&
              c0["control_entries"][1]["parameters"]["M211"]["reader_value"] == 4 &&
              c0["control_entries"][0]["unmapped_source_parameters"]["unknown"] == "value",
          "control entry order and explicit zero normalization preserve original values");
    type10_owner["children"][0]["children"][0]["children"][0]["attributes"].erase("M174");
    first = material_procedure_nodes(type10_owner)["entries"][0];
    check(first["channels"][0]["channel"]["entries"][0]["reader_selector"]["value"] == 3,
          "missing channel selector uses earlier explicitly stored value");
    auto layer_proc = xml_node("Layer", {{"LayerType", "layer GRADIENT"}}, Json::array({type10}));
    auto map_proc = xml_node("Map", {{"Type", "1"}}, Json::array({layer_proc}));
    auto root_proc = xml_node("Material", Json::object(), Json::array({map_proc}));
    const auto original_proc = root_proc;
    auto integration = material_settings(root_proc);
    check(integration["maps"][0]["procedures"]["entries"].empty() &&
              integration["maps"][0]["texture_layers"]["entries"][0]["procedures"]["entries"][0]
                         ["type"]["value"] == 10 &&
              root_proc == original_proc,
          "nested procedure belongs to its layer, not the enclosing map");
    auto replication = xml_node("M541", {{"M542", "2.5"},
                                         {"M547", "-3"},
                                         {"M548", "4294967295"},
                                         {"M549", "0"},
                                         {"M550", "1e-3"},
                                         {"M554", "17"},
                                         {"M555", "4294967296"},
                                         {"unmapped", "preserved"}});
    auto repl_owner = xml_node("Layer", Json::object(),
                               Json::array({xml_node("m541", {{"M542", "99"}}), replication,
                                            xml_node("M541", {{"M542", "6"}})}));
    auto replicas = material_replicator_nodes(repl_owner);
    const auto &rp = replicas["entries"][0]["parameters"];
    check(
        replicas["entries"].size() == 2 && replicas["entries"][0]["child_index"] == 1 &&
            replicas["entries"][1]["selection"] == "later_matching_node" && rp.size() == 14 &&
            rp["M542"]["value"] == 2.5 && rp["M547"]["value"] == -3 &&
            rp["M548"]["value"] == 4294967295u && rp["M548"]["reader_value"] == true &&
            rp["M549"]["reader_value"] == false && rp["M550"]["value"] == .001 &&
            rp["M554"]["value"] == 17 && rp["M555"]["value"] == 0 &&
            rp["M543"]["status"] == "missing" &&
            replicas["entries"][0]["unmapped_source_parameters"] ==
                Json({{"unmapped", "preserved"}}),
        "replicator selects exact first child and preserves source boolean integers and unknowns");
    auto reader_tree = [&](const Json &attrs, const Json &children = Json::array(),
                           const Json &root_attrs = Json({{"material_version", "9"}})) {
        return xml_node("Material", root_attrs, Json::array({xml_node("Map", attrs, children)}));
    };
    const auto packages = Json::array({xml_node("M633", {{"M634", "1"}}), replication});
    auto reader = material_settings(reader_tree({{"Type", "1"}, {"layer", "6"}}, packages));
    check(reader["reader_profile"]["mode"] == 9 &&
              reader["reader_profile"]["provider_type_source_key"] == "layer" &&
              reader["maps"][0]["reader_path"]["single_provider_type_before_preset"]
                    ["reader_value"] == 4 &&
              reader["maps"][0]["procedures"]["reader_applicability"]["status"] == "read" &&
              reader["maps"][0]["replicators"]["reader_applicability"]["status"] == "skipped",
          "single Map type six normalizes before procedure selection when resource list is empty");
    for (const auto &attrs : {Json::object(), Json({{"material_version", "0"}})}) {
        reader = material_settings(
            reader_tree({{"Type", "1"}, {"layer", "3"}, {"layer_state", "7"}}, packages, attrs));
        check(reader["reader_profile"]["mode"] == 0 &&
                  reader["reader_profile"]["provider_type_source_key"] == "layer_state" &&
                  reader["maps"][0]["replicators"]["reader_applicability"]["status"] == "read" &&
                  reader["maps"][0]["procedures"]["reader_applicability"]["status"] == "skipped",
              "XML file reader missing or zero version uses layer_state instead of layer");
    }
    reader = material_settings(reader_tree({{"Type", "1"}}, packages));
    check(reader["maps"][0]["reader_path"]["single_provider_type_before_preset"]["status"] ==
                  "missing" &&
              reader["maps"][0]["reader_path"]["single_provider_type_before_preset"]
                    ["reader_value"] == 1 &&
              reader["maps"][0]["procedures"]["reader_applicability"]["status"] == "skipped",
          "missing provider type retains source absence while reporting native constructor type");
    reader = material_settings(reader_tree({{"Type", "1"}, {"layer", "0"}}, packages));
    check(reader["maps"][0]["reader_path"]["single_provider_type_before_preset"]["reader_value"] ==
              1,
          "explicit zero provider type uses image type for an empty resource list");
    for (const auto filename : {"image.jpg", "wood.pma", "", "LAYEREDPROCEDURALNAME"}) {
        reader = material_settings(
            reader_tree({{"Type", "1"}, {"layer", "0"}, {"Filename", filename}}, packages));
        check(reader["maps"][0]["reader_path"]["branch"] == "single_provider" &&
                  reader["maps"][0]["reader_path"]["single_provider_type_before_preset"]
                        ["reader_value"] == (std::string(filename) == "wood.pma" ? 2 : 1) &&
                  reader["maps"][0]["reader_path"]["first_resource_reference"]["value"] ==
                      filename &&
                  reader["maps"][0]["procedures"]["reader_applicability"]["status"] == "skipped",
              "ordinary Map filenames pass through the default resource service unchanged");
    }
    for (const auto &root_attrs :
         {Json({{"material_version", "9"}}), Json({{"material_version", "bad"}})}) {
        reader = material_settings(
            reader_tree({{"Type", "30"}, {"layer", "bad"}}, packages, root_attrs));
        check(reader["maps"][0]["reader_path"]["single_provider_type_before_preset"]
                    ["reader_value"] == 5 &&
                  reader["maps"][0]["procedures"]["reader_applicability"]["status"] == "skipped",
              "Map type thirty overrides the single provider even with an undecodable source "
              "selector");
    }
    reader = material_settings(
        reader_tree({{"Type", "1"}, {"layer", "3"}}, packages, {{"material_version", 9}}));
    check(reader["reader_profile"]["mode"].is_null() &&
              reader["maps"][0]["procedures"]["reader_applicability"]["status"] == "unresolved",
          "non-text XML input remains unresolved instead of fabricating an attribute value");
    Json layers_for_reader = packages;
    layers_for_reader.push_back(
        xml_node("AnyTag", {{"LayerType", "layer GRADIENT image.jpg"}}, packages));
    layers_for_reader.push_back(
        xml_node("AnyTag", {{"LayerType", "layer TEXTURE_REPLICATOR image.jpg"}}, packages));
    layers_for_reader.push_back(xml_node("AnyTag", {{"LayerType", "layer GAMMA"}}, packages));
    layers_for_reader.push_back(xml_node("AnyTag", {{"LayerType", "LAYER GRADIENT"}}, packages));
    for (const auto filename : {u8"E:\\材质\\LaYeRs.PmA", "catalog:layers.pma", "layers.pma"}) {
        auto source = reader_tree(
            {{"Type", "1"}, {"Filename", filename}, {"layer", "7"}, {"M556", "ignored.jpg"}},
            layers_for_reader);
        auto original = source;
        reader = material_settings(source);
        const auto &m = reader["maps"][0];
        const auto &le = m["texture_layers"]["entries"];
        check(
            m["reader_path"]["branch"] == "layer_collection" &&
                m["procedures"]["reader_applicability"]["status"] == "skipped" &&
                m["semantics"]["additional_texture_references"]["reader_applicability"]["status"] ==
                    "skipped" &&
                le[0]["child_index"] == 2 && le[0]["reader_applicability"]["status"] == "read" &&
                le[0]["procedures"]["reader_applicability"]["status"] == "read" &&
                le[1]["replicators"]["reader_applicability"]["status"] == "read" &&
                le[2]["procedures"]["reader_applicability"]["status"] == "skipped" &&
                le[3]["reader_applicability"]["status"] == "skipped" && source == original,
            "layers.pma activates direct source layers, not Map-level packages or texture "
            "references");
    }
    reader = material_settings(
        reader_tree({{"Type", "1"}, {"Filename", "layers.pma.backup"}}, layers_for_reader));
    check(reader["maps"][0]["texture_layers"]["entries"][0]["procedures"]["reader_applicability"]
                ["status"] == "skipped",
          "layer nodes are not read by a single provider just because they exist in XML");
    for (const auto &attrs : {Json::object(), Json({{"Type", "0"}, {"Filename", "layers.pma"}})}) {
        reader = material_settings(reader_tree(attrs, layers_for_reader));
        check(reader["maps"][0]["reader_path"]["branch"] == "skipped" &&
                  reader["maps"][0]["texture_layers"]["reader_applicability"]["status"] ==
                      "skipped",
              "missing and zero Map type cannot activate source layers");
    }
    reader = material_settings(xml_node(
        "Material", Json::object(),
        Json::array(
            {xml_node("Wrapper", Json::object(),
                      Json::array({xml_node("Map", {{"Type", "1"}, {"Filename", "layers.pma"}},
                                            layers_for_reader)}))})));
    check(reader["maps"][0]["reader_path"]["branch"] == "skipped" &&
              reader["maps"][0]["procedures"]["reader_applicability"]["reason"] ==
                  "outside_native_table",
          "nested Map inventory is separate from native direct Map reader");
    for (const auto &preset : std::vector<std::pair<std::string, unsigned>>{
             {"checker", 14}, {"wood01", 29}, {"grad1d", 31}}) {
        auto name = preset.first;
        for (auto &c : name)
            if (c >= 'a' && c <= 'z')
                c += 'A' - 'a';
        const auto p = material_layer_semantics(
            {{"LayerType", "layer TEXTURE_REPLICATOR " + name + ".PMA"}})["reader_path"];
        check(p["branch"] == "pma_preset" && p["preset"]["procedure_type"] == preset.second &&
                  p["provider_type_after_dispatch"] == (preset.second == 31 ? 3 : 4),
              "PMA basename selects native preset and replaces the previous provider type");
    }
    auto layer_reader = [&](const std::string &reference) {
        return material_layer_semantics(
            {{"LayerType", "layer LXOPROCEDURE " + reference}})["reader_path"];
    };
    check(layer_reader("custom.pma")["branch"] == "pma_user_data" &&
              layer_reader("custom.pma")["provider_type_after_dispatch"] == 4 &&
              layer_reader("custom.jpg")["branch"] == "M633" &&
              layer_reader("library:WOOD.PMA")["preset"]["procedure_type"] == 28 &&
              layer_reader("lib:E:wood.pma")["branch"] == "pma_user_data" &&
              layer_reader(u8"E:\\材质\\wood.pma")["preset"]["procedure_type"] == 28,
          "layer dispatch observes native prefix splitting and PMA priority over M633");
    auto overlong = layer_reader(std::string(260, 'x') + ".pma");
    auto boundary = layer_reader(std::string(259, 'x') + ".pma");
    check(overlong["source_reference"]["status"] == "native_buffer_limit_clears_parts" &&
              overlong["branch"] == "M633" && boundary["branch"] == "pma_user_data" &&
              layer_reader(std::string(259, 'x') + "/wood.pma")["branch"] == "M633" &&
              layer_reader(std::string(258, 'x') + "/wood.pma")["preset"]["procedure_type"] == 28,
          "native per-component UTF16 buffer limits affect dispatch, not total path length");
    std::string unicode_dir;
    for (unsigned i = 0; i < 129; ++i)
        unicode_dir += u8"\U0001f333";
    check(layer_reader(unicode_dir + "/wood.pma")["preset"]["procedure_type"] == 28 &&
              layer_reader(unicode_dir + "x/wood.pma")["source_reference"]["status"] ==
                  "native_buffer_limit_clears_parts" &&
              layer_reader(u8"wood.p\u00e1")["status"] == "unresolved",
          "supplementary Unicode counts as two native wchar units and locale comparisons remain "
          "explicit");
    auto preset_layer = xml_node("L", {{"LayerType", "layer GRADIENT wood.pma"}}, packages);
    reader = material_settings(
        reader_tree({{"Type", "1"}, {"Filename", "layers.pma"}}, Json::array({preset_layer})));
    check(reader["maps"][0]["texture_layers"]["entries"][0]["procedures"]["reader_applicability"]
                ["status"] == "skipped" &&
              reader["maps"][0]["texture_layers"]["entries"][0]["replicators"]
                    ["reader_applicability"]["status"] == "skipped" &&
              reader["maps"][0]["texture_layers"]["entries"][0]["semantics"]["reader_path"]
                    ["preset"]["parameter_status"] == "decoded",
          "recognized legacy preset uses its content reader and skips modern parameter packages");
    return checks;
}
