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
    for (auto bad : {"-1", "4294967296", "3.5", "1e2", "0x10", "", "+", "NaN", "1junk"}) {
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
    check(material_parameter_semantics(a)["parameters"]["roughness_factor"]["status"] == "invalid",
          "nonfinite source coefficient not treated as valid");

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
    check(material_map_semantics(map)["enabled"]["status"] == "invalid", "signed off overflow");
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
    check(material_map_semantics(map)["uv_scale"]["status"] == "invalid",
          "invalid scale component");
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
    return checks;
}
