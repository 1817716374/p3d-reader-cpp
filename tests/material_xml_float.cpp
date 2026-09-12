#include "internal.hpp"

unsigned material_xml_float_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto read = [](const std::string &s) { return material_xml_float({{"v", s}}, "v"); };
    for (const auto &entry : std::vector<std::pair<std::string, std::string>>{
             {"  +12.5tail", "4029000000000000"},
             {"0XAp1", "4034000000000000"},
             {"0x1.8", "3ff8000000000000"},
             {"0x1p-2tail", "3fd0000000000000"},
             {"0x1e+2", "403e000000000000"},
             {"1.#INF", "3ff0000000000000"},
             {"-0", "8000000000000000"},
             {"-1e-99999", "8000000000000000"},
             {"1e99999", "7ff0000000000000"},
             {"-inf", "fff0000000000000"},
             {"NaN", "7fffffffffffffff"},
             {"-nan", "ffffffffffffffff"},
             {"nan(snan)", "7ff0000000000001"},
             {"-nan(ind)", "fff8000000000000"},
             {"NAN(IND)", "fff8000000000000"},
             {"nan(0x123)", "7fffffffffffffff"},
             {"2.4703282292062327e-324", "0000000000000000"},
             {"2.4703282292062328e-324", "0000000000000001"},
             {"4.9406564584124654e-324", "0000000000000001"}}) {
        const auto r = read(entry.first);
        check(r["status"] == "decoded" && r["conversion"]["destination_bits_hex"] == entry.second,
              ("native floating IEEE representation: " + entry.first + " => " +
               r["conversion"].dump())
                  .c_str());
    }
    for (const auto text : {"", "bad", "+", ".", "1e", "1e+", "1e-x", "0x", "0x1p", "0x.p1", "infi",
                            "infix", "nan(a", "nan(!)", "nan(a b)"}) {
        const auto r = read(text);
        check(r["status"] == "invalid" && material_xml_numeric_failed(r),
              "incomplete or malformed scanner input does not fall back to a shorter number");
    }
    auto r = read("\v\f +1.25tail");
    check(r["value"] == 1.25 && r["conversion"]["consumed_bytes"] == 8 &&
              r["conversion"]["ignored_suffix"] == "tail",
          "whitespace sign and consumed prefix retain native source positions");
    r = read(std::string("2\0e9", 4));
    check(r["value"] == 2 && r["conversion"]["consumed_bytes"] == 1 &&
              r["conversion"]["ignored_suffix"] == std::string("\0e9", 3),
          "native NUL termination does not discard the preserved XML source suffix");
    r = read("-nan");
    check(r["value"]["floating_point"] == "nan" && r["value"]["negative"] == true &&
              Json::parse(r.dump()) == r,
          "nonfinite values survive ordinary JSON serialization without becoming null");
    r = material_xml_float({{"v", 4}}, "v");
    check(r["status"] == "unresolved" && !material_xml_numeric_failed(r),
          "nontext input is distinct from a proved failed native attribute assignment");
    check(material_xml_numeric_failed(material_xml_float(Json::object(), "v")),
          "missing source attributes do not write a caller destination");
    auto node = [](const char *tag, Json a, Json c = Json::array()) {
        return Json{{"tag", tag}, {"attributes", a}, {"children", c}, {"text", ""}};
    };
    Json map = {{"Type", "2"}, {"origin_uv_pro_matrix_on", "0"}};
    for (const auto axis : {"xa", "ya", "za"})
        for (const auto component : {"x", "y", "z"})
            map[std::string("origin_uv_pro_matrix_") + axis + "." + component] = "0";
    map["origin_uv_pro_matrix_xa.x"] = "-nan";
    auto s = material_settings(node("Material",
                                    {{"material_version", "3"}, {"displacement_distance", "nan"}},
                                    Json::array({node("Map", map)})));
    const auto projection = s["maps"][0]["initial_projection_state"];
    check(projection["status"] == "resolved" &&
              projection["matrix"]["all_components_finite"] == false &&
              projection["matrix"]["storage_values"].size() == 9 &&
              projection["matrix"]["storage_values"][0]["ieee754_hex"] == "ffffffffffffffff" &&
              Json::parse(projection.dump()) == projection,
          "complete axis containing native NaN has a known stored value even with its switch off");
    check(s["version_conversion"]["map_topology"]["active_object_ids"] == Json::array({1}) &&
              s["version_conversion"]["map_topology"]["objects"][1]["native_type_key"] == 15,
          "unordered displacement distance follows the native nonzero conversion branch");
    map["pattern_proj_offset.x"] = "1e+";
    map["pattern_proj_offset.y"] = "2";
    map["pattern_proj_offset.z"] = "3";
    map["origin_uv_pro_matrix_xa.x"] = "1e+";
    s = material_settings(
        node("Material", {{"material_version", "9"}}, Json::array({node("Map", map)})));
    const auto failed = s["maps"][0]["initial_projection_state"];
    check(failed["parameters"]["pattern_proj_offset"]["value"] == Json::array({0., 0., 0.}) &&
              failed["matrix"]["axes"]["xa"]["state"] == "indeterminate_native_temporary",
          "the same failed vector conversion leaves frame constructor state but indeterminate "
          "matrix temporary");
    return checks;
}
