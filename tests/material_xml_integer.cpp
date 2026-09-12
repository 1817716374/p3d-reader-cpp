#include "internal.hpp"

unsigned material_xml_integer_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    struct Case {
        const char *text;
        std::int64_t signed_value;
        std::uint64_t unsigned_value;
    };
    const Case cases[] = {
        {"9tail", 9, 9},
        {"-9tail", -9, 4294967287u},
        {"0x10", 0, 0},
        {"3.5", 3, 3},
        {"1e2", 1, 1},
        {"0010", 10, 10},
        {"+12.5", 12, 12},
        {"\v\f 7x", 7, 7},
        {"2147483648", INT32_MIN, 2147483648u},
        {"4294967295", -1, UINT32_MAX},
        {"4294967296", 0, 0},
        {"4294967297", 1, 1},
        {"-2147483649", INT32_MAX, INT32_MAX},
        {"-4294967295", 1, 1},
        {"9223372036854775807", -1, UINT32_MAX},
        {"9223372036854775808", -1, 0},
        {"18446744073709551615", -1, UINT32_MAX},
        {"18446744073709551616", -1, UINT32_MAX},
        {"-9223372036854775808", 0, 0},
        {"-9223372036854775809", 0, UINT32_MAX},
        {"-18446744073709551615", 0, 1},
        {"-18446744073709551616", 0, UINT32_MAX},
    };
    for (const auto &c : cases) {
        auto s = material_xml_integer({{"v", c.text}}, "v", true);
        auto u = material_xml_integer({{"v", c.text}}, "v", false);
        check(s["status"] == "decoded" && s["value"] == c.signed_value &&
                  u["status"] == "decoded" && u["value"] == c.unsigned_value,
              "XML integer conversion matches native signed and unsigned decimal scanning");
        check(s["source_text"] == c.text && u["source_text"] == c.text,
              "integer source spelling survives normalization");
    }
    for (const auto text : {"", "bad", "+", "-", "  - 3", "\t", "--1"}) {
        const auto r = material_xml_integer({{"v", text}}, "v", true);
        check(r["status"] == "invalid" && r["value"].is_null() &&
                  r["conversion"]["status"] == "no_integer_assignment",
              "failed integer scan is distinct from a successful zero");
    }
    check(material_xml_integer(Json::object(), "v", true)["status"] == "missing" &&
              material_xml_integer({{"v", 7}}, "v", true)["status"] == "unresolved",
          "absent XML attribute and invalid non-text caller input remain distinct");
    auto n = material_xml_integer({{"v", " +12suffix"}}, "v", true);
    check(n["conversion"]["consumed_bytes"] == 4 && n["conversion"]["ignored_suffix"] == "suffix",
          "native prefix boundary is reported in UTF-8 bytes");
    n = material_xml_integer({{"v", std::string("12\0more", 7)}}, "v", false);
    check(n["value"] == 12 && n["source_text"].get<std::string>().size() == 7,
          "C-string numeric end does not discard the separately retained source text");
    auto node = [](const std::string &tag, const Json &a, const Json &children = Json::array()) {
        return Json{{"tag", tag}, {"attributes", a}, {"children", children}, {"text", ""}};
    };
    auto parse = [&](const Json &root, const Json &attrs) {
        return material_settings(node("Material", root, Json::array({node("Map", attrs)})));
    };
    auto s = parse({{"material_version", "9tail"}}, {{"Type", "1tail"}, {"layer", "6suffix"}});
    check(s["reader_profile"]["mode"] == 9 &&
              s["maps"][0]["reader_path"]["single_provider_type_before_preset"]["reader_value"] ==
                  4,
          "numeric prefixes select the actual version, Map type and normalized provider");
    s = parse({{"material_version", "bad"}}, {{"Type", "1"}, {"layer_state", "6"}, {"layer", "3"}});
    check(s["reader_profile"]["mode"] == 0 &&
              s["reader_profile"]["mode_source"] == "failed_read_uses_zero" &&
              s["reader_profile"]["version"]["status"] == "invalid" &&
              s["maps"][0]["reader_path"]["single_provider_type_before_preset"]["reader_value"] ==
                  4,
          "failed material version selects legacy field names while retaining the failed read");
    s = parse({{"material_version", "9"}}, {{"Type", "1"}, {"layer", "bad"}});
    check(s["maps"][0]["reader_path"]["single_provider_type_before_preset"]["reader_value"] == 1,
          "provider integer read failure leaves the constructor type unchanged");
    s = parse({{"material_version", "9"}}, {{"Type", "bad"}});
    check(s["maps"][0]["reader_path"]["branch"] == "skipped" &&
              s["map_bindings"]["entries"][0]["selection"] == "skipped_type_read_failure" &&
              s["map_bindings"]["status"] == "resolved",
          "failed Map type read is a confirmed skipped entry");
    s = parse({{"material_version", "9"}},
              {{"Type", "1"}, {"map_link", "bad"}, {"pattern_off", "bad"}});
    check(s["map_bindings"]["entries"][0]["status"] == "local" &&
              s["map_bindings"]["entries"][0]["link_normalization"] == "failed_read_uses_zero" &&
              s["maps"][0]["numeric_reader"]["parameters"]["pattern_off"]["write_status"] ==
                  "not_written",
          "failed link read uses zero but failed ordinary integer field leaves its prior value");
    s = parse({{"material_version", "9"}}, {{"Type", "1"}, {"pattern_off", "0junk"}});
    check(s["maps"][0]["numeric_reader"]["parameters"]["pattern_off"]["reader_value"] == true,
          "integer prefix participates in native zero-is-enabled normalization");
    return checks;
}
