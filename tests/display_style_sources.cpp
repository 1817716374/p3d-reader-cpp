#include "internal.hpp"
using namespace p3d;
namespace {
Json attr(unsigned key, unsigned index, const Json &decoded) {
    return {{"group", 0}, {"key", key}, {"index", index}, {"offset", 100 + index},
            {"decoded", decoded}};
}
Json marker(std::uint32_t registration) {
    return attr(22900, 1, {{"encoding", "native_handler_reference"},
                           {"registration_key", registration}});
}
Json style(unsigned index, const char *name) {
    return attr(22903, index, {{"encoding", "compressed_utf16_xml"},
                              {"tree", {{"tag", "ShowStyle"},
                                        {"attributes", {{"Name", name}, {"Unknown", "keep"}}},
                                        {"children", Json::array()}}}});
}
Json graphic(const Json &attrs, unsigned id = 42) {
    return {{"stream", StreamPath{"FILE", "ATTR", "M7"}}, {"id", id},
            {"offset", 90}, {"attributes", attrs}};
}
Json native(std::int32_t maximum) {
    Bytes base(44, 0);
    base[4] = 92;
    const auto word = std::uint32_t(maximum);
    for (unsigned i = 0; i < 4; ++i)
        base[40 + i] = std::uint8_t(word >> (8 * i));
    return {{"stream", StreamPath{"FILE", "SYS", "M7"}}, {"id", 42},
            {"element_type", 92}, {"data", rawbytes(base)}};
}
} // namespace
unsigned display_style_sources_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) { ++checks; require(value, message); };
    const auto ref = decode_attribute(0, 20080, Bytes{9, 0, 0, 0}, 0);
    check(ref.at("display_style_index") == 9 && ref.at("native_lookup_eligible") == true &&
              ref.at("runtime_resolution") == "not_evaluated", "display style signed index prefix");
    for (const Bytes bytes : {Bytes{0xff, 0xff, 0xff, 0xff}, Bytes{0, 0, 0, 0x80}}) {
        const auto negative = decode_attribute(0, 20080, bytes, 0);
        check(negative.at("display_style_index").get<std::int32_t>() < 0 &&
                  negative.at("native_lookup_eligible") == false, "negative style index is not a huge ID");
    }
    for (std::size_t n : {0u, 1u, 2u, 3u, 5u, 8u}) {
        bool rejected = false;
        try { decode_attribute(0, 20080, Bytes(n), 0); }
        catch (const std::exception &) { rejected = true; }
        check(rejected, "native style reference requires exactly four bytes");
    }
    check(decode_attribute(1, 20080, Bytes{9, 0, 0, 0}, 0).at("encoding") == "opaque" &&
              decode_attribute(0, 20080, Bytes{9, 0, 0, 0}, 1).at("encoding") == "opaque",
          "display style reference dispatch respects group and index");
    const Json index = {{"P3D-SSYS", "SYS"}, {"P3D-SSYSA", "ATTR"}};
    Json graphics = Json::array({graphic(Json::array({marker(0x006f0000), style(9, "Smooth")})),
                                  graphic(Json::array({attr(20080, 0, ref)}), 71)});
    const Json headers = Json::array({native(10)});
    auto out = build_display_style_sources(index, headers, graphics);
    check(out["tables"][0]["maximum_entry_index"] == 10 &&
              out["tables"][0]["entries"][0]["within_declared_range"] == true &&
              out["references"][0]["candidates"][0]["name"] == "Smooth",
          "source attribute index joins file style table without a model-ID collision");
    check(out["status"] == "partial" && out["runtime_resolution"] == "not_evaluated" &&
              out["references"][0]["status"] == "source_candidates" &&
              out["tables"][0]["entries"][0]["xml_tree"]["attributes"]["Unknown"] == "keep",
          "XML semantics and source candidates do not claim final runtime binding");
    auto small = build_display_style_sources(index, Json::array({native(8)}), graphics);
    check(small["references"][0]["candidates"].empty() &&
              small["tables"][0]["entries"].size() == 1,
          "out-of-range declaration is retained but not offered as an indexed source");
    auto missing = build_display_style_sources(index, Json::array(), graphics);
    check(missing["references"][0]["candidates"][0]["header_status"] == "missing",
          "missing native header remains visible on a tentative source candidate");
    out = build_display_style_sources(index, Json::array({native(-1)}), graphics);
    check(out["references"][0]["candidates"].empty(),
          "negative maximum declares an empty list rather than billions of slots");
    out = build_display_style_sources(index, Json::array({native(2147483647)}), graphics);
    check(out["tables"][0]["entries"].size() == 1 &&
              out["references"][0]["candidates"].size() == 1,
          "sparse source catalog does not allocate declared empty slots");
    auto duplicate = build_display_style_sources(index, Json::array({native(10), native(10)}), graphics);
    check(duplicate["tables"][0]["header_status"] == "ambiguous" &&
              duplicate["tables"][0]["native_record_candidates"].size() == 2,
          "duplicate table identities are not silently collapsed");
    auto invalid_header = native(10);
    invalid_header["data"] = rawbytes(Bytes(40));
    out = build_display_style_sources(index, Json::array({invalid_header}), graphics);
    check(out["tables"][0]["header_status"] == "invalid", "truncated maximum index is not guessed");
    graphics.push_back(graphic(Json::array({marker(0x597e0000), style(9, "Lite")})));
    out = build_display_style_sources(index, headers, graphics);
    check(out["references"][0]["candidates"].size() == 2 &&
              out["references"][0]["candidates"][1]["table_kind"] == "lite",
          "common and lite candidates remain distinct pending registry and conversion");
    graphics[2]["attributes"].push_back(style(9, "Duplicate"));
    out = build_display_style_sources(index, headers, graphics);
    check(out["references"][0]["candidates"].size() == 3,
          "duplicate attribute indices retain occurrence identities");
    graphics[0]["stream"] = StreamPath{"FILE", "MODEL", "ATTR", "M7"};
    graphics[2]["stream"] = StreamPath{"FILE", "MODEL", "ATTR", "M7"};
    out = build_display_style_sources(index, headers, graphics);
    check(out["references"][0]["candidates"].empty() && out["tables"].size() == 2,
          "model-local marker does not establish the file style namespace");
    auto common = graphic(Json::array({marker(0x006f0000), style(9, "Original")}));
    auto &tree = common["attributes"][1]["decoded"]["tree"];
    tree["children"] = Json::array({
        {{"tag", "Flags"}, {"attributes", {{"DisplayVisibleEdges", "true"},
            {"VisibleEdgeColor", "true"}, {"VisibleEdgeWeight", "true"},
            {"Transparency", "true"}, {"Material", "true"},
            {"DisplayHiddenEdges", "true"}, {"DisplayShadows", "true"},
            {"LineStyle", "true"}, {"FutureFlag", "retain"}}}},
        {{"tag", "Overrides"}, {"attributes", {{"DisplayMode", "3"},
            {"VisibleEdgeColor", "65535"}, {"VisibleEdgeWeight", "23"},
            {"Transparency", "0.75"}, {"Material", "18446744073709551615"},
            {"HiddenEdgeWeight", "17"}, {"LineStyle", "8"}}}},
        {{"tag", "Extension"}, {"attributes", {{"value", "unchanged"}}}}
    });
    const auto original = common;
    out = build_display_style_sources(index, headers, Json::array({common}));
    const auto &entry = out["tables"][0]["entries"][0];
    const auto &projection = entry["lite_copy_projection"];
    const auto &derived = projection["xml_tree"];
    const auto &f = derived["children"][0]["attributes"];
    const auto &o = derived["children"][1]["attributes"];
    check(projection["status"] == "conditional" &&
              projection["runtime_application"] == "not_evaluated" &&
              common == original && entry["xml_tree"] == tree,
          "conditional native copy projection preserves source XML and does not apply runtime selection");
    check(o["DisplayMode"] == "6" && f["DisplayVisibleEdges"] == "false" &&
              f["VisibleEdgeColor"] == "false" && f["VisibleEdgeWeight"] == "false" &&
              f["Transparency"] == "false" && f["Material"] == "false" &&
              o["VisibleEdgeColor"] == "0" && o["VisibleEdgeWeight"] == "0" && o["Transparency"] == "0",
          "native Lite copy clears the confirmed enable bits and corresponding stored edge/transparency values");
    check(o["Material"] == "18446744073709551615" && o["HiddenEdgeWeight"] == "17" &&
              o["LineStyle"] == "8" && f["LineStyle"] == "true" &&
              f["DisplayHiddenEdges"] == "true" && f["DisplayShadows"] == "true" &&
              f["FutureFlag"] == "retain" && derived["children"][2] == tree["children"][2],
          "disabling material must not erase its ID or other unaffected style fields");
    common["attributes"][0] = marker(0x597e0000);
    out = build_display_style_sources(index, headers, Json::array({common}));
    check(!out["tables"][0]["entries"][0].contains("lite_copy_projection"),
          "an existing Lite style must not be converted again");
    common["attributes"][0] = marker(0x006f0000);
    tree["children"].push_back(tree["children"][0]);
    out = build_display_style_sources(index, headers, Json::array({common}));
    check(out["tables"][0]["entries"][0]["lite_copy_projection"]["status"] == "unresolved",
          "duplicate XML sections do not receive a guessed projection");
    tree["children"] = Json::array();
    out = build_display_style_sources(index, headers, Json::array({common}));
    check(out["tables"][0]["entries"][0]["lite_copy_projection"]["status"] == "unresolved",
          "missing XML sections do not receive fabricated native defaults");
    return checks;
}
