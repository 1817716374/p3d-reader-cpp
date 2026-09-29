#include "internal.hpp"
#include "display_style_xml.hpp"
#include "display_style_xml_oracle.hpp"
#include "display_style_usages_oracle.hpp"
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
    Json input = {{"tag", "ShowStyle"}, {"attributes", {{"Name", ""}}},
                  {"children", Json::array({
                      {{"tag", "Flags"}, {"attributes", Json::object()}},
                      {{"tag", "Overrides"}, {"attributes", {{"DisplayMode", "6"}}}}
                  })}};
    const auto minimal = input;
    auto decoded = decode_display_style_xml(input);
    check(decoded["status"] == "partial" && decoded["runtime_resolution"] == "not_evaluated" &&
              decoded["packed_flags_at_48"] == 6 && decoded["packed_flags_at_50"] == 0x582,
          "minimal source style preserves native true defaults without claiming complete import");
    check(decoded["fields"]["Overrides.HiddenEdgeWeight"]["value"] == 65535 &&
              decoded["fields"]["Overrides.Material"]["value"] == 0 &&
              decoded["fields"]["Flags.VisibleEdgeStyle"]["default_applied"] == true,
          "missing override fields preserve documented constructor and reader defaults");
    auto status = [](unsigned code) { return code == 0 ? "parsed" : code == 4100 ? "missing" : "invalid"; };
    for (const auto &sample : style_xml_oracle) {
        input = minimal;
        auto &fa = input["children"][0]["attributes"];
        auto &oa = input["children"][1]["attributes"];
        if (sample.source) {
            fa["Material"] = fa["VisibleEdgeStyle"] = sample.source;
            oa["VisibleEdgeColor"] = oa["Material"] = sample.source;
            input["attributes"]["ShowGroundFromBelow"] = sample.source;
            oa["Transparency"] = oa["HLineTransparencyThreshold"] = sample.source;
            input["attributes"]["GroundPlaneHeight"] = sample.source;
        }
        decoded = decode_display_style_xml(input);
        const auto &fields = decoded.at("fields");
        check(fields["Flags.Material"]["value"] == sample.boolean &&
                  fields["Flags.Material"]["read_status"] == status(sample.bool_status) &&
                  fields["Flags.Material"]["default_applied"] == (sample.bool_status != 0),
              "boolean false-default behavior matches the actual official XML DLL");
        check(fields["Flags.VisibleEdgeStyle"]["value"] == (sample.bool_status ? true : sample.boolean) &&
                  fields["ShowGroundFromBelow"]["value"] == sample.boolean,
              "invalid booleans apply per-field defaults, not numeric truthiness");
        check(fields["Overrides.VisibleEdgeColor"]["value"] == sample.u32 &&
                  fields["Overrides.VisibleEdgeColor"]["read_status"] == status(sample.u32_status) &&
                  fields["Overrides.Material"]["value"] == sample.u64 &&
                  fields["Overrides.Material"]["read_status"] == status(sample.u64_status),
              "unsigned numeric prefixes, negative values and saturation match the official DLL");
        const auto &floating = fields["Overrides.Transparency"];
        check(floating["read_status"] == status(sample.double_status) &&
                  floating["ieee754_hex"] == sample.double_bits &&
                  floating["default_applied"] == (sample.double_status != 0) &&
                  fields["GroundPlaneHeight"] == floating,
              "style float parsing and zero fallback match the official getter bit for bit");
        const auto &threshold = fields["Overrides.HLineTransparencyThreshold"];
        check(threshold["ieee754_hex"] == (sample.double_status ? "3fd3333333333333" : sample.double_bits) &&
                  threshold["read_status"] == status(sample.double_status) &&
                  threshold["default_applied"] == (sample.double_status != 0),
              "threshold fallback applies on failed reads, never on valid nonfinite or zero values");
        check(Json::parse(floating.dump()) == floating && Json::parse(threshold.dump()) == threshold,
              "style float evidence including nonfinite payloads survives JSON export");
    }
    decoded = decode_display_style_xml(minimal);
    check(decoded["fields"].size() == 47 &&
              decoded["fields"]["GroundPlaneColor.R"]["value"] == 0 &&
              decoded["fields"]["GroundPlaneColor.G"]["value"] == 0 &&
              decoded["fields"]["GroundPlaneColor.B"]["value"] == 0 &&
              decoded["fields"]["GroundPlaneTransparency"]["value"] == 0 &&
              decoded["fields"]["Overrides.HLineTransparencyThreshold"]["value"] == 0.3,
          "absent ground-plane doubles overwrite constructor defaults but threshold restores 0.3");
    input = minimal;
    input["attributes"].update({{"GroundPlaneColor.R", "0.125"}, {"GroundPlaneColor.G", "0.25"},
                                 {"GroundPlaneColor.B", "0.5"}, {"GroundPlaneTransparency", "0.75"}});
    decoded = decode_display_style_xml(input);
    check(decoded["fields"]["GroundPlaneColor.R"]["value"] == 0.125 &&
              decoded["fields"]["GroundPlaneColor.G"]["value"] == 0.25 &&
              decoded["fields"]["GroundPlaneColor.B"]["value"] == 0.5 &&
              decoded["fields"]["GroundPlaneTransparency"]["value"] == 0.75,
          "dotted RGB attributes remain independent floating fields, without byte-color conversion");
    input["attributes"]["GroundPlaneHeight"] = 1.0;
    decoded = decode_display_style_xml(input);
    check(decoded["fields"]["GroundPlaneHeight"]["read_status"] == "unresolved" &&
              decoded["fields"]["GroundPlaneHeight"]["value"].is_null() &&
              decoded["fields"]["GroundPlaneHeight"]["default_applied"] == false,
          "unsupported non-text JSON input is not reported as an observed native failure or default");
    input = minimal;
    input["children"][1]["attributes"]["DisplayMode"] = "-1";
    input["children"][1]["attributes"]["HiddenEdgeLineStyle"] = "10suffix";
    decoded = decode_display_style_xml(input);
    check(decoded["packed_flags_at_48"] == 0x23f &&
              decoded["fields"]["Overrides.DisplayMode"]["value"] == 4294967295u &&
              decoded["fields"]["Overrides.DisplayMode"]["stored_value"] == 63 &&
              decoded["fields"]["Overrides.HiddenEdgeLineStyle"]["stored_value"] == 2,
          "bitfield truncation retains both parsed scalar and stored subfield values");
    // Expected masks come from the DLL read/write instructions, independently
    // exercising each switch to catch accidental shifts and word crossover.
    const std::pair<const char *, unsigned> first_word[] = {
        {"DisplayVisibleEdges", 0x40}, {"DisplayHiddenEdges", 0x80}, {"DisplayShadows", 0x800},
        {"LegacyDrawOrder", 0x1000}, {"BackgroundColor", 0x2000}, {"ApplyEdgeStyleToLines", 0x4000},
        {"IgnoreGeometryMaps", 0x8000}, {"IgnoreImageMaps", 0x10000}, {"HideInPickers", 0x20000},
        {"InvisibleToCamera", 0x40000}, {"DisplayGroundPlane", 0x80000}};
    for (const auto &[name, mask] : first_word) {
        input = minimal;
        input["children"][0]["attributes"][name] = "TrUe";
        decoded = decode_display_style_xml(input);
        check(decoded["packed_flags_at_48"] == (6 | mask) && decoded["packed_flags_at_50"] == 0x582,
              "main style switch changes exactly the confirmed bit");
    }
    const char *second_word[] = {"VisibleEdgeColor", "VisibleEdgeWeight", "Transparency", "FillColor",
        "LineStyle", "LineWeight", "Material", "VisibleEdgeStyle", "HiddenEdgeLineStyle", "HiddenEdgeWeight",
        "HLineTransparency", "HLineMaterialColors", "SmoothIgnoreLights", "UseDisplayHandler"};
    for (unsigned bit = 0; bit < 14; ++bit) {
        input = minimal;
        for (const auto *name : second_word) input["children"][0]["attributes"][name] = "false";
        input["children"][0]["attributes"][second_word[bit]] = "true";
        decoded = decode_display_style_xml(input);
        check(decoded["packed_flags_at_48"] == 6 && decoded["packed_flags_at_50"] == (1u << bit),
              "override switch changes exactly the confirmed bit, including true-default switches");
    }
    for (const auto *bad_mode : {"", "true", "+", "--1"}) {
        input = minimal;
        input["children"][1]["attributes"]["DisplayMode"] = bad_mode;
        decoded = decode_display_style_xml(input);
        check(decoded["status"] == "rejected" && !decoded.contains("fields"),
              "required display mode must parse before publishing an imported state");
    }
    input = minimal;
    input["children"][1]["attributes"].erase("DisplayMode");
    check(decode_display_style_xml(input)["status"] == "rejected", "missing required mode rejects import");
    input = minimal;
    input["attributes"].erase("Name");
    check(decode_display_style_xml(input)["status"] == "rejected", "missing Name differs from present empty Name");
    input = minimal;
    input["children"].erase(0);
    check(decode_display_style_xml(input)["status"] == "rejected", "absent Flags is not an empty Flags node");
    input = minimal;
    input["children"].push_back(input["children"][0]);
    check(decode_display_style_xml(input)["status"] == "unresolved", "ambiguous XML layout remains unresolved");
    tree = minimal;
    const auto before_import = common;
    out = build_display_style_sources(index, headers, Json::array({common}));
    check(out["tables"][0]["entries"][0]["native_xml_import"]["packed_flags_at_50"] == 0x582 &&
              common == before_import && out["tables"][0]["entries"][0]["xml_tree"] == minimal,
          "source catalog exposes typed import evidence while preserving the original XML");
    for (const auto &sample : style_usages_oracle) {
        const auto usage = decode_display_style_usages({{"Usages", sample.source}});
        std::vector<unsigned> bits;
        for (const auto &range : usage.at("ranges"))
            for (unsigned i = range.at("first"); i <= range.at("last").get<unsigned>(); ++i)
                bits.push_back(i);
        check(usage["status"] == "decoded" && usage["bit_length"] == sample.size &&
                  bits == sample.bits && usage["set_bit_count"] == bits.size(),
              "usage range parsing matches actual native bitset including delimiter and trim behavior");
    }
    auto usage = decode_display_style_usages({{"Usages", "9-7,1-3,3-8,2,12"}});
    check(usage["ranges"] == Json::array({{{"first", 1}, {"last", 9}}, {{"first", 12}, {"last", 12}}}) &&
              usage["tokens"].size() == 5 && usage["set_bit_count"] == 10,
          "overlapping and reversed ranges normalize set membership while retaining ordered source tokens");
    usage = decode_display_style_usages({{"Usages", "0-1000000000"}});
    check(usage["status"] == "decoded" && usage["ranges"].size() == 1 &&
              usage["set_bit_count"] == 1000000001 && usage["native_execution"] == "not_evaluated",
          "large bounded ranges have exact compact semantics without allocating or executing native bit loops");
    for (const auto *text : {"0-4294967295", "4294967295", "1--1", "18446744073709551616"}) {
        usage = decode_display_style_usages({{"Usages", text}});
        check(usage["status"] == "unresolved" && usage["reason"] == "native_uint32_range_loop_wrap" &&
                  !usage.contains("ranges") && !usage.contains("set_bit_count"),
              "native nonterminating uint32 endpoint does not masquerade as a completed bitset");
    }
    usage = decode_display_style_usages({{"Usages", "2,4294967295,4"}});
    check(usage["completed_prefix_ranges"] == Json::array({{{"first", 2}, {"last", 2}}}) &&
              usage["ignored_suffix"] == "4294967295,4",
          "failure retains only prior completed tokens and does not continue to later source values");
    usage = decode_display_style_usages({{"Usages", "1,,3"}});
    check(usage["ignored_suffix"] == ",3" && usage["stop_reason"] == "non_digit_prefix",
          "native stops before attempting tokenizer on a non-digit prefix");
    usage = decode_display_style_usages({{"Usages", std::string("1\0,3", 4)}});
    check(usage["set_bit_count"] == 1 && usage["ignored_suffix"] == std::string("\0,3", 3),
          "native text stops at NUL while preserving the complete source suffix");
    usage = decode_display_style_usages(Json::object());
    check(usage["read_status"] == "missing" && usage["bit_length"] == 0 && usage["ranges"].empty(),
          "absent Usages leaves the constructor's empty default-false bitset");
    check(decode_display_style_usages({{"Usages", 3}})["status"] == "unresolved",
          "usage input requires actual XML text");
    input = minimal;
    input["attributes"].update({{"Name", u8"光滑"}, {"EnvironmentName", u8"夜景"}, {"Usages", "0,2-4"}});
    const auto input_copy = input;
    decoded = decode_display_style_xml(input);
    check(decoded["fields"]["Name"]["value"] == u8"光滑" &&
              decoded["fields"]["EnvironmentName"]["value"] == u8"夜景" &&
              decoded["usages"]["set_bit_count"] == 4 && input == input_copy,
          "style importer joins names and the initial usage set without rewriting the source tree");
    check(decode_display_style_xml(minimal)["fields"]["EnvironmentName"]["read_status"] == "missing" &&
              decode_display_style_xml(minimal)["fields"]["EnvironmentName"]["value"] == "",
          "environment name defaults to an empty string rather than a guessed external resource");
    return checks;
}
