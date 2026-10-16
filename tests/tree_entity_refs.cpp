#include "tree_entity_refs.hpp"
#include <future>
#include <stdexcept>

unsigned tree_entity_reference_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        if (!ok)
            throw std::runtime_error(why);
    };
    const std::set<std::string> live = {"0:0", "13:99", "4294967295:1"};
    const Json bindings = {{"0:0", {7}}, {"13:99", {12}}, {"4294967295:1", {21}}};
    auto parse = [&](const std::string &text) {
        Json refs = Json::array();
        auto meta = append_tree_entity_references(Json::array({"", "", "", text}), "1:10", live,
                                                  bindings, refs);
        return Json{{"meta", std::move(meta)}, {"refs", std::move(refs)}};
    };
    auto r = parse("55834574947,"); // (13 << 32) + 99
    check(r["refs"].size() == 1 && r["refs"][0]["status"] == "resolved" &&
              r["refs"][0]["element_key"] == "13:99" &&
              r["refs"][0]["bindings"] == Json::array({12}),
          "canonical terminated reference reaches its source binding");
    r = parse("-4294967295,");
    check(r["refs"][0]["native_model_id"] == -1 && r["refs"][0]["model_id"] == UINT32_MAX &&
              r["refs"][0]["element_id"] == 1 && r["refs"][0]["element_key"] == "4294967295:1" &&
              r["refs"][0]["status"] == "resolved" && r["refs"][0]["bindings"] == Json::array({21}),
          "signed native model and unsigned storage key preserve the same 32-bit identity");
    r = parse("1");
    check(r["refs"].empty() && r["meta"]["ignored_tail_byte_count"] == 1,
          "unterminated first token is not consumed by the native loop");
    r = parse("55834574947,123");
    check(r["refs"].size() == 1 && r["meta"]["ignored_tail_byte_count"] == 3,
          "unterminated final token is retained as an unconsumed tail");
    r = parse(",,");
    check(r["refs"].size() == 2 && r["refs"][0]["packed_value"] == 0 &&
              r["refs"][0]["conversion_status"] == "no_digits" &&
              r["refs"][0]["status"] == "resolved" && r["refs"][1]["source_token_index"] == 1,
          "empty terminated tokens each read zero instead of disappearing");
    r = parse("55834574947,55834574947,");
    check(r["refs"].size() == 2 && r["refs"][0]["element_key"] == r["refs"][1]["element_key"] &&
              r["meta"]["reference_indices"] == Json::array({0, 1}),
          "source duplicate references and order survive runtime set deduplication");
    for (const auto &text : {" +55834574947suffix,", "55834574947.9,", "55834574947e2,"}) {
        r = parse(text);
        check(r["refs"][0]["element_key"] == "13:99",
              "decimal prefix conversion retains the established reference");
    }
    for (const auto &text : {"junk,", "+,", "-,", "0x100,"}) {
        r = parse(text);
        check(r["refs"][0]["packed_value"] == 0 && r["refs"][0]["element_key"] == "0:0",
              "failed or hexadecimal-looking decimal input follows native integer conversion");
    }
    r = parse("18446744073709551615,-18446744073709551615,");
    check(r["refs"][0]["packed_value"] == std::numeric_limits<std::int64_t>::max() &&
              r["refs"][0]["native_model_id"] == INT32_MAX &&
              r["refs"][0]["element_id"] == UINT32_MAX &&
              r["refs"][0]["conversion_status"] == "saturated",
          "positive overflow saturates rather than becoming an unsigned 64-bit reference");
    check(r["refs"][1]["packed_value"] == std::numeric_limits<std::int64_t>::min() &&
              r["refs"][1]["native_model_id"] == INT32_MIN && r["refs"][1]["element_id"] == 0 &&
              r["refs"][1]["native_entity_id_validity"] == false,
          "negative saturation keeps the known tuple even when its native validity is false");
    r = parse("-1,55834574947,");
    check(r["meta"]["status"] == "partial" && r["refs"][0]["native_model_id"] == -2 &&
              r["refs"][0]["element_id"].is_null() && !r["refs"][0].contains("element_key") &&
              r["refs"][0]["status"] == "indeterminate_native_temporary",
          "minus-one preserves the native unassigned component instead of fabricating zero");
    check(r["refs"][1]["element_key"] == "13:99" && r["refs"][1]["status"] == "resolved",
          "a later known token remains usable after a runtime-dependent token");
    r = parse("-0001tail,");
    check(r["refs"][0]["status"] == "indeterminate_native_temporary",
          "the sentinel branch depends on converted value rather than exact token spelling");
    r = parse(u8"\u3000\uff15\uff15\uff18\uff13\uff14\uff15\uff17\uff14\uff19\uff14\uff17,");
    check(r["refs"][0]["element_key"] == "13:99",
          "UCRT wide whitespace and full-width digits are supported in C locale");
    r = parse(u8"\u0661\u06f2\u0969,");
    check(r["refs"][0]["packed_value"] == 123,
          "mixed confirmed wide digit blocks use their decimal values");
    r = parse(u8"\U0001d7cf,");
    check(r["refs"][0]["packed_value"] == 0 && r["refs"][0]["conversion_status"] == "no_digits",
          "supplementary Unicode digits are not substituted for the UCRT UTF16-unit table");
    const std::string text = std::string("55834574947,\0", 13) + "123,";
    r = parse(text);
    check(r["refs"].size() == 1 && r["meta"]["input_prefix_byte_count"] == 12 &&
              r["meta"]["ignored_suffix_byte_count"] == 5,
          "outer native C-string truncation precedes comma scanning");
    Json refs = Json::array({{{"existing", true}}});
    auto meta = append_tree_entity_references(Json::array({"", "", "", "1,2,"}), "node", {},
                                              Json::object(), refs);
    check(meta["reference_indices"] == Json::array({1, 2}) && refs[0]["existing"] == true &&
              refs[2]["source_byte_offset"] == 2 && refs[2]["source_byte_count"] == 1,
          "source byte ranges and appended graph reference indices remain distinct");
    refs = Json::array();
    check(append_tree_entity_references(Json::array(), "node", {}, Json::object(),
                                        refs)["status"] == "absent" &&
              refs.empty(),
          "missing resource slot is distinct from malformed text");
    check(append_tree_entity_references(Json::array({"", "", "", 1}), "node", {}, Json::object(),
                                        refs)["status"] == "not_evaluated",
          "nontext resource input is not interpreted as empty references");
    check(append_tree_entity_references(nullptr, "node", {}, Json::object(), refs)["status"] ==
              "not_evaluated",
          "nonarray resources do not fabricate an absent native vector");
    const auto expected = parse("55834574947,-4294967295,,");
    auto future =
        std::async(std::launch::async, [&] { return parse("55834574947,-4294967295,,"); });
    check(future.get() == expected,
          "integer parsing and graph reference emission have no mutable locale or shared state");
    auto query = [](const Json &type, const Json &resources) {
        return native_tree_node_queries({{"NodeType", type}, {"ResourceRefers", resources}},
                                        {9, 12, UINT32_MAX});
    };
    for (int type : {3, 7, 35, 36, 38, 39}) {
        auto q = query(type, {"12", "other", "9"});
        check(q["model_self"]["native_model_id"] == 12 && q["model_self"]["source_slot"] == 0 &&
                  q["model_self"]["model_storage_status"] == "present",
              "native model node query selects slot zero for each supported node type");
    }
    auto q = query(9, {"12", "other", "9suffix"});
    check(q["model_self"]["native_model_id"] == 9 && q["model_self"]["source_slot"] == 2,
          "document node model query reads slot two and accepts decimal prefix");
    q = query(17, {"42"});
    check(q["domain_self"]["value"] == 42 && q["model_self"]["native_return_value"] == true &&
              !q["model_self"].contains("model_id"),
          "domain node slot zero is a domain code rather than a model identity");
    q = query(11, {"12"});
    check(q["domain_self"]["value"] == -1 && q["model_self"]["status"] == "not_applicable" &&
              q["model_self"]["native_return_value"] == true,
          "entity node query does not interpret arbitrary resources as model or domain");
    q = query(7, Json::array());
    check(q["model_self"]["status"] == "empty" && q["model_self"]["native_return_value"] == false &&
              !q["model_self"].contains("model_id"),
          "absent slot has an empty model query instead of a fabricated zero identity");
    check(query(17, {""})["domain_self"]["value"] == -1,
          "empty domain slot returns native minus one default");
    check(query(7, {"-1"})["model_self"]["model_id"] == UINT32_MAX,
          "negative model is retained as signed native value and unsigned storage identity");
    check(query(7, {"-2"})["model_self"]["native_return_value"] == true,
          "native model set query does not add an identity validity filter");
    q = query(7, {"2147483648"});
    check(q["model_self"]["native_model_id"] == INT32_MAX &&
              q["model_self"]["conversion_status"] == "saturated",
          "wtoi positive overflow saturates at signed 32 bits rather than wrapping");
    check(query(7, {"-9223372036854775809"})["model_self"]["native_model_id"] == INT32_MIN,
          "wtoi negative overflow saturates at signed 32 bits");
    check(query(7, {"garbage"})["model_self"]["native_model_id"] == 0 &&
              query(7, {"garbage"})["model_self"]["native_return_value"] == true,
          "nonempty unconverted model text inserts zero rather than returning false");
    q = query(7, {std::string("\0ignored", 8)});
    check(q["model_self"]["native_model_id"] == 0 &&
              q["model_self"]["ignored_suffix_byte_count"] == 8,
          "nonempty stored string with initial NUL still invokes native conversion");
    check(query(17, {"\xe3\x80\x80-\xef\xbc\x91\xef\xbc\x92tail"})["domain_self"]["value"] == -12,
          "domain conversion uses the confirmed UCRT wide character tables");
    check(query(nullptr, {"12"})["model_self"]["status"] == "not_evaluated",
          "missing native node type does not fabricate a query branch");
    check(query(7, {12})["model_self"]["status"] == "not_evaluated" &&
              query(17, nullptr)["domain_self"]["status"] == "not_evaluated",
          "malformed source resource types are retained as unevaluated");
    check(query(7, {std::string("\xff", 1)})["model_self"]["status"] == "not_evaluated",
          "invalid UTF8 source prefix is not converted through unchecked decoding");
    return checks;
}
