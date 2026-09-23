#include "internal.hpp"
#include <p3d/graphics_material.hpp>
#include <future>

unsigned graphics_material_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto attribute = [](unsigned group, unsigned index, const Bytes &bytes) {
        return Json{
            {"group", group}, {"key", 10001}, {"index", index}, {"payload", rawbytes(bytes)}};
    };
    auto narrow = [](const std::string &s) { return Bytes(s.begin(), s.end()); };
    auto wide = [](const std::u16string &s) {
        Bytes b;
        for (auto c : s) {
            b.push_back(c & 255);
            b.push_back(c >> 8);
        }
        return b;
    };
    const Json entries = {{{"geometry_type", 0}, {"inline_material", {{"id", 7}}}},
                          {{"geometry_type", 6}, {"inline_material", {{"id", 8}}}},
                          {{"geometry_type", 3}, {"inline_material", nullptr}}};
    Json attributes = Json::array(
        {attribute(4, 1, narrow(R"({"0":"A","1":"Missing"})")), attribute(2, 1, wide(u"Legacy"))});
    GraphicsMaterialContext c;
    c.native_entity_material = {true, 2};
    std::vector<std::pair<std::u16string, GraphicsMaterialNameQuery>> calls;
    unsigned inline_calls = 0;
    c.lookup_name = [&](const std::u16string &name, GraphicsMaterialNameQuery q) {
        calls.emplace_back(name, q);
        if (name == u"A")
            return GraphicsMaterialResult{true, 0};
        if (name == u"Legacy")
            return GraphicsMaterialResult{true, 1};
        if (name == u"Entity")
            return GraphicsMaterialResult{true, 3};
        return GraphicsMaterialResult{true, {}};
    };
    c.load_inline_material = [&](const Json &m) {
        ++inline_calls;
        return GraphicsMaterialResult{true, m.at("id").get<std::size_t>()};
    };
    const auto original_entries = entries.dump(), original_attributes = attributes.dump();
    auto result = resolve_rebuilt_graphics_materials(entries, attributes, c);
    check(result.at("status") == "resolved", "rebuilt material sequence resolved");
    for (unsigned i = 0; i < 3; ++i)
        check(result.at("entries")[i].at("material_index") == i,
              "part overrides precede element material and index zero remains valid");
    check(result.at("entries")[1].at("part_index") == 1,
          "a non-emitting entry consumes a material part index");
    check(calls.size() == 3 && calls[0].second == GraphicsMaterialNameQuery::advanced_part &&
              calls[2].second == GraphicsMaterialNameQuery::legacy_part,
          "advanced and legacy searches retain distinct native scope and update mode");
    check(inline_calls == 0, "higher priority hit does not convert inline materials");

    c.native_entity_material = {true, {}};
    result = resolve_rebuilt_graphics_materials(entries, Json::array(), c);
    check(result.at("entries")[0].at("material_index") == 7 &&
              result.at("entries")[1].at("material_index") == 8 &&
              result.at("entries")[2].at("status") == "unassigned",
          "known misses reach per-entry materials and confirmed null remains unassigned");
    auto entity_attrs = Json::array({attribute(4, 0, wide(u"Entity"))});
    result = resolve_rebuilt_graphics_materials(entries, entity_attrs, c);
    check(result.at("entity_material").at("source") == "advanced_entity_name" &&
              result.at("entries")[0].at("material_index") == 3 &&
              calls.back().second == GraphicsMaterialNameQuery::advanced_entity,
          "advanced entity name supplies missing native entity material");
    auto previous_calls = calls.size();
    c.native_entity_material = {true, 20};
    result = resolve_rebuilt_graphics_materials(entries, entity_attrs, c);
    check(result.at("entries")[0].at("material_index") == 20 && calls.size() == previous_calls,
          "native entity result wins without reading lower-priority advanced entity name");

    c.native_entity_material = {};
    result = resolve_rebuilt_graphics_materials(entries, attributes, c);
    check(result.at("entries")[0].at("status") == "resolved" &&
              result.at("entries")[1].at("status") == "resolved" &&
              result.at("entries")[2].at("status") == "unresolved",
          "part hits remain usable even with unresolved element material");
    auto unknown = c;
    unknown.lookup_name = {};
    unknown.native_entity_material = {true, 20};
    result = resolve_rebuilt_graphics_materials(entries, attributes, unknown);
    check(result.at("entries")[0].at("status") == "unresolved" &&
              !result.at("entries")[0].contains("material_index") &&
              result.at("entries")[0].at("lookups").size() == 1,
          "unavailable higher priority name lookup cannot become a fallback miss");
    unknown.load_inline_material = {};
    unknown.native_entity_material = {true, {}};
    result = resolve_rebuilt_graphics_materials(entries, Json::array(), unknown);
    check(result.at("entries")[0].at("status") == "unresolved" &&
              result.at("entries")[2].at("status") == "unassigned",
          "missing inline loader differs from an absent inline material");

    c.native_entity_material = {true, 20};
    auto source = narrow(R"({"0":"A","01":"Legacy","-0":"Legacy"})");
    source.push_back(0);
    source.push_back(0xff);
    result = resolve_rebuilt_graphics_materials(entries, Json::array({attribute(4, 1, source)}), c);
    check(result.at("entries")[0].at("material_index") == 0 &&
              result.at("entries")[1].at("material_index") == 20,
          "native JSON input ends at first NUL; aliases are not normalized to part indices");
    result = resolve_rebuilt_graphics_materials(
        entries, Json::array({attribute(4, 1, narrow(R"({"0":"A\u0000ignored"})"))}), c);
    check(result.at("entries")[0].at("material_index") == 0,
          "JSON name has its own NUL-terminated UTF-8 lookup boundary");
    result = resolve_rebuilt_graphics_materials(
        entries,
        Json::array(
            {attribute(2, 0, wide(std::u16string{u'L', u'e', u'g', u'a', u'c', u'y', 0, 0xd800}))}),
        c);
    check(result.at("entries")[0].at("material_index") == 1,
          "legacy UTF-16 lookup ignores malformed suffix after NUL");
    result =
        resolve_rebuilt_graphics_materials(entries, Json::array({attribute(2, 0, Bytes{'A'})}), c);
    check(result.at("entries")[0].at("material_index") == 0,
          "native appended zeros supply odd UTF-16 name high byte");

    auto duplicate = Json::array(
        {attribute(4, 1, narrow(R"({"0":"Missing"})")), attribute(4, 1, narrow(R"({"0":"A"})"))});
    result = resolve_rebuilt_graphics_materials(entries, duplicate, c);
    check(result.at("entries")[0].at("material_index") == 0 &&
              result.at("part_name_table").at("attribute_ordinal") == 1,
          "small native attribute collection selects last equal key");
    for (unsigned i = 0; i < 4; ++i)
        duplicate.push_back(attribute(8, i, {}));
    result = resolve_rebuilt_graphics_materials(entries, duplicate, c);
    check(result.at("entries")[0].at("material_index") == 20 &&
              result.at("part_name_table").at("attribute_ordinal") == 0,
          "larger native collection uses lower bound rather than arbitrary last-wins");
    result = resolve_rebuilt_graphics_materials(
        entries, Json::array({attribute(4, 2, narrow(R"({"A":{"0":"0"}})"))}), c);
    check(result.at("entries")[0].at("material_index") == 20,
          "reverse index cannot fabricate a forward material assignment");
    result = resolve_rebuilt_graphics_materials(entries,
                                                Json::array({attribute(4, 1, narrow("null"))}), c);
    check(result.at("entries")[0].at("material_index") == 20, "null JSON has no member assignment");
    for (const auto *bad : {"{", "[]", R"({"0":3.0})", R"({"0":{}})", R"({"0":[]})"}) {
        result = resolve_rebuilt_graphics_materials(entries,
                                                    Json::array({attribute(4, 1, narrow(bad))}), c);
        check(result.at("entries")[0].at("status") == "unresolved",
              "unsupported JSON parse or string conversion cannot silently fall back");
    }
    // Check the actual lookup strings, not just a successful fallback. In
    // particular, null converts to an empty name, rejected by the native loader.
    const std::vector<std::pair<std::string, std::u16string>> scalar_cases = {
        {"null", u""},
        {"true", u"true"},
        {"false", u"false"},
        {"0", u"0"},
        {"-0", u"0"},
        {"-1", u"-1"},
        {"2147483648", u"2147483648"},
        {"9007199254740993", u"9007199254740993"},
        {"9223372036854775807", u"9223372036854775807"},
        {"9223372036854775808", u"9223372036854775808"},
        {"18446744073709551615", u"18446744073709551615"},
        {"-9223372036854775808", u"-9223372036854775808"}};
    for (const auto &test : scalar_cases) {
        auto scalar_context = c;
        unsigned name_calls = 0;
        scalar_context.lookup_name = [&](const auto &name, auto query) {
            ++name_calls;
            check(name == test.second && query == GraphicsMaterialNameQuery::advanced_part,
                  "native scalar name conversion preserves exact text and lookup scope");
            return GraphicsMaterialResult{true, name.empty() ? std::optional<std::size_t>()
                                                             : std::optional<std::size_t>(77)};
        };
        result = resolve_rebuilt_graphics_materials(
            entries, Json::array({attribute(4, 1, narrow("{\"0\":" + test.first + "}"))}),
            scalar_context);
        const bool empty_name = test.second.empty();
        check(name_calls == 1 &&
                  result.at("entries")[0].at("material_index") == (empty_name ? 20 : 77) &&
                  result.at("entries")[1].at("material_index") == 20,
              "scalar assignment overrides its own part without affecting other entries");
        const auto source = narrow("{\"0\":" + test.first + "}");
        const auto view = decode_material_assignment(1, source);
        const auto &item = view.at("entries")[0];
        check(item.at("status") == "recognized" && item.at("material_name").is_string() &&
                  item.at("source_value") == Json::parse(test.first) &&
                  bytesof(view.at("source_bytes")) == source,
              "source assignment view preserves scalar value and exposes its converted name");
    }
    for (const auto *unhandled : {"18446744073709551616", "-9223372036854775809", "1e2"}) {
        const auto before = calls.size();
        result = resolve_rebuilt_graphics_materials(
            entries,
            Json::array({attribute(4, 1, narrow(std::string("{\"0\":") + unhandled + "}"))}), c);
        check(calls.size() == before && result.at("entries")[0].at("status") == "unresolved" &&
                  result.at("entries")[1].at("material_index") == 20,
              "floating or overflowing integer syntax cannot become an invented material name");
        const auto view = decode_material_assignment(
            1, narrow(std::string("{\"0\":") + unhandled + ",\"1\":\"A\"}"));
        check(view.at("entries")[0].contains("name_conversion_error") &&
                  view.at("entries")[1].at("material_name") == "A",
              "unsupported numeric name leaves other source assignments usable");
    }
    for (const auto *commented : {"/*before*/{\"0\"/*key*/:/*value*/\"A\"}/*after*/",
                                  "//before\r\n{\"0\":\"A\"//value\n}//after",
                                  "{\"0\":\"Missing\",/*later wins*/\"0\":\"A\"}"}) {
        auto bytes = narrow(commented);
        result =
            resolve_rebuilt_graphics_materials(entries, Json::array({attribute(4, 1, bytes)}), c);
        check(result.at("entries")[0].at("material_index") == 0 &&
                  result.at("part_name_table").at("status") == "parsed",
              "native line and block comments do not obscure the material assignment");
        const auto decoded = decode_material_assignment(1, bytes);
        check(decoded.at("json_status") == "parsed" && decoded.at("value").at("0") == "A" &&
                  bytesof(decoded.at("source_bytes")) == bytes,
              "attribute view supports comments while preserving all original source bytes");
    }
    auto literal = c;
    std::u16string literal_name;
    literal.lookup_name = [&](const auto &name, auto) {
        literal_name = name;
        return GraphicsMaterialResult{true, 77};
    };
    result = resolve_rebuilt_graphics_materials(
        entries, Json::array({attribute(4, 1, narrow(R"({"0":"a/*b*/c//d"})"))}), literal);
    check(literal_name == u"a/*b*/c//d" && result.at("entries")[0].at("material_index") == 77,
          "comment markers within quoted names remain literal");
    auto null_missing =
        Json::array({attribute(4, 1, narrow(R"({"0":null})")), attribute(2, 0, wide(u"Legacy"))});
    result = resolve_rebuilt_graphics_materials(entries, null_missing, c);
    check(result.at("entries")[0].at("source") == "legacy_part_name" &&
              result.at("entries")[0].at("lookups")[0].at("lookup_name_utf16_code_units").empty(),
          "a confirmed empty-name miss reaches legacy part material");
    auto null_unknown = c;
    null_unknown.lookup_name = {};
    result = resolve_rebuilt_graphics_materials(entries, null_missing, null_unknown);
    check(result.at("entries")[0].at("status") == "unresolved" &&
              result.at("entries")[0].at("lookups").size() == 1 &&
              result.at("entries")[0].at("source") == "advanced_part_name",
          "unknown empty-name project preparation does not silently become a completed lookup");
    for (const auto *empty_value : {"null", "\"\"", "\"\\u0000ignored\""}) {
        unsigned calls_for_empty = 0;
        auto empty_context = c;
        empty_context.lookup_name = [&](const auto &name, auto) {
            ++calls_for_empty;
            check(name.empty(), "empty name reaches complete native query callback");
            return GraphicsMaterialResult{true, {}};
        };
        result = resolve_rebuilt_graphics_materials(
            entries,
            Json::array({attribute(4, 1, narrow(std::string("{\"0\":") + empty_value + "}"))}),
            empty_context);
        check(calls_for_empty == 1 && result.at("entries")[0].at("material_index") == 20 &&
                  result.at("entries")[0].at("lookups")[0].at("status") == "not_found",
              "empty advanced name miss falls back after project preparation callback");
        empty_context.lookup_name = [](const auto &, auto) {
            return GraphicsMaterialResult{true, 77};
        };
        result = resolve_rebuilt_graphics_materials(
            entries,
            Json::array({attribute(4, 1, narrow(std::string("{\"0\":") + empty_value + "}"))}),
            empty_context);
        check(result.at("entries")[0].at("status") == "unresolved" &&
                  !result.at("entries")[0].contains("material_index"),
              "contradictory empty-name material hit is not accepted as a native result");
    }
    auto empty_layers = null_unknown;
    empty_layers.native_entity_material = {true, {}};
    unsigned empty_layer_calls = 0;
    empty_layers.lookup_name = [&](const auto &name, auto) {
        ++empty_layer_calls;
        check(name.empty(), "empty legacy and entity names retain project query boundary");
        return GraphicsMaterialResult{true, {}};
    };
    result = resolve_rebuilt_graphics_materials(
        entries, Json::array({attribute(4, 0, {}), attribute(2, 0, wide(u""))}), empty_layers);
    check(result.at("entries")[0].at("material_index") == 7 &&
              result.at("entity_material").at("status") == "not_found" && empty_layer_calls == 2,
          "empty legacy and advanced entity names fall through after confirmed native misses");
    result = resolve_rebuilt_graphics_materials(
        entries, Json::array({attribute(4, 1, narrow("{\"0\":\"A\",/*unterminated"))}), c);
    check(result.at("part_name_table").at("status") == "unresolved" &&
              result.at("entries")[0].at("status") == "unresolved",
          "unterminated comment cannot yield a partially accepted name table");
    auto throwing = c;
    throwing.lookup_name = [](const auto &, auto) -> GraphicsMaterialResult {
        throw std::runtime_error("loader failed");
    };
    result = resolve_rebuilt_graphics_materials(entries, attributes, throwing);
    check(result.at("entries")[0].at("status") == "unresolved" &&
              result.at("entries")[2].at("material_index") == 20,
          "lookup exception affects dependent entries, not unrelated parts");
    throwing.lookup_name = [](const auto &, auto) { return GraphicsMaterialResult{false, 99}; };
    result = resolve_rebuilt_graphics_materials(entries, attributes, throwing);
    check(result.at("entries")[0].at("status") == "unresolved" &&
              !result.at("entries")[0].contains("material_index"),
          "contradictory callback identity does not leak into a resolved result");
    result = resolve_rebuilt_graphics_materials(Json::array(), Json::array(), {});
    check(result.at("status") == "resolved" && result.at("entries").empty(),
          "empty entry sequence needs no per-entry material decision");
    result = resolve_rebuilt_graphics_materials(entries, Json::object(), c);
    check(result.at("status") == "unresolved" && result.at("entries").empty(),
          "wrong attribute container does not imply no assignments");
    check(entries.dump() == original_entries && attributes.dump() == original_attributes,
          "resolution preserves original geometry and raw source assignments");

    auto parallel = c;
    parallel.lookup_name = [](const std::u16string &name, auto) {
        return GraphicsMaterialResult{true,
                                      name == u"A" ? std::optional<std::size_t>(0) : std::nullopt};
    };
    parallel.load_inline_material = {};
    const auto expected = resolve_rebuilt_graphics_materials(entries, attributes, parallel);
    std::vector<std::future<Json>> tasks;
    for (unsigned i = 0; i < 4; ++i)
        tasks.push_back(std::async(std::launch::async, [&] {
            return resolve_rebuilt_graphics_materials(entries, attributes, parallel);
        }));
    for (auto &task : tasks)
        check(task.get() == expected, "read-only material resolution is independently concurrent");
    return checks;
}
