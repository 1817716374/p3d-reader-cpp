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
    for (const auto *bad : {"{", "[]", R"({"0":3})", "{/*native extension*/}"}) {
        result = resolve_rebuilt_graphics_materials(entries,
                                                    Json::array({attribute(4, 1, narrow(bad))}), c);
        check(result.at("entries")[0].at("status") == "unresolved",
              "unsupported JSON parse or string conversion cannot silently fall back");
    }
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
