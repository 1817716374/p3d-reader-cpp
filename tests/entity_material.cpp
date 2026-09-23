#include "internal.hpp"
#include <p3d/entity_material.hpp>
#include <stdexcept>

unsigned entity_material_tests() {
    using namespace p3d;
    using Query = NativeEntityMaterialIdQuery;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto put = [](Bytes &b, std::size_t offset, std::uint64_t n, unsigned size) {
        for (unsigned i = 0; i < size; ++i)
            b.at(offset + i) = std::uint8_t(n >> (8 * i));
    };
    auto id_link = [&](std::uint64_t id) {
        Bytes b(12);
        put(b, 0, 0x1000e, 4);
        put(b, 4, id, 8);
        return Json{{"header", 0x1007}, {"app", 0x41}, {"payload", rawbytes(b)}};
    };
    auto name_link = [&](const Bytes &name) {
        Bytes b(2);
        put(b, 0, name.size(), 2);
        b.insert(b.end(), name.begin(), name.end());
        b.push_back(0);
        if (b.size() % 2)
            b.push_back(0);
        return Json{{"header", 0x1000 | ((b.size() + 4) / 2 - 1)},
                    {"app", 0x4f5a},
                    {"payload", rawbytes(b)}};
    };
    const auto named = name_link(Bytes{'O', 'a', 'k'});
    auto record = [](const Json &links) { return Json{{"id", 37}, {"links", links}}; };
    const auto source = record(Json::array({id_link(7), named}));
    const auto original = source.dump();
    const GraphicsMaterialResult miss{true, {}}, unknown{}, hit{true, 0};
    NativeEntityMaterialContext context;
    std::vector<std::string> calls;
    auto loaders = [&](GraphicsMaterialResult primary, GraphicsMaterialResult fallback,
                       GraphicsMaterialResult name) {
        calls.clear();
        context.lookup_id = [&, primary, fallback](std::uint64_t id, Query query) {
            check(id == 7, "source ID, not source record identity, reaches the loader");
            const bool first = query == Query::primary_resource_update_true;
            calls.push_back(first ? "id_update_true" : "id_update_false");
            return first ? primary : fallback;
        };
        context.lookup_name = [&, name](const std::u16string &s) {
            check(s == u"Oak", "entity name query uses decoded source name");
            calls.push_back("name_update_true");
            return name;
        };
    };
    // Expected call sequences are taken from the separate primary and outer
    // native wrappers: ID failure skips name, whereas the name branch can reach ID.
    for (const auto primary : {miss, unknown, hit}) {
        for (const auto fallback : {miss, unknown, hit}) {
            loaders(primary, fallback, hit);
            context.entity_model_resource_available = true;
            const auto result = resolve_native_entity_material(source, context);
            const bool second = primary.known && !primary.material_index;
            const auto expected = second ? fallback : primary;
            check(result.material.known == expected.known &&
                      result.material.material_index == expected.material_index,
                  "primary ID miss uses fallback ID, never the available source name");
            check(calls == (second ? std::vector<std::string>{"id_update_true", "id_update_false"}
                                   : std::vector<std::string>{"id_update_true"}),
                  "known miss alone permits the outer ID operation");
            check(result.report.at("name_reference").at("status") == "not_inspected",
                  "unused source name remains unread on primary ID path");
            check(result.report.at("lookups")[0].at("update") == true &&
                      (!second || result.report.at("lookups")[1].at("update") == false),
                  "two ID requests keep distinct update flags");
        }
    }
    for (const auto name : {miss, unknown, hit}) {
        loaders(hit, hit, name);
        context.entity_model_resource_available = false;
        const auto result = resolve_native_entity_material(source, context);
        const bool second = name.known && !name.material_index;
        check(calls == (second ? std::vector<std::string>{"name_update_true", "id_update_false"}
                               : std::vector<std::string>{"name_update_true"}),
              "unavailable model resource selects name, with outer ID only after known miss");
        check(result.material.known == (second ? true : name.known) &&
                  result.material.material_index ==
                      (second ? hit.material_index : name.material_index),
              "name and fallback ID return the loaded material identity");
    }
    loaders(hit, hit, hit);
    context.entity_model_resource_available.reset();
    auto result = resolve_native_entity_material(source, context);
    check(!result.material.known && calls.empty() &&
              result.report.at("unresolved_phase") == "primary_route_selection",
          "unknown resource availability cannot choose ID over name");
    for (const auto &links :
         {Json::array({named}), Json::array({id_link(UINT64_MAX), id_link(7), named})}) {
        calls.clear();
        result = resolve_native_entity_material(record(links), context);
        check(
            result.material.material_index == 0 &&
                calls == std::vector<std::string>{"name_update_true"},
            "absent or first sentinel ID selects name, even when resource availability is unknown");
    }
    calls.clear();
    result = resolve_native_entity_material(record(Json::array()), context);
    check(result.material.known && !result.material.material_index && calls.empty(),
          "complete empty linkage collection proves no entity material without loaders");
    result = resolve_native_entity_material(Json::object(), context);
    check(!result.material.known, "missing linkage collection is unknown, not an empty record");

    auto truncated_id = id_link(7);
    truncated_id["payload"] = rawbytes(Bytes{0x0e, 0, 1, 0, 7, 0});
    const auto duplicate = record(Json::array({truncated_id, id_link(7), named}));
    context.entity_model_resource_available = true;
    result = resolve_native_entity_material(duplicate, context);
    check(!result.material.known && result.report.at("id_reference").at("linkage_index") == 0,
          "invalid first selected ID shadows a later valid ID");
    context.entity_model_resource_available = false;
    result = resolve_native_entity_material(duplicate, context);
    check(result.material.material_index == 0 &&
              result.report.at("id_reference").at("status") == "not_inspected",
          "successful name branch does not inspect malformed unused ID");

    auto bad_name = named;
    bad_name["payload"] = rawbytes(Bytes{3, 0, 'O', 'a', 'k', 'X'});
    const auto name_duplicates = record(Json::array({id_link(7), bad_name, named}));
    calls.clear();
    result = resolve_native_entity_material(name_duplicates, context);
    check(!result.material.known && calls.empty() &&
              result.report.at("name_reference").at("linkage_index") == 1,
          "unterminated first name does not fall through to a duplicate or fallback ID");
    context.entity_model_resource_available = true;
    result = resolve_native_entity_material(name_duplicates, context);
    check(result.material.material_index == 0, "successful primary ID ignores malformed name");
    auto stale_annotations = source;
    stale_annotations["links"][0]["decoded"] = {{"material_id", 19},
                                                {"reader_selection", "shadowed"}};
    result = resolve_native_entity_material(stale_annotations, context);
    check(result.material.material_index == 0 &&
              result.report.at("id_reference").at("material_id") == 7,
          "binding reads preserved source bytes, not edited derived annotations");

    auto nonuser = id_link(88);
    nonuser["header"] = 7;
    auto other_key = id_link(99);
    auto other_payload = bytesof(other_key["payload"]);
    other_payload[0] = 15;
    other_key["payload"] = rawbytes(other_payload);
    result = resolve_native_entity_material(record(Json::array({nonuser, other_key, id_link(7)})),
                                            context);
    check(result.report.at("id_reference").at("linkage_index") == 2 &&
              result.material.material_index == 0,
          "only user linkage app 0x41 with exact key 0x1000e supplies ID");
    for (const auto id : {std::uint64_t(0), std::uint64_t(0xfedcba9876543210), UINT64_MAX - 1}) {
        context.lookup_id = [&, id](std::uint64_t actual, Query query) {
            check(actual == id && query == Query::primary_resource_update_true,
                  "full unsigned source ID retained");
            return hit;
        };
        result = resolve_native_entity_material(record(Json::array({id_link(id)})), context);
        check(result.material.material_index == 0,
              "zero and high-bit IDs are valid, only all-ones is sentinel");
    }

    // Go through the actual binary record parser, including linkage offsets and
    // duplicate selection, rather than testing only hand-assembled JSON.
    Bytes binary(36);
    put(binary, 4, 1, 2);
    put(binary, 12, 16, 4);
    put(binary, 20, 37, 8);
    for (const auto &link : Json::array({id_link(7), id_link(8), named})) {
        const auto start = binary.size();
        binary.resize(start + 4);
        put(binary, start, link.at("header").get<unsigned>(), 2);
        put(binary, start + 2, link.at("app").get<unsigned>(), 2);
        const auto bytes = bytesof(link.at("payload"));
        binary.insert(binary.end(), bytes.begin(), bytes.end());
    }
    put(binary, 8, (binary.size() - 4) / 2, 4);
    const auto parsed = parse_native(binary);
    loaders(hit, miss, miss);
    result = resolve_native_entity_material(parsed[0], context);
    check(result.material.material_index == 0 &&
              result.report.at("id_reference").at("linkage_offset") == 36 &&
              result.report.at("source_record").at("id") == 37,
          "binary record selection connects to material query with exact provenance");

    context.entity_model_resource_available = false;
    auto misleading_length = named;
    misleading_length["payload"] = rawbytes(Bytes{0xff, 0xff, 'O', 'a', 'k', 0, 0xff, 0xff});
    result = resolve_native_entity_material(record(Json::array({misleading_length})), context);
    check(result.material.known && !result.material.material_index &&
              result.report.at("name_reference").at("decoded").at("declared_length_matches") ==
                  false,
          "NUL terminator, not declared length or invalid suffix, bounds the native name");
    calls.clear();
    auto nonascii = name_link(Bytes{0xd6, 0xd0});
    result = resolve_native_entity_material(record(Json::array({nonascii, id_link(7)})), context);
    check(!result.material.known && calls.empty(), "unknown ANSI mapping blocks lower ID fallback");
    context.ansi_decoder = [&](const Bytes &b) {
        check(b == Bytes({0xd6, 0xd0}), "decoder receives exactly source bytes before NUL");
        return std::string("\xe4\xb8\xad\xf0\x9f\x8c\xb2");
    };
    context.lookup_name = [&](const std::u16string &name) {
        check(name == std::u16string({0x4e2d, 0xd83c, 0xdf32}),
              "ANSI conversion retains UTF-16 surrogate pairs");
        return hit;
    };
    result = resolve_native_entity_material(record(Json::array({nonascii})), context);
    check(result.material.material_index == 0, "caller-decoded non-ASCII name can bind");
    context.ansi_decoder = [](const Bytes &) { return std::string("\xff"); };
    result = resolve_native_entity_material(record(Json::array({nonascii})), context);
    check(!result.material.known, "invalid UTF-8 conversion cannot become a native miss");
    context.ansi_decoder = {};
    unsigned empty_calls = 0;
    context.lookup_name = [&](const std::u16string &name) {
        check(name.empty(), "empty native name is preserved");
        ++empty_calls;
        return miss;
    };
    result = resolve_native_entity_material(record(Json::array({name_link({})})), context);
    check(result.material.known && !result.material.material_index && empty_calls == 1,
          "empty name still performs outer project preparation through the name callback");

    loaders(miss, hit, hit);
    context.entity_model_resource_available = true;
    context.lookup_id = {};
    result = resolve_native_entity_material(source, context);
    check(!result.material.known && result.report.at("lookups").size() == 1 && calls.empty(),
          "missing primary loader cannot become a fallback miss");
    context.lookup_id = [](std::uint64_t, Query) -> GraphicsMaterialResult {
        throw std::runtime_error("load error");
    };
    result = resolve_native_entity_material(source, context);
    check(!result.material.known && result.report.at("lookups")[0].at("reason") == "load error",
          "loader exception stays unresolved with its call trace");
    context.lookup_id = [](std::uint64_t, Query) { return GraphicsMaterialResult{false, 7}; };
    result = resolve_native_entity_material(source, context);
    check(!result.material.known && !result.material.material_index &&
              result.report.at("lookups")[0].at("reason") == "unknown_material_has_identity",
          "inconsistent unknown callback result cannot publish an identity");

    // The result feeds the existing outer entity/part material selection without
    // inventing a serialized geometry-to-rebuilt-part correspondence.
    GraphicsMaterialContext graphics;
    graphics.native_entity_material = result.material;
    unsigned advanced_calls = 0;
    graphics.lookup_name = [&](const std::u16string &, GraphicsMaterialNameQuery) {
        ++advanced_calls;
        return hit;
    };
    const auto entries = Json::array({Json{{"inline_material", nullptr}}});
    const auto attrs = Json::array(
        {Json{{"group", 4}, {"key", 10001}, {"index", 0}, {"payload", rawbytes(Bytes{'X', 0})}}});
    auto built = resolve_rebuilt_graphics_materials(entries, attrs, graphics);
    check(built.at("entries")[0].at("status") == "unresolved" && advanced_calls == 0,
          "unresolved entity lookup blocks advanced entity fallback in the graphics builder");
    graphics.native_entity_material =
        resolve_native_entity_material(record(Json::array()), {}).material;
    built = resolve_rebuilt_graphics_materials(entries, attrs, graphics);
    check(built.at("entries")[0].at("material_index") == 0 && advanced_calls == 1,
          "confirmed null entity lookup enables advanced entity name in existing builder");
    check(source.dump() == original, "entity source record is never mutated");
    return checks;
}
