#include "internal.hpp"
#include <p3d/material_assignment_table.hpp>
#include <future>

unsigned material_assignment_table_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        ++checks;
        require(value, why);
    };
    NativeAssignmentTextContext text;
    text.uppercase = [](char16_t c) -> std::optional<char16_t> {
        if (c >= u'a' && c <= u'z')
            return c - 32;
        return c;
    };
    text.compare = [&](const std::u16string &a, const std::u16string &b) -> std::optional<int> {
        auto x = a, y = b;
        for (auto &c : x)
            c = *text.uppercase(c);
        for (auto &c : y)
            c = *text.uppercase(c);
        return x < y ? -1 : x > y ? 1 : 0;
    };
    auto read = [&](const std::string &body) {
        return decode_native_material_assignment_table(
            "<table><assignments>" + body + "</assignments></table>", text);
    };
    const std::string body =
        "<a id='1' materialName='old'><layer cMsk0='15'>Wall<rgb>01,02,03</rgb></layer></a>"
        "<a id='2' materialName='new'><layer cMsk0='6'>wall<rgb>01,02,03</rgb></layer></a>"
        "<a id='2' materialName='NEW'><layer cMsk0='16'>WALL<rgb>aa,bb,cc</rgb></layer></a>";
    auto table = read(body);
    check(table.at("status") == "resolved" && table.at("source_entries").size() == 3 &&
              table.at("entries").size() == 2,
          "native insertion merges same material and retains source rows");
    check(table.at("entries")[0]["color_mask_words"][0] == 9 &&
              table.at("entries")[0]["rgb_keys"].empty(),
          "later different material subtracts index bits and true-color keys");
    check(table.at("entries")[1]["color_mask_words"][0] == 22 &&
              table.at("entries")[1]["rgb_keys"].size() == 2 &&
              table.at("entries")[1]["layer_name"] == "wall" &&
              table.at("entries")[1]["material_name"] == "new",
          "same material unions colors without replacing original name and order");
    check(table.at("source_entries")[0]["color_mask_words"][0] == 15,
          "normalization never changes saved source masks");
    NativeAssignmentQuery query;
    query.layer_name = u"Wall";
    query.color = 1;
    query.text = text;
    auto result = lookup_native_material_assignment(table, query);
    check(result.at("material_id") == 2 && result.at("pass") == "pattern",
          "exact layer with rejected color continues to pattern pass");
    query.color = 0;
    check(lookup_native_material_assignment(table, query).at("material_id") == 1,
          "remaining old color stays assigned");
    table = read("<a id='1'><l cMsk0='1'>*</l></a><a id='2'><l cMsk0='1'>Wall</l></a>");
    check(lookup_native_material_assignment(table, query).at("material_id") == 2,
          "all exact rules precede earlier wildcard rules");
    query.layer_name = u"Other";
    check(lookup_native_material_assignment(table, query).at("material_id") == 1,
          "trailing star matches any suffix");
    auto match = [&](const std::string &pattern, std::u16string name) {
        auto t = read("<a id='7'><l cMsk0='1'>" + pattern + "</l></a>");
        query.layer_name = std::move(name);
        query.color = 0;
        return lookup_native_material_assignment(t, query);
    };
    check(match("Wall", u"wall-extra").at("status") == "matched",
          "pattern comparison is a prefix match without final name check");
    check(match("*ab", u"aab").at("status") == "not_found",
          "native star does not backtrack to a later candidate");
    check(match("**ab*", u"xxABmore").at("status") == "matched",
          "consecutive stars and source uppercase conversion");
    check(match("?", u"x").at("status") == "not_found", "question mark is literal");
    check(match("", u"anything").at("status") == "matched", "empty pattern matches any name");
    check(match("*a", u"").at("status") == "unresolved",
          "unsafe native read after empty query is not emulated");
    table =
        read("<a id='8'><l cMsk0='1'>x<RGB>11,22,33</RGB><rgb>111,122,133 trailing</rgb></l></a>");
    query.layer_name = u"x";
    query.color = 0x100;
    check(lookup_native_material_assignment(table, query).at("status") == "unresolved",
          "extended color requires its model table");
    query.extended_colors = std::vector<std::array<std::uint8_t, 3>>{{0x11, 0x22, 0x33}};
    check(lookup_native_material_assignment(table, query).at("status") == "matched",
          "extended color is a one-based palette reference");
    query.color = 0xf00001a5;
    check(lookup_native_material_assignment(table, query).at("status") == "matched",
          "extended index discards top nibble and low byte");
    query.color = 0x200;
    check(lookup_native_material_assignment(table, query).at("status") == "not_found",
          "extended color out of range returns no match");
    query.extended_colors.reset();
    for (auto c : {0xffffff00u, 0xfffffffeu, 0xffffffffu, 0x10000000u}) {
        query.color = c;
        check(lookup_native_material_assignment(table, query).at("status") == "not_found",
              "color sentinels and zero masked index reject");
    }
    check(table.at("entries")[0]["rgb_keys"].size() == 1,
          "RGB components use low bytes and duplicate keys collapse natively");
    table = read(
        "<a id='0'><l cMsk0='1'>skip</l></a><a id='18446744073709551615'><l cMsk0='1'>skip</l></a>"
        "<a id='-2junk'><l cMsk7='2147483648'>last</l></a>");
    check(table.at("source_entries").size() == 1 &&
              table.at("entries")[0]["material_id"] == UINT64_MAX - 1,
          "material ID excludes zero and all ones but retains signed scanf wrapping");
    query.layer_name = u"last";
    query.color = 255;
    check(lookup_native_material_assignment(table, query).at("status") == "matched",
          "eight mask words include bit 255");
    table = read("<a materialName='name'><l cMsk0='bad'><![CDATA[ignored]]><!-- split "
                 "-->actual<rgb>1,2</rgb></l></a>");
    check(table.at("status") == "unresolved" &&
              table.at("source_entries")[0]["layer_name"] == "actual" &&
              table.at("source_entries")[0]["color_mask_words"][0] == 0,
          "first text node differs from CDATA and malformed RGB cannot fabricate stack values");
    table = read("<a id='1'><l cMsk0='1'>x</l></a><a id='2'><l cMsk0='1'>x</l><l>empty</l></a>");
    check(table.at("entries").size() == 2 && table.at("entries")[0]["material_id"] == 2 &&
              table.at("entries")[1]["layer_name"] == "empty",
          "insertion removes old empty rules before appending a new empty row");
    table = read("<a id='1'><l>empty</l><l cMsk0='1'>x</l></a>");
    check(table.at("entries").size() == 1, "later insertion removes a previously empty rule");
    auto unknown =
        decode_native_material_assignment_table("<t><assignments>" + body + "</assignments></t>");
    check(unknown.at("status") == "unresolved" && unknown.at("source_entries").size() == 3,
          "missing source locale does not guess normalization or lose source rules");
    check(decode_native_material_assignment_table("<t xmlns='x'/>").at("status") == "unresolved",
          "namespace XPath behavior is not guessed");
    check(decode_native_material_assignment_table("<!DOCTYPE t><t/>").at("status") == "unresolved",
          "DTD requires separate native XML semantics");
    check(decode_native_material_assignment_table("<t>").at("status") == "unresolved",
          "invalid XML remains unresolved");
    auto xml = std::string("<t><assignments><a id='9'><l cMsk0='1'>x</l></a></assignments></t>");
    Bytes payload(8, 0);
    payload[0] = 1;
    for (auto c : xml) {
        payload.push_back(c);
        payload.push_back(0);
    }
    payload.push_back(0);
    payload.push_back(0);
    const auto n = std::uint32_t(payload.size() - 8);
    for (unsigned i = 0; i < 4; ++i)
        payload[4 + i] = (n >> (8 * i)) & 255;
    auto decoded = decode_attribute(0, 20015, payload, 0);
    check(decoded.at("material_assignment_table").at("entries")[0]["material_id"] == 9,
          "stored attribute feeds real assignment parser");
    const auto original = read(body);
    query.layer_name = u"Wall";
    query.color = 2;
    const auto expected = lookup_native_material_assignment(original, query);
    std::vector<std::future<Json>> futures;
    for (unsigned i = 0; i < 4; ++i)
        futures.push_back(std::async(std::launch::async, [&] {
            return lookup_native_material_assignment(original, query);
        }));
    for (auto &f : futures)
        check(f.get() == expected, "parallel lookups have no shared mutable parser state");
    return checks;
}
