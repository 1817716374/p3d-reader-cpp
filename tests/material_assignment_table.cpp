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

    for (auto bad : {"<a/><b/>", "outside<a/>", "<a/>outside"})
        check(decode_native_material_assignment_table(bad).at("status") == "unresolved",
              "multiple roots and outer text cannot select the first XML fragment");
    auto attribute = [&](unsigned index, const std::string &label, const std::string &rows) {
        const auto s = "<t><name>" + label + "</name><assignments>" + rows + "</assignments></t>";
        Bytes raw;
        for (unsigned char c : s) {
            raw.push_back(c);
            raw.push_back(0);
        }
        raw.insert(raw.end(), {0, 0, 0xff}); // Invalid suffix lies AFTER native terminator.
        Bytes b(8, 0);
        b[0] = 1;
        for (unsigned i = 0; i < 4; ++i)
            b[4 + i] = (raw.size() >> (8 * i)) & 255;
        b.insert(b.end(), raw.begin(), raw.end());
        return Json{{"group", 0},
                    {"key", 20015},
                    {"index", index},
                    {"payload", rawbytes(b)},
                    {"decoded", {{"material_assignment_table", {{"name", "forged"}}}}}};
    };
    const std::string one = "<a id='9' materialName='Paint'><l cMsk0='1'>x</l></a>";
    Json attributes = Json::array(
        {attribute(8, "Late", one), attribute(2, "First", one), attribute(3, "First", one)});
    const auto attribute_snapshot = attributes.dump();
    auto selected = select_native_material_assignment_table(attributes, u"", text);
    check(selected.at("status") == "selected" && selected.at("source_ordinal") == 1 &&
              selected.at("attribute_index") == 2,
          "empty table name selects first native index, not first source occurrence");
    check(bytesof(selected.at("ignored_suffix")) == Bytes{0xff} &&
              selected.at("table").at("name") == "First",
          "table selector uses payload prefix and preserves invalid suffix after NUL");
    auto raw = bytesof(attributes[1]["payload"]);
    decoded = decode_attribute(0, 20015, raw, 2);
    check(decoded.at("material_assignment_table").at("name") == "First" &&
              bytesof(decoded.at("ignored_suffix")) == Bytes{0xff},
          "automatic attribute decoding uses same first UTF16 NUL boundary");
    selected = select_native_material_assignment_table(attributes, u"lATE", text);
    check(selected.at("source_ordinal") == 0 && selected.at("candidates").size() == 3,
          "named table lookup visits duplicate indices and applies source locale");
    check(select_native_material_assignment_table(attributes, u"Missing", text).at("status") ==
              "not_found",
          "complete named table miss");
    check(select_native_material_assignment_table(attributes, u"Late").at("status") == "unresolved",
          "unknown earlier comparison blocks later exact match");
    std::u16string nul_name = u"First";
    nul_name.append({u'\0', u'X'});
    check(select_native_material_assignment_table(attributes, nul_name).at("source_ordinal") == 1,
          "requested table name ends at native NUL");
    auto duplicates = Json::array({attribute(2, "First", one), attribute(2, "First", one)});
    check(select_native_material_assignment_table(duplicates, u"First").at("source_ordinal") == 0,
          "iterator uses first sorted duplicate, not small exact-key lookup's last duplicate");
    duplicates[0]["payload"] = rawbytes(Bytes{0});
    check(select_native_material_assignment_table(duplicates, u"").at("status") == "unresolved",
          "unsupported preceding table cannot silently choose later table");
    check(attributes.dump() == attribute_snapshot,
          "selection does not sort or modify source attributes");
    raw.resize(raw.size() - 3); // Remove terminator and suffix, update stored length.
    for (unsigned i = 0; i < 4; ++i)
        raw[4 + i] = ((raw.size() - 8) >> (8 * i)) & 255;
    check(decode_attribute(0, 20015, raw).at("material_assignment_table").at("status") ==
              "unresolved",
          "missing terminator does not read beyond attribute boundary");

    const Json current = {{"kind", "current_resource_context"}};
    Json catalog = {
        {"status", "resolved"},
        {"current_context", current},
        {"entries",
         Json::array(
             {{{"id", 9}, {"name", "Paint"}, {"resource", {{"primary_context", current}}}},
              {{"id", 10}, {"name", "PAINT"}, {"resource", {{"primary_context", current}}}}})}};
    NativeAssignmentMaterialContext loader;
    std::vector<std::size_t> id_loads, name_loads;
    loader.id.load = [&](std::size_t i) {
        id_loads.push_back(i);
        return GraphicsMaterialResult{true, 100 + i};
    };
    loader.compare_name = text.compare;
    loader.load_name = [&](std::size_t i) {
        name_loads.push_back(i);
        return GraphicsMaterialResult{true, 200 + i};
    };
    query.layer_name = u"x";
    query.color = 0;
    table = read(one);
    result = resolve_native_assignment_material(table, query, catalog, loader);
    check(result.at("status") == "loaded" && result.at("material_index") == 100 &&
              id_loads == std::vector<std::size_t>{0} && name_loads.empty(),
          "successful ID load stops before name candidates");
    loader.id.load = [&](std::size_t) { return GraphicsMaterialResult{true, {}}; };
    result = resolve_native_assignment_material(table, query, catalog, loader);
    check(result.at("status") == "load_failed" && result.at("native_diagnostic_code") == 4 &&
              name_loads.empty(),
          "ID provider failure must not fall back to name");
    loader.id.load = [&](std::size_t) { return GraphicsMaterialResult{false, {}}; };
    check(resolve_native_assignment_material(table, query, catalog, loader).at("status") ==
                  "unresolved" &&
              name_loads.empty(),
          "unknown ID loading blocks name fallback");
    table = read("<a id='99' materialName='Paint'><l cMsk0='1'>x</l></a>");
    result = resolve_native_assignment_material(table, query, catalog, loader);
    check(result.at("material_index") == 200 && name_loads == std::vector<std::size_t>({0, 1}) &&
              result.at("loads").size() == 2,
          "known ID miss loads all same-name candidates then selects first success");
    name_loads.clear();
    table = read("<a materialName='Paint'><l cMsk0='1'>x</l></a>");
    result = resolve_native_assignment_material(table, query, catalog, loader);
    check(result.at("status") == "loaded" && result.at("loads").size() == 1 &&
              result.at("loads")[0]["query"] == "name",
          "sentinel ID goes directly to name route");
    loader.load_name = [&](std::size_t i) {
        return GraphicsMaterialResult{true, i ? std::optional<std::size_t>{201} : std::nullopt};
    };
    check(resolve_native_assignment_material(table, query, catalog, loader).at("material_index") ==
              201,
          "failed first name candidate allows later successful name candidate");
    loader.load_name = [&](std::size_t i) {
        return i ? GraphicsMaterialResult{false, {}} : GraphicsMaterialResult{true, 200};
    };
    check(resolve_native_assignment_material(table, query, catalog, loader).at("status") ==
              "unresolved",
          "unknown later name load cannot be ignored after first success");
    loader.load_name = [&](std::size_t) { return GraphicsMaterialResult{true, {}}; };
    check(resolve_native_assignment_material(table, query, catalog, loader).at("status") ==
              "load_failed",
          "all name candidates failing yields provider failure");
    table = read("<a id='99'><l cMsk0='1'>x</l></a>");
    result = resolve_native_assignment_material(table, query, catalog, loader);
    check(result.at("status") == "not_found" && result.at("loads").size() == 2,
          "empty fallback name still records completed name query without matching");
    query.color = 1;
    check(resolve_native_assignment_material(table, query, Json(), {}).at("status") ==
              "rule_not_found",
          "rule miss does not access catalog or invent parent layer fallback");

    auto palette = decode_native_palette_reference("$(_P3DLIB)|shared.p3d|Materials/Wood.pal");
    check(palette.at("status") == "decoded" && palette.at("reference_kind") == "library" &&
              palette.at("library_reference") == "shared.p3d" &&
              palette.at("palette_name") == "Wood",
          "library palette splits resource and filename stem");
    check(palette.at("palette_path") == "Materials/Wood.pal" &&
              palette.at("resource_lookup") == "not_performed" &&
              palette.at("table_registration") == "not_evaluated",
          "reference interpretation does not resolve external resources or claim registration");
    palette = decode_native_palette_reference("$(_P3DPROJECT)\\Materials\\Paint.pal");
    check(palette.at("reference_kind") == "project" && palette.at("library_reference") == "" &&
              palette.at("palette_name") == "Paint" &&
              palette.at("palette_path") == "Materials\\Paint.pal",
          "project prefix strips exact native 15 UTF16 units");
    palette = decode_native_palette_reference("E:\\Materials\\Mixed.case.pal");
    check(palette.at("reference_kind") == "ordinary" && palette.at("palette_name") == "Mixed.case",
          "ordinary reference uses Windows stem without filesystem access");
    check(decode_native_palette_reference("logical:dir/Named.pal").at("palette_name") == "Named",
          "native logical path prefix uses existing path parser");
    check(decode_native_palette_reference("$(_P3DLIB)").at("palette_name") == "" &&
              decode_native_palette_reference("$(_P3DLIB)").at("library_reference") == "",
          "bare library marker returns empty outputs");
    check(decode_native_palette_reference("").at("status") == "rejected",
          "empty palette input is rejected before provider calls");
    check(decode_native_palette_reference("$(_p3dlib)|shared|Wood.pal").at("status") ==
              "unresolved",
          "case-equivalent special prefix needs source locale");
    check(decode_native_palette_reference("$(_p3dlib)|shared|Wood.pal", text).at("palette_name") ==
              "Wood",
          "explicit source comparison enables lowercase library marker");
    check(decode_native_palette_reference("$(_P3DPROJECT)/Wood.pal").at("reference_kind") ==
              "ordinary",
          "project prefix requires native backslash rather than slash normalization");
    palette = decode_native_palette_reference("$(_P3DLIB)|outer|$(_P3DLIB)|inner|Nested.pal");
    check(palette.at("library_reference") == "outer" && palette.at("palette_name") == "Nested",
          "one nested library expression strips member prefix but retains outer resource");
    palette = decode_native_palette_reference("$(_P3DLIB)NoPipes");
    check(palette.at("library_reference") == "$(_P3DLIB)NoPipes" &&
              palette.at("palette_path") == "$(_P3DLIB)NoPipes",
          "native npos wrap with absent separators preserves its odd source split");
    palette = decode_native_palette_reference("$(_P3DLIB)||Wood.pal");
    check(palette.at("library_reference") == "|Wood.pal" &&
              palette.at("palette_path") == "$(_P3DLIB)||Wood.pal",
          "second delimiter search skips immediately adjacent delimiter");
    std::string nul_reference = "prefix.pal";
    nul_reference.append("\0ignored.pal", 12);
    palette = decode_native_palette_reference(nul_reference);
    check(palette.at("palette_name") == "prefix" && palette.at("source_value") == nul_reference,
          "native palette NUL cutoff retains full supplied source");
    palette = decode_native_palette_reference(std::string(260, 'x') + ".pal");
    check(palette.at("native_path_buffer_limit") == true && palette.at("palette_name") == "",
          "native fixed path buffers clear stem on component overflow");
    table = decode_native_material_assignment_table(
        "<t><paletteList>ignored<!--x--><one>first.pal</one><anything>first.pal</anything>"
        "<x>$(_P3DLIB)|shared|dir/<![CDATA[Wood]]>.pal</x><empty/>"
        "</paletteList><paletteList><x>ignored.pal</x></paletteList></t>");
    check(table.at("palette_references").size() == 4 &&
              table.at("palette_references")[2]["palette_name"] == "Wood",
          "first paletteList visits every direct element and concatenates text plus CDATA");
    check(table.at("palette_references")[0]["source_value"] == "first.pal" &&
              table.at("palette_references")[1]["source_value"] == "first.pal" &&
              table.at("palette_references")[1]["source_index"] == 1,
          "duplicate palette references retain source order before provider equality");
    check(table.at("palette_references")[3]["status"] == "rejected" &&
              table.at("palette_membership") == "not_evaluated" && table.at("status") == "resolved",
          "palette membership and rejected entries remain separate from rule normalization");
    return checks;
}
