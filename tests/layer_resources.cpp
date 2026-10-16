#include "internal.hpp"
#include <p3d/layer_resources.hpp>

unsigned layer_resource_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) {
        require(value, message);
        ++checks;
    };
    auto units = [](const std::u16string &text) {
        return Json(std::vector<std::uint16_t>(text.begin(), text.end()));
    };
    auto material = decode_native_layer_material_name(u"\"Palette\",\"Concrete\" suffix");
    check(material["status"] == "decoded" && material["name_utf16"] == units(u"Concrete") &&
              material["ignored_suffix_utf16"] == units(u" suffix"),
          "material reader discards the first quoted component and keeps the suffix separate");
    check(decode_native_layer_material_name(u"X\"!?Name\"")["name_utf16"] == units(u"Name"),
          "native quote scan does not validate the initial quote or skipped separator characters");
    check(decode_native_layer_material_name(u"\"P\",\"\"")["name_utf16"].empty() &&
              decode_native_layer_material_name(u"")["selection"] == "empty_input",
          "empty input and an empty extracted material name remain known empty");
    for (const auto &value : {u"No quote", u"\"No closing component", u"X\"x", u"\"P\",\"Name"})
        check(decode_native_layer_material_name(value)["status"] == "unresolved",
              "unsafe native material quote scans do not read past owned text");
    check(decode_native_layer_material_name(u"\"P\",\"" + std::u16string(511, u'A') +
                                            u"\"")["status"] == "decoded" &&
              decode_native_layer_material_name(u"\"P\",\"" + std::u16string(512, u'A') +
                                                u"\"")["status"] == "unresolved",
          "material copy includes the closing quote in its 512-unit buffer limit");
    const auto color = decode_native_layer_color_name(u"254 254 254 \"\" \"\"");
    check(color["status"] == "decoded" && color["rgb_bytes"] == Json::array({254, 254, 254}) &&
              color["first_name_utf16"].empty() && color["second_name_utf16"].empty(),
          "fixed-width RGB and empty color names decode independently of palette lookup");
    auto named_color = decode_native_layer_color_name(u"  1   2   3 \"Book\" \"Name\"tail");
    check(named_color["first_name_utf16"] == units(u"Book") &&
              named_color["second_name_utf16"] == units(u"Name") &&
              named_color["ignored_suffix_utf16"] == units(u"tail"),
          "color names use fixed first offset and closing-quote plus three for the second");
    check(decode_native_layer_color_name(u" -1 256 511")["rgb_bytes"] == Json::array({255, 0, 255}),
          "color decimal values truncate to bytes instead of clamping");
    for (const auto &value :
         {u"1 2 3", u"  1   X   3 \"\" \"\"", u"2147483648 0 0", u"  1   2   3 \"Book"})
        check(decode_native_layer_color_name(value)["status"] == "unresolved",
              "short, failed, overflowed or unterminated native color reads stay unknown");
    check(decode_native_layer_color_name(u"")["selection"] == "empty_input",
          "empty color input bypasses uninitialized RGB conversion");
    unsigned fallback_calls = 0;
    auto fallback = [&](const std::u16string &name) -> std::optional<std::int32_t> {
        ++fallback_calls;
        return name == u"Custom" ? -17 : 0;
    };
    for (int i = 0; i < 8; ++i) {
        const std::u16string name(1, char16_t(u'0' + i));
        auto value = lookup_native_line_style_name(name, fallback);
        check(value["line_style"] == i && value["assign_to_layer"] == true,
              "all eight exact numeric builtin names bypass external style lookup");
    }
    check(
        lookup_native_line_style_name(u"STYLE_ByLayer", fallback)["line_style"] == 0x7fffffff &&
            lookup_native_line_style_name(u"STYLE_ByBlock", fallback)["line_style"] == 0x7ffffffe &&
            lookup_native_line_style_name(u"STYLE_Invalid", fallback)["assign_to_layer"] == false &&
            fallback_calls == 0,
        "symbolic builtin names use native positive sentinel values without provider calls");
    check(lookup_native_line_style_name(u"style_ByLayer", fallback)["assign_to_layer"] == false &&
              fallback_calls == 1 &&
              lookup_native_line_style_name(u"Custom", fallback)["line_style"] == -17,
          "builtin matching is case sensitive; nonzero signed provider results are preserved");
    check(lookup_native_line_style_name(u"")["assign_to_layer"] == false &&
              lookup_native_line_style_name(u"Missing")["status"] == "unresolved",
          "empty name is a known miss while a missing prepared lookup is unknown");
    auto current = Json{{"kind", "current_resource_context"}};
    auto key = [](const std::string &index, const std::string &comparison) {
        return Json{{"kind", "explicit_context_keys"},
                    {"index_key", index},
                    {"comparison_key", comparison}};
    };
    auto entry = [&](std::uint64_t id, const std::string &name, Json context) {
        return Json{{"id", id}, {"name", name}, {"resource", {{"primary_context", context}}}};
    };
    auto catalog = [&](Json entries, Json context) {
        return Json{{"status", "resolved"}, {"entries", entries}, {"current_context", context}};
    };
    NativeMaterialReferenceNameContext context;
    context.compare = [](std::u16string a, std::u16string b) -> std::optional<int> {
        for (auto *s : {&a, &b})
            for (auto &c : *s)
                if (c >= u'A' && c <= u'Z')
                    c += 32;
        return a.compare(b);
    };
    const auto prepared = catalog(Json::array({entry(5, "Concrete", key("Wrong ID key", "Other")),
                                               entry(0, "concrete", key("Another ID key", "Main")),
                                               entry(9, "Concrete", current)}),
                                  key("Not used", "MAIN"));
    auto found = lookup_native_material_reference_name(u"CONCRETE", prepared, context);
    check(found["status"] == "matched" && found["selected_catalog_index"] == 1 &&
              found["material_id"] == 0 && found["comparisons"].size() == 2,
          "reference name lookup selects first name/context match using comparison key, never ID "
          "index key");
    const auto sentinel = catalog(
        Json::array({entry(UINT64_MAX, "Concrete", current), entry(7, "Concrete", current)}),
        current);
    check(lookup_native_material_reference_name(u"Concrete", sentinel)["material_id"] == UINT64_MAX,
          "the first matching sentinel reference is returned without searching a later valid ID");
    check(lookup_native_material_reference_name(u"", Json())["status"] == "not_found" &&
              lookup_native_material_reference_name(u"X", {{"status", "absent"}})["status"] ==
                  "not_found",
          "empty name and confirmed absent catalog do not access unavailable entries or keys");
    check(lookup_native_material_reference_name(
              u"Concrete", catalog(Json::array({entry(1, "Concrete", current)}),
                                   current))["status"] == "matched",
          "exact names and a shared symbolic resource context need no guessed locale");
    auto explicit_context = context;
    explicit_context.resource_comparison_key = u"OTHER";
    check(lookup_native_material_reference_name(u"Concrete", prepared,
                                                explicit_context)["selected_catalog_index"] == 0,
          "explicit resource comparison context selects the actual first compatible declaration");
    check(lookup_native_material_reference_name(
              u"Concrete", catalog(Json::array({entry(1, "Concrete", key("same", "External"))}),
                                   current))["status"] == "unresolved",
          "unknown current comparison identity cannot be fabricated from an ID index key");
    check(lookup_native_material_reference_name(u"concrete", sentinel)["status"] == "unresolved",
          "unavailable source-locale comparison does not become a miss");
    auto blocked = context;
    blocked.compare = [](const auto &, const auto &) -> std::optional<int> { return {}; };
    check(lookup_native_material_reference_name(u"CONCRETE", prepared, blocked)["status"] ==
              "unresolved",
          "unknown comparison stops before later candidates can falsely win");
    auto suffix_catalog = catalog(Json::array({entry(1, "First", current), Json()}), current);
    check(lookup_native_material_reference_name(u"First", suffix_catalog)["status"] == "matched",
          "first-match reference lookup never inspects a malformed later declaration");
    // Independent first-match oracle keeps catalog order and uses the actual
    // context equality key. Duplicate names/IDs are deliberately retained.
    const std::array<std::tuple<std::uint64_t, std::string, std::string>, 8> choices{
        {{0, "A", "Main"},
         {1, "a", "Main"},
         {UINT64_MAX, "A", "Main"},
         {2, "A", "Other"},
         {3, "B", "Main"},
         {4, "b", "Other"},
         {5, "A", "Other"},
         {6, "", "Main"}}};
    auto canonical = [](std::string name) {
        for (auto &c : name)
            if (c >= 'A' && c <= 'Z')
                c += 32;
        return name;
    };
    for (const auto &a : choices)
        for (const auto &b : choices)
            for (const auto &c : choices)
                for (const std::string query_key : {"Main", "Other"}) {
                    const std::array<std::tuple<std::uint64_t, std::string, std::string>, 3>
                        sequence{{a, b, c}};
                    Json entries = Json::array();
                    std::optional<std::size_t> winner;
                    for (std::size_t i = 0; i < sequence.size(); ++i) {
                        const auto &[id, name, resource] = sequence[i];
                        entries.push_back(entry(id, name, key("Deliberately unrelated", resource)));
                        if (!winner && canonical(name) == "a" && resource == query_key)
                            winner = i;
                    }
                    const auto actual = lookup_native_material_reference_name(
                        u"a", catalog(entries, key("Another unrelated ID key", query_key)),
                        context);
                    check(
                        winner ? actual["status"] == "matched" &&
                                     actual["selected_catalog_index"] == *winner &&
                                     actual["material_id"] == std::get<0>(sequence[*winner])
                               : actual["status"] == "not_found",
                        "reference selection agrees with independent ordered-name/context oracle");
                }

    auto put = [](Bytes &b, auto value) {
        auto *p = reinterpret_cast<const std::uint8_t *>(&value);
        b.insert(b.end(), p, p + sizeof(value));
    };
    auto set = [](Bytes &b, std::size_t offset, auto value) {
        std::memcpy(b.data() + offset, &value, sizeof(value));
    };
    auto link = [&](std::uint16_t k, const std::u16string &value, std::uint16_t flags = 0) {
        Bytes text, payload;
        put(text, std::uint16_t(0xfeff));
        for (auto c : value)
            put(text, std::uint16_t(c));
        put(payload, k);
        put(payload, flags);
        put(payload, std::uint32_t(text.size()));
        payload.insert(payload.end(), text.begin(), text.end());
        return Json{{"app", 0x56d2},
                    {"header", 0x1000},
                    {"offset", 108 + k},
                    {"payload", rawbytes(payload)}};
    };
    auto record = [&](Json links) {
        Bytes b(108);
        set(b, 4, std::uint16_t(49));
        set(b, 6, std::uint16_t(0x80));
        set(b, 8, std::uint32_t(52));
        set(b, 16, std::uint32_t(1));
        set(b, 36, std::uint32_t(1));
        set(b, 44, std::uint16_t(6));
        return Json{{"data", rawbytes(b)}, {"links", links}};
    };
    auto input = record(Json::array(
        {link(0x48, u"\"P\",\"Concrete\""), link(0x45, u"2"), link(0x47, u"\"P\",\"Old\""),
         link(0x47, u"\"Ignored\",\"Concrete\"", 17), link(0x43, u"  1   2   3 \"\" \"\"")}));
    auto decoded = decode_native_layer(bytesof(input["data"]), input["links"]);
    const auto &requests = decoded["resource_name_inputs"]["requests"];
    check(requests.size() == 4 && requests[0]["key"] == 0x43 && requests[1]["key"] == 0x45 &&
              requests[2]["source_string_index"] == 3 && requests[3]["key"] == 0x48 &&
              decoded["resource_name_inputs"]["entries"][2]["selected"] == false,
          "resource name map loads last per uint16 key, ignores string flags, then visits "
          "ascending keys");
    auto final = resolve_native_layer_material_names(Json::array({input}), 0, sentinel);
    check(final["status"] == "evaluated" && final["primary"]["material_id"] == UINT64_MAX &&
              final["primary"]["selection"] == "saved_reference",
          "first selected sentinel ID preserves the saved layer material instead of trying the "
          "next match");
    auto matched = resolve_native_layer_material_names(Json::array({input}), 0, prepared, context);
    check(matched["primary"]["material_id"] == 0 && matched["by_layer"]["material_id"] == 0 &&
              matched["primary"]["selection"] == "named_catalog_reference",
          "both material channels accept zero IDs from the actual prepared name/context query");
    auto no_names = record(Json::array());
    check(resolve_native_layer_material_names(Json::array({no_names}), 0, Json())["status"] ==
              "evaluated",
          "saved references without name requests do not require any catalog query");
    auto unknown = record(Json::array({link(0x47, u"Broken")}));
    auto partial = resolve_native_layer_material_names(Json::array({unknown}), 0, sentinel);
    check(partial["status"] == "partially_evaluated" &&
              partial["primary"]["material_id"].is_null() &&
              partial["by_layer"]["status"] == "resolved",
          "unknown primary name text does not erase the independent by-layer result");
    auto empty = record(Json::array({link(0x47, u"\"P\",\"\"")}));
    check(resolve_native_layer_material_names(Json::array({empty}), 0, Json())["status"] ==
              "evaluated",
          "empty extracted material name bypasses the prepared catalog");
    auto invalid = input;
    invalid["links"][1]["payload"] = rawbytes(Bytes{1, 2});
    auto broken = decode_native_layer(bytesof(invalid["data"]), invalid["links"]);
    check(broken["resource_name_inputs"]["status"] == "unresolved" &&
              broken["resource_name_inputs"]["requests"].is_null(),
          "an unsafe string load withholds the final resource request map");
    auto forged = input;
    forged["layer_definition"] = {{"material_reference", {{"material_id", 77}}}};
    check(resolve_native_layer_material_names(Json::array({forged}), 0, prepared, context) ==
              matched,
          "material name assignments read raw layer bodies and links, not replaceable annotations");
    return checks;
}
