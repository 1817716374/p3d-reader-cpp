#include "internal.hpp"
#include "material_source_graph.hpp"
#include <numeric>
#include <thread>

unsigned material_sources_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool b, const char *why) {
        require(b, why);
        ++checks;
    };
    const Json none = Json::array();
    auto definition = [](std::uint64_t record, std::uint64_t offset, unsigned ordinal) {
        return Json{{"id", 7},
                    {"scope", "global"},
                    {"stream", {"source", "attributes"}},
                    {"record_offset", record},
                    {"attribute_header_offset", offset},
                    {"attribute_offset", offset + 16},
                    {"attribute_ordinal", ordinal},
                    {"attribute_index", 0}};
    };
    auto attribute = [](const Json &d) {
        return Json{{"stream", d.at("stream")},
                    {"record_offset", d.at("record_offset")},
                    {"offset", d.at("attribute_header_offset")},
                    {"source_ordinal", d.at("attribute_ordinal")}};
    };
    auto entry = [&](std::uint64_t id, std::string name, const Json *def) {
        Json e = {
            {"id", id},
            {"name", name},
            {"resource",
             {{"member_name", name}, {"primary_context", {{"kind", "current_resource_context"}}}}}};
        if (def)
            e["source_attribute"] = attribute(*def);
        return e;
    };
    auto catalog = [](Json entries) {
        return Json{{"status", "resolved"}, {"entries", std::move(entries)}};
    };
    auto bytes = [](const std::u16string &s) {
        Bytes b;
        for (auto c : s) {
            b.push_back(std::uint8_t(c));
            b.push_back(std::uint8_t(c >> 8));
        }
        return b;
    };
    auto prop = [](unsigned group, unsigned index, Bytes b) {
        return Json{{"group", group},
                    {"key", 10001},
                    {"index", index},
                    {"offset", 88 + index},
                    {"payload", rawbytes(b)}};
    };
    auto graphics = [](Json attributes) {
        return Json::array({{{"id", 99},
                             {"offset", 40},
                             {"stream", {"model", "attributes"}},
                             {"attributes", std::move(attributes)}}});
    };
    auto defs = Json::array({definition(100, 128, 0), definition(100, 256, 1),
                             definition(200, 228, 0), definition(200, 228, 0)});
    Json materials = {{"definitions", defs}};
    auto c0 = catalog(Json::array({entry(77, "Wood", &defs[1]), entry(0, "Zero", &defs[0]),
                                   entry(UINT64_MAX, "Sentinel", &defs[2])}));
    c0["initial_source_materials"] = {
        {"entries", Json::array({{{"catalog_entry_index", 0},
                                  {"status", "decoded"},
                                  {"source_attribute", attribute(defs[0])}},
                                 {{"catalog_entry_index", 1}, {"status", "not_found"}}})}};
    auto c1 = catalog(Json::array({entry(77, "wood", nullptr)}));
    const auto catalogs = Json::array({c0, c1});
    auto g = graphics(Json::array({prop(4, 0, bytes(u"WOOD"))}));
    auto result = detail::material_source_graph(materials, catalogs, g, none, {});
    const auto &nodes = result.at("catalogs")[0].at("entries");
    check(result.at("status") == "resolved" &&
              nodes[0].at("declared_source").at("definition_indices") == Json::array({1}) &&
              nodes[0].at("initial_read_source").at("definition_indices") == Json::array({0}),
          "physical selected attribute identity overrides equal saved IDs; declared and provider "
          "inputs remain distinct");
    const auto &candidate =
        result.at("attribute_assignments")[0].at("queries")[0].at("candidates")[0];
    check(candidate.at("catalog_entry_index") == 0 && candidate.at("catalog_id") == 77 &&
              candidate.at("catalog_name") == "Wood" &&
              candidate.at("declared_source").at("definition_indices") == Json::array({1}) &&
              candidate.at("initial_read_status") == "decoded" &&
              candidate.at("initial_read_source").at("definition_indices") == Json::array({0}),
          "query candidate carries both independent source-definition edges without selecting a "
          "runtime winner");
    check(
        nodes[0].at("declared_source").at("source_identity").at("attribute_header_offset") == 256 &&
            nodes[0].at("declared_source").at("source_identity").at("attribute_payload_offset") ==
                272 &&
            result.at("definitions")[1].at("attribute_offset") == 272,
        "catalog header offsets and graphics payload offsets differ by the actual 16-byte header");
    auto wrong_header = materials;
    wrong_header["definitions"][1]["attribute_header_offset"] = 272;
    const auto mismatch = detail::material_source_graph(wrong_header, catalogs, g, none, {});
    check(mismatch.at("status") == "partial" &&
              mismatch.at("definitions")[1].at("source_identity_status") == "unresolved" &&
              mismatch.at("catalogs")[0].at("entries")[0].at("declared_source").at("status") ==
                  "not_found",
          "inconsistent header and payload offsets cannot silently produce a resolved source edge");
    check(
        nodes[2].at("declared_source").at("status") == "ambiguous" &&
            nodes[2].at("declared_source").at("definition_indices") == Json::array({2, 3}),
        "duplicate physical identity remains ambiguous rather than content or name deduplication");
    check(result.at("definitions").size() == 4 &&
              result.at("attribute_assignments")[0].at("queries")[0].at("candidates").size() == 2 &&
              result.at("attribute_assignments")[0].at("queries")[0].at("candidates")[0].at(
                  "catalog_context_index") == 0 &&
              result.at("attribute_assignments")[0].at("queries")[0].at("candidates")[1].at(
                  "catalog_context_index") == 1,
          "matching names preserve catalog namespaces and native registered order without "
          "selecting a runtime winner");
    check(nodes[1].at("initial_read_source").at("status") == "not_evaluated" &&
              result.at("catalogs")[1].at("entries")[0].at("declared_source").at("status") ==
                  "not_evaluated",
          "missing provider or external source does not borrow an unrelated same-ID definition");
    StoredMaterialSourceOptions exact;
    exact.catalog.case_equal = [](const std::string &a, const std::string &b) { return a == b; };
    auto exact_result = detail::material_source_graph(materials, catalogs, g, none, exact);
    check(exact_result.at("attribute_assignments")[0].at("queries")[0].at("status") ==
              "no_candidates",
          "source comparison callback is respected rather than forced ASCII folding");
    // Every permutation of independent descriptors and source definitions.
    Json distinct = Json::array();
    for (unsigned i = 0; i < 5; ++i)
        distinct.push_back(definition(300 + i, 500 + i * 32, i));
    Json inputs = Json::array();
    for (unsigned i = 0; i < 5; ++i)
        inputs.push_back(prop(2, i, bytes(std::u16string{u'M', char16_t(u'0' + i)})));
    std::array<unsigned, 5> order{{0, 1, 2, 3, 4}};
    do {
        Json registered = Json::array();
        for (auto i : order)
            registered.push_back(entry(UINT64_MAX - 1 - i, "M" + std::to_string(i), &distinct[i]));
        const auto r = detail::material_source_graph({{"definitions", distinct}},
                                                     Json::array({catalog(registered)}),
                                                     graphics(inputs), none, {});
        bool correct = r.at("status") == "resolved";
        for (unsigned i = 0; i < 5; ++i) {
            const auto expected =
                std::size_t(std::find(order.begin(), order.end(), i) - order.begin());
            const auto &q = r.at("attribute_assignments")[i].at("queries")[0];
            correct = correct && q.at("candidates").size() == 1 &&
                      q.at("candidates")[0].at("catalog_entry_index") == expected &&
                      r.at("catalogs")[0]
                              .at("entries")[expected]
                              .at("declared_source")
                              .at("definition_indices") == Json::array({i});
        }
        check(correct, "independent permutation oracle for material references and source "
                       "attribute occurrences");
    } while (std::next_permutation(order.begin(), order.end()));
    for (unsigned count : {2u, 6u}) {
        Json attrs = Json::array();
        attrs.push_back(prop(4, 0, bytes(u"Wood")));
        for (unsigned i = 1; i < count - 1; ++i)
            attrs.push_back({{"group", 0}, {"key", i}, {"index", 0}});
        attrs.push_back(prop(4, 0, bytes(u"Zero")));
        auto r =
            detail::material_source_graph(materials, Json::array({c0}), graphics(attrs), none, {});
        const auto &rows = r.at("attribute_assignments");
        check(rows.size() == 2 &&
                  rows[0].at("attribute_lookup_selection") ==
                      (count == 2 ? "shadowed" : "selected") &&
                  rows[1].at("attribute_lookup_selection") ==
                      (count == 2 ? "selected" : "shadowed") &&
                  rows[0].at("queries")[0].at("candidates")[0].at("catalog_entry_index") == 0 &&
                  rows[1].at("queries")[0].at("candidates")[0].at("catalog_entry_index") == 1,
              "all declared names survive while native duplicate selection uses complete attribute "
              "count");
    }
    auto part = prop(4, 1, Bytes{'{', '}'});
    const std::string json =
        "{/*comment*/"
        "\"0\":\"Wood\",\"1\":77,\"2\":true,\"3\":null,\"4\":1.5,\"01\":\"Zero\",\"-1\":\"Zero\"}";
    part["payload"] = rawbytes(Bytes(json.begin(), json.end()));
    auto parts =
        detail::material_source_graph(materials, catalogs, graphics(Json::array({part})), none, {});
    check(parts.at("status") == "partial" &&
              parts.at("attribute_assignments")[0].at("status") == "partial",
          "unsupported JsonCpp float name conversion stays unknown rather than inventing text");
    std::map<std::string, Json> part_rows;
    for (const auto &q : parts.at("attribute_assignments")[0].at("queries"))
        part_rows[q.at("source_key").get<std::string>()] = q;
    check(part_rows.at("0").at("query").at("candidates").size() == 2 &&
              part_rows.at("1").at("query").at("name") == "77" &&
              part_rows.at("2").at("query").at("name") == "true" &&
              part_rows.at("3").at("query").at("status") == "not_requested" &&
              part_rows.at("01").at("status") == "not_requested" &&
              part_rows.at("-1").at("status") == "not_requested",
          "native scalar name conversion, empty names and canonical nonnegative part keys are "
          "distinct");
    const std::string reverse_json = "{\"Wood\":{\"0\":\"0\",\"1\":\"1\"}}";
    auto reverse = detail::material_source_graph(
        materials, catalogs,
        graphics(Json::array({prop(4, 2, Bytes(reverse_json.begin(), reverse_json.end()))})), none,
        {});
    check(reverse.at("status") == "resolved" &&
              reverse.at("attribute_assignments")[0].at("queries")[0].at("direction") ==
                  "reverse_index_only" &&
              reverse.at("attribute_assignments")[0]
                      .at("queries")[0]
                      .at("query")
                      .at("candidates")
                      .size() == 2 &&
              reverse.at("attribute_assignments")[0].at("queries")[0].at("source_part_keys") ==
                  Json::array({"0", "1"}),
          "reverse index names retain source references without becoming forward part or triangle "
          "assignments");
    check(detail::material_source_graph(materials, catalogs,
                                        graphics(Json::array({prop(4, 2, Bytes{0xff})})), none, {})
                  .at("status") == "partial",
          "unreadable reverse-index data remains explicit rather than silently omitted");
    auto partial_catalog = c0;
    partial_catalog["status"] = "partial";
    const auto partial =
        detail::material_source_graph(materials, Json::array({partial_catalog}), g, none, {});
    check(partial.at("status") == "partial" &&
              partial.at("attribute_assignments")[0].at("queries")[0].at("status") ==
                  "partial_candidates" &&
              !partial.at("attribute_assignments")[0]
                   .at("queries")[0]
                   .at("candidate_set_complete")
                   .get<bool>(),
          "a catalog prefix hit never hides an unknown suffix");
    auto broken = materials;
    broken["definitions"][1].erase("attribute_offset");
    auto missing = detail::material_source_graph(broken, Json::array({c0}), g, none, {});
    check(missing.at("status") == "partial" &&
              missing.at("catalogs")[0].at("entries")[0].at("declared_source").at("status") ==
                  "not_found",
          "missing physical identity is not repaired by identical IDs or names");
    auto put = [](Bytes &b, std::size_t off, std::uint64_t v, unsigned n) {
        for (unsigned j = 0; j < n; ++j)
            b[off + j] = std::uint8_t(v >> (8 * j));
    };
    auto id_link = [&](std::uint64_t id, unsigned key = 0x1000e) {
        Bytes b(12);
        put(b, 0, key, 4);
        put(b, 4, id, 8);
        return Json{{"header", 0x1000}, {"app", 0x41}, {"payload", rawbytes(b)}};
    };
    auto name_link = [&](Bytes name) {
        Bytes b(name.size() + 3);
        put(b, 0, name.size(), 2);
        std::copy(name.begin(), name.end(), b.begin() + 2);
        return Json{{"header", 0x1000}, {"app", 0x4f5a}, {"payload", rawbytes(b)}};
    };
    auto native = Json::array(
        {{{"id", 7},
          {"stream", {"native"}},
          {"offset", 0},
          {"layer_definition", Json::object()},
          {"links", Json::array({id_link(77), id_link(0), id_link(UINT64_MAX), id_link(0, 0x1000c),
                                 name_link(Bytes{'W', 'o', 'o', 'd'}),
                                 name_link(Bytes{'Z', 'e', 'r', 'o'})})}}});
    auto links = detail::material_source_graph(materials, catalogs, none, native, {});
    const auto &refs = links.at("native_link_references");
    check(links.at("status") == "resolved" && refs.size() == 6 &&
              refs[0].at("query").at("candidates").size() == 2 &&
              refs[1].at("query").at("candidates").size() == 1 &&
              refs[2].at("query").at("status") == "not_requested" &&
              refs[3].at("role") == "layer_primary_material_id",
          "native full-width ID declarations, zero, sentinel and layer-primary channel remain "
          "distinct");
    check(refs[0].at("entity_getter_selection") == "first_match" &&
              refs[1].at("entity_getter_selection") == "later_match" &&
              refs[4].at("entity_getter_selection") == "first_match" &&
              refs[5].at("entity_getter_selection") == "later_match" &&
              refs[5].at("query").at("candidates").size() == 1,
          "later stored links keep source candidates without pretending the entity getter selects "
          "them");
    auto high_native = native;
    high_native[0]["links"] = Json::array({id_link(UINT64_MAX - 9)});
    auto high_catalog = catalog(Json::array({entry(UINT64_MAX - 9, "High", &defs[0])}));
    check(
        detail::material_source_graph(materials, Json::array({high_catalog}), none, high_native, {})
                .at("native_link_references")[0]
                .at("query")
                .at("candidates")
                .size() == 1,
        "material IDs above signed and double exact ranges remain uint64");
    auto malformed = native;
    malformed[0]["links"] = Json::array(
        {{{"header", 0x1000}, {"app", 0x41}, {"payload", rawbytes(Bytes{1})}}, id_link(77)});
    const auto bad = detail::material_source_graph(materials, catalogs, none, malformed, {});
    check(bad.at("status") == "partial" &&
              bad.at("native_link_references")[1].at("entity_getter_selection") ==
                  "earlier_key_unresolved" &&
              bad.at("native_link_references")[1].at("query").at("candidates").size() == 2,
          "an unreadable earlier key leaves getter selection unknown but does not erase a later "
          "declared reference");
    auto encoded = native;
    encoded[0]["links"] = Json::array({name_link(Bytes{0x80})});
    check(detail::material_source_graph(materials, catalogs, none, encoded, {}).at("status") ==
              "partial",
          "unknown Windows ANSI code page does not become an empty or guessed name");
    StoredMaterialSourceOptions ansi;
    ansi.ansi_decoder = [](const Bytes &b) {
        require(b == Bytes{0x80}, "ANSI source mismatch");
        return std::string("Wood");
    };
    check(detail::material_source_graph(materials, catalogs, none, encoded, ansi)
                  .at("native_link_references")[0]
                  .at("query")
                  .at("candidates")
                  .size() == 2,
          "caller ANSI decoding is used for the actual stored prefix only");
    auto nul = detail::material_source_graph(
        materials, Json::array({c0}),
        graphics(Json::array({prop(4, 0, bytes(std::u16string(u"Wood\0Zero", 9)))})), none, {});
    check(nul.at("attribute_assignments")[0].at("queries")[0].at("name") == "Wood",
          "wide names stop at the first NUL without altering the stored source payload");
    auto odd_catalog = catalog(Json::array({entry(1, "a", &defs[0])}));
    check(detail::material_source_graph(materials, Json::array({odd_catalog}),
                                        graphics(Json::array({prop(4, 0, Bytes{'A'})})), none, {})
                  .at("attribute_assignments")[0]
                  .at("queries")[0]
                  .at("candidates")
                  .size() == 1,
          "native zero padding makes an odd final UTF16 byte readable");
    check(detail::material_source_graph(
              materials, catalogs, graphics(Json::array({prop(4, 0, Bytes{0, 0xd8})})), none, {})
                  .at("status") == "partial",
          "invalid Unicode remains unresolved with its source identity intact");
    const auto expected = detail::material_source_graph(materials, catalogs, g, native, {});
    std::array<Json, 4> threaded;
    std::array<std::thread, 4> threads;
    for (unsigned i = 0; i < 4; ++i)
        threads[i] = std::thread([&, i] {
            threaded[i] = detail::material_source_graph(materials, catalogs, g, native, {});
        });
    for (auto &t : threads)
        t.join();
    check(
        std::all_of(threaded.begin(), threaded.end(), [&](const Json &r) { return r == expected; }),
        "parallel source graph construction shares no mutable catalog or definition state");
    return checks;
}
