#include "internal.hpp"
#include "extended_colors.hpp"
#include "color_registration_oracle.hpp"
#include <p3d/color_registration.hpp>
#include <codecvt>
#include <locale>

unsigned color_registration_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) { ++checks; require(ok, message); };
    std::string fixture;
    for (const auto *part : color_registration_oracle_parts) fixture += part;
    const auto oracle = Json::parse(fixture);
    for (const auto &row : oracle.at("hsv"))
        check(Json(native_color_hsv(row.at("rgb").get<NativeRgb>())) == row.at("hsv"),
              "native integer HSV matches original DLL over grid, near-greys and random RGB");
    for (const auto &row : oracle.at("palettes")) {
        const auto palette = row.at("palette").get<NativeColorPalette>();
        for (const auto &query : row.at("queries"))
            check(nearest_native_palette_color(palette, query.at("rgb").get<NativeRgb>()) == query.at("index"),
                  "palette match reproduces quantized distance, ties, excluded slot and early exit");
    }
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> utf;
    auto pair = [&](const Json &value) -> std::optional<NativeColorBookName> {
        if (value.is_null()) return std::nullopt;
        return NativeColorBookName{utf.from_bytes(value.at("book").get<std::string>()),
                                   utf.from_bytes(value.at("name").get<std::string>())};
    };
    auto state = [&](const NativeExtendedColorCache &cache) {
        Json entries = Json::array();
        for (const auto &entry : cache.entries) {
            Json names;
            if (entry.book_name) names = {{"book", utf.to_bytes(entry.book_name->book)},
                                          {"name", utf.to_bytes(entry.book_name->name)}};
            entries.push_back({{"rgb", entry.rgb}, {"book_name", std::move(names)}});
        }
        return entries;
    };
    auto text = [&](const Json &value) -> std::optional<std::u16string> {
        return value.is_null() ? std::nullopt : std::optional<std::u16string>(utf.from_bytes(value.get<std::string>()));
    };
    for (const auto &row : oracle.at("registrations")) {
        NativeExtendedColorCache cache;
        // Feed the actual importer into registration as a consumer would.
        if (!row.at("xml").is_null()) {
            const auto imported = decode_native_extended_colors(xml_tree(row.at("xml")));
            require(imported.at("status") == "resolved", "color cache import prerequisite");
            for (const auto &entry : imported.at("entries"))
                cache.entries.push_back({entry.at("rgb").get<NativeRgb>(), pair(entry.at("book_name"))});
        }
        check(state(cache) == row.at("initial").at("entries") && cache.dirty == row.at("initial").at("dirty"),
              "imported cache preserves original native initial state before registration");
        const auto palette = oracle.at("palettes").at(row.at("palette").get<std::size_t>())
                                 .at("palette").get<NativeColorPalette>();
        for (const auto &query : row.at("queries")) {
            const auto result = register_native_color(cache, query.at("rgb").get<NativeRgb>(), &palette,
                text(query.at("book")), text(query.at("name")), query.at("allow_append").get<bool>());
            check(result.native_index == query.at("native_index") && result.color_id == query.at("color_id"),
                  "registration slot and encoded ID match original 1e41c0");
            check(state(cache) == query.at("entries") && cache.dirty == query.at("dirty"),
                  "registration reproduces ordered duplicate slots, optional name pairs and sticky dirty flag");
        }
    }
    NativeExtendedColorCache cache{{{{1,2,3}, std::nullopt}, {{1,2,3}, std::nullopt}}, false};
    auto result = register_native_color(cache, {1,2,3}, nullptr, u"B", u"N", false);
    check(result.action == NativeColorRegistrationAction::AttachedBookName && result.native_index == 2 &&
              result.color_id == 512 && !cache.entries[0].book_name && cache.entries[1].book_name && cache.dirty,
          "disallowing append still names the last duplicate and known-null palette contributes zero");
    cache.dirty = false;
    result = register_native_color(cache, {1,2,3}, nullptr, u"New", u"New", true);
    check(result.action == NativeColorRegistrationAction::Existing && cache.entries[1].book_name->book == u"B" &&
              !cache.dirty, "existing pair never overwritten and a no-op does not mark the cache dirty");
    result = register_native_color(cache, {4,5,6}, nullptr, u"", u"", true);
    check(result.action == NativeColorRegistrationAction::Appended && result.native_index == 3 &&
              cache.entries.back().book_name && cache.entries.back().book_name->book.empty(),
          "new RGB accepts an allocated pair of empty strings");
    result = register_native_color(cache, {7,8,9}, nullptr, {}, {}, false);
    check(result.action == NativeColorRegistrationAction::Missing && result.color_id == 0 && cache.entries.size() == 3,
          "missing RGB with appending disabled is zero slot and leaves prior dirty state intact");
    NativeColorPalette palette;
    palette.fill({255,255,255}); palette[0] = {0,0,0}; palette[1] = {20,20,20}; palette[255] = {255,0,0};
    check(nearest_native_palette_color(palette, {1,1,1}) == 0 &&
              nearest_native_palette_color(palette, {2,2,2}) == 1 &&
              nearest_native_palette_color(palette, {255,0,0}) == 2,
          "zero-distance early exit precedes grey rescue; excluded 255 cannot win even when exact");
    Bytes root_bytes(24); root_bytes[16] = 8;
    const auto records = Json::array({{{"element_type", 46}, {"element_flags", 0}, {"id", 4},
                                       {"data", rawbytes(root_bytes)}},
                                      {{"element_type", 46}, {"element_flags", 0}, {"id", 4},
                                       {"data", rawbytes(root_bytes)}}});
    Json list = {{"scope", "empty_list_before_runtime_registration"}, {"status", "resolved"},
        {"system_bootstrap_required", true}, {"system_bootstrap_found", true},
        {"bootstrap_root_record_index", 0}, {"roots", Json::array()}};
    for (unsigned i = 0; i < 2; ++i)
        list["roots"].push_back({{"native_record_index", i}, {"block_number", 1},
            {"headers", Json::array({{{"native_record_index", i}, {"status", "resolved"},
                                     {"parent_record_index", nullptr}, {"output_element_flags", 0}}})}});
    const auto ids = native_system_id_assignments(list, records,
        {{"status", "resolved"}, {"initial_probe", {{"action", "read_header_payload"}, {"id_counter", 10}}}});
    auto attribute = [&](unsigned key, unsigned index, const std::string &xml) {
        Bytes b(8); b[0] = 1;
        for (const auto c : xml) { b.push_back(std::uint8_t(c)); b.push_back(0); }
        const auto length = b.size() - 8;
        for (unsigned i = 0; i < 4; ++i) b[4+i] = std::uint8_t(length >> (8*i));
        return Json{{"group", 0}, {"key", key}, {"index", index}, {"offset", index + 200},
                    {"payload", rawbytes(b)}, {"decoded", decode_attribute(0, key, b, index)}};
    };
    const auto first = attribute(22902, 0, "<Colors><Entry Color=\"(1,2,3)\"/></Colors>");
    const auto later = attribute(22902, 0, "<Colors><Entry Color=\"(4,5,6)\"/></Colors>");
    auto attachments = [&](const Json &arrays) {
        Json input = {{"status", "resolved"}, {"attachments", Json::array()}};
        for (std::size_t i = 0; i < arrays.size(); ++i)
            if (!arrays[i].is_null())
                input["attachments"].push_back({{"target", {{"input_occurrence_index", i}}},
                    {"stream", {"FILE", "ATTR", "B1"}}, {"attributes", arrays[i]},
                    {"lookup", native_attribute_lookup(arrays[i])}});
        return input;
    };
    auto arrays = Json::array({Json::array({first}), Json::array({later})});
    auto input = attachments(arrays);
    auto run = [&]() { return initial_native_extended_color_table(list, records, ids, input); };
    auto table = run();
    check(table["status"] == "resolved" && table["selected_record"]["native_record_index"] == 0 &&
              table["cache_input"]["entries"][0]["rgb"] == Json::array({1,2,3}) &&
              table["runtime_cache"] == "not_evaluated", "system slot zero selects its own color input, not the last table");
    input = attachments(Json::array({nullptr, Json::array({later})}));
    table = run();
    check(table["outcome"] == "empty_cache_missing_attribute" && table["cache_input"]["slot_count"] == 0 &&
              table["selected_attribute"].is_null(), "missing slot-zero attribute never borrows another record's colors");
    for (const auto *status : {"absent", "not_loaded"}) {
        input = {{"status", status}, {"attachments", Json::array()}};
        check(run()["outcome"] == "empty_cache_missing_attribute", "known absent attribute input produces empty initial cache");
    }
    arrays[0] = Json::array({first, later}); input = attachments(arrays); table = run();
    check(table["selected_attribute"]["source_ordinal"] == 1 &&
              table["cache_input"]["entries"][0]["rgb"] == Json::array({4,5,6}),
          "small attribute collection takes the last equal color key");
    for (unsigned i = 0; i < 4; ++i) arrays[0].push_back(attribute(100+i, 0, "<X/>"));
    input = attachments(arrays); table = run();
    check(table["selected_attribute"]["source_ordinal"] == 0 &&
              table["cache_input"]["entries"][0]["rgb"] == Json::array({1,2,3}),
          "large attribute collection follows native sorted lower_bound for duplicate keys");
    arrays[0] = Json::array({first, later}); arrays[0][1]["decoded"] = Json::object(); input = attachments(arrays);
    table = run();
    check(table["status"] == "unresolved" && table["selected_attribute"]["source_ordinal"] == 1 &&
              !table.contains("cache_input"), "unknown selected import does not fall back to another color payload");
    arrays[0] = Json::array({attribute(22902, 1, "<Colors/>")}); input = attachments(arrays);
    check(run()["outcome"] == "empty_cache_missing_attribute", "color cache requires exact index zero");
    input = attachments(Json::array({Json::array({first}), nullptr}));
    input["status"] = "partial";
    check(run()["reason"] == "complete_initial_attribute_input_required", "partial attachments cannot prove final slot-zero colors");
    input["status"] = "resolved"; input["attachments"].push_back(input["attachments"][0]);
    check(run()["reason"] == "ambiguous_initial_attribute_attachment", "duplicate attachment identity is not silently selected");
    input["attachments"].erase(1); input["attachments"][0]["lookup"]["status"] = "unresolved";
    check(run()["reason"] == "initial_attribute_lookup_required", "unresolved native ordering prevents color attribute selection");
    input = attachments(Json::array({Json::array({first}), nullptr}));
    list["bootstrap_root_record_index"] = 1;
    check(run()["reason"] == "system_slot_zero_identity_mismatch", "bootstrap identity must equal first prepared occurrence");
    list["bootstrap_root_record_index"] = 0; list["system_bootstrap_found"] = false;
    check(run()["status"] == "unresolved", "missing bootstrap does not claim a valid empty native cache");
    return checks;
}
