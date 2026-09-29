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
    return checks;
}
