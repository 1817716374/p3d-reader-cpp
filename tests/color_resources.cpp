#include "internal.hpp"
#include <p3d/color_resources.hpp>
#include <thread>

unsigned color_resource_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        require(value, why);
        ++checks;
    };
    auto units = [](const std::u16string &s) {
        return Json(std::vector<std::uint16_t>(s.begin(), s.end()));
    };
    auto hsv_oracle = [](NativeRgb rgb) {
        const int r = rgb[0], g = rgb[1], b = rgb[2];
        const int maximum = std::max({r, g, b}), minimum = std::min({r, g, b}),
                  delta = maximum - minimum;
        auto rounded = [](int numerator, int denominator) {
            return unsigned((2 * numerator + denominator) / (2 * denominator));
        };
        std::array<unsigned, 3> out{
            {0, maximum ? rounded(100 * delta, maximum) : 0, rounded(100 * maximum, 255)}};
        if (out[1]) {
            int numerator = maximum == r   ? 60 * (g - b)
                            : maximum == g ? 120 * delta + 60 * (b - r)
                                           : 240 * delta + 60 * (r - g);
            if (numerator < 0)
                numerator += 360 * delta;
            out[0] = rounded(numerator, delta) % 360;
        }
        return out;
    };
    for (unsigned r = 0; r < 256; r += 17)
        for (unsigned g = 0; g < 256; g += 17)
            for (unsigned b = 0; b < 256; b += 17) {
                NativeRgb rgb{{std::uint8_t(r), std::uint8_t(g), std::uint8_t(b)}};
                check(native_color_hsv(rgb) == hsv_oracle(rgb),
                      "native floating HSV agrees with independent rational oracle on RGB grid");
            }
    NativeColorPalette palette;
    palette.fill({255, 0, 0});
    palette[255] = {1, 2, 3};
    auto closest = nearest_native_palette_color({1, 2, 3}, palette);
    check(
        closest["palette_index"] == 0 && closest["candidates_visited"] == 255,
        "palette entry 255 is excluded even when it is the only exact RGB match; ties keep first");
    palette.fill({255, 0, 0});
    palette[0] = {5, 5, 5};
    palette[1] = {0, 0, 0};
    palette[2] = {3, 3, 3};
    closest = nearest_native_palette_color({1, 1, 1}, palette);
    check(closest["palette_index"] == 0 && closest["nearest_index_before_fallback"] == 1 &&
              closest["candidates_visited"] == 2 && closest["gray_fallback"] == true,
          "early exact HSV termination restricts gray fallback to already visited candidates");
    palette[0] = {0, 0, 0};
    palette[1] = {3, 3, 3};
    check(nearest_native_palette_color({1, 1, 1}, palette)["gray_fallback"] == false,
          "an initial exact black HSV match does not inspect later nonblack gray candidates");
    palette[0] = {5, 5, 5};
    palette[1] = {0, 0, 0};
    check(nearest_native_palette_color({0, 0, 0}, palette)["palette_index"] == 1,
          "an actual black RGB request preserves the nearest black entry");
    palette.fill({0, 255, 0});
    palette[0] = {250, 255, 250};
    palette[1] = {255, 250, 250};
    check(nearest_native_palette_color({255, 250, 250}, palette)["palette_index"] == 0,
          "hue is ignored at query saturation two, so native first ties can precede exact RGB");
    palette[0] = {248, 255, 248};
    palette[1] = {255, 248, 248};
    check(nearest_native_palette_color({255, 248, 248}, palette)["palette_index"] == 1,
          "hue participates above query saturation two");
    // Independent full-distance oracle computes all candidates, then restricts
    // gray candidates to the prefix before native exact-score termination.
    std::uint32_t random = 0x73b129u;
    auto next = [&] {
        random ^= random << 13;
        random ^= random >> 17;
        random ^= random << 5;
        return random;
    };
    for (unsigned trial = 0; trial < 2048; ++trial) {
        NativeColorPalette source;
        for (auto &rgb : source)
            for (auto &c : rgb)
                c = std::uint8_t((next() % 16) * 17);
        NativeRgb query;
        for (auto &c : query)
            c = std::uint8_t((next() % 16) * 17);
        const auto q = hsv_oracle(query);
        std::array<unsigned, 255> distances;
        std::array<std::array<unsigned, 3>, 255> values;
        unsigned visited = 255;
        for (unsigned i = 0; i < 255; ++i) {
            values[i] = hsv_oracle(source[i]);
            const auto &v = values[i];
            int hue = std::abs(int(v[0]) - int(q[0]));
            hue = std::min(hue, 360 - hue);
            if (q[1] <= 2)
                hue = 0;
            const int sat = int(v[1]) - int(q[1]), value = int(v[2]) - int(q[2]);
            distances[i] = 2 * hue * hue + sat * sat + value * value;
            if (!distances[i] && visited == 255)
                visited = i + 1;
        }
        const auto winner = unsigned(
            std::min_element(distances.begin(), distances.begin() + visited) - distances.begin());
        std::optional<unsigned> gray;
        const unsigned gray_limit = distances[winner] == 0 ? visited - 1 : visited;
        for (unsigned i = 0; i < gray_limit; ++i)
            if (values[i][1] == 0 && values[i][2] > 0 && (!gray || values[i][2] < values[*gray][2]))
                gray = i;
        const bool fallback =
            values[winner][2] == 0 && (query[0] || query[1] || query[2]) && gray.has_value();
        const auto expected = fallback ? *gray : winner;
        const auto actual = nearest_native_palette_color(query, source);
        check(actual["palette_index"] == expected &&
                  actual["nearest_index_before_fallback"] == winner &&
                  actual["candidates_visited"] == visited &&
                  actual["squared_distance"] == distances[winner] &&
                  actual["gray_fallback"] == fallback,
              "palette selection agrees with independent complete-score/prefix-fallback oracle");
    }
    const std::string xml =
        "<Root><Entry/><Other Color='(4,5,6)'/><Entry Color=''/><Branch>"
        "<Entry Color='(1,2,3)' Book='B' Name='N'/></Branch><Entry Color='(1,2,3)' Book='Only'/>"
        "<Entry Color='bad'/><Entry Color='(256,-1,5)'/><Entry Color='(7,8)'/></Root>";
    NativeExtendedColorCache cache(xml);
    const auto before = cache.metadata();
    check(
        before["status"] == "resolved" && before["entries"].size() == 5 &&
            before["load_steps"].size() == 7,
        "native XML selection visits all descendant Entry nodes, skipping missing and empty Color");
    check(cache.lookup({1, 2, 3})["extended_color_id"] == 2 &&
              before["entries"][0]["names_present"] == true &&
              before["entries"][1]["names_present"] == false,
          "duplicate loaded RGB entries remain distinct while the RGB index selects the last");
    check(before["entries"][2]["rgb"] == Json::array({255, 255, 255}) &&
              before["entries"][3]["rgb"] == Json::array({0, 255, 5}) &&
              before["entries"][4]["rgb"] == Json::array({7, 8, 255}),
          "XML RGB uses initialized 255 defaults, partial conversion and byte modulo rather than "
          "range checks");
    NativeExtendedColorCache partial(
        "<x><Entry Color='(1 ,2,3)'/><Entry Color='(4,5,6missing'/><Entry Color=' (7,8,9)'/></x>");
    check(partial.metadata()["entries"][0]["rgb"] == Json::array({1, 255, 255}) &&
              partial.metadata()["entries"][1]["rgb"] == Json::array({4, 5, 6}) &&
              partial.metadata()["entries"][2]["rgb"] == Json::array({255, 255, 255}),
          "native scanf literals do not skip spaces and failed trailing literals do not undo "
          "conversions");
    auto attached = cache.resolve({{{1, 2, 3}, u"NewBook", u"NewName"}}, {}, true);
    check(attached.metadata()["entries"][1]["book_utf16"] == units(u"NewBook") &&
              attached.metadata()["operations"][0]["extended_color_id"] == 2 &&
              attached.metadata()["operations"][0]["names_attached"] == true &&
              attached.metadata()["operations"][0]["packed_color"] == 0x200 &&
              cache.metadata() == before,
          "request attaches names to the indexed nameless entry and preserves source cache "
          "identity/state");
    NativeExtendedColorCache fresh;
    auto allocated_empty =
        fresh.resolve({{{9, 8, 7}, u"", u""}, {{9, 8, 7}, u"Book", u"Name"}}, {}, true);
    check(allocated_empty.metadata()["entries"].size() == 1 &&
              allocated_empty.metadata()["entries"][0]["names_present"] == true &&
              allocated_empty.metadata()["entries"][0]["book_utf16"].empty() &&
              allocated_empty.metadata()["operations"][1]["names_attached"] == false,
          "allocated empty names are distinct from null and block later name replacement");
    auto unnamed = fresh.resolve({{{9, 8, 7}, {}, {}}});
    check(unnamed.metadata()["status"] == "resolved" &&
              unnamed.metadata()["entries"][0]["names_present"] == false &&
              unnamed.metadata()["operations"][0]["packed_color"].is_null() &&
              unnamed.metadata()["color_resolution_status"] == "partially_evaluated",
          "unknown actual palette does not fabricate a low byte or discard the independent "
          "RGB-cache transition");
    auto later = unnamed.resolve({{{9, 8, 7}, u"Book", u"Name"}, {{5, 4, 3}, {}, {}}}, {}, true);
    check(later.metadata()["entries"].size() == 2 &&
              later.metadata()["operations"][0]["extended_color_id"] == 1 &&
              later.metadata()["operations"][0]["names_attached"] == true &&
              later.metadata()["operations"][1]["extended_color_id"] == 2,
          "derived snapshots continue native RGB reuse and append IDs without merging source "
          "entries");
    auto preserved = later.resolve({{{9, 8, 7}, u"Different", u"Different"}}, {}, true);
    check(preserved.metadata()["entries"][0]["book_utf16"] == units(u"Book"),
          "existing allocated names are not overwritten by a different name for the same RGB");
    palette.fill({0, 0, 0});
    palette[7] = {9, 8, 7};
    auto colored = fresh.resolve({{{9, 8, 7}, {}, {}}}, palette);
    check(colored.metadata()["operations"][0]["packed_color"] == 0x107 &&
              colored.metadata()["operations"][0]["assign_to_layer"] == true,
          "actual palette index combines with one-based extended ID in the source 32-bit encoding");
    check(fresh.resolve({}, palette, true).metadata()["status"] == "unresolved",
          "contradictory palette presence does not publish a fabricated final cache");
    for (const auto &bad :
         {"<x>", "<!DOCTYPE x><x/>", "<x xmlns='urn:color'><Entry Color='(1,2,3)'/></x>",
          "<x><Entry Color='(2147483648,0,0)'/></x>",
          "<x><Entry Color='(1,2,3)' Name='&unknown;'/></x>",
          "<x><Entry Color='(1,2,3)' Name='&#0;'/></x>"}) {
        NativeExtendedColorCache unknown(bad);
        check(unknown.metadata()["status"] == "unresolved" &&
                  unknown.metadata()["entries"].is_null() &&
                  unknown.lookup({1, 2, 3})["status"] == "unresolved",
              "unsupported XML and unsafe numeric conversion withhold final color state");
    }
    NativeExtendedColorCache escaped("<x><!-- &unknown; --><![CDATA[&other;]]><Entry "
                                     "Color='(1,2,3)' Book='A&amp;B' Name='&#x4E2D;'/></x>");
    check(escaped.metadata()["status"] == "resolved" &&
              escaped.metadata()["entries"][0]["book_utf16"] == units(u"A&B") &&
              escaped.metadata()["entries"][0]["name_utf16"] == units(u"中"),
          "XML entities decode names while comment and CDATA text are not treated as entity "
          "references");
    const auto bad_utf8 = NativeExtendedColorCache(std::string(1, char(0xff))).metadata();
    check(bad_utf8["status"] == "unresolved" && bad_utf8["source_xml_bytes"]["bytes"] == 1 &&
              !bad_utf8.dump().empty(),
          "invalid UTF8 input retains source bytes without breaking diagnostic JSON serialization");
    Bytes payload(8);
    payload[0] = 1;
    for (auto c : xml) {
        payload.push_back(std::uint8_t(c));
        payload.push_back(0);
    }
    payload.push_back(0);
    payload.push_back(0);
    const auto length = std::uint32_t(payload.size() - 8);
    std::memcpy(payload.data() + 4, &length, 4);
    const auto decoded = decode_attribute(0, 22902, payload, 0);
    check(decoded["native_extended_colors"]["status"] == "resolved" &&
              decoded["color_entries"].size() == 5 &&
              decoded["color_entries"][3]["rgb"] == Json::array({0, 255, 5}),
          "stored color attribute feeds native selection and correct loaded ordinal projection");
    std::array<bool, 4> ok{{true, true, true, true}};
    std::array<std::thread, 4> threads;
    for (unsigned i = 0; i < threads.size(); ++i)
        threads[i] = std::thread([&, i] {
            for (unsigned n = 0; n < 100; ++n)
                if (cache.lookup({1, 2, 3})["extended_color_id"] != 2)
                    ok[i] = false;
        });
    for (auto &t : threads)
        t.join();
    check(
        std::all_of(ok.begin(), ok.end(), [](bool v) { return v; }) && cache.metadata() == before,
        "owned immutable loaded cache supports concurrent lookup without changing RGB reuse state");
    return checks;
}
