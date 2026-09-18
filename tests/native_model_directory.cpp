#include "internal.hpp"

unsigned native_model_directory_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool condition, const char *message) {
        ++checks;
        require(condition, message);
    };
    auto put = [](Bytes &b, std::size_t at, std::uint64_t value, unsigned n) {
        for (unsigned i = 0; i < n; ++i)
            b.at(at + i) = std::uint8_t(value >> (8 * i));
    };
    auto wide = [](const std::u16string &text) {
        Bytes b;
        for (auto unit : text) {
            b.push_back(std::uint8_t(unit));
            b.push_back(std::uint8_t(unit >> 8));
        }
        return b;
    };
    auto entry = [&](std::uint32_t id, unsigned flags, std::u16string name,
                     std::u16string secondary = u"", Bytes mask = {}) {
        Bytes b(32, 0);
        put(b, 0, 7, 2);
        put(b, 2, flags, 2);
        put(b, 4, id, 4);
        const auto a = wide(name), s = wide(secondary);
        put(b, 18, a.size(), 2);
        put(b, 20, s.size(), 2);
        put(b, 24, mask.size(), 2);
        put(b, 26, 9, 2);
        b.insert(b.end(), a.begin(), a.end());
        b.insert(b.end(), s.begin(), s.end());
        b.insert(b.end(), mask.begin(), mask.end());
        return b;
    };
    auto wire = [&](const std::vector<Bytes> &entries, unsigned version = 4) {
        Bytes b(16, 0);
        put(b, 0, 0xaa00ba11, 4);
        put(b, 4, version, 4);
        put(b, 8, entries.size(), 4);
        for (const auto &e : entries)
            b.insert(b.end(), e.begin(), e.end());
        return b;
    };
    const auto basic = entry(5, 0x8025, u"Model", u"description");
    auto parsed = parse_native_model_directory(wire({basic}));
    check(parsed["status"] == "decoded" && parsed["native_input_status"] == "decoded" &&
              parsed["entries"][0]["model_id"] == 5 && parsed["entries"][0]["kind"] == 2 &&
              parsed["entries"][0]["source_kind"] == 7 &&
              parsed["entries"][0]["excluded_from_name_lookup"] == true &&
              parsed["entries"][0]["name"]["text"] == "Model" &&
              parsed["entries"][0]["secondary_string"]["text"] == "description",
          "model directory header, strings, kind override and lookup exclusion");
    check(lookup_native_model_directory(parsed, u"Model")["status"] == "missing",
          "excluded directory entries never participate in name lookup");
    parsed =
        parse_native_model_directory(wire({basic, entry(8, 0, u"Model"), entry(9, 0, u"Model")}));
    check(parsed["entries"].size() == 3 &&
              lookup_native_model_directory(parsed, u"Model")["model_id"] == 8,
          "directory order and duplicate names are retained with first eligible match");
    parsed = parse_native_model_directory(wire({entry(1, 0, u"other"), entry(2, 0, u"Model")}));
    check(lookup_native_model_directory(parsed, u"Model")["status"] == "not_evaluated",
          "later exact match cannot bypass earlier locale-dependent comparison");
    auto ascii_equal = [](std::u16string a, std::u16string b) {
        for (auto *s : {&a, &b})
            for (auto &c : *s)
                if (c >= u'A' && c <= u'Z')
                    c += u'a' - u'A';
        return a == b;
    };
    check(lookup_native_model_directory(parsed, u"model", ascii_equal)["model_id"] == 2 &&
              lookup_native_model_directory(parsed, u"absent", ascii_equal)["status"] == "missing",
          "explicit comparison context supports ordered case-insensitive search and miss");
    check(lookup_native_model_directory(parsed, std::u16string(u"Model\0ignored", 13),
                                        ascii_equal)["model_id"] == 2,
          "query uses native wide null termination");
    parsed = parse_native_model_directory(wire({entry(1, 0, u"")}));
    check(parsed["entries"][0]["name"]["status"] == "not_evaluated" &&
              lookup_native_model_directory(parsed, u"Default", ascii_equal)["status"] ==
                  "not_evaluated",
          "zero stored name requires default or file-name fallback context");
    parsed = parse_native_model_directory(wire({entry(1, 0, u"")}), 1);
    check(lookup_native_model_directory(parsed, u"Default")["model_id"] == 1,
          "known default model ID supplies native Default fallback name");
    parsed = parse_native_model_directory(wire({entry(1, 0, std::u16string(1, 0))}), 1);
    check(parsed["entries"][0]["name"]["text"] == "" &&
              lookup_native_model_directory(parsed, u"")["model_id"] == 1,
          "present null name remains empty and differs from missing stored name");
    auto odd = entry(1, 0, u"AB", u"CD");
    put(odd, 18, 5, 2);
    parsed = parse_native_model_directory(wire({odd}));
    check(parsed["entries"][0]["name"]["text"] == "AB" &&
              parsed["entries"][0]["secondary_string"]["text"] == "CD" &&
              bytesof(parsed["trailing_storage"]).empty(),
          "odd directory string length consumes whole units without an extra-byte skip");
    check(parse_native_model_directory(wire({entry(1, 0, std::u16string(259, u'x'))}))["status"] ==
                  "decoded" &&
              parse_native_model_directory(
                  wire({entry(1, 0, std::u16string(260, u'x'))}))["status"] == "not_evaluated",
          "native 520-byte string buffer rejects at 260 code units");
    parsed = parse_native_model_directory(wire({entry(1, 0, std::u16string(1, char16_t(0xd800)))}));
    check(parsed["entries"][0]["name"]["text_status"] == "invalid_utf16" &&
              lookup_native_model_directory(parsed,
                                            std::u16string(1, char16_t(0xd800)))["model_id"] == 1 &&
              Json::parse(parsed.dump()) == parsed,
          "directory lookup can retain and compare raw surrogate units without invalid JSON");
    Bytes mask(12);
    put(mask, 0, 17, 4);
    put(mask, 4, 2, 4);
    put(mask, 8, 0x8001, 2);
    put(mask, 10, 0xabcd, 2);
    parsed = parse_native_model_directory(wire({entry(1, 0, u"A", u"", mask)}));
    check(parsed["entries"][0]["bitmap"]["effective_bit_count"] == 32 &&
              parsed["entries"][0]["bitmap"]["default_bit"] == true &&
              parsed["entries"][0]["bitmap"]["packed_words"] == Json::array({0x8001, 0xabcd}),
          "directory bitmap loader rounds effective capacity rather than masking source tail bits");
    mask.push_back(0);
    parsed = parse_native_model_directory(wire({entry(1, 0, u"A", u"", mask)}));
    check(parsed["native_input_status"] == "decoded" &&
              parsed["entries"][0]["bitmap"]["status"] == "rejected",
          "rejected bitmap becomes null without rejecting directory entry");
    parsed = parse_native_model_directory(wire({entry(1, 0, u"A", u"", Bytes{1})}));
    check(parsed["native_input_status"] == "not_evaluated" &&
              lookup_native_model_directory(parsed, u"A")["status"] == "not_evaluated",
          "unsafe native bitmap header cannot establish completed native input");
    auto v5 = entry(2, 0, u"A");
    v5.resize(v5.size() + 4);
    parsed = parse_native_model_directory(wire({v5}, 5));
    check(parsed["native_input_status"] == "decoded" &&
              parsed["entries"][0]["extension_map_status"] == "empty",
          "version five empty extension map consumes its count");
    put(v5, v5.size() - 4, 1, 4);
    const auto ex = v5.size();
    v5.resize(ex + 15);
    put(v5, ex, 7, 4);
    put(v5, ex + 4, 3, 8);
    put(v5, ex + 12, 0x030201, 3);
    parsed = parse_native_model_directory(wire({v5}, 5));
    check(parsed["status"] == "decoded" && parsed["native_input_status"] == "not_evaluated" &&
              bytesof(parsed["entries"][0]["extensions"][0]["data"]) == Bytes({1, 2, 3}),
          "nonempty version five extension wire storage does not imply reconstructed runtime map");
    check(parse_native_model_directory(wire({}, 3))["status"] == "not_evaluated",
          "native rejects directory versions below four for fallback rebuilding");
    auto truncated = wire({entry(1, 0, u"A")});
    truncated.pop_back();
    check(parse_native_model_directory(truncated)["status"] == "not_evaluated",
          "truncated string does not publish an accepted entry");
    auto trailing = wire({entry(1, 0, u"A")});
    trailing.push_back(0xef);
    parsed = parse_native_model_directory(trailing);
    check(parsed["status"] == "decoded" && bytesof(parsed["trailing_storage"]) == Bytes({0xef}),
          "native record count ends directory reading while preserving unconsumed storage");
    return checks;
}
