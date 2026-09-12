#include "internal.hpp"

unsigned native_list_input_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto put = [](Bytes &b, std::size_t at, std::uint32_t value, unsigned length) {
        for (unsigned i = 0; i < length; ++i)
            b.at(at + i) = static_cast<std::uint8_t>(value >> (8 * i));
    };
    auto make = [&](unsigned type, unsigned flags, unsigned words, unsigned base_words,
                    unsigned subtype = 0, bool dimension = false) {
        Bytes b(128, 0);
        put(b, 4, type, 2);
        put(b, 6, flags, 2);
        put(b, 8, words, 4);
        put(b, 12, base_words, 4);
        put(b, 16, subtype, 4);
        put(b, 36, dimension ? 0x1000 : 0, 2);
        return Json{{"element_type", type}, {"element_flags", flags}, {"data", rawbytes(b)}};
    };
    // Expected byte minima from the native branch table, independent of how
    // the C++ implementation groups switch cases. Default subtype is zero.
    const unsigned minimum[] = {
        40,  276, 16,  368, 60,  128, 16,  52,  112, 112, 168, 192, 112, 56,  120, 164, 120, 120,
        114, 114, 144, 16,  256, 52,  16,  120, 120, 136, 144, 16,  128, 128, 16,  16,  128, 16,
        16,  16,  40,  16,  116, 208, 160, 128, 170, 40,  56,  128, 32,  316, 352, 16,  256, 288};
    const unsigned delta[] = {0,  0,  36, 0,  0, 0, 36, 0, 0,  0,  32, 64, 0, 0, 0,  0,  0, 0,
                              0,  0,  0,  36, 0, 0, 36, 0, 0,  16, 32, 36, 8, 8, 36, 36, 8, 36,
                              36, 36, 0,  0,  0, 0, 32, 8, 32, 0,  0,  8,  0, 0, 0,  36, 0, 0};
    static_assert(std::size(minimum) == 54 && std::size(delta) == 54);
    for (unsigned type = 10; type <= 63; ++type) {
        auto n = make(type, 0, 1000, 16);
        auto h = native_list_record_header(n, Json(), false, type <= 32, 0, true);
        check(h["validation"]["minimum_base_bytes"] == minimum[type - 10] &&
                  h["output_base_word_count"] == std::max(16u, minimum[type - 10] / 2),
              "native type minimum updates only insufficient base lengths");
        n = make(type, 0x20, 1000, 16, 0, true);
        h = native_list_record_header(n, Json(), false, type <= 32, 0, true);
        check(h["validation"]["minimum_base_bytes"] == minimum[type - 10] + delta[type - 10],
              "extended dimensional and default branches use the native minima");
    }
    const unsigned table_minima[] = {64, 40, 40, 40, 64, 48, 40, 40, 48, 40};
    const unsigned entry_minima[] = {0, 46, 234, 0, 784, 320, 96, 204, 40, 104};
    for (unsigned subtype = 1; subtype <= 10; ++subtype) {
        check(native_list_record_header(make(10, 0, 1000, 16, subtype), Json(), false, true, 0,
                                        true)["validation"]["minimum_base_bytes"] ==
                  table_minima[subtype - 1],
              "table subtype dispatch retains its individual minimum");
        check(native_list_record_header(make(49, 0, 1000, 16, subtype), Json(), false, false, 0,
                                        true)["validation"]["minimum_base_bytes"] ==
                  entry_minima[subtype - 1],
              "entry subtype dispatch includes explicit zero minima");
    }
    auto n = make(37, 0x48, 16, 16);
    auto h = native_list_record_header(n, Json(), false, false, 0, true);
    check(
        h["validation"]["return_code"] == 0x11016 &&
            h["validation"]["return_code_ignored_by_file_loader"] == true &&
            h["output_base_word_count"] == 68 && h["output_element_flags"] == 0,
        "length failure is reported while the file-input path keeps base-length and flag updates");
    h = native_list_record_header(make(49, 0, 16, 17, 1), Json(), false, false, 0, true);
    check(h["validation"]["return_code"] == 0x11017 && h["output_base_word_count"] == 17,
          "existing base length above total length returns the separate base-length error");
    h = native_list_record_header(make(0, 0x48, 16, 16), Json(), false, false, 0, true);
    check(h["validation"]["return_code"] == 0x11002 && h["output_element_flags"] == 0x40,
          "type-zero early return is distinct from the file input filter");
    h = native_list_record_header(make(37, 0, 65536, 16), Json(), false, false, 0, true);
    check(h["validation"]["return_code"] == 0x11015 && h["output_base_word_count"] == 16,
          "word-limit failure precedes base-length mutation");
    h = native_list_record_header(make(24, 0x20, 80, 80),
                                  Json{{"output_element_flags", 0},
                                       {"output_record_word_count", 44},
                                       {"output_base_word_count", 44}},
                                  true, true, 2, true);
    check(h["output_element_flags"] == 0xc0 && h["output_base_word_count"] == 60 &&
              h["validation"]["return_code"] == 0x11016 &&
              h["descendant_count_update"]["header_offset"] == 32 &&
              h["descendant_count_update"]["output_value"] == 2,
          "type-24 converted word counts and flags feed list preparation before minimum checks");
    h = native_list_record_header(make(70, 0x60, 53, 53), Json(), true, true, 5, true);
    check(h["descendant_count_update"]["written"] == false && h["output_element_flags"] == 0xa0,
          "a count that cannot fit clears the compound flag without dropping source ancestry");
    h = native_list_record_header(make(70, 0x40, 18, 18), Json(), false, true, 0x80000000u, true);
    check(h["descendant_count_update"]["written"] == true &&
              h["descendant_count_update"]["output_value"] == 0,
          "descendant write uses the native signed-positive count rule at the exact size boundary");
    h = native_list_record_header(make(37, 0x20, 1000, 80), Json(), false, false, 0, false);
    check(h["validation"]["return_code"].is_null() &&
              h["validation"]["dimension_match"] == "requires_model_context",
          "model dimensional validation is not guessed without its model state");
    h = native_list_record_header(make(54, 0x20, 1000, 100), Json(), false, false, 0, false);
    check(h["validation"]["return_code"] == 0,
          "type-54 dimensional exemption applies outside system containers too");
    return checks;
}
