#include "internal.hpp"
#include <p3d/native_input.hpp>
#include <future>

unsigned native_record_preparation_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        require(value, why);
        ++checks;
    };
    auto put = [](Bytes &b, std::size_t p, auto value) {
        require(p <= b.size() && sizeof(value) <= b.size() - p, "preparation fixture bounds");
        std::memcpy(b.data() + p, &value, sizeof(value));
    };
    auto make = [&](unsigned type, unsigned flags, std::size_t base_size, bool links = true) {
        Bytes b(base_size + (links ? 8 : 0), 0);
        put(b, 4, std::uint16_t(type));
        put(b, 6, std::uint16_t(flags));
        put(b, 8, std::uint32_t((b.size() - 4) / 2));
        put(b, 12, std::uint32_t((base_size - 4) / 2));
        put(b, 20, std::uint64_t(17));
        if (links) {
            put(b, base_size, std::uint16_t(0));
            put(b, base_size + 2, std::uint16_t(0x1234));
            put(b, base_size + 4, std::uint32_t(0xdeadbeef));
        }
        return b;
    };
    const std::array<std::uint64_t, 4> ids{{0, 1, 0x100000000ull, UINT64_MAX}};
    for (auto flags : {0u, 0x20u, 0x40u, 0x60u, 0x80u, 0xc0u, 0x200u, 0xa0u})
        for (bool child : {false, true})
            for (bool system : {false, true})
                for (auto id : ids) {
                    auto source = make(37, flags, 156);
                    put(source, 36, std::uint16_t(0x1000));
                    auto expected = source;
                    put(expected, 6, std::uint16_t((flags & ~0xc8u) | (child ? 0x80u : 0u)));
                    put(expected, 20, id);
                    const auto j =
                        prepare_native_record_input(source, id, UINT32_MAX, child, system);
                    check(j.at("status") == "prepared" && j.at("decoded_status") == "decoded" &&
                              bytesof(j.at("prepared_record")) == expected &&
                              bytesof(j.at("source_record")) == source &&
                              j.at("decoded_record").at("id") == id &&
                              j.at("decoded_record").at("links").size() == 1 &&
                              bytesof(j.at("decoded_record").at("links")[0].at("payload")) ==
                                  slice(source, 160, 4),
                          "leaf preparation changes only native input flags and assigned ID "
                          "including full unsigned boundaries");
                }
    for (bool extended : {false, true})
        for (bool child : {false, true})
            for (auto count : {0u, 3u, 0x7fffffffu, 0x80000000u, UINT32_MAX}) {
                auto source = make(24, extended ? 0xa0 : 0x80, extended ? 200 : 128);
                for (std::size_t p = 36; p < source.size() - 8; ++p)
                    source[p] = std::uint8_t(p * 17);
                put(source, extended ? 108 : 36, std::uint32_t(123));
                Bytes expected(source.begin(), source.begin() + 36);
                expected.insert(expected.end(), source.begin() + (extended ? 108 : 36),
                                source.end());
                put(expected, 6, std::uint16_t(0x40 | (child ? 0x80 : 0)));
                put(expected, 8, std::uint32_t((expected.size() - 4) / 2));
                put(expected, 12, std::uint32_t(62));
                put(expected, 20, UINT64_MAX);
                put(expected, 36, count > 0x7fffffff ? 0u : count);
                const auto j = prepare_native_record_input(source, UINT64_MAX, count, child, true);
                check(j.at("status") == "prepared" &&
                          bytesof(j.at("prepared_record")) == expected &&
                          bytesof(j.at("source_record")) == source &&
                          j.at("decoded_status") == "decoded" &&
                          bytesof(j.at("decoded_record").at("links")[0].at("payload")) ==
                              slice(source, source.size() - 4, 4),
                      "type24 removes exactly the extended region shifts complete linkage tail and "
                      "updates accepted count independently of saved count");
            }
    for (bool extended : {false, true}) {
        auto source = make(13, extended ? 0x20 : 0, 380);
        for (std::size_t p = 36; p < 380; ++p)
            source[p] = std::uint8_t(p * 23);
        put(source, 160, std::uint32_t(0));
        put(source, 342, std::uint16_t(0));
        put(source, 344, std::uint16_t(0));
        put(source, 346, std::uint16_t(2));
        put(source, 64, std::uint16_t(0x1234));
        const auto j = prepare_native_record_input(source, 0x1234567800000000ull, 5, false, true);
        const auto prepared = bytesof(j.at("prepared_record"));
        bool valid = j.at("status") == "prepared" && prepared.size() == source.size() + 24 &&
                     j.at("decoded_status") == "decoded" &&
                     j.at("source_topology").at("conversion").at("applied") == true &&
                     Reader(prepared).at<std::uint32_t>(8) == 204 &&
                     Reader(prepared).at<std::uint32_t>(12) == 200 &&
                     Reader(prepared).at<std::uint32_t>(extended ? 108 : 36) == 5 &&
                     Reader(prepared).at<std::uint64_t>(20) == 0x1234567800000000ull &&
                     slice(prepared, 372, 32) == slice(source, 348, 32) &&
                     slice(prepared, 404, 8) == slice(source, 380, 8) &&
                     bytesof(j.at("source_record")) == source;
        for (unsigned i = 0; i < 8; ++i)
            if (!(extended && i == 3))
                valid = valid && Reader(prepared).at<std::uint16_t>(72 + i * 12) == 0x1234 &&
                        Reader(prepared).at<std::uint16_t>(74 + i * 12) == 0;
        check(valid, "legacy reference upgrade retains entries and every linkage byte while fresh "
                     "header count and ID are applied after layout migration");
    }
    const std::array<Matrix3, 3> matrices{{{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}},
                                           {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}},
                                           {{{2, 0, 0}, {0, 3, 0}, {0, 0, 0}}}}};
    const std::array<Matrix3, 3> expected_matrices{{{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}},
                                                    {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}},
                                                    {{{2, 0, 0}, {0, 3, 0}, {0, 0, 1}}}}};
    const std::array<std::uint64_t, 4> translations{{0x7ff8000000001234ull, 0x7ff0000000000000ull,
                                                     0x8000000000000000ull, 0x4004000000000000ull}};
    for (std::size_t m = 0; m < matrices.size(); ++m)
        for (auto translation_bits : translations) {
            auto source = make(62, 0, 260);
            for (unsigned row = 0; row < 3; ++row)
                for (unsigned column = 0; column < 3; ++column)
                    put(source, 164 + (row * 3 + column) * 8, matrices[m][row][column]);
            for (unsigned a = 0; a < 3; ++a)
                put(source, 236 + a * 8, translation_bits);
            const auto j = prepare_native_record_input(source, 99, 0, false, true);
            const auto prepared = bytesof(j.at("prepared_record"));
            bool valid = j.at("status") == "prepared" &&
                         slice(prepared, 236, source.size() - 236) ==
                             slice(source, 236, source.size() - 236) &&
                         j.at("conversion").at("matrix_input").at("matrix") == expected_matrices[m];
            for (unsigned row = 0; row < 3; ++row)
                for (unsigned column = 0; column < 3; ++column)
                    valid = valid && Reader(prepared).at<double>(164 + (row * 3 + column) * 8) ==
                                         expected_matrices[m][row][column];
            check(valid, "matrix-only input repairs known rank cases without reading replacing or "
                         "rejecting unconsumed nonfinite translation");
        }
    auto source = make(62, 0, 236, false);
    auto j = prepare_native_record_input(source, 5, 0, false, false);
    check(j.at("status") == "prepared" && j.at("decoded_status") == "not_decoded" &&
              bytesof(j.at("prepared_record")).size() == 236 &&
              Reader(bytesof(j.at("prepared_record"))).at<std::uint32_t>(12) == 128 &&
              j.at("header_preparation").at("validation").at("return_code") == 0x11016,
          "ignored minimum-base validation retains exact short buffer rather than inventing "
          "missing translation bytes");
    source = make(37, 0, 156, false);
    put(source, 12, std::uint32_t(1));
    check(prepare_native_record_input(source, 0, 0, false, true).at("status") == "not_evaluated",
          "an accepted record with invalid base has no prepared buffer");
    put(source, 6, std::uint16_t(8));
    check(prepare_native_record_input(source, 0, 0, false, true).at("status") == "skipped",
          "physical skip occurs before the accepted-record base check");
    source = make(37, 0, 156, false);
    put(source, 0, std::uint32_t(1));
    check(prepare_native_record_input(source, 0, 0, false, true).at("status") == "skipped",
          "nonzero stream prefix is preserved and skipped");
    source.resize(140004, 0);
    put(source, 8, std::uint32_t(70000));
    check(prepare_native_record_input(source, 0, 0, false, true).at("status") == "read_error",
          "physical upper word bound precedes prefix skipping");
    source = make(37, 0, 156, false);
    source.pop_back();
    check(prepare_native_record_input(source, 0, 0, false, true).at("prepared_record").is_null(),
          "partial physical record is not completed with zeros");
    source = make(13, 0, 380, false);
    source.resize(131074, 0);
    put(source, 8, std::uint32_t(65535));
    put(source, 346, std::uint16_t(2));
    j = prepare_native_record_input(source, 7, 0, false, true);
    check(j.at("status") == "prepared" && bytesof(j.at("prepared_record")).size() == 131098 &&
              j.at("native_descriptor_copy_bytes") == 131070 &&
              j.at("header_preparation").at("validation").at("return_code") == 0x11015,
          "upgraded record length and later bounded descriptor copy remain distinct");
    source = make(62, 0, 260, false);
    put(source, 164, std::uint64_t(0x7ff8000000004321ull));
    j = prepare_native_record_input(source, 1, 0, false, true);
    check(j.at("status") == "not_evaluated" && j.at("prepared_record").is_null() &&
              bytesof(j.at("source_record")) == source,
          "unknown nonfinite matrix arithmetic does not become a default usable transform");
    source = make(24, 0x20, 200);
    const auto original = source;
    const auto expected = prepare_native_record_input(source, UINT64_MAX, 3, true, true);
    std::array<std::future<Json>, 4> tasks;
    for (auto &task : tasks)
        task = std::async(std::launch::async, [&] {
            return prepare_native_record_input(source, UINT64_MAX, 3, true, true);
        });
    for (auto &task : tasks)
        check(task.get() == expected, "record preparation is independent across threads");
    check(source == original, "preparation leaves input source storage unchanged");
    return checks;
}
