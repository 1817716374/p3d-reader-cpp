#include "internal.hpp"
#include "lz4/lz4.h"

unsigned material_auxiliary_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto put = [](Bytes &b, std::size_t offset, auto value) {
        require(offset + sizeof(value) <= b.size(), "auxiliary fixture bounds");
        std::memcpy(b.data() + offset, &value, sizeof(value));
    };
    auto packet = [&](std::uint16_t word, std::uint8_t byte, const Bytes &body) {
        Bytes b(64 + body.size(), 0);
        put(b, 0, std::uint32_t(b.size()));
        put(b, 4, std::uint32_t(2));
        put(b, 8, std::uint32_t(100));
        put(b, 12, std::uint32_t(200));
        put(b, 16, word);
        put(b, 18, std::uint8_t(11));
        put(b, 19, byte);
        put(b, 20, std::uint16_t(300));
        put(b, 22, std::uint16_t(0xabcd));
        put(b, 24, 1.25);
        put(b, 32, -2.5);
        put(b, 40, std::uint32_t(0xfedcba98));
        b[63] = 123;
        std::copy(body.begin(), body.end(), b.begin() + 64);
        return b;
    };
    auto join = [](Bytes &a, const Bytes &b) { a.insert(a.end(), b.begin(), b.end()); };
    auto a = packet(65535, 255, {1, 2, 3});
    auto r = decode_material_auxiliary_records(a);
    const auto &first = r["records"][0];
    check(r["status"] == "resolved" && r["consumed_bytes"] == a.size() && r["remaining_bytes"] == 0,
          "material auxiliary packet consumes its declared size");
    check(first["key"] == Json({{"byte_19", 255}, {"word_16", 65535}}) &&
              bytesof(first["data"]) == Bytes({1, 2, 3}),
          "material auxiliary identity retains unsigned byte/word and exact body");
    check(first["unassigned_fields"]["u32_8"] == 100 &&
              first["unassigned_fields"]["u32_12"] == 200 &&
              first["unassigned_fields"]["u8_18"] == 11 &&
              first["unassigned_fields"]["u16_20"] == 300 &&
              first["unassigned_fields"]["u32_40"] == 0xfedcba98u &&
              first["unassigned_fields"]["f64_24"] == 1.25 &&
              first["unassigned_fields"]["f64_32"] == -2.5,
          "native-copied auxiliary fields retain widths without invented business names");
    check(bytesof(first["header"]) == slice(a, 0, 64) &&
              first["ignored_header_fields"]["u16_22"] == 0xabcd &&
              bytesof(first["ignored_header_fields"]["bytes_44_63"])[19] == 123,
          "unused header bytes remain available without requiring native writer zeros");
    join(a, packet(9, 1, {4}));
    join(a, packet(2, 1, {5}));
    auto replacement = packet(65535, 255, {});
    replacement[18] = 77;
    join(a, replacement);
    r = decode_material_auxiliary_records(a);
    check(r["selected_record_indices"] == Json::array({2, 1, 3}),
          "effective auxiliary entries sort by byte first and unsigned word second");
    check(r["records"][3]["action"] == "replaced" &&
              r["records"][3]["replaced_record_index"] == 0 &&
              bytesof(r["records"][3]["data"]).empty(),
          "duplicate key fully replaces earlier data including replacement with empty body");
    check(r["records"][0]["offset"] == 0 && r["records"][1]["offset"] == 67 &&
              r["records"].size() == 4,
          "source order and prior duplicate records are not discarded");
    Bytes skipped(8);
    put(skipped, 0, std::uint32_t(8));
    put(skipped, 4, std::uint32_t(3));
    join(skipped, packet(8, 2, {9}));
    r = decode_material_auxiliary_records(skipped);
    check(r["status"] == "resolved" && r["records"][0]["action"] == "skipped_version" &&
              r["selected_record_indices"] == Json::array({1}),
          "unsupported record versions skip their declared bytes without ending input");
    put(skipped, 0, std::uint32_t(1000));
    r = decode_material_auxiliary_records(skipped);
    check(r["status"] == "resolved" && r["native_cursor"] == 1000 &&
              r["consumed_bytes"] == skipped.size() && r["remaining_bytes"] == 0 &&
              r["selected_record_indices"].empty() && r["records"].size() == 1,
          "skipped version can advance past the end without interpreting its body or next packet");
    auto damaged = packet(1, 0, {3});
    join(damaged, Bytes(7));
    r = decode_material_auxiliary_records(damaged);
    check(r["status"] == "partial" && r["selected_record_indices"] == Json::array({0}) &&
              r["consumed_bytes"] == 65 && r["remaining_bytes"] == 7,
          "truncated header retains confirmed prefix and unconsumed tail");
    for (auto size : {0u, 63u, 1000u}) {
        auto bad = packet(1, 0, {});
        put(bad, 0, std::uint32_t(size));
        r = decode_material_auxiliary_records(bad);
        check(r["status"] == "partial" && r["selected_record_indices"].empty() &&
                  r["consumed_bytes"] == 0,
              "unsafe auxiliary frame never becomes a registered value");
    }
    auto nan = packet(1, 0, {});
    put(nan, 24, std::uint64_t(0x7ff8000000000042));
    r = decode_material_auxiliary_records(nan);
    check(r["records"][0]["unassigned_fields"]["f64_24"].is_null() &&
              bytesof(r["records"][0]["header"]) == nan,
          "nonfinite numeric bits survive in the original header");
    check(decode_material_auxiliary_records({})["status"] == "resolved",
          "empty decoded auxiliary collection has no records");
    auto envelope = [&](const Bytes &raw, unsigned version) {
        Bytes b(8);
        put(b, 0, std::uint32_t(version));
        put(b, 4, std::uint32_t(raw.size()));
        if (version != 3)
            join(b, raw);
        else {
            Bytes compressed(LZ4_compressBound(int(raw.size())));
            auto n = LZ4_compress_default(reinterpret_cast<const char *>(raw.data()),
                                          reinterpret_cast<char *>(compressed.data()),
                                          int(raw.size()), int(compressed.size()));
            require(n > 0, "auxiliary fixture compression");
            compressed.resize(n);
            join(b, compressed);
        }
        return b;
    };
    for (auto version : {1u, 2u, 3u}) {
        r = decode_attribute(0, 22906, envelope(a, version), 0);
        check(r["encoding"] == "material_auxiliary_records" && bytesof(r["data"]) == a &&
                  r["native_input"]["selected_record_indices"] == Json::array({2, 1, 3}),
              "stored and compressed attribute paths expose auxiliary structure");
    }
    r = decode_attribute(0, 20091, envelope({1, 2, 3}, 1), 0);
    check(r["encoding"] == "material_auxiliary_buffer" && r["content_semantics"] == "unresolved" &&
              bytesof(r["data"]) == Bytes({1, 2, 3}),
          "ancillary material buffer remains explicit without guessed image semantics");
    check(decode_attribute(1, 22906, envelope(a, 1), 0)["encoding"] == "uncompressed_binary",
          "material-specific group zero reader does not capture unrelated groups");
    return checks;
}
