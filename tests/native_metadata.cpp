#include "internal.hpp"
#include <p3d/native_metadata.hpp>

unsigned native_metadata_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto storage = [](std::uint64_t bits) {
        Bytes out(8);
        for (unsigned i = 0; i < 8; ++i)
            out[i] = std::uint8_t(bits >> (8 * i));
        return out;
    };
    for (const auto bits :
         {0ULL, 0x8000000000000000ULL, 0x3ff4000000000000ULL, 0xbff4000000000000ULL, 1ULL,
          0x7fefffffffffffffULL, 0x7ff0000000000000ULL, 0xfff0000000000000ULL,
          0x7ff8000000001234ULL, 0xfff8000000005678ULL}) {
        const auto bytes = storage(bits);
        const auto copy = bytes;
        const auto value = decode_native_modification_time(bytes);
        const auto finite = (bits & 0x7ff0000000000000ULL) != 0x7ff0000000000000ULL;
        check(value.at("status") == "decoded" && value.at("finite") == finite &&
                  value.at("time_basis") == "local" && value.at("epoch") == "1970-01-01T00:00:00" &&
                  value.at("source_offset") == 0 && bytesof(value.at("source_storage")) == copy &&
                  bytes == copy,
              "modification time keeps its local basis and every source bit");
        if (finite) {
            const auto number = value.at("milliseconds").get<double>();
            std::uint64_t roundtrip;
            std::memcpy(&roundtrip, &number, 8);
            check(roundtrip == bits, "finite timestamp keeps fraction, sign and subnormal bits");
        } else {
            check(value.at("milliseconds").is_null(),
                  "nonfinite time is null without loss of source bits");
        }
    }
    for (unsigned n = 0; n < 20; ++n)
        if (n != 8)
            check(decode_native_modification_time(Bytes(n)).at("status") == "not_evaluated",
                  "timestamp decoder rejects incomplete or oversized scalar storage");
    auto put = [](Bytes &out, std::size_t at, auto value) {
        std::memcpy(out.data() + at, &value, sizeof(value));
    };
    Bytes stream;
    std::vector<Bytes> expected;
    std::vector<std::size_t> offsets;
    for (const auto type : {0u, 1u, 33u, 63u, 127u}) {
        for (const auto flags : {0u, 8u, 0x20u, 0x80u}) {
            Bytes record((flags & 0x20) ? 108 : 36);
            put(record, 0, std::uint32_t(type == 0 ? 3 : 0));
            put(record, 4, std::uint16_t(type));
            put(record, 6, std::uint16_t(flags));
            put(record, 8, std::uint32_t((record.size() - 4) / 2));
            put(record, 12, std::uint32_t((record.size() - 4) / 2));
            put(record, 20, UINT64_MAX);
            put(record, 28, 1700000000000.25 + type + flags);
            offsets.push_back(stream.size());
            expected.push_back(slice(record, 28, 8));
            stream.insert(stream.end(), record.begin(), record.end());
        }
    }
    const auto original = stream;
    const auto records = parse_native(stream);
    check(records.size() == expected.size() && stream == original,
          "common header metadata does not filter or rewrite saved records");
    for (std::size_t i = 0; i < records.size(); ++i) {
        const auto &record = records[i], &time = record.at("modification_time");
        check(record.at("offset") == offsets[i] && record.at("id") == UINT64_MAX &&
                  time.at("source_offset") == 28 &&
                  time.at("source_offsets_include_stream_prefix") == true &&
                  bytesof(time.at("source_storage")) == expected[i] &&
                  time.at("milliseconds") == Reader(expected[i]).f64(),
              "record-relative timestamp is independent of stream offset, extended header and "
              "input filter");
    }
    auto nan_record = slice(stream, offsets[4], 36);
    const auto nan = storage(0x7ff8000000004321ULL);
    std::copy(nan.begin(), nan.end(), nan_record.begin() + 28);
    const auto parsed = parse_native(nan_record);
    check(parsed[0].at("modification_time").at("milliseconds").is_null() &&
              bytesof(parsed[0].at("modification_time").at("source_storage")) == nan,
          "record adapter retains nonfinite timestamp payloads");
    return checks;
}
