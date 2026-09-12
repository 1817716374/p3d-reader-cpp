#include "internal.hpp"

unsigned native_application_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto put = [](Bytes &b, std::size_t at, std::uint64_t v, unsigned n) {
        for (unsigned i = 0; i < n; ++i)
            b.at(at + i) = std::uint8_t(v >> (8 * i));
    };
    auto record = [&](unsigned signature, std::size_t n) {
        Bytes b(n);
        put(b, 4, 47, 2);
        put(b, 16, 20, 4);
        put(b, 8, (n - 4) / 2, 4);
        put(b, 12, (n - 4) / 2, 4);
        put(b, 36, signature, 2);
        return b;
    };
    auto b = record(0x56df, 52);
    put(b, 38, 7, 2);
    put(b, 40, 4, 2);
    for (std::size_t i = 42; i < b.size(); ++i)
        b[i] = std::uint8_t(i);
    auto d = native_application_record(b);
    check(d["status"] == "partial" && d["version"] == 7 && d["declared_storage_length"] == 4,
          "versioned application header is decoded without claiming payload semantics");
    check(bytesof(d["decoder_input"]["data"]) == slice(b, 42, 10),
          "resource decoder bound is native base size, not the declared storage length");
    check(d["conversion"]["attempt_versions"] == Json::array({7, 6}) &&
              d["conversion"]["status"] == "not_evaluated",
          "input tries current then immediately previous version without claiming success");
    for (unsigned v = 0; v < 8; ++v) {
        put(b, 38, v, 2);
        d = native_application_record(b);
        Json expected = Json::array({v});
        if (v)
            expected.push_back(v - 1);
        check(d["conversion"]["attempt_versions"] == expected,
              "version fallback stops after one predecessor");
    }
    for (unsigned v : {8u, 32767u, 32768u, 65535u}) {
        put(b, 38, v, 2);
        d = native_application_record(b);
        check(d["status"] == "unsupported" && d["conversion"]["attempt_versions"].empty() &&
                  d["conversion"]["dispatch"] == "initialize_defaults",
              "out of range and negative versions do not index the converter table");
    }
    put(b, 38, 7, 2);
    put(b, 40, 65535, 2);
    d = native_application_record(b);
    check(d["declared_storage_length"] == -1 && bytesof(d["decoder_input"]["data"]).size() == 10,
          "signed adapter length is kept independently of resource input bounds");
    for (std::size_t n = 0; n < 42; ++n)
        check(native_application_record(slice(b, 0, n))["status"] == "invalid",
              "truncated header is rejected");
    for (unsigned words : {0u, 18u, 25u, UINT32_MAX}) {
        auto bad = b;
        put(bad, 12, words, 4);
        check(native_application_record(bad)["status"] == "invalid",
              "native base bound is checked without overflow");
    }
    auto extra = b;
    extra.insert(extra.end(), {1, 2});
    d = native_application_record(extra);
    check(bytesof(d["decoder_input"]["data"]).size() == 10 &&
              bytesof(d["outside_native_base"]["data"]) == Bytes({1, 2}),
          "bytes outside the native header bound are not included in decoder input");
    auto marker = record(0x5704, 44);
    marker[41] = 9;
    d = native_application_record(marker);
    check(d["role"] == "owner_application_marker" && d["status"] == "partial" &&
              bytesof(d["unassigned_ranges"][0]["data"]) == slice(marker, 38, 6),
          "owner marker retains unassigned bytes and is not treated as an empty property set");
    check(native_application_record(slice(marker, 0, 42))["status"] == "invalid",
          "marker requires its complete header");
    for (unsigned sig : {0x56dfu, 0x5704u, 0x56e7u}) {
        auto raw = sig == 0x5704 ? marker : b;
        put(raw, 36, sig, 2);
        auto rows = parse_native(raw);
        check(rows.size() == 1 && bytesof(rows[0]["data"]) == raw && rows[0]["children"].empty() &&
                  rows[0].contains("application_record") == (sig != 0x56e7),
              "record integration preserves bytes and selects only the two application signatures");
    }
    return checks;
}
