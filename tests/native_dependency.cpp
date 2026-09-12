#include "internal.hpp"
#include <cmath>
#include <cstring>

unsigned native_dependency_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto put = [](Bytes &b, std::size_t at, std::uint64_t value, unsigned n) {
        for (unsigned i = 0; i < n; ++i)
            b.at(at + i) = std::uint8_t(value >> (8 * i));
    };
    auto make = [&](unsigned format, unsigned count, std::size_t size) {
        Bytes b(size, 0);
        put(b, 0, 12000, 2);
        put(b, 2, 19, 2);
        put(b, 4, format << 10, 2);
        put(b, 6, count, 2);
        return b;
    };
    const std::uint64_t high = 0xfedcba9876543210ull;
    auto b = make(0, 3, 32);
    put(b, 8, high, 8);
    put(b, 16, 0, 8);
    put(b, 24, ~std::uint64_t(0), 8);
    auto d = native_dependency_link(b);
    check(d["status"] == "decoded" && d["entries"][0]["references"][0]["element_id"] == high &&
              d["entries"][1]["element_id"] == 0 &&
              d["entries"][2]["element_id"] == ~std::uint64_t(0),
          "generic dependency IDs preserve full width and sentinels without filtering");
    put(b, 0, 10000, 2);
    put(b, 2, 4, 2);
    d = native_dependency_link(b);
    check(d["lookup_rule"] == "reverse_owner_path" && d["entries"][0]["references"].empty(),
          "special dependency path must not become three independent local references");
    put(b, 4, 513, 2);
    d = native_dependency_link(b);
    check(d["dependency_index_input"] == "skipped_flag_1" && d["unassigned_flag_bits"] == 512 &&
              d["status"] == "partial" && d["entries"].size() == 3,
          "disabled dependency still exposes source data and unidentified flags");
    for (unsigned format : {1u, 4u, 5u, 8u}) {
        const unsigned stride = format == 5 ? 24 : 16;
        auto p = make(format, 2, 8 + 2 * stride);
        put(p, 8, high, 8);
        put(p, 8 + stride, 37, 8);
        if (format == 8) {
            put(p, 8, 0xffffffff, 4);
            put(p, 24, 0xfffffffe, 4);
        }
        if (format != 1) {
            put(p, 8 + stride - 8, 71, 8);
            put(p, 8 + 2 * stride - 8, 72, 8);
        }
        auto x = native_dependency_link(p);
        check(x["entries"].size() == 2 && x["entries"][1]["payload_offset"] == 8 + stride,
              "multiple dependency entries retain independent source offsets");
        if (format == 1)
            check(x["entries"][0]["element_id"] == high && x["status"] == "partial",
                  "format one preserves unassigned second word instead of inventing a reference");
        else if (format == 8)
            check(x["entries"][0]["references"][0]["lookup"] == "system_owner" &&
                      x["entries"][1]["references"][0]["model_index"] == -2 &&
                      x["entries"][1]["references"][0]["lookup"] == "model_index",
                  "only signed model index minus one selects the system owner");
        else
            check(x["entries"][0]["references"][0]["owner_reference_id"] == 71 &&
                      x["entries"][1]["references"][1]["element_id"] == 72,
                  "cross-owner dependency emits target then the local owner-reference dependency");
        for (std::size_t n = 0; n < p.size(); ++n)
            check(native_dependency_link(slice(p, 0, n))["status"] == "invalid",
                  "truncated fixed dependency entry must not expose a complete list");
    }
    for (unsigned format : {2u, 3u})
        for (unsigned kind : {0u, 2u, 8u, 255u}) {
            const unsigned stride = format == 2 ? 40 : 48;
            auto p = make(format, 2, 8 + 2 * stride);
            put(p, 8, kind, 1);
            put(p, 16, 10, 8);
            put(p, 24, 20, 8);
            put(p, 32, 30, 8);
            put(p, 40, 40, 8);
            put(p, 16 + stride, high, 8);
            auto x = native_dependency_link(p);
            const auto &refs = x["entries"][0]["references"];
            check(refs.size() == ((kind == 2 || kind == 8) ? 4 : 2) &&
                      refs[0]["owner_reference_id"] == ((kind == 2 || kind == 8) ? 30 : 20) &&
                      x["entries"][1]["references"][0]["element_id"] == high,
                  "selector two and eight dispatch two target-owner pairs; all other selectors "
                  "dispatch one");
        }
    b = make(6, 1, 48);
    put(b, 8, 3, 4);
    put(b, 24, 11, 8);
    put(b, 32, 22, 8);
    put(b, 40, 33, 8);
    d = native_dependency_link(b);
    check(d["owner_path"]["ids"] == Json::array({11, 22, 33}) &&
              d["owner_path"]["lookup_order"] == "reverse" && d["entries"].empty(),
          "variable owner path has its own count and is not a flat reference array");
    put(b, 8, 0xffffffff, 4);
    check(native_dependency_link(b)["status"] == "invalid", "variable path overflow is bounded");
    put(b, 8, 0, 4);
    check(native_dependency_link(b)["status"] == "invalid", "empty active owner path is invalid");
    check(native_dependency_link(make(6, 0, 8))["status"] == "decoded",
          "zero input iterations need no path");
    b = make(7, 2, 56);
    put(b, 8, 0xfedcba98, 4);
    put(b, 12, 0x76543210, 4);
    put(b, 16, 2, 1);
    put(b, 40, 4, 1);
    d = native_dependency_link(b);
    check(d["entries"][0]["element_id"] == high && d["entries"][0]["expanded_selector_code"] == 3 &&
              d["entries"][0]["references"][0]["lookup"] == "current_owner" &&
              d["entries"][1]["reference_status"] == "unsupported_compact_selector" &&
              d["entries"][1]["references"].empty(),
          "compact ID word order and unsupported selector are explicit");
    b = make(15, 1, 16);
    d = native_dependency_link(b);
    check(d["status"] == "invalid" && d["dependency_index_input"] == "skipped_reference_format" &&
              bytesof(d["unassigned_ranges"][0]["data"]) == slice(b, 8, 8),
          "unknown format retains its payload without dispatching an invalid handler");
    b = make(0, 1, 18);
    put(b, 8, high, 8);
    b[16] = 0xab;
    b[17] = 0xcd;
    d = native_dependency_link(b);
    check(d["status"] == "partial" && d["unassigned_ranges"][0]["payload_offset"] == 16 &&
              bytesof(d["unassigned_ranges"][0]["data"]) == Bytes({0xab, 0xcd}),
          "unknown suffix preserved");
    // Exercise actual record linkage dispatch, including duplicates and the legacy children view.
    auto p = make(0, 1, 16);
    put(p, 0, 10000, 2);
    put(p, 2, 17, 2);
    put(p, 4, 512, 2);
    put(p, 8, high, 8);
    Bytes record(36);
    put(record, 4, 1, 2);
    put(record, 12, 16, 4);
    for (unsigned i = 0; i < 2; ++i) {
        const auto start = record.size();
        record.resize(start + 20);
        put(record, start, 0x1009, 2);
        put(record, start + 2, 0x56d0, 2);
        std::copy(p.begin(), p.end(), record.begin() + start + 4);
    }
    put(record, 8, (record.size() - 4) / 2, 4);
    auto rows = parse_native(record);
    check(rows[0]["links"].size() == 2 && rows[0]["children"] == Json::array({high, high}) &&
              rows[0]["links"][1]["decoded"]["entries"][0]["element_id"] == high &&
              bytesof(rows[0]["links"][0]["payload"]) == p,
          "record integration preserves duplicates, original linkage bytes and established child "
          "extraction");
    // An eight-byte non-user linkage with the same app number is not this format.
    record.resize(44);
    put(record, 36, 0, 2);
    put(record, 38, 0x56d0, 2);
    put(record, 8, 20, 4);
    rows = parse_native(record);
    check(!rows[0]["links"][0].contains("decoded"), "dependency decoder requires a user linkage");
    // Compact selectors are normalized before being consumed by the native reader.
    // Check their complete descriptor bytes independently of the exposed mappings.
    for (unsigned kind = 0; kind < 4; ++kind) {
        auto p = make(7, 1, 32);
        put(p, 8, 0x89abcdef, 4);
        put(p, 12, 0x01234567, 4);
        put(p, 16, kind, 1);
        put(p, 20, 0x1122, 2);
        put(p, 22, 0x3344, 2);
        put(p, 24, 0x5566778899aabbccull, 8);
        auto x = native_dependency_link(p);
        const auto &sel = x["entries"][0]["expanded_selector"];
        Bytes expected(40, 0);
        const unsigned kinds[] = {1, 6, 3, 7};
        put(expected, 0, kinds[kind], 1);
        put(expected, 2, 0x1122, 2);
        put(expected, 8, 0x89abcdef01234567ull, 8);
        if (kind == 0) {
            put(expected, 24, 0x3344, 2);
            put(expected, 4, 0xbbcc, 2);
            put(expected, 6, 0x99aa, 2);
        }
        if (kind == 1)
            put(expected, 4, 0x3344, 2);
        if (kind == 1 || kind == 2)
            put(expected, 24, 0x5566778899aabbccull, 8);
        check(sel["status"] == "resolved" && bytesof(sel["descriptor"]) == expected,
              "compact parameter fields map to the exact expanded descriptor without guessing slot "
              "names");
        check(!sel.contains("normalization"),
              "nonzero selector parameter suppresses angle normalization");
        for (const auto &field : sel["field_mappings"])
            check(bytesof(field["source"]) == slice(p, 8 + field["entry_offset"].get<std::size_t>(),
                                                    field["byte_count"].get<std::size_t>()),
                  "compact parameter source bytes retain provenance");
    }
    constexpr double tau = 0x1.921fb54442d18p+2;
    for (double source : {-0.25, tau + 0.25, -0.0, -2 * tau, 1e100}) {
        auto p = make(7, 1, 32);
        put(p, 16, 2, 1);
        std::uint64_t bits;
        std::memcpy(&bits, &source, 8);
        put(p, 24, bits, 8);
        const auto sel = native_dependency_link(p)["entries"][0]["expanded_selector"];
        auto expanded = bytesof(sel["descriptor"]);
        double value = Reader(expanded, 24).f64();
        double expected = std::fmod(source, tau);
        if (expected < 0)
            expected += tau;
        check(sel["normalization"]["kind"] == "angle_modulo_two_pi" && value == expected &&
                  (value != 0 || std::signbit(value) == std::signbit(source)),
              "angle projection matches modulo and retains signed zero");
        check(bytesof(sel["field_mappings"].back()["source"]) == slice(p, 24, 8),
              "angle normalization does not overwrite the serialized angle");
    }
    for (auto bits : {0x7ff0000000000000ull, 0xfff0000000000000ull, 0x7ff8123456789abcull}) {
        auto p = make(7, 1, 32);
        put(p, 16, 2, 1);
        put(p, 24, bits, 8);
        auto x = native_dependency_link(p);
        auto sel = x["entries"][0]["expanded_selector"];
        check(sel["status"] == "invalid" && !sel.contains("descriptor") &&
                  Reader(bytesof(sel["field_mappings"].back()["source"])).u64() == bits,
              "nonfinite angles do not turn into a successful null-valued JSON projection");
        put(p, 16, 1, 1);
        x = native_dependency_link(p);
        sel = x["entries"][0]["expanded_selector"];
        check(sel["status"] == "resolved" && Reader(bytesof(sel["descriptor"]), 24).u64() == bits,
              "unmodified floating parameters preserve every bit including NaN payloads");
    }
    for (unsigned flags : {0u, 0x78u, 0xffffu, 0x1e01u}) {
        auto p = make(0, 0, 8);
        put(p, 4, flags, 2);
        auto x = native_dependency_link(p);
        check(x["flags"] == flags && x["write_flag_projection"]["flags"] == (flags & 0xff87u) &&
                  x["write_flag_projection"]["cleared_bits"] == (flags & 0x78u),
              "writer flag projection clears only confirmed bits without changing source flags");
    }
    // The same local ID has different fallback rules depending on which native
    // entry point emitted it. Preserve both references rather than merging them.
    for (unsigned format : {0u, 1u, 8u}) {
        const auto stride = format == 0 ? 8u : 16u;
        auto p = make(format, 1, 8 + stride);
        put(p, format == 8 ? 16 : 8, UINT64_MAX, 8);
        auto x = native_dependency_link(p);
        const auto ref = x["entries"][0]["references"][0];
        check(ref["lookup_profile"] == "owner_system_file" && !ref.contains("target_lookup"),
              "ordinary IDs including maximum are not rejected by the cross-owner sentinel rule");
    }
    for (unsigned format : {2u, 3u, 4u, 5u, 7u}) {
        const unsigned sizes[] = {8, 16, 40, 48, 16, 24, 0, 24, 16};
        auto p = make(format, 1, 8 + sizes[format]);
        const auto entry = native_dependency_link(p)["entries"][0];
        check(entry["references"].size() == 2 &&
                  entry["references"][0]["lookup_profile"] == "owner_system_file" &&
                  entry["references"][1]["lookup_profile"] == "owner_system" &&
                  entry["references"][0]["element_id"] == entry["references"][1]["element_id"],
              "target through zero and local owner dependency retain distinct native fallback "
              "profiles");
    }
    for (unsigned format : {4u, 5u}) {
        const auto stride = format == 4 ? 16u : 24u;
        for (auto pair :
             {std::pair<std::uint64_t, std::uint64_t>{41, 42}, {UINT64_MAX, 0}, {41, UINT64_MAX}}) {
            auto p = make(format, 1, 8 + stride);
            put(p, 8, pair.first, 8);
            put(p, stride, pair.second, 8);
            const auto refs = native_dependency_link(p)["entries"][0]["references"];
            check(refs[0]["lookup_profile"] ==
                          (pair.second ? "reference_path_owner_system" : "owner_system_file") &&
                      refs[0]["target_lookup"] ==
                          (pair.first == UINT64_MAX || pair.second == UINT64_MAX
                               ? "skipped_maximum_id"
                               : "requires_runtime_state") &&
                      refs[1]["element_id"] == pair.second &&
                      refs[1]["lookup_profile"] == "owner_system",
                  "maximum target or owner reference rejects only the paired target lookup");
        }
    }
    return checks;
}
