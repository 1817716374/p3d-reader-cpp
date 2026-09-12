#include "internal.hpp"

unsigned native_reference_path_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto put = [](Bytes &b, std::size_t at, std::uint64_t v, unsigned n) {
        for (unsigned i = 0; i < n; ++i)
            b.at(at + i) = std::uint8_t(v >> (i * 8));
    };
    Bytes base(40);
    put(base, 4, 47, 2);
    put(base, 16, 20, 4);
    put(base, 36, 0x56e6, 2);
    auto link = [&](Bytes p, unsigned h = 0x1000) {
        return Json{{"app", 0x56d0}, {"header", h}, {"payload", rawbytes(p)}, {"offset", 40}};
    };
    auto payload = [&](unsigned format, unsigned n) {
        Bytes p(format == 6 ? 24 + n * 8 : 8 + n * 8);
        put(p, 0, 10000, 2);
        put(p, 2, 4, 2);
        put(p, 4, format << 10, 2);
        put(p, 6, format == 6 ? 1 : n, 2);
        if (format == 6)
            put(p, 8, n, 4);
        for (unsigned i = 0; i < n; ++i)
            put(p, (format == 6 ? 24 : 8) + i * 8, 100 + i, 8);
        return p;
    };
    auto p = payload(0, 3);
    put(p, 8, UINT64_MAX, 8);
    put(p, 16, 0, 8);
    auto d = native_reference_path(base, Json::array({link(p)}));
    check(d["status"] == "resolved" && d["steps"].size() == 3 &&
              d["steps"][0]["element_id"] == 102 && d["steps"][1]["element_id"] == 0 &&
              d["steps"][2]["element_id"] == UINT64_MAX,
          "reference ID lists expand in reverse order and retain sentinel IDs");
    check(d["steps"][2]["operation"] == "resolve_and_expand_reference" &&
              d["steps"][0]["runtime_reject_mask"] == 8 &&
              d["steps"][0]["lookup_profile"] == "owner_system",
          "ordinary path last source ID is not treated as a special terminal target");
    put(p, 4, 513, 2);
    d = native_reference_path(base, Json::array({link(p)}));
    check(d["status"] == "resolved" && d["dependency_disable_bit_applied"] == false &&
              native_dependency_link(p)["dependency_index_input"] == "skipped_flag_1",
          "reference path reader does not reuse dependency index suppression flags");
    auto first = p;
    put(p, 8, 777, 8);
    d = native_reference_path(base, Json::array({link(first), link(p)}));
    check(d["steps"][2]["element_id"] == UINT64_MAX && d["selected_linkage_index"] == 0 &&
              d["matching_links"][1]["selection"] == "shadowed",
          "first matching path wins without merging duplicates");
    auto short_first = slice(first, 0, 4);
    d = native_reference_path(base, Json::array({link(short_first), link(p)}));
    check(d["status"] == "invalid" && d["selected_linkage_index"] == 0 && d["steps"].empty() &&
              d["matching_links"][1]["selection"] == "shadowed",
          "damaged selected path cannot fall back to a later match");
    auto short_key = slice(first, 0, 2);
    d = native_reference_path(base, Json::array({link(short_key), link(p)}));
    check(d["status"] == "invalid" && !d.contains("selected_linkage_index") &&
              d["matching_links"][1]["selection"] == "undetermined",
          "incomplete early matching key prevents guessed selection");
    put(short_key, 0, 123, 2);
    d = native_reference_path(base, Json::array({link(short_key), link(p)}));
    check(d["selected_linkage_index"] == 1 && d["status"] == "resolved",
          "a complete mismatching owner code needs no relation bytes");
    d = native_reference_path(base, Json::array({link(p, 0), link(p)}));
    check(d["selected_linkage_index"] == 1, "path getter only matches user linkages");
    p = payload(6, 3);
    d = native_reference_path(base, Json::array({link(p)}));
    check(d["steps"].size() == 3 && d["steps"][0]["element_id"] == 102 &&
              d["steps"][1]["element_id"] == 101 && d["steps"][2]["element_id"] == 100 &&
              d["steps"][2]["operation"] == "resolve_and_append_target" &&
              d["terminal_owner_update"] == "none",
          "variable path reverses owner references and reserves the first source ID for the "
          "terminal target");
    check(d["steps"][2]["append_rule"] == "first_ancestor_without_child_flag_then_target" &&
              !d["steps"][2].contains("owner_update"),
          "terminal target uses runtime ancestry without changing current owner");
    for (unsigned n : {0u, 2u, 65535u}) {
        auto bad = p;
        put(bad, 6, n, 2);
        check(native_reference_path(base, Json::array({link(bad)}))["status"] == "invalid",
              "variable path requires exactly one outer entry");
    }
    for (auto n : {0u, 4u, 0x80000000u, 0xffffffffu}) {
        auto bad = p;
        put(bad, 8, n, 4);
        check(native_reference_path(base, Json::array({link(bad)}))["status"] == "invalid",
              "variable path rejects zero, truncated and signed-overflow counts safely");
    }
    d = native_reference_path(base, Json::array({link(payload(0, 0))}));
    check(d["status"] == "resolved" && d["steps"].empty() &&
              d["empty_path_owner"] == "keep_existing_or_use_current",
          "empty fixed path preserves the native owner rule");
    d = native_reference_path(base, Json::array({link(payload(6, 1))}));
    check(d["steps"].size() == 1 && d["steps"][0]["operation"] == "resolve_and_append_target",
          "single variable target does not fabricate an owner expansion step");
    for (unsigned format : {1u, 2u, 3u, 4u, 5u, 7u, 8u, 15u}) {
        auto bad = p;
        put(bad, 4, format << 10, 2);
        check(native_reference_path(base, Json::array({link(bad)}))["status"] == "invalid",
              "path reader accepts fewer formats than the generic dependency reader");
    }
    for (unsigned format : {0u, 6u}) {
        auto full = payload(format, 3);
        for (std::size_t n = 0; n < full.size(); ++n)
            check(native_reference_path(base, Json::array({link(slice(full, 0, n))}))["status"] ==
                      "invalid",
                  "truncated paths publish no successful program");
    }
    for (std::size_t n = 0; n < 38; ++n)
        check(native_reference_path(slice(base, 0, n), Json::array({link(p)}))["status"] ==
                  "invalid",
              "reference path signature is bounded by the source base record");
    auto wrong = base;
    put(wrong, 36, 0x56e7, 2);
    check(native_reference_path(wrong, Json::array({link(p)}))["status"] == "invalid",
          "wrong signature does not identify a path");
    check(native_reference_path(base, Json::array())["status"] == "invalid",
          "missing path link remains explicit");
    // End-to-end parse keeps raw linkage offsets and the two independent interpretations.
    Bytes raw = base;
    put(raw, 12, 18, 4);
    p = payload(6, 2);
    put(p, 4, 0x1801, 2);
    const auto at = raw.size();
    raw.resize(at + 4 + p.size());
    put(raw, at, 0x1000 | ((p.size() + 4) / 2 - 1), 2);
    put(raw, at + 2, 0x56d0, 2);
    std::copy(p.begin(), p.end(), raw.begin() + at + 4);
    put(raw, 8, (raw.size() - 4) / 2, 4);
    auto rows = parse_native(raw);
    check(rows[0]["owner_reference_path"]["status"] == "resolved" &&
              rows[0]["owner_reference_path"]["selected_linkage_offset"] == at &&
              rows[0]["links"][0]["decoded"]["dependency_index_input"] == "skipped_flag_1" &&
              bytesof(rows[0]["links"][0]["payload"]) == p && rows[0]["children"].empty(),
          "native record exposes path program without altering raw links or converting references "
          "to children");
    put(raw, 36, 0x56e7, 2);
    rows = parse_native(raw);
    check(
        !rows[0].contains("owner_reference_path"),
        "other application signatures sharing the record type are not mislabeled as broken paths");
    return checks;
}
