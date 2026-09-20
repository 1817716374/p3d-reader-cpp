#include "internal.hpp"
#include <p3d/reference_loading.hpp>
#include <future>

unsigned reference_loading_input_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto put = [](Bytes &b, std::size_t at, std::uint64_t value, unsigned n) {
        for (unsigned i = 0; i < n; ++i)
            b.at(at + i) = std::uint8_t(value >> (i * 8));
    };
    auto number = [&](Bytes &b, std::size_t at, double value) {
        std::uint64_t bits;
        std::memcpy(&bits, &value, 8);
        put(b, at, bits, 8);
    };
    auto link = [&](Bytes p, bool user = true) {
        Bytes b(p.size() + 4);
        put(b, 0, (user ? 0x1000 : 0) | (b.size() / 2 - 1), 2);
        put(b, 2, 0x56d0, 2);
        std::copy(p.begin(), p.end(), b.begin() + 4);
        return b;
    };
    auto dependency = [&](unsigned flags, unsigned count, std::uint64_t id) {
        Bytes p(24);
        put(p, 0, 10000, 2);
        put(p, 2, 16, 2);
        put(p, 4, flags, 2);
        put(p, 6, count, 2);
        put(p, 8, id, 8);
        put(p, 16, 0x12345678, 8);
        return link(p);
    };
    auto wire = [&](unsigned primary, unsigned secondary, double diagonal = 1., double scale = 1.,
                    double lower = -3., double upper = 4.) {
        Bytes b(372);
        put(b, 4, 13, 2);
        put(b, 8, 184, 4);
        put(b, 12, 184, 4);
        put(b, 60, primary, 4);
        put(b, 64, secondary, 4);
        put(b, 160, 1, 4);
        for (auto offset : {220, 252, 284})
            number(b, offset, diagonal);
        number(b, 292, scale);
        number(b, 316, upper);
        number(b, 324, lower);
        // Points are nonzero in the file, but are loaded AFTER bit-zero recomputation.
        number(b, 172, 11.);
        number(b, 196, 23.);
        return b;
    };
    auto record = [&](Bytes bytes, const std::vector<Bytes> &links = {}) {
        for (const auto &link : links)
            bytes.insert(bytes.end(), link.begin(), link.end());
        put(bytes, 8, (bytes.size() - 4) / 2, 4);
        return parse_native(bytes).at(0);
    };
    auto flags = [](const Json &r) -> const Json & {
        return r.at("reference_target").at("initial_loaded_flags");
    };
    const auto valid = dependency(0x1000, 1, UINT64_C(0xfedcba9876543210));
    auto r = record(wire(UINT32_MAX, UINT32_MAX));
    check(flags(r).at("status") == "decoded" && flags(r).at("primary").at("value") == 0x7dfffeffu &&
              flags(r).at("secondary").at("value") == UINT32_MAX,
          "missing controlling dependency clears only primary bits8/25/31, valid depths preserve "
          "secondary");
    check(flags(r).at("primary").at("bit0_recomputed") == false,
          "unit input preserves primary bit zero without rechecking source points");
    r = record(wire(UINT32_MAX, UINT32_MAX), {valid});
    check(flags(r).at("primary").at("value") == UINT32_MAX &&
              flags(r).at("control_dependency").at("element_id") == UINT64_C(0xfedcba9876543210) &&
              flags(r).at("control_dependency").at("source_offset") == 372,
          "nonzero uint64 controlling ID preserves three primary bits and source position");
    r = record(wire(UINT32_MAX, UINT32_MAX), {dependency(0x9001, 1, 8)});
    check(flags(r).at("primary").at("value") == UINT32_MAX,
          "dependency consumer ignores flag one and unrelated bits instead of generic index "
          "eligibility");
    r = record(wire(UINT32_MAX, 0), {dependency(0x1000, 1, 0), valid});
    check(
        flags(r).at("primary").at("value") == 0x7dfffeffu &&
            flags(r).at("control_dependency").at("selected_linkage_index") == 0,
        "first zero element ID stays zero even with nonzero second ID and later valid dependency");
    r = record(wire(UINT32_MAX, 0), {dependency(0, 1, 8), valid});
    check(flags(r).at("control_dependency").at("status") == "ignored_format" &&
              flags(r).at("primary").at("value") == 0x7dfffeffu,
          "first owner/relation match wins before format validation");
    for (auto count : {0u, 2u, 65535u}) {
        r = record(wire(UINT32_MAX, 0), {dependency(0x1000, count, 8), valid});
        check(flags(r).at("control_dependency").at("status") == "ignored_count" &&
                  flags(r).at("primary").at("value") == 0x7dfffeffu,
              "non-unit count does not fall through to later controlling dependency");
    }
    auto other = valid;
    put(other, 6, 15, 2);
    auto non_user = valid;
    non_user.resize(8);
    non_user[1] &= ~0x10;
    r = record(wire(UINT32_MAX, 0), {other, non_user, valid});
    check(flags(r).at("control_dependency").at("selected_linkage_index") == 2,
          "unrelated relation and non-user dependency do not shadow the matching user record");
    auto first_only = valid;
    first_only.resize(20);
    put(first_only, 0, 0x1009, 2);
    r = record(wire(UINT32_MAX, 0), {first_only});
    check(flags(r).at("primary").at("value") == UINT32_MAX &&
              r.at("links")[0].at("decoded").at("status") == "invalid",
          "specific consumer needs first ID only despite generic format4 requiring its second ID");
    auto malformed = valid;
    malformed.resize(12);
    put(malformed, 0, 0x1005, 2);
    r = record(wire(UINT32_MAX, UINT32_MAX), {malformed, valid});
    check(flags(r).at("status") == "partial" && !flags(r).at("primary").contains("value") &&
              flags(r).at("secondary").at("status") == "decoded" &&
              flags(r).at("control_dependency").at("selected_linkage_index") == 0,
          "truncated selected ID leaves primary unresolved while independent secondary remains "
          "decoded");
    Bytes short_owner{1, 0};
    r = record(wire(UINT32_MAX, 0), {link(short_owner), valid});
    check(flags(r).at("primary").at("value") == UINT32_MAX,
          "mismatched owner short circuits before reading the absent relation");
    auto bad_format = valid;
    bad_format.resize(10);
    put(bad_format, 0, 0x1004, 2);
    put(bad_format, 8, 0, 2);
    r = record(wire(UINT32_MAX, 0), {bad_format});
    check(flags(r).at("control_dependency").at("status") == "ignored_format",
          "unsupported dependency format does not require an absent count");
    for (auto diagonal : {0.5, 2., 3., 1. + 1e-9}) {
        r = record(wire(UINT32_MAX, 0, diagonal, 1. / diagonal), {valid});
        check(flags(r).at("primary").at("bit0_recomputed") == true &&
                  flags(r).at("primary").at("value") == 0xfffffffeu,
              "rescaling recomputes primary bit zero from loaded scale and post-adjustment matrix");
    }
    for (auto diagonal : {1., 1. + 1e-11, -1.}) {
        r = record(wire(UINT32_MAX, 0, diagonal), {valid});
        check(flags(r).at("primary").at("bit0_recomputed") == false &&
                  flags(r).at("primary").at("value") == UINT32_MAX,
              "no adjustment and rejected rigid-scale predicate preserve saved bit zero");
    }
    const auto nan = std::numeric_limits<double>::quiet_NaN(),
               inf = std::numeric_limits<double>::infinity();
    for (auto pair : {std::pair<double, double>{4, 4}, {5, 4}, {inf, inf}, {-inf, -inf}}) {
        r = record(wire(0, UINT32_MAX, 1, 1, pair.first, pair.second));
        check(flags(r).at("secondary").at("value") == 0xfffff3ffu,
              "equal or reversed ordered depth endpoints clear secondary bits10/11 including equal "
              "infinities");
    }
    for (auto pair : {std::pair<double, double>{nan, 4}, {-3, nan}, {-inf, inf}}) {
        r = record(wire(0, UINT32_MAX, 1, 1, pair.first, pair.second));
        check(flags(r).at("secondary").at("value") == UINT32_MAX,
              "unordered depth comparisons and valid infinite interval preserve secondary bits");
    }
    r = record(wire(UINT32_MAX, 0, nan));
    check(flags(r).at("primary").at("status") == "not_evaluated" &&
              flags(r).at("secondary").at("status") == "decoded",
          "unknown transform effect is not presented as saved primary flags");

    ReferenceLoadingDecisionContext c;
    c.native_input_status = 0;
    c.host_filter_excluded = false;
    c.source_runtime_deleted = false;
    c.descendant_filter_required = false;
    c.ancestor_repeated = false;
    r = record(wire(0, 0));
    c.loaded_primary_flags = 0x80;
    check(initial_reference_loading_decision(r, c).at("loading_state") == 1,
          "record-derived initial primary overrides contradictory manual loaded flags");
    r = record(wire(0x80, 0));
    c.loaded_primary_flags = 0;
    check(initial_reference_loading_decision(r, c).at("loading_state") == 5,
          "actual loaded bit7 selects the native rejected branch");
    r = record(wire(0, 0));
    c.require_source_bit14 = true;
    c.source_primary_flags = 0x4000;
    check(initial_reference_loading_decision(r, c).at("loading_state") == 7,
          "source-bit requirement comes from the record rather than caller flag member");
    r = record(wire(0x4000, 0));
    check(initial_reference_loading_decision(r, c).at("loading_state") == 1,
          "record source bit14 permits the otherwise accepted loading branch");
    r.erase("reference_target");
    c.native_input_status = 42;
    check(initial_reference_loading_decision(r, c).at("native_return") == 42,
          "known input failure does not require unavailable loaded flag state");
    c.require_source_bit14 = false;
    r["reference_input"] = "unused";
    check(initial_reference_loading_decision(r, c).at("native_return") == 42,
          "unused source input is not decoded when bit14 filtering is disabled");
    c.native_input_status.reset();
    check(initial_reference_loading_decision(r, c).at("status") == "unresolved",
          "parsed record cannot invent a successful native input return code");
    c.native_input_status = 0;
    check(initial_reference_loading_decision(r, c).at("status") == "unresolved",
          "successful input requires confirmed loaded flags before selecting a list");
    check(initial_reference_loading_decision(Json{{"element_type", 47}}, c).at("loading_state") ==
              4,
          "non-reference native record uses ordinary rejection without reference fields");
    r = record(wire(0, 0, 2), {valid});
    const auto baseline = initial_reference_loading_decision(r, c);
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async,
                                  [=] { return initial_reference_loading_decision(r, c); }));
    for (auto &job : jobs)
        check(job.get() == baseline, "initial loading adapter has no shared mutable state");
    return checks;
}
