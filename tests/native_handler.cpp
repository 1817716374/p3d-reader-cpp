#include "internal.hpp"
using namespace p3d;

unsigned native_handler_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    // Eight-byte payload from the public R1.18 WelcomeFile/案例1.p3d.
    const Bytes sample{0, 0, 0x80, 0x59, 0, 0, 0, 0};
    const auto value = decode_attribute(0, 22900, sample, 1);
    check(value.at("encoding") == "native_handler_reference" &&
              value.at("registration_key") == 0x59800000u &&
              value.at("registration_lookup_keys") == Json::array({0x5980, 0}),
          "handler registry uses high then low WORD, not a floating-point payload");
    check(value.at("status") == "partial" && value.at("runtime_resolution") == "not_evaluated" &&
              value.at("unassigned_word_at_4") == 0 &&
              bytesof(value.at("trailing_bytes")).empty(),
          "handler identity does not claim runtime registration or semantics are resolved");
    const Bytes extended{0x34, 0x12, 0xcd, 0xab, 0xff, 0xee, 0xdd, 0xcc, 0, 7, 0xff};
    const auto other = decode_attribute(0, 22900, extended, 1);
    check(other.at("registration_key") == 0xabcd1234u &&
              other.at("registration_lookup_keys") == Json::array({0xabcd, 0x1234}) &&
              other.at("unassigned_word_at_4") == 0xccddeeffu &&
              bytesof(other.at("trailing_bytes")) == Bytes({0, 7, 0xff}),
          "unknown handler word and native-unconsumed suffix remain lossless");
    for (std::size_t length = 0; length < 8; ++length) {
        bool rejected = false;
        try {
            decode_attribute(0, 22900, Bytes(sample.begin(), sample.begin() + length), 1);
        } catch (const std::exception &) {
            rejected = true;
        }
        check(rejected, "truncated handler identity is rejected before prefix access");
    }
    check(decode_attribute(0, 22900, sample, 0).at("encoding") == "opaque" &&
              decode_attribute(1, 22900, sample, 1).at("encoding") == "opaque",
          "handler decoder is restricted to the observed group and occurrence index");
    const Bytes grouping_sample{0, 0, 0xc0, 0x5d, 0xff, 0xff, 0xff, 0xff};
    const auto grouping = decode_attribute(3, 20031, grouping_sample, 0);
    check(grouping.at("encoding") == "native_geometry_grouping_key" &&
              grouping.at("first_unsigned") == 0x5dc00000u &&
              grouping.at("second_signed") == -1 && grouping.at("status") == "partial",
          "native grouping key has mixed signedness, not two floats or a uint64 id");
    const Bytes boundary{0xff, 0xff, 0xff, 0xff, 0, 0, 0, 0x80};
    const auto extreme = decode_attribute(3, 20031, boundary, 0);
    check(extreme.at("first_unsigned") == 0xffffffffu &&
              extreme.at("second_signed") == (-2147483647 - 1) &&
              extreme.at("component_semantics") == "unresolved",
          "grouping key preserves unsigned high bit and signed minimum without inferred roles");
    for (std::size_t length = 0; length <= 10; ++length) {
        if (length == 8)
            continue;
        bool rejected = false;
        try {
            decode_attribute(3, 20031, Bytes(length, 0), 0);
        } catch (const std::exception &) {
            rejected = true;
        }
        check(rejected, "native grouping lookup requires exactly eight bytes");
    }
    check(decode_attribute(3, 20031, grouping_sample, 1).at("encoding") == "opaque" &&
              decode_attribute(2, 20031, grouping_sample, 0).at("encoding") == "opaque",
          "grouping decoder does not reinterpret other groups or indices");
    return checks;
}
