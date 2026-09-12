#include "internal.hpp"

unsigned model_units_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    Bytes b(500, 0);
    auto put = [&](std::size_t offset, auto value) {
        std::memcpy(b.data() + offset, &value, sizeof value);
    };
    put(4, std::uint16_t(47));
    put(8, std::uint32_t(248));
    put(12, std::uint32_t(248));
    put(16, std::uint32_t(32));
    put(68, std::uint32_t(17));
    put(76, std::uint32_t(17));
    put(80, std::uint32_t(9));
    put(228, 1.);
    put(236, 1000.);
    put(244, 1.);
    put(84, 1000.);
    put(92, 1.);
    put(100, 100.);
    put(108, 1.);
    const auto source = b;
    auto state = decode_model_units(b);
    check(state["status"] == "decoded" && b == source && state["storage_unit"]["base_key"] == 1 &&
              state["storage_unit"]["system_key"] == 2 &&
              state["display_unit_2"]["numerator"] == 100.,
          "decode three separate unit definitions without changing source");
    check(state["factors"]["data_units_per_meter"]["value"] == 1000. &&
              state["factors"]["meters_per_data_unit"]["value"] == .001 &&
              state["factors"]["material_projection_unit_factor"]["value"] == 1.,
          "millimeter data factor differs from absolute material projection factor");
    auto records = parse_native(b);
    check(records.size() == 1 && records[0]["model_unit_state"] == state &&
              bytesof(records[0]["data"]) == source,
          "native model-header records expose unit semantics and retain raw bytes");
    put(228, 3.);
    put(236, 7.);
    put(244, 2.);
    put(84, 5.);
    put(92, 4.);
    state = decode_model_units(b);
    check(
        state["factors"]["data_units_per_meter"]["value"] == 10.5 &&
            state["factors"]["material_projection_unit_factor"]["value"] == 8.4,
        "unit conversion uses stored rational values and base factor, not a millimeter heuristic");
    put(76, std::uint32_t(0xffffffc9));
    state = decode_model_units(b);
    check(state["display_unit_1"]["packed_flags"] == 0xffffffc9u &&
              state["display_unit_1"]["base_key"] == 1 &&
              state["display_unit_1"]["system_key"] == 1 &&
              state["factors"]["material_projection_unit_factor"]["branch"] ==
                  "compatible_positive_ratio",
          "projection compares only base keys, retaining system and reserved source bits");
    put(76, std::uint32_t(18));
    state = decode_model_units(b);
    check(state["factors"]["material_projection_unit_factor"]["value"] == 3. &&
              state["factors"]["material_projection_unit_factor"]["branch"] ==
                  "base_factor_fallback",
          "different bases use the stored base factor");
    put(76, std::uint32_t(17));
    for (const auto offset : {84u, 92u, 236u, 244u}) {
        const auto saved = Reader(b).at<double>(offset);
        for (const double invalid : {0., -1., std::numeric_limits<double>::quiet_NaN()}) {
            put(offset, invalid);
            state = decode_model_units(b);
            check(state["factors"]["material_projection_unit_factor"]["value"] == 3. &&
                      state["factors"]["material_projection_unit_factor"]["branch"] ==
                          "base_factor_fallback",
                  "nonpositive or NaN ratio component follows the native base-factor fallback");
        }
        put(offset, saved);
    }
    put(100, std::uint64_t(0xfff8000000000012));
    state = decode_model_units(b);
    check(state["display_unit_2"]["numerator"]["ieee754_hex"] == "fff8000000000012" &&
              state["factors"]["data_units_per_meter"]["status"] == "computed" &&
              state["factors"]["material_projection_unit_factor"]["status"] == "computed",
          "unused nonfinite second display unit preserves NaN payload without obscuring factors");
    put(84, std::numeric_limits<double>::infinity());
    state = decode_model_units(b);
    check(state["factors"]["material_projection_unit_factor"]["value"] == 0. &&
              state["factors"]["material_projection_unit_factor"]["branch"] ==
                  "compatible_positive_ratio",
          "positive infinity passes the native positive comparison and can yield a finite zero "
          "ratio");
    put(84, 5.);
    put(244, 0.);
    state = decode_model_units(b);
    check(state["factors"]["data_units_per_meter"]["status"] == "not_evaluated" &&
              state["factors"]["meters_per_data_unit"]["value"].is_null() &&
              state["factors"]["material_projection_unit_factor"]["value"] == 3.,
          "unavailable meter conversion does not hide a separately valid projection fallback");
    put(244, 2.);
    put(228, 0.);
    state = decode_model_units(b);
    check(state["factors"]["data_units_per_meter"]["value"] == 0. &&
              state["factors"]["meters_per_data_unit"]["status"] == "not_evaluated",
          "zero data scale cannot become a usable reciprocal conversion");
    put(228, -3.);
    state = decode_model_units(b);
    check(state["factors"]["data_units_per_meter"]["value"] == -10.5 &&
              state["factors"]["data_units_per_meter"]["positive"] == false,
          "negative stored base is preserved and distinguished from a usable positive scale");
    put(228, 1e308);
    put(236, 1e308);
    state = decode_model_units(b);
    check(state["factors"]["material_projection_unit_factor"]["status"] == "not_evaluated",
          "intermediate multiplication overflow is not algebraically rearranged into a finite "
          "answer");
    check(decode_model_units(Bytes(b.begin(), b.end() - 1))["status"] == "unsupported_header",
          "short model header remains unsupported");
    put(16, std::uint32_t(33));
    check(decode_model_units(b)["status"] == "unsupported_header",
          "other type-47 subtypes are not decoded as model units");
    return checks;
}
