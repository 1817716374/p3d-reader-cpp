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
    b = source;
    state = decode_model_units(b);
    const std::array<double, 7> unit_values{1, 1, 10, 1000, 1, 304.8, 25.4};
    check(state["factors"]["secondary_to_primary_display_ratio"]["value"] == .1,
          "display unit ratio reads the second display definition independently");
    for (int mode = 0; mode < 7; ++mode) {
        const auto &entry = state.at("factors").at("material_mapping_units_before_reference")[mode];
        check(entry.at("status") == "computed" && entry.at("value") == unit_values[mode],
              "native mapping unit table and master/subunit selection");
        const auto selected = material_uv_mapping_unit_factor(state, 3, mode, 2.);
        check(selected.at("status") == "computed" &&
                  selected.at("value") == (mode == 0 ? 1. : unit_values[mode] * 2) &&
                  selected.at("reference_scale_applied") == (mode != 0),
              "relative mode bypasses reference scaling and other modes apply it");
        const auto elevation = material_uv_mapping_unit_factor(state, 1, mode);
        check(elevation.at("status") == "computed" && elevation.at("value") == unit_values[mode] &&
                  elevation.at("reference_scale_applied") == false,
              "elevation drape does not use the reference-chain scale");
    }
    check(material_uv_mapping_unit_factor(Json(), 1, 0).at("value") == 1.,
          "relative mode needs neither a model nor a reference context");
    check(material_uv_mapping_unit_factor(state, 3, 3).at("reason") ==
              "missing_or_nonfinite_mapping_reference_scale",
          "missing reference scale is not replaced by one");
    for (int mode : {-1, 7, 100})
        check(material_uv_mapping_unit_factor(state, 3, mode, -2.).at("value") == -2.,
              "out-of-table signed modes follow native master-unit fallback");
    check(material_uv_mapping_unit_factor(state, 3, 3, 0.).at("value") == 0.,
          "explicit zero reference multiplier remains zero");
    check(material_uv_mapping_unit_factor(state, 3, 3, std::numeric_limits<double>::infinity())
                  .at("status") == "not_evaluated",
          "nonfinite reference multiplier is rejected");
    put(80, std::uint32_t(10));
    state = decode_model_units(b);
    check(state["factors"]["secondary_to_primary_display_ratio"]["value"] == 1 &&
              state["factors"]["material_mapping_units_before_reference"][2]["value"] == 1,
          "incompatible display base keys use native unit-ratio fallback");
    put(76, std::uint32_t(18));
    state = decode_model_units(b);
    for (int mode = 3; mode <= 6; ++mode)
        check(state["factors"]["material_mapping_units_before_reference"][mode]["value"] == 1 &&
                  state["factors"]["material_mapping_units_before_reference"][mode]["branch"] ==
                      "primary_unit_factor_fallback",
              "named unit conversion compares base keys, not guessed physical units");
    for (const auto offset : {84u, 92u, 100u, 108u}) {
        b = source;
        put(offset, 0.);
        state = decode_model_units(b);
        check(state["factors"]["secondary_to_primary_display_ratio"]["value"] == 1,
              "zero display ratio components follow native fallback");
    }
    b = source;
    put(100, std::numeric_limits<double>::quiet_NaN());
    state = decode_model_units(b);
    check(state["factors"]["secondary_to_primary_display_ratio"]["value"] == 1 &&
              material_uv_mapping_unit_factor(state, 1, 2).at("value") == 1,
          "unused NaN display numerator does not poison the native fallback");
    put(100, std::numeric_limits<double>::infinity());
    state = decode_model_units(b);
    check(state["factors"]["secondary_to_primary_display_ratio"]["status"] == "not_evaluated" &&
              state["factors"]["material_mapping_units_before_reference"][2]["value"] == 0,
          "positive infinity passes native ratio tests and master/infinity produces zero");
    b = source;
    put(84, 1e308);
    state = decode_model_units(b);
    check(state["factors"]["material_mapping_units_before_reference"][5]["status"] ==
              "not_evaluated",
          "named-unit intermediate overflow is not hidden by reordered arithmetic");
    return checks;
}
