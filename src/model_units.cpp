#include "internal.hpp"

namespace p3d {
namespace {
Json numeric(Reader &r, std::size_t offset) {
    const auto bits = r.at<std::uint64_t>(offset);
    const auto value = r.at<double>(offset);
    if (std::isfinite(value))
        return value;
    std::string encoded(16, '0');
    for (unsigned i = 0; i < 16; ++i)
        encoded[15 - i] = "0123456789abcdef"[(bits >> (i * 4)) & 15];
    return {{"floating_point", std::isinf(value) ? "infinity" : "nan"},
            {"negative", bool(bits >> 63)},
            {"ieee754_hex", encoded}};
}
Json definition(Reader &r, std::size_t flags_offset, std::size_t ratio_offset) {
    const auto flags = r.at<std::uint32_t>(flags_offset);
    return {{"packed_flags", flags},
            {"base_key", flags & 7u},
            {"system_key", (flags >> 3) & 7u},
            {"numerator", numeric(r, ratio_offset)},
            {"denominator", numeric(r, ratio_offset + 8)},
            {"source_offsets",
             {{"packed_flags", flags_offset},
              {"numerator", ratio_offset},
              {"denominator", ratio_offset + 8}}}};
}
Json factor(double value) {
    if (!std::isfinite(value))
        return {{"status", "not_evaluated"},
                {"reason", "nonfinite_arithmetic_result"},
                {"value", nullptr}};
    return {{"status", "computed"}, {"value", value}, {"positive", value > 0}};
}
} // namespace

Json decode_model_coordinates(const Bytes &b) {
    Json out = {{"scope", "native_model_header_coordinate_state"},
                {"status", "unsupported_header"},
                {"source_offsets_include_stream_prefix", true}};
    if (b.size() < 500 || Reader(b, 4).u16() != 47 || Reader(b, 16).u32() != 32)
        return out;
    Reader r(b);
    const auto source_flags = r.at<std::uint32_t>(72);
    const auto source_kind = r.at<std::uint16_t>(38);
    const unsigned kind = source_kind == 0 && (source_flags & 0x400) ? 2 : source_kind;
    const auto flags = (source_flags & ~0x400u) | (kind == 2 ? 0x400u : 0u);
    auto point = [&](std::size_t offset) {
        return Json::array({numeric(r, offset), numeric(r, offset + 8), numeric(r, offset + 16)});
    };
    const auto primary = point(404);
    auto effective = primary;
    if (!(flags & 1))
        effective[2] = 0.;
    out.update({{"status", "decoded"},
                {"source_flags", source_flags},
                {"flags", flags},
                {"flags_source_offset", 72},
                {"model_kind",
                 {{"source_value", source_kind},
                  {"value", kind},
                  {"source_offset", 38},
                  {"name", kind == 0   ? "physical"
                           : kind == 1 ? "sheet"
                           : kind == 2 ? "drawing"
                                       : "unknown"}}},
                {"reference_origin",
                 {{"source_value", primary},
                  {"value", effective},
                  {"source_offset", 404},
                  {"z_enabled", bool(flags & 1)}}},
                {"auxiliary_origin", {{"value", point(116)}, {"source_offset", 116}}}});
    return out;
}

Json decode_model_units(const Bytes &b) {
    Json out = {{"scope", "native_model_header_unit_state"},
                {"status", "unsupported_header"},
                {"source_offsets_include_stream_prefix", true}};
    if (b.size() < 500)
        return out;
    Reader r(b);
    if (r.at<std::uint16_t>(4) != 47 || r.at<std::uint32_t>(16) != 32)
        return out;
    out.update(
        {{"status", "decoded"},
         {"data_units_per_storage_unit", {{"value", numeric(r, 228)}, {"source_offset", 228}}},
         {"storage_unit", definition(r, 68, 236)},
         {"display_unit_1", definition(r, 76, 84)},
         {"display_unit_2", definition(r, 80, 100)},
         {"unassigned_adjacent_value", {{"source_offset", 252}, {"value", numeric(r, 252)}}}});
    const auto base = r.at<double>(228), storage_n = r.at<double>(236),
               storage_d = r.at<double>(244), display_n = r.at<double>(84),
               display_d = r.at<double>(92);
    const auto per_meter = (storage_n * base) / storage_d;
    const bool ratio = (r.at<std::uint32_t>(68) & 7u) == (r.at<std::uint32_t>(76) & 7u) &&
                       storage_n > 0 && storage_d > 0 && display_n > 0 && display_d > 0;
    const auto projection =
        ratio ? ((storage_n * base) * display_d) / (display_n * storage_d) : base;
    out["factors"]["data_units_per_meter"] = factor(per_meter);
    out["factors"]["data_units_per_meter"]["source_offsets"] = {228, 236, 244};
    out["factors"]["material_projection_unit_factor"] = factor(projection);
    out["factors"]["material_projection_unit_factor"]["branch"] =
        ratio ? "compatible_positive_ratio" : "base_factor_fallback";
    out["factors"]["material_projection_unit_factor"]["source_offsets"] =
        Json::array({68, 76, 228, 236, 244, 84, 92});
    const auto secondary_n = r.at<double>(100), secondary_d = r.at<double>(108);
    const bool display_ratio = (r.at<std::uint32_t>(76) & 7u) == (r.at<std::uint32_t>(80) & 7u) &&
                               secondary_n > 0 && secondary_d > 0 && display_n > 0 && display_d > 0;
    const double sub_ratio =
        display_ratio ? (display_d * secondary_n) / (display_n * secondary_d) : 1;
    out["factors"]["secondary_to_primary_display_ratio"] = factor(sub_ratio);
    out["factors"]["secondary_to_primary_display_ratio"]["branch"] =
        display_ratio ? "compatible_positive_ratio" : "unit_ratio_fallback";
    out["factors"]["secondary_to_primary_display_ratio"]["source_offsets"] = {76, 80,  84,
                                                                              92, 100, 108};
    // The native unit table stores rational scales; preserve its multiplication
    // order rather than replacing imperial ratios with rounded decimal units.
    Json mapping_units = Json::array();
    for (unsigned mode = 0; mode < 7; ++mode) {
        const char *names[] = {"relative",    "master_units", "sub_units", "meters",
                               "millimeters", "feet",         "inches"};
        Json entry = {{"scale_mode", mode}, {"name", names[mode]}};
        double value = mode == 0 ? 1 : mode == 2 ? projection / sub_ratio : projection;
        const char *branch = mode == 0   ? "relative_one"
                             : mode == 2 ? "primary_factor_divided_by_display_ratio"
                                         : "primary_unit_factor";
        if (mode >= 3) {
            const double target_n = mode == 4 ? 1000 : mode >= 5 ? 10000 : 1;
            const double target_d = mode == 5 ? 3048 : mode == 6 ? 254 : 1;
            const bool compatible =
                (r.at<std::uint32_t>(76) & 7u) == 1 && display_n > 0 && display_d > 0;
            if (compatible)
                value = ((display_n * target_d) / (display_d * target_n)) * projection;
            branch = compatible ? "target_unit_ratio_times_primary_factor"
                                : "primary_unit_factor_fallback";
            entry["target_unit"] = {{"base_key", 1},
                                    {"system_key", mode >= 5 ? 1 : 2},
                                    {"numerator", target_n},
                                    {"denominator", target_d}};
        }
        entry.update(factor(value));
        entry["branch"] = branch;
        mapping_units.push_back(std::move(entry));
    }
    out["factors"]["material_mapping_units_before_reference"] = std::move(mapping_units);
    auto &meters = out["factors"]["meters_per_data_unit"];
    if (!std::isfinite(per_meter) || per_meter == 0)
        meters = {{"status", "not_evaluated"},
                  {"reason", "nonfinite_or_zero_meter_factor"},
                  {"value", nullptr}};
    else
        meters = factor(1 / per_meter);
    meters["derivation"] = "reciprocal_of_data_units_per_meter";
    return out;
}

Json material_uv_mapping_unit_factor(const Json &units, std::int32_t mapping_mode,
                                     std::int32_t scale_mode,
                                     const std::optional<double> &reference_scale) {
    Json out = {{"scope", "native_uv_mapping_unit_factor"},
                {"status", "not_evaluated"},
                {"mapping_mode", mapping_mode},
                {"scale_mode", scale_mode}};
    auto fail = [&](const char *reason) {
        out["reason"] = reason;
        return out;
    };
    if (scale_mode == 0) {
        out.update({{"status", "computed"},
                    {"value", 1.},
                    {"before_reference", 1.},
                    {"reference_scale_applied", false}});
        return out;
    }
    try {
        if (!units.is_object() || !units.contains("status") || units.at("status") != "decoded")
            return fail("selected_mapping_model_units_unavailable");
        const auto &factors = units.at("factors");
        const auto &source = scale_mode >= 1 && scale_mode <= 6
                                 ? factors.at("material_mapping_units_before_reference")
                                       .at(static_cast<unsigned>(scale_mode))
                                 : factors.at("material_projection_unit_factor");
        if (source.at("status") != "computed" || !source.at("value").is_number())
            return fail("mapping_unit_arithmetic_unavailable");
        const double initial = source.at("value").get<double>();
        double value = initial;
        const bool apply_reference = mapping_mode != 1;
        if (apply_reference) {
            if (!reference_scale || !std::isfinite(*reference_scale))
                return fail("missing_or_nonfinite_mapping_reference_scale");
            value = *reference_scale * initial;
        }
        if (!std::isfinite(initial) || !std::isfinite(value))
            return fail("nonfinite_mapping_unit_arithmetic");
        out.update({{"status", "computed"},
                    {"value", value},
                    {"before_reference", initial},
                    {"reference_scale_applied", apply_reference},
                    {"unit_branch", source.at("branch")}});
    } catch (const Json::exception &) {
        return fail("missing_or_invalid_mapping_unit_fields");
    }
    return out;
}
} // namespace p3d
