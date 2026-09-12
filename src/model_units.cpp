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
} // namespace p3d
