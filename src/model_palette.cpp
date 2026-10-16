#include "internal.hpp"
#include <p3d/color_resources.hpp>

namespace p3d {
namespace {
std::int32_t cvtt(double value) {
    const auto v = std::trunc(value);
    return v >= -2147483648. && v < 2147483648. ? static_cast<std::int32_t>(v) : INT32_MIN;
}
std::int16_t word(std::int32_t value) {
    const auto low = std::uint32_t(value) & 65535u;
    return std::int16_t(low < 32768 ? std::int32_t(low) : std::int32_t(low) - 65536);
}
Json fields(const NativeModelColorAdjustment &s) {
    return {{"value_reduction_percent", s.value_reduction_percent},
            {"saturation_reduction_percent", s.saturation_reduction_percent},
            {"hue_degrees", s.hue_degrees},
            {"flags", s.flags},
            {"absolute_saturation_and_value", bool(std::uint16_t(s.flags) & 1)},
            {"replace_hue", bool(std::uint16_t(s.flags) & 2)}};
}
Json fp(double v) {
    Bytes b(8);
    std::memcpy(b.data(), &v, 8);
    return {{"value", std::isfinite(v) ? Json(v) : Json(nullptr)}, {"bits", rawbytes(b)}};
}
// Native integer HSV -> RGB. Invalid sectors return without writing the output.
std::optional<NativeRgb> rgb(std::int32_t h, std::int32_t s, std::int32_t v) {
    if (s == 0 || h == -1) {
        const auto byte = std::uint8_t(std::uint32_t(cvtt((double(v) * 255.0) / 100.0 + .5)));
        return NativeRgb{byte, byte, byte};
    }
    double hue = h == 360 ? 0. : double(h);
    hue /= 60.;
    const auto sector = cvtt(hue);
    const double fraction = hue - double(sector), sat = double(s) / 100., val = double(v) / 100.;
    const auto p = std::uint8_t(std::uint32_t(cvtt(((1. - sat) * val) * 255. + .5)));
    const auto q = std::uint8_t(std::uint32_t(cvtt(((1. - fraction * sat) * val) * 255. + .5)));
    const auto t =
        std::uint8_t(std::uint32_t(cvtt(((1. - (1. - fraction) * sat) * val) * 255. + .5)));
    const auto top = std::uint8_t(std::uint32_t(cvtt(val * 255. + .5)));
    switch (sector) {
    case 0:
        return NativeRgb{top, t, p};
    case 1:
        return NativeRgb{q, top, p};
    case 2:
        return NativeRgb{p, top, t};
    case 3:
        return NativeRgb{p, q, top};
    case 4:
        return NativeRgb{t, p, top};
    case 5:
        return NativeRgb{top, p, q};
    default:
        return {};
    }
}
} // namespace
Json read_native_reference_palette_adjustment(const Json &record) {
    Json out = {{"status", "not_evaluated"},
                {"scope", "reference_palette_adjustment_after_persisted_links"},
                {"applies_when", "ordinary_reference_input_succeeds"},
                {"source", "initial_zero_state"},
                {"links", Json::array()}};
    NativeModelColorAdjustment state;
    try {
        const auto data = bytesof(record.at("data"));
        require(Reader(data).at<std::uint16_t>(4) == 13, "palette_adjustment_requires_type_13");
        const auto &links = record.at("links");
        require(links.is_array(), "complete_reference_link_array_required");
        for (std::size_t i = 0; i < links.size(); ++i) {
            const auto &link = links[i];
            if (link.at("app") != 0x56d5 || !(link.at("header").get<unsigned>() & 0x1000))
                continue;
            const auto payload = bytesof(link.at("payload"));
            require(payload.size() >= 2, "reference_numeric_link_key_truncated");
            const auto key = Reader(payload).u16();
            if (key != 9 && key != 11)
                continue;
            Json item = {{"linkage_index", i},
                         {"key", key},
                         {"source_payload", link.at("payload")},
                         {"status", "ignored"}};
            if (link.contains("offset"))
                item["linkage_offset"] = link.at("offset");
            require(payload.size() >= 8, "reference_palette_link_header_truncated");
            const auto count = Reader(payload).at<std::uint32_t>(4);
            item["reserved_word"] = Reader(payload).at<std::uint16_t>(2);
            item["declared_count"] = count;
            if (count != (key == 9 ? 2u : 4u)) {
                item["reason"] = "native_count_mismatch";
                out["links"].push_back(std::move(item));
                continue;
            }
            require(payload.size() >= 8 + 8 * std::size_t(count),
                    "reference_palette_values_truncated");
            std::array<std::int32_t, 4> ints{};
            Json converted = Json::array();
            for (unsigned j = 0; j < count; ++j)
                converted.push_back(nullptr);
            item["conversion_order"] = Json::array();
            auto convert = [&](unsigned j) {
                ints[j] = cvtt(Reader(payload).at<double>(8 + 8 * j));
                converted[j] = ints[j];
                item["conversion_order"].push_back(j);
            };
            if (key == 9) {
                convert(0);
                convert(1);
            } else {
                convert(1);
                convert(2);
                convert(3);
                if (std::uint32_t(ints[3]) & 3u)
                    convert(0);
            }
            item["converted_signed_32"] = std::move(converted);
            if (key == 9 &&
                (std::uint16_t(word(ints[0])) > 100 || std::uint16_t(word(ints[1])) > 100))
                item["reason"] = "native_unsigned_word_range_check";
            else if (key == 11 && !(std::uint32_t(ints[3]) & 3u))
                item["reason"] = "native_adjustment_flags_clear";
            else {
                state.value_reduction_percent = word(ints[0]);
                state.saturation_reduction_percent = word(ints[1]);
                if (key == 11) {
                    state.hue_degrees = word(ints[2]);
                    state.flags = word(ints[3]);
                }
                item["status"] = "applied";
                item["state_after"] = fields(state);
                out["source"] = "ordered_persisted_links";
                out["selected_linkage_index"] = i;
            }
            item["trailing_storage"] =
                rawbytes(slice(payload, 8 + 8 * count, payload.size() - 8 - 8 * count));
            out["links"].push_back(std::move(item));
        }
        out["status"] = "decoded";
        out["state"] = fields(state);
    } catch (const std::exception &e) {
        out["reason"] = e.what();
        out["partial_state"] = fields(state);
    }
    return out;
}
Json prepare_native_model_palette(const std::array<std::uint32_t, 256> &base,
                                  const std::vector<NativeModelColorAdjustment> &chain,
                                  std::optional<bool> combine, bool complete) {
    Json out = {{"status", "not_evaluated"},
                {"scope", "initial_actual_model_palette_with_explicit_native_ancestry"},
                {"packed_colors", nullptr},
                {"ancestry_complete", complete},
                {"combine_ancestor_adjustments", combine ? Json(*combine) : Json(nullptr)},
                {"selected_ancestors", Json::array()}};
    try {
        require(!chain.empty(), "actual_current_model_adjustment_required");
        require(complete, "complete_actual_model_ancestry_required");
        require(chain.size() <= 100000, "model_palette_ancestry_budget_exceeded");
        if (!combine && chain.size() > 1) {
            const bool any_adjustment = std::any_of(chain.begin(), chain.end(), [](const auto &s) {
                return s.value_reduction_percent || s.saturation_reduction_percent ||
                       (std::uint16_t(s.flags) & 2u);
            });
            require(!any_adjustment, "native_palette_ancestry_setting_required");
            out.update({{"status", "prepared"},
                        {"adjustment_needed", false},
                        {"selection", "both_native_setting_branches_share_base"},
                        {"native_storage_action", "share_base_palette"},
                        {"packed_colors", base}});
            return out;
        }
        const std::size_t first = combine.value_or(false) ? 0 : chain.size() - 1;
        bool needed = false;
        for (std::size_t i = first; i < chain.size(); ++i) {
            const auto &s = chain[i];
            needed = needed || s.value_reduction_percent || s.saturation_reduction_percent ||
                     (std::uint16_t(s.flags) & 2u);
            out["selected_ancestors"].push_back({{"chain_index", i}, {"state", fields(s)}});
        }
        out["adjustment_needed"] = needed;
        if (!needed) {
            out.update({{"status", "prepared"},
                        {"native_storage_action", "share_base_palette"},
                        {"packed_colors", base}});
            return out;
        }
        double value_scale = (100. - chain[first].value_reduction_percent) / 100.;
        double saturation_scale = (100. - chain[first].saturation_reduction_percent) / 100.;
        std::int16_t hue = chain[first].hue_degrees;
        auto flags = std::uint16_t(chain[first].flags);
        for (std::size_t i = first + 1; i < chain.size(); ++i) {
            value_scale *= (100. - chain[i].value_reduction_percent) / 100.;
            saturation_scale *= (100. - chain[i].saturation_reduction_percent) / 100.;
            flags |= std::uint16_t(chain[i].flags) & 3u;
            if (std::uint16_t(chain[i].flags) & 2u)
                hue = chain[i].hue_degrees;
        }
        out["effective_parameters"] = {{"value_scale", fp(value_scale)},
                                       {"saturation_scale", fp(saturation_scale)},
                                       {"hue_degrees", hue},
                                       {"absolute_saturation_and_value", bool(flags & 1)},
                                       {"replace_hue", bool(flags & 2)}};
        std::optional<NativeRgb> constant;
        if ((flags & 3u) == 3u) {
            constant = rgb(hue, cvtt(saturation_scale * 100. + .5), cvtt(value_scale * 100. + .5));
            require(constant.has_value(), "native_constant_color_output_uninitialized");
        }
        std::array<std::uint32_t, 256> result = base;
        for (std::size_t i = 0; i < 255; ++i) {
            const NativeRgb original{std::uint8_t(base[i]), std::uint8_t(base[i] >> 8),
                                     std::uint8_t(base[i] >> 16)};
            auto color = constant;
            if (!color) {
                const auto hsv = native_color_hsv(original);
                double v, s;
                if (flags & 1) {
                    v = value_scale * 100.;
                    s = saturation_scale * 100.;
                } else {
                    const auto vx = double(hsv[2]) * value_scale,
                               sx = double(hsv[1]) * saturation_scale;
                    v = std::isnan(vx) ? vx : std::max(0., std::min(100., vx));
                    s = std::isnan(sx) ? sx : std::max(0., std::min(100., sx));
                }
                if (flags & 2)
                    s = std::isnan(s) ? s : std::max(40., s);
                color = rgb(flags & 2 ? hue : std::int32_t(hsv[0]), cvtt(s + .5), cvtt(v + .5));
                if (!color)
                    color = original;
            }
            result[i] = std::uint32_t((*color)[0]) | (std::uint32_t((*color)[1]) << 8) |
                        (std::uint32_t((*color)[2]) << 16);
        }
        result[255] &= 0xffffffu;
        out.update({{"status", "prepared"},
                    {"native_storage_action", "build_adjusted_palette"},
                    {"packed_colors", result},
                    {"adjusted_indices", {0, 254}},
                    {"background_rgb_preserved", true}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
