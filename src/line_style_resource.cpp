#include "line_style_source.hpp"
#include <p3d/line_style.hpp>

namespace p3d {
namespace {
Json range(const Bytes &b, std::size_t start, std::size_t count) {
    return {{"source_offset", start}, {"data", rawbytes(slice(b, start, count))}};
}
// Bounded source text, not a locale-dependent replacement of invalid UTF-16.
Json name(const Bytes &b, std::size_t start) {
    Json out = {{"source_offset", start},
                {"source_units", Json::array()},
                {"value", nullptr},
                {"status", "unresolved"}};
    const auto source = slice(b, start, 128);
    std::size_t end = source.size();
    for (std::size_t i = 0; i < source.size(); i += 2) {
        const auto u = Reader(source).at<std::uint16_t>(i);
        out["source_units"].push_back(u);
        if (u == 0 && end == source.size())
            end = i;
    }
    try {
        require(end < source.size(), "line_style_name_not_terminated_in_saved_field");
        for (std::size_t i = 0; i < end; i += 2) {
            const auto u = Reader(source).at<std::uint16_t>(i);
            require(u < 0xdc00 || u > 0xdfff, "line_style_name_lone_low_surrogate");
            if (u >= 0xd800 && u <= 0xdbff) {
                require(i + 2 < end, "line_style_name_missing_low_surrogate");
                const auto low = Reader(source).at<std::uint16_t>(i + 2);
                require(low >= 0xdc00 && low <= 0xdfff, "line_style_name_surrogate_pair");
                i += 2;
            }
        }
        out.update({{"value", utf16(slice(source, 0, end))}, {"status", "decoded"}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json real(const Bytes &b, std::size_t offset) {
    const auto v = Reader(b).at<double>(offset);
    return {{"source_offset", offset},
            {"value", std::isfinite(v) ? Json(v) : Json()},
            {"finite", std::isfinite(v)},
            {"source", rawbytes(slice(b, offset, 8))}};
}
std::uint32_t resource_type(std::uint16_t kind) {
    switch (kind) {
    case 1:
        return 2004;
    case 2:
        return 2001;
    case 3:
        return 2002;
    case 4:
        return 2003;
    case 6:
        return 0;
    default:
        return kind;
    }
}
Json line_code(const Bytes &b) {
    constexpr std::size_t start = 208;
    Json out = {{"status", "unresolved"},
                {"source_offset", start},
                {"strokes", Json::array()},
                {"geometry_generation", "not_evaluated"}};
    try {
        Reader r(b);
        require(b.size() >= start + 208, "line_code_header_truncated");
        const auto count = r.at<std::uint32_t>(start + 204);
        out["stroke_count"] = count;
        out["stroke_count_source_offset"] = start + 204;
        // Native loading derives the copy size from the count, not from the
        // component's saved payload-size word. Preserve that distinction.
        require(count <= (b.size() - start - 208) / 48, "line_code_strokes_truncated");
        const auto consumed = 208 + std::size_t(count) * 48;
        out["native_consumed_bytes"] = consumed;
        out["description_source"] = range(b, start, 128);
        out["phase"] = real(b, start + 144);
        out["native_numeric_parameters"] =
            Json::array({real(b, start + 152), real(b, start + 160)});
        const auto options = r.at<std::uint32_t>(start + 192);
        const auto phase = r.at<double>(start + 144);
        out["options"] = options;
        out["iteration_limit"] = r.at<std::uint32_t>(start + 196);
        out["native_option_bits"] = {{"bit_3", bool(options & 8)}, {"bit_4", bool(options & 16)}};
        out["native_phase_mode"] = options & 1 ? 2 : phase != 0 ? 1 : options & 32 ? 3 : 0;
        out["unassigned_ranges"] = Json::array(
            {range(b, start + 128, 16), range(b, start + 168, 24), range(b, start + 200, 4)});
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto p = start + 208 + std::size_t(i) * 48;
            const auto mode = r.at<std::uint8_t>(p + 44);
            const auto width = r.at<std::uint8_t>(p + 45);
            const bool independent_end_width = width & 4;
            out["strokes"].push_back(
                {{"source_offset", p},
                 {"length", real(b, p)},
                 {"start_width", real(b, p + 8)},
                 {"saved_end_width", real(b, p + 16)},
                 {"effective_end_width", real(b, p + (independent_end_width ? 16 : 8))},
                 {"independent_end_width", independent_end_width},
                 {"is_dash", bool(mode & 1)},
                 {"stroke_mode", mode},
                 {"width_mode", width},
                 {"caps_type", r.at<std::uint8_t>(p + 46)},
                 {"native_stroke_bits",
                  {{"bit_1", bool(mode & 2)},
                   {"bit_2", bool(mode & 4)},
                   {"bit_3", bool(mode & 8)},
                   {"bit_4", bool(mode & 16)}}},
                 {"native_width_bits", {{"bit_0", bool(width & 1)}, {"bit_1", bool(width & 2)}}},
                 {"unassigned_ranges", Json::array({range(b, p + 24, 20), range(b, p + 47, 1)})}});
        }
        out["unconsumed_suffix"] = range(b, start + consumed, b.size() - start - consumed);
        out["status"] = "decoded";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace

Json decode_native_line_style_resource(const Bytes &b) {
    Json out = {{"status", "unresolved"},
                {"source_offsets_include_stream_prefix", true},
                {"resource_loading", "not_evaluated"}};
    try {
        Reader r(b);
        require(b.size() >= 36, "line_style_record_header_truncated");
        const auto type = r.at<std::uint16_t>(4);
        const auto subtype = r.at<std::uint32_t>(16);
        require((type == 10 || type == 49) && (subtype == 7 || subtype == 8),
                "line_style_resource_record_required");
        const bool names = subtype == 7;
        out["record_class"] = subtype;
        if (type == 10) {
            const auto p = r.at<std::uint16_t>(6) & 0x20 ? 108u : 36u;
            out["encoding"] =
                names ? "native_line_style_name_table" : "native_line_style_component_table";
            out["declared_descendant_count"] = r.at<std::uint32_t>(p);
            out["descendant_count_source_offset"] = p;
            out["unassigned_ranges"] =
                Json::array({range(b, 36, p - 36), range(b, p + 4, b.size() - p - 4)});
            out["status"] = "decoded";
            return out;
        }
        out["source_id"] = r.at<std::uint64_t>(20);
        if (names) {
            require(b.size() >= 228, "line_style_name_record_truncated");
            out["encoding"] = "native_line_style_name";
            const auto kind = r.at<std::uint16_t>(92);
            const auto flags = r.at<std::uint16_t>(48);
            out["name"] = name(b, 100);
            out["line_style_number"] = r.at<std::int32_t>(96);
            out["loader_flags"] = flags;
            out["creates_local_definition"] = bool(flags & 1);
            out["definition_flags"] = r.at<std::uint32_t>(68);
            out["native_numeric_parameters"] =
                Json::array({{{"source_offset", 52}, {"value", r.at<std::int32_t>(52)}},
                             {{"source_offset", 56}, {"value", r.at<std::int32_t>(56)}}});
            out["native_parameter_ratio"] = double(r.at<std::int32_t>(56)) / 100000.0;
            out["root_component"] = {{"kind", kind},
                                     {"resource_type", resource_type(kind)},
                                     {"source_id", r.at<std::uint64_t>(84)},
                                     {"source_offset", 84},
                                     {"target_resolution", "not_evaluated"}};
            out["unassigned_ranges"] =
                Json::array({range(b, 36, 12), range(b, 50, 2), range(b, 60, 8), range(b, 72, 12),
                             range(b, 94, 2), range(b, 228, b.size() - 228)});
            out["status"] = out["name"].at("status") == "decoded" ? "decoded" : "partial";
        } else {
            require(b.size() >= 208, "line_style_component_header_truncated");
            const auto kind = r.at<std::uint16_t>(70);
            out["encoding"] = "native_line_style_component";
            out["component_kind"] = kind;
            out["resource_type"] = resource_type(kind);
            out["name"] = name(b, 72);
            out["reference_count"] = r.at<std::uint16_t>(200);
            out["reference_storage_bytes"] = r.at<std::uint16_t>(202);
            out["declared_payload_bytes"] = r.at<std::uint16_t>(204);
            out["unassigned_ranges"] = Json::array({range(b, 36, 34), range(b, 206, 2)});
            if (kind == 3)
                out["line_code"] = line_code(b);
            else if (kind == 2)
                out["compound"] = decode_native_compound_line_style(b);
            else
                out["component_payload"] = {{"status", "unresolved"},
                                            {"reason", "line_style_component_kind_not_decoded"},
                                            {"source", range(b, 208, b.size() - 208)}};
            out["status"] = ((kind == 3 && out["line_code"].at("status") == "decoded") ||
                             (kind == 2 && out["compound"].at("status") == "decoded")) &&
                                    out["name"].at("status") == "decoded"
                                ? "decoded"
                                : "partial";
        }
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json read_native_line_style_resource(const Json &record) {
    try {
        return decode_native_line_style_resource(bytesof(record.at("data")));
    } catch (const std::exception &e) {
        return {{"status", "unresolved"}, {"reason", e.what()}};
    }
}
} // namespace p3d
