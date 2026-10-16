#include "line_style_source.hpp"
#include "geometry.hpp"
#include <p3d/line_style.hpp>

namespace p3d {
namespace {
constexpr std::size_t payload = 208, buffer_start = 536;
thread_local unsigned symbol_depth = 0;
struct Depth {
    Depth() {
        ++symbol_depth;
    }
    ~Depth() {
        --symbol_depth;
    }
};
Json number(double value) {
    Bytes bits(8);
    std::memcpy(bits.data(), &value, 8);
    return {{"value", std::isfinite(value) ? Json(value) : Json()},
            {"finite", std::isfinite(value)},
            {"source", rawbytes(bits)}};
}
Json real(const Bytes &b, std::size_t p) {
    auto out = number(Reader(b).at<double>(p));
    out["source"] = rawbytes(slice(b, p, 8));
    out["source_offset"] = p;
    return out;
}
Json point(const Bytes &b, std::size_t p) {
    return {{"source_offset", p},
            {"components", Json::array({real(b, p), real(b, p + 8), real(b, p + 16)})}};
}
Json range(const Bytes &b, std::size_t p, std::size_t size) {
    return {{"source_offset", p}, {"data", rawbytes(slice(b, p, size))}};
}
void require_symbol(const Bytes &b) {
    Reader r(b);
    require(b.size() >= 532, "point_symbol_fixed_parameters_truncated");
    require(r.at<std::uint16_t>(4) == 49 && r.at<std::uint32_t>(16) == 8 &&
                r.at<std::uint16_t>(70) == 1,
            "point_symbol_component_required");
}
Json parameters(const Bytes &b, const NativeLineStyleSymbolContext &context) {
    Json out = {{"status", "unresolved"},
                {"scope", "fresh_point_symbol_parameters_before_geometry_lookup"},
                {"native_scale", nullptr},
                {"geometry_loading", "not_evaluated"}};
    try {
        require_symbol(b);
        Reader r(b);
        out["range_low"] = point(b, 352);
        out["range_high"] = point(b, 376);
        out["native_option_bit_1"] = bool(r.at<std::uint32_t>(528) & 2);
        out["saved_scale"] = real(b, 424);
        const auto saved = r.at<double>(424);
        double scale;
        if (saved > 0) {
            scale = saved; // Native COMISD/JA also accepts positive infinity.
            out["selection"] = "positive_saved_scale";
        } else if (context.owner_confirmed_absent && !context.owner_unit_factor) {
            scale = 1;
            out["selection"] = "absent_owner_default_one";
        } else {
            require(!context.owner_confirmed_absent, "conflicting_symbol_owner_context");
            require(bool(context.owner_unit_factor), "actual_symbol_owner_unit_context_required");
            out["owner_unit_factor"] = number(*context.owner_unit_factor);
            scale = 1 / *context.owner_unit_factor;
            out["selection"] = "reciprocal_owner_unit_factor";
        }
        out.update({{"native_scale", number(scale)}, {"status", "computed"}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json embedded_records(const Bytes &b) {
    Json out = {{"status", "unresolved"},
                {"records", Json::array()},
                {"encoding", "native_headers_without_stream_prefix"},
                {"geometry_reconstruction", "not_evaluated"}};
    try {
        Reader r(b);
        const auto size = r.at<std::uint32_t>(532);
        out["declared_bytes"] = size;
        out["source_offset"] = buffer_start;
        require(buffer_start <= b.size() && size <= b.size() - buffer_start,
                "point_symbol_buffer_truncated");
        const auto source = slice(b, buffer_start, size);
        out["source"] = rawbytes(source);
        out["unassigned_suffix"] = range(b, buffer_start + size, b.size() - buffer_start - size);
        bool nested_complete = true;
        for (std::size_t p = 0; p < source.size();) {
            Reader header(source, p);
            header.need(32);
            const auto words = header.at<std::uint32_t>(p + 4);
            require(words >= 16 && words <= 65535, "point_symbol_native_buffer_word_count");
            const auto bytes = std::size_t(words) * 2;
            require(bytes <= source.size() - p, "point_symbol_native_buffer_record_truncated");
            // The buffer writer copies descriptor+48, omitting the file's
            // four-byte stream prefix. Rebuild that prefix solely for the
            // existing parser and preserve the exact native source mapping.
            Bytes framed(4, 0);
            framed.insert(framed.end(), source.begin() + p, source.begin() + p + bytes);
            auto parsed = parse_native(framed);
            require(parsed.size() == 1, "point_symbol_native_buffer_record_count");
            auto record = std::move(parsed[0]);
            record["embedded_byte_offset"] = p;
            record["parent_record_source_offset"] = buffer_start + p;
            record["synthetic_stream_prefix"] = true;
            record["source_offsets_include_synthetic_prefix"] = true;
            record["native_source"] = rawbytes(slice(source, p, bytes));
            if (record.contains("line_style_component") &&
                record.at("line_style_component").at("status") != "decoded")
                nested_complete = false;
            out["records"].push_back(std::move(record));
            p += bytes;
        }
        out["status"] = nested_complete ? "decoded" : "partial";
        if (!nested_complete)
            out["reason"] = "embedded_resource_record_not_fully_decoded";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace

Json decode_native_line_style_symbol(const Bytes &b) {
    Json out = {{"status", "unresolved"},
                {"source_status", "unresolved"},
                {"source_offset", payload},
                {"resource_loading", "not_evaluated"}};
    try {
        require(symbol_depth < 32, "point_symbol_nested_resource_depth_limit");
        Depth depth;
        require_symbol(b);
        Reader r(b);
        out["source_payload"] = range(b, payload, b.size() - payload);
        out["description_source"] = range(b, payload, 128);
        out["range_low"] = point(b, 352);
        out["range_high"] = point(b, 376);
        out["offset"] = point(b, 400);
        out["saved_scale"] = real(b, 424);
        const auto flags = r.at<std::uint32_t>(528);
        out["symbol_flags"] = flags;
        out["is_3d"] = bool(flags & 1);
        out["native_option_bit_1"] = bool(flags & 2);
        out["unassigned_symbol_flags"] = flags & ~3u;
        out["unassigned_ranges"] = Json::array({range(b, 336, 16), range(b, 432, 96)});
        out["source_status"] = "decoded";
        out["initial_object_parameters"] = parameters(b, {});
        out["embedded_geometry"] = embedded_records(b);
        Json reference = {{"status", "unresolved"}, {"target_resolution", "not_evaluated"}};
        try {
            const auto p = payload + std::size_t(r.at<std::uint16_t>(204));
            const auto id = r.at<std::uint64_t>(p); // Not gated by saved reference count.
            reference.update(
                {{"status", "decoded"},
                 {"source_id", id},
                 {"source_offset", p},
                 {"selection", id ? "request_owner_geometry" : "no_geometry_request"}});
        } catch (const std::exception &e) {
            reference["reason"] = e.what();
        }
        out["geometry_reference"] = std::move(reference);
        out["status"] = out["embedded_geometry"].at("status") == "decoded" &&
                                out["geometry_reference"].at("status") == "decoded"
                            ? "decoded"
                            : "partial";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json prepare_native_line_style_symbol_parameters(const Json &record,
                                                 const NativeLineStyleSymbolContext &context) {
    try {
        return parameters(bytesof(record.at("data")), context);
    } catch (const std::exception &e) {
        return {{"status", "unresolved"}, {"reason", e.what()}};
    }
}

Geometry reconstruct_native_line_style_symbol_record(const Json &source,
                                                     const Tessellation &policy) {
    auto part = reconstruct_native(source, policy);
    const auto type = source.at("element_type").get<unsigned>();
    if (type == 97)
        part.unknown.push_back({{"reason", "symbol_graphics_commands_require_companion_records"}});
    if (type == 19 || type == 20 || type == 21)
        part.unknown.push_back({{"reason", "symbol_composite_assembly_not_evaluated"}});
    if (!part.texts.empty())
        part.unknown.push_back({{"reason", "symbol_text_glyph_geometry_not_loaded"}});
    return part;
}

Geometry reconstruct_native_line_style_symbol_buffer(const Json &record,
                                                     const Tessellation &policy) {
    Geometry result;
    try {
        const auto decoded = read_native_line_style_resource(record);
        const auto &embedded = decoded.at("point_symbol").at("embedded_geometry");
        if (embedded.at("status") != "decoded")
            result.unknown.push_back({{"reason", "incomplete_native_symbol_buffer"},
                                      {"detail", embedded.value("reason", std::string())}});
        for (std::size_t i = 0; i < embedded.at("records").size(); ++i) {
            const auto &source = embedded.at("records")[i];
            auto part = reconstruct_native_line_style_symbol_record(source, policy);
            for (auto &item : part.unknown)
                item["symbol_record_index"] = i;
            for (auto &item : part.primitive_ranges)
                item["symbol_record_index"] = i;
            merge_geometry(result, part, identity());
        }
        result.notes.push_back("Source buffer coordinates; owner geometry reference, symbol "
                               "placement and appearance are not evaluated.");
    } catch (const std::exception &e) {
        result.unknown.push_back({{"reason", e.what()}});
    }
    return result;
}
} // namespace p3d
