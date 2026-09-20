#include "internal.hpp"
#include "text_bytes.hpp"

namespace p3d {
namespace {
Json symbology(Reader &r) {
    const auto style = r.i32();
    const auto weight = r.u32(), color = r.u32();
    // The writer resolves the project color index before storing these four bytes.
    return {{"line_style", style},
            {"line_weight", weight},
            {"resolved_color", color},
            {"color_source", "serialized_resolved_color"}};
}
Json source_name(std::int32_t source) {
    switch (source) {
    case 0:
        return "part";
    case 1:
        return "entity";
    case 2:
        return "level";
    case 8:
        return "reserve";
    default:
        return nullptr;
    }
}
void material(Reader &r, std::size_t size, Json &out) {
    out["inline_material"] = nullptr;
    if (!size)
        return;
    const auto data = r.take(size);
    Json m = {{"raw_base64", base64(data)}};
    try {
        m.update(decode_inline_material(data));
    } catch (const std::exception &e) {
        m["decode_error"] = e.what();
    }
    out["inline_material"] = std::move(m);
}
void footer(Reader &r, Json &out, bool entry) {
    const auto start = r.p;
    out["inline_material"] = nullptr;
    if (!r.left()) {
        out["footer_status"] = "not_present";
        return;
    }
    const auto marker = r.b[r.p];
    if (marker <= 1) {
        // Legacy form starts with BPMaterial::is_valid, without a length prefix.
        material(r, r.left(), out);
        out["footer_format"] = "legacy_inline_material";
    } else {
        require(marker == 2 || marker == 3, "graphics footer marker");
        out["footer_version"] = r.u8();
        const auto size = r.i32();
        require(size >= 0, "graphics material byte count");
        material(r, std::size_t(size), out);
        out["footer_format"] = "length_prefixed_material";
        const auto extension_size = entry ? 28u : 8u;
        if (r.left() >= extension_size) {
            out["line_style_scale"] = r.f64();
            if (entry) {
                out["start_width"] = r.f64();
                out["end_width"] = r.f64();
                out["layer_id"] = r.u32();
            }
        }
    }
    out["footer_status"] = r.left() ? "unassigned_suffix" : "decoded";
    if (r.left())
        out["unassigned_suffix_hex"] = hex(r.take(r.left()));
    out["footer_hex"] = hex(slice(r.b, start, r.p - start));
}
Json entry_bytes(const Bytes &b) {
    Reader r(b);
    const auto source = r.i32();
    const auto type = r.i32();
    Json out = {{"symbology_source", source}, {"symbology_source_name", source_name(source)},
                {"geometry_type", type},      {"symbology", symbology(r)},
                {"transparency", r.f64()},    {"view_flag_status", "not_serialized"}};
    static const std::map<int, const char *> types = {
        {0, "None"},           {1, "GeCurveBase"},      {2, "GeCurveArray"}, {3, "Polyface"},
        {4, "GeBsplineCurve"}, {5, "GeBsplineSurface"}, {6, "GeSolidBase"},  {7, "Text"},
        {8, "SolidEntity"},    {10, "GeCsgTree"}};
    out["geometry_type_name"] = types.count(type) ? Json(types.at(type)) : Json();
    out["header_hex"] = hex(slice(b, 0, r.p));
    const auto size = r.u64();
    out["geometry_offset"] = r.p;
    require(size <= r.left(), "graphics geometry byte count");
    const auto geometry = r.take(std::size_t(size));
    out["geometry_bytes"] = size;
    out["geometry_status"] = geometry.empty() ? "empty" : "unsupported_encoding";
    if (type == 7 && !geometry.empty()) {
        try {
            out["geometry"] = decode_text_bytes(geometry);
            out["geometry"]["_type"] = "TextEntity";
            out["geometry_status"] = "decoded";
        } catch (const std::exception &e) {
            out["geometry_status"] = "invalid";
            out["geometry_decode_error"] = e.what();
        }
    } else if (geometry.size() >= 8 && hex(slice(geometry, 0, 8)) == "6267303030316662") {
        try {
            out["geometry"] = decode_bgfb(geometry);
            out["geometry_status"] = "decoded";
        } catch (const std::exception &e) {
            out["geometry_status"] = "invalid";
            out["geometry_decode_error"] = e.what();
        }
    }
    out["suffix_hex"] = hex(slice(b, r.p, r.left()));
    footer(r, out, true);
    r.finish();
    return out;
}
} // namespace
Json decode_graphics_bytes(const Bytes &b) {
    Reader r(b);
    const auto version = r.u64();
    require(version == 1, "serialized graphics version");
    const auto include_model = r.u8();
    Json out = {{"encoding", "serialized_graphics"},
                {"version", version},
                {"includes_model_id", include_model != 0},
                {"include_model_value", include_model}};
    out["model_id"] = include_model ? Json(r.u32()) : Json();
    out["symbology"] = symbology(r);
    const auto source = r.i32();
    out["symbology_source"] = source;
    out["symbology_source_name"] = source_name(source);
    out["transparency"] = r.f64();
    out["header_hex"] = hex(slice(b, 0, r.p));
    Json matrix = Json::array();
    for (unsigned i = 0; i < 3; ++i)
        matrix.push_back(r.doubles(4));
    out["transform_3x4_rows"] = std::move(matrix);
    const auto transform = r.u8();
    out["transform_enabled"] = transform != 0;
    out["transform_flag_value"] = transform;
    out["layer_id"] = r.u32();
    const auto count = r.count('Q', 8);
    out["geometry_packets"] = Json::array();
    for (std::uint64_t i = 0; i < count; ++i) {
        const auto size = r.u64();
        const auto offset = r.p;
        require(size <= r.left(), "graphics entry byte count");
        const auto data = r.take(std::size_t(size));
        Json packet = {{"entry_index", i},
                       {"source_offset", offset},
                       {"source_bytes", size},
                       {"raw_base64", base64(data)}};
        packet["native_geometry_input"] = graphics_entry_native_input(data);
        if (data.size() >= 8)
            packet["parametric_append_rule"] =
                parametric_graphics_append_rule(Reader(data, 4).i32());
        try {
            packet.update(entry_bytes(data));
        } catch (const std::exception &e) {
            packet["decode_error"] = e.what();
        }
        out["geometry_packets"].push_back(std::move(packet));
    }
    footer(r, out, false);
    out["entry_index_scope"] = "serialized_graphics";
    out["view_flag_status"] = "not_serialized";
    out["owning_element_part_mapping_status"] = "not_established";
    out["native_geometry_input"] = {{"status", "not_evaluated"},
                                    {"failure_policy", "reject_entire_graphics_container"}};
    for (const auto &packet : out.at("geometry_packets"))
        if (packet.at("native_geometry_input").at("status") == "rejected") {
            out["native_geometry_input"].update(
                {{"status", "rejected"}, {"rejecting_entry_index", packet.at("entry_index")}});
            break;
        }
    r.finish();
    return out;
}
} // namespace p3d
