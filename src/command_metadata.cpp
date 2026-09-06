#include "internal.hpp"

namespace p3d {
static Json curve_topology(const Bytes &b) {
    Json out = {{"source_bytes", rawbytes(b)}, {"ids", Json::array()}};
    if (b.size() <= 2) {
        out["status"] = "native_empty";
        out["native_type"] = 0;
        return out;
    }
    Reader r(b);
    auto type = r.u8(), code = r.u8();
    out["type"] = type;
    out["width_code"] = code;
    require(code <= 2, "curve topology identifier width");
    unsigned width = 1u << code;
    require(r.left() % width == 0, "curve topology partial identifier");
    auto count = r.left() / width;
    Json ids = Json::array();
    while (r.left())
        ids.push_back(width == 1 ? r.u8() : width == 2 ? r.u16() : r.u32());
    out["ids"] = std::move(ids);
    out["id_width_bytes"] = width;
    out["native_count"] = count & 255u;
    out["status"] = count > 255 ? "exceeds_native_count_range" : "decoded";
    static const char *labels[] = {"unknown",
                                   "brep_shared_edge",
                                   "brep_sheet_edge",
                                   "brep_silhouette",
                                   "mesh_shared_edge",
                                   "mesh_sheet_edge",
                                   "mesh_unknown",
                                   "indexed_mesh_edge",
                                   "wire",
                                   "unannounced_section_shape",
                                   "unannounced_section_wire",
                                   "visible_analytic",
                                   "visible_bounded_plane",
                                   "geometry_map",
                                   "sweep_profile",
                                   "sweep_lateral",
                                   "cut_wires",
                                   "cut_fill",
                                   "visible_intersection",
                                   "sweep_silhouette",
                                   "brep_isoline",
                                   "curve_array",
                                   "polyface_cut",
                                   "polyface_edge",
                                   "mesh_edge_vertices",
                                   "brep_planar_face"};
    out["type_name"] = type < sizeof(labels) / sizeof(*labels) ? labels[type] : "unassigned";
    return out;
}
Json decode_curve_identifier(const Bytes &b) {
    Reader r(b);
    auto marker = r.u32();
    Json out = {{"marker_hex", hex(slice(b, 0, 4))}};
    // Retain the old eight-byte view for callers; it is not a native object ID.
    if (b.size() == 12)
        out["identifier"] = Reader(b, 4).u64();
    if (marker != 0x45644964) {
        out["identifier_status"] = "unrecognized_marker";
        out["identifier_data"] = rawbytes(r.take(r.left()));
        return out;
    }
    auto header = r.u32();
    auto type = header & 65535u, extra = header >> 16;
    auto data = r.take(r.left());
    Json id = {{"encoding", "native_curve_identifier"},
               {"type", type},
               {"header_word", header},
               {"declared_id_bytes", extra}};
    static const char *labels[] = {
        "parasolid_cut",   "unused_facet_set",     "acis_cut",        "cached_edge", "cached_cut",
        "cached_underlay", "parasolid_body",       "solid_primitive", "curve_array", "polyface_cut",
        "polyface_edge",   "unspecified_topology", "opencascade_body"};
    id["type_name"] = type < sizeof(labels) / sizeof(*labels) ? labels[type]
                      : type == 99                            ? "cut_geometry"
                                                              : "unassigned";
    id["compound_draw_state_status"] = extra ? "invalid_fallback_to_id_data" : "absent";
    if (extra && std::size_t(extra) + 2 < data.size()) {
        auto size = Reader(data, extra).u16();
        if (size >= 2 && std::size_t(extra) + 2 + size == data.size()) {
            auto state = slice(data, extra + 2, size);
            id["compound_draw_state"] = rawbytes(state);
            Reader s(state);
            Json decoded = {{"function_code", s.u16()}, {"unassigned_words", Json::array()}};
            while (s.left() >= 2)
                decoded["unassigned_words"].push_back(s.u16());
            // The native loader uses floor(byte_count / 2); preserve its ignored byte too.
            decoded["trailing_bytes"] = rawbytes(s.take(s.left()));
            decoded["semantics_status"] = "unassigned_function_and_word_meanings";
            id["compound_draw_state_decoded"] = std::move(decoded);
            id["compound_draw_state_status"] =
                size % 2 ? "decoded_with_trailing_byte" : "decoded_layout";
            data.resize(extra);
        }
    }
    id["id_data"] = rawbytes(data);
    // The native accessor treats types 0..3 and 99 as opaque ID bytes.
    if (type > 3 && type != 99) {
        try {
            id["topology"] = curve_topology(data);
        } catch (const std::exception &e) {
            id["topology_decode_error"] = e.what();
        }
    } else
        id["topology_status"] = "opaque_for_native_type";
    out["curve_identifier"] = std::move(id);
    out["identifier_status"] = "decoded_container";
    return out;
}
Json decode_symbology_extension(const Bytes &b) {
    Reader r(b);
    auto flags = r.u32();
    require((flags & ~7u) == 0, "unsupported symbology extension flags");
    Json out = {{"extension_flags", flags}};
    // Compatibility views of the previously assumed fixed eight-byte layout.
    if (b.size() == 8) {
        out["kind"] = flags;
        out["value"] = Reader(b, 4).u32();
    }
    if (flags & 1) {
        auto n = r.u16();
        auto raw = r.take(n);
        Json block = {{"source_bytes", rawbytes(raw)}, {"byte_count", n}};
        if (!n)
            block["status"] = "clear";
        else {
            require(n <= 160, "symbology fill block exceeds native storage");
            auto padded = raw;
            padded.resize(160, 0); // Native reader explicitly zeroes the entire fixed buffer.
            Reader f(padded);
            block["unassigned_scalars"] = f.doubles(3);
            block["angle"] = block["unassigned_scalars"][0];
            block["white_intensity"] = block["unassigned_scalars"][1];
            block["shift"] = block["unassigned_scalars"][2];
            auto count = f.u16();
            block["declared_entry_count"] = count;
            block["mode_code"] = f.u16();
            auto mode = block["mode_code"].get<unsigned>();
            const char *modes[] = {"unassigned",  "linear",    "curved",
                                   "cylindrical", "spherical", "hemispherical"};
            block["mode_name"] = mode < 6 ? modes[mode] : "unassigned";
            block["flags"] = f.u16();
            block["unassigned_header_bytes"] = rawbytes(f.take(2));
            block["zero_padded_bytes"] = 160 - n;
            Json entries = Json::array();
            for (unsigned i = 0; i < std::min<unsigned>(count, 8); ++i) {
                auto position = f.f64();
                auto channels = f.take(3);
                entries.push_back({{"position", position},
                                   {"color_bytes", channels},
                                   {"rgb", {channels[0], channels[2], channels[1]}},
                                   {"unassigned_bytes", rawbytes(f.take(5))}});
            }
            block["entries"] = std::move(entries);
            block["unused_storage"] = rawbytes(f.take(f.left()));
            block["status"] = "decoded_gradient_fill_with_unassigned_flags";
        }
        out["fill_style_block"] = std::move(block);
    }
    if (flags & 2) {
        auto inherited = r.u32();
        out["inheritance_flags"] = inherited;
        out["by_layer"] = {{"color", bool(inherited & 1)},
                           {"fill_color", bool(inherited & 2)},
                           {"line_style", bool(inherited & 4)},
                           {"line_weight", bool(inherited & 8)},
                           {"unassigned_bits", inherited & ~15u}};
    }
    if (flags & 4)
        out["layer_id"] = r.u32();
    r.finish();
    return out;
}
void apply_symbology_extension(Json &style, const Json &d) {
    require(!d.contains("field_decode_error"), "invalid symbology extension");
    if (d.contains("layer_id"))
        style["layer_id"] = d["layer_id"];
    for (auto key : {"fill_style_block", "inheritance_flags", "by_layer"})
        if (d.contains(key))
            style["native_symbology_extension"][key] = d[key];
}
void apply_symbology(Json &style, const Json &d) {
    require(!d.contains("field_decode_error"), "invalid symbology");
    if (d.contains("color_index") && !d.contains("true_color_packed")) {
        style.erase("true_color_packed");
        style.erase("true_color_rgb");
        style.erase("true_color_unassigned_high_byte");
    }
    if (d.contains("line_style"))
        style.erase("line_style_modifiers");
    auto fields = d;
    fields.erase("flags");
    style.update(fields);
}
} // namespace p3d
