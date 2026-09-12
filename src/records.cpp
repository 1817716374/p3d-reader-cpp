#include "internal.hpp"
#include <lz4/lz4.h>
#include <miniz/miniz_tinfl.h>
namespace p3d {
class Dex : public Reader {
  public:
    Json table = Json::array();
    using Reader::Reader;
    std::uint64_t integer() {
        auto t = u8();
        if (t < 0xf0)
            return t;
        switch (t) {
        case 0xf1:
            return u8();
        case 0xf2:
            return u16();
        case 0xf3:
            return u32();
        case 0xf4:
            return u64();
        default:
            throw std::runtime_error("DEX integer tag");
        }
    }
    std::string text() {
        auto t = u8();
        require(t == 0xfa || t == 0xfb, "DEX string tag");
        auto n = integer();
        require(n <= left() / (t == 0xfb ? 2 : 1), "DEX string length");
        auto b = take(n * (t == 0xfb ? 2 : 1));
        return t == 0xfb ? utf16(b) : latin1(b);
    }
    Json indexed() {
        auto n = integer();
        require(n < table.size(), "DEX string index");
        return table[n];
    }
    Json scalar() {
        need(1);
        auto t = b[p];
        if (t < 0xf0 || (t >= 0xf1 && t <= 0xf4))
            return integer();
        p++;
        switch (t) {
        case 0xf5:
            return doubles(2);
        case 0xf7:
            return f64();
        case 0xf8:
            return Json{{"int64", i64()}};
        case 0xf9:
            return doubles(3);
        case 0xf6: {
            auto n = integer();
            auto raw = take(n);
            return {{"binary_base64", base64(raw)}, {"bytes", raw.size()}};
        }
        case 0xfa:
        case 0xfb:
            --p;
            return text();
        case 0xfd:
        case 0xfe: {
            auto st = u8();
            auto n = integer();
            require(n <= 10000000, "DEX array limit");
            Json a = Json::array();
            for (std::uint64_t i = 0; i < n; ++i) {
                Json v;
                std::uint64_t sentinel = 0;
                switch (st) {
                case 0xf1:
                    v = u8();
                    sentinel = 255;
                    break;
                case 0xf2:
                    v = u16();
                    sentinel = 65535;
                    break;
                case 0xf3:
                    v = u32();
                    sentinel = UINT32_MAX;
                    break;
                case 0xf4:
                    v = u64();
                    sentinel = UINT64_MAX;
                    break;
                case 0xf5:
                    v = doubles(2);
                    break;
                case 0xf7:
                    v = f64();
                    break;
                case 0xf8:
                    v = i64();
                    sentinel = UINT64_MAX;
                    break;
                case 0xf9:
                    v = doubles(3);
                    break;
                default:
                    throw std::runtime_error("DEX array type");
                }
                if (t == 0xfe) {
                    require(v.is_number_integer(), "DEX string array type");
                    auto k = v.get<std::uint64_t>();
                    require(k < table.size() || k == sentinel, "DEX string array index");
                    v = k == sentinel ? Json{{"null_string_index", sentinel}} : table[k];
                }
                a.push_back(std::move(v));
            }
            return a;
        }
        default:
            throw std::runtime_error("DEX scalar tag");
        }
    }
    Json value() {
        auto t = u8();
        if (t == 0x10)
            return scalar();
        if (t == 0x11)
            return indexed();
        throw std::runtime_error("DEX value token");
    }
    Json node(unsigned depth = 0) {
        require(depth <= 200, "DEX node depth");
        auto t = u8();
        require(t == 2 || t == 3, "DEX node token");
        Json n = {{"name", indexed()}};
        if (t == 3) {
            Json attrs = Json::object();
            while (true) {
                auto tok = u8();
                if (tok == 6)
                    break;
                require(tok == 5, "DEX attribute token");
                auto k = indexed().get<std::string>();
                attrs[k] = value();
            }
            n["attributes"] = std::move(attrs);
        }
        Json children = Json::array(), values = Json::array();
        while (left()) {
            auto tok = b[p];
            if (tok == 4) {
                p++;
                if (!children.empty())
                    n["children"] = std::move(children);
                if (!values.empty())
                    n["value"] = values.size() == 1 ? std::move(values[0]) : std::move(values);
                return n;
            }
            if (tok == 2 || tok == 3)
                children.push_back(node(depth + 1));
            else if (tok == 0x10 || tok == 0x11)
                values.push_back(value());
            else
                throw std::runtime_error("DEX unknown node token");
        }
        throw std::runtime_error("DEX unclosed node");
    }
};
Json parse_dex(const Bytes &b, std::size_t &offset, bool enrich) {
    Dex r(b, offset);
    require(hex(r.take(10)) == "50334444455800ff0a0d", "DEX magic");
    auto ver = hex(r.take(6));
    require(r.u8() == 0x30, "DEX string table marker");
    auto count = r.integer();
    require(count <= 1000000, "DEX table count");
    for (std::uint64_t i = 0; i < count; ++i)
        r.table.push_back(r.text());
    auto root = r.node();
    if (enrich)
        enrich_tree(root);
    offset = r.p;
    return {{"version_hex", ver}, {"string_table", std::move(r.table)}, {"root", std::move(root)}};
}
Json parse_native(const Bytes &b) {
    Json out = Json::array();
    Reader r(b);
    auto linkage_bytes = [](unsigned h) -> std::size_t {
        if (!(h & 0x1000))
            return 8;
        if (!(h & 0x4000))
            return ((h & 255) + 1) * 2;
        auto words = (h & 255) << ((h >> 8) & 15);
        require(words >= 2 && words <= 65535, "native extended linkage length");
        return words * 2;
    };
    std::size_t pos = 0;
    while (pos < b.size()) {
        r.p = pos;
        r.need(36);
        r.skip(4);
        auto type = r.u16(), flags = r.u16();
        auto words = r.u32(), base = r.u32();
        std::size_t end = pos + 4 + std::uint64_t(words) * 2,
                    attr = pos + 4 + std::uint64_t(base) * 2;
        require(pos + 36 <= attr && attr <= end && end <= b.size(), "native record length");
        auto id = r.at<std::uint64_t>(pos + 20);
        unsigned adjustment = 0;
        auto valid = [&](std::size_t p) {
            while (p < end) {
                if (std::all_of(b.begin() + p, b.begin() + end, [](auto c) { return c == 0; }))
                    return true;
                if (p + 4 > end)
                    return false;
                auto h = r.at<std::uint16_t>(p);
                std::size_t n;
                try {
                    n = linkage_bytes(h);
                } catch (const std::exception &) {
                    return false;
                }
                if (n < 4 || p + n > end)
                    return false;
                p += n;
            }
            return p == end;
        };
        // Some legacy type-49 records understate the base by four words. A
        // string linkage in that base can straddle the stated boundary; its
        // text tail also happens to look like an eight-byte non-user linkage.
        // Prefer its explicit length over that otherwise valid interpretation.
        bool crossing_string = false;
        if (type == 49 && base == 392 && attr + 8 <= end) {
            for (auto p = pos + 36; p + 12 <= attr; p += 2) {
                auto h = r.at<std::uint16_t>(p);
                if (!(h & 0x1000) || (h & 0x4000) || r.at<std::uint16_t>(p + 2) != 0x56d2)
                    continue;
                auto n = linkage_bytes(h);
                if (p + n != attr + 8)
                    continue;
                auto size = r.at<std::uint32_t>(p + 8);
                if (size <= n - 12 && p + 12 + size > attr)
                    crossing_string = true;
            }
        }
        if (type == 49 && base == 392 && (!valid(attr) || crossing_string) && attr + 8 <= end &&
            valid(attr + 8)) {
            attr += 8;
            adjustment = 8;
        }
        r.p = attr;
        Json links = Json::array(), children = Json::array();
        std::size_t pad = 0;
        while (r.p + 4 <= end) {
            if (std::all_of(b.begin() + r.p, b.begin() + end, [](auto c) { return c == 0; })) {
                pad = end - r.p;
                r.p = end;
                break;
            }
            auto off = r.p;
            auto h = r.u16(), app = r.u16();
            auto n = linkage_bytes(h);
            require(n >= 4 && off + n <= end, "native linkage length");
            auto p = r.take(n - 4);
            links.push_back(
                {{"app", app}, {"header", h}, {"offset", off}, {"payload", rawbytes(p)}});
            if (app == 0x56d0 && p.size() >= 8) {
                Reader c(p);
                auto a = c.u16(), bb = c.u16(), cc = c.u16(), num = c.u16();
                if (a == 10000 && bb == 17 && cc == 512 && 8u + 8u * num <= p.size())
                    for (unsigned j = 0; j < num; ++j)
                        children.push_back(c.u64());
            }
        }
        require(r.p == end, "native linkage alignment");
        out.push_back({{"offset", pos},
                       {"length", end - pos},
                       {"element_type", type},
                       {"element_flags", flags},
                       {"id", id},
                       {"base_length", attr - pos},
                       {"links", std::move(links)},
                       {"children", std::move(children)},
                       {"data", rawbytes(slice(b, pos, attr - pos))},
                       {"zero_padding", pad},
                       {"base_boundary_adjustment", adjustment}});
        if (type == 49 && r.at<std::uint32_t>(pos + 16) == 1)
            out.back()["layer_definition"] =
                decode_native_layer(slice(b, pos, attr - pos), out.back()["links"]);
        if (type == 10 && r.at<std::uint32_t>(pos + 16) == 1)
            out.back()["layer_table"] =
                decode_native_layer_table(slice(b, pos, attr - pos), out.back()["links"]);
        if (type == 47 && r.at<std::uint32_t>(pos + 16) == 32) {
            Json reference = {{"encoding", "model_layer_group_reference"},
                              {"source_offset", 492},
                              {"status", "unsupported_header"}};
            if (attr - pos >= 500) {
                auto table_id = r.at<std::uint64_t>(pos + 492);
                reference.update(
                    {{"table_id", table_id}, {"status", table_id ? "reference" : "none"}});
            }
            out.back()["model_layer_group_reference"] = std::move(reference);
        }
        // The in-memory native header starts after the four-byte stream prefix.
        // Type 47 / subtype 33 persists the root model's explicit link ordering.
        if (type == 47 && attr - pos >= 20 && r.at<std::uint32_t>(pos + 16) == 33) {
            Json sequence = {{"encoding", "view_link_sequence"}};
            try {
                auto data = slice(b, pos, attr - pos);
                Reader order(data, 38);
                auto flag = order.u16();
                auto count = order.u32();
                require(count <= order.left() / 8, "view link sequence count");
                Json ids = Json::array(), entries = Json::array();
                for (std::uint32_t i = 0; i < count; ++i) {
                    auto id = order.u64();
                    ids.push_back(id);
                    entries.push_back({{"source_index", i},
                                       {"source_id", id},
                                       {"kind", id == 0 ? "current_model" : "model_link"}});
                }
                order.finish();
                sequence.update({{"sequence_flag", flag},
                                 {"entry_count", count},
                                 {"entry_ids", ids},
                                 {"entries", entries},
                                 {"entry_ids_source_offset", 44},
                                 {"ordering", "source_order"},
                                 {"runtime_reconciliation", "not_evaluated"}});
            } catch (const std::exception &e) {
                sequence["decode_error"] = e.what();
            }
            out.back()["view_link_sequence"] = std::move(sequence);
        }
        pos = end;
    }
    return out;
}
Json decode_symbology(const Bytes &b) {
    Reader r(b);
    auto flags = r.u16();
    Json v = {{"flags", flags}};
    // Payload order is native branch order, not increasing bit order.
    for (auto pair : std::vector<std::pair<unsigned, std::string>>{{1, "field_0001"},
                                                                   {2, "field_0002"},
                                                                   {4, "field_0004"},
                                                                   {8, "field_0008"},
                                                                   {0x20, "field_0020"},
                                                                   {0x10, "field_0010"},
                                                                   {0x40, "field_0040"},
                                                                   {0x80, "color_index"},
                                                                   {0x100, "fill_color_index"},
                                                                   {0x200, "line_weight"},
                                                                   {0x800, "transparency"},
                                                                   {0x1000, "field_1000"},
                                                                   {0x2000, "material_id"},
                                                                   {0x400, "line_style"},
                                                                   {0x4000, "true_color_packed"}})
        if (flags & pair.first) {
            if (pair.first == 0x800)
                v[pair.second] = r.f64();
            else if (pair.first == 0x2000)
                v[pair.second] = r.u64();
            else if (pair.first == 0x400)
                v[pair.second] = r.i32();
            else
                v[pair.second] = r.u32();
        }
    if (v.contains("field_0040"))
        v["fill_mode"] = v["field_0040"];
    if (v.contains("field_1000"))
        v["subitem_index"] = v["field_1000"];
    if (v.contains("true_color_packed")) {
        auto packed = v["true_color_packed"].get<std::uint32_t>();
        v["true_color_rgb"] = {packed & 255u, (packed >> 8) & 255u, (packed >> 16) & 255u};
        v["true_color_unassigned_high_byte"] = packed >> 24;
    }
    if (flags & 0x8000) {
        auto n = r.u16();
        require(n >= 4, "line modifier size");
        auto raw = r.take(n);
        Reader m(raw);
        Json mod = {{"bytes", n}, {"flags", m.u32()}, {"raw_hex", hex(raw)}};
        if ((n - 4) % 8 == 0)
            mod["unassigned_doubles"] = Reader(raw, 4).doubles((n - 4) / 8);
        auto bits = mod["flags"].get<std::uint32_t>();
        Json fields = Json::array();
        Json named = Json::object();
        const char *scalar_names[] = {"scale",     "dash_scale", "gap_scale",     "start_width",
                                      "end_width", "shift",      "fraction_phase"};
        for (unsigned bit = 0; bit < 7; ++bit)
            if (bits & (1u << bit)) {
                auto offset = m.p;
                auto storage = Reader(raw, offset).u64();
                auto value = m.f64();
                named[scalar_names[bit]] = value;
                fields.push_back({{"flag_mask", 1u << bit},
                                  {"source_offset", offset},
                                  {"native_member_offset", 8u * (bit + 1)},
                                  {"name", scalar_names[bit]},
                                  {"storage_uint64", storage},
                                  {"float64_view", value}});
            }
        if (bits & 0x100)
            mod["orientation_vector"] = m.doubles(3);
        if (bits & 0x200)
            mod["orientation_quaternion"] = m.doubles(4);
        for (auto pair : {std::make_pair(0x800u, 0x40u), std::make_pair(0x1000u, 0x44u)})
            if (bits & pair.first) {
                auto offset = m.p;
                auto value = m.u32();
                auto name = pair.first == 0x800 ? "multiline_index" : "multiline_data_type";
                named[name] = value;
                fields.push_back({{"flag_mask", pair.first},
                                  {"source_offset", offset},
                                  {"native_member_offset", pair.second},
                                  {"name", name},
                                  {"storage_uint32", value}});
            }
        mod["fields"] = std::move(fields);
        mod["named_values"] = std::move(named);
        mod["centered_shift"] = bool(bits & 0x80);
        mod["true_scale"] = bool(bits & 0x2000);
        mod["break_at_corners"] = bool(bits & 0x40000000u);
        mod["run_through_corners"] = bool(bits & 0x80000000u);
        mod["consumed_bytes"] = m.p;
        mod["unassigned_flag_bits"] = bits & ~0xc0003bffu;
        mod["trailing_bytes"] = rawbytes(m.take(m.left()));
        mod["layout_status"] = "decoded";
        mod["semantics_status"] = "named_fields_with_unassigned_extensions_preserved";
        v["line_style_modifiers"] = mod;
    }
    r.finish();
    return v;
}
Json decode_polyface(const Bytes &b) {
    Reader r(b);
    auto flags = r.u32(), reported = r.u32();
    Json arrays = Json::array();
    for (int j = 0; j < 3; ++j) {
        auto n = r.u32();
        require(n <= r.left() / 4, "polyface indices");
        Json a = Json::array();
        for (unsigned i = 0; i < n; ++i)
            a.push_back(r.i32());
        arrays.push_back(std::move(a));
    }
    auto points = [&](unsigned stride) {
        auto n = r.u32();
        require(n <= r.left() / (8 * stride), "polyface point count");
        Json a = Json::array();
        for (unsigned i = 0; i < n; ++i)
            a.push_back(r.doubles(stride));
        return a;
    };
    auto p = points(3);
    auto tail = r.p;
    auto normals = points(3), uv = points(2);
    auto extra = r.take(r.left());
    std::array<std::size_t, 3> sizes = {p.size(), normals.size(), uv.size()};
    for (unsigned j = 0; j < 3; ++j) {
        auto &a = arrays[j];
        require(a.empty() || a.size() == arrays[0].size(), "polyface corner count");
        for (std::size_t i = 0; i < a.size(); ++i) {
            // The command reader also uses parameter indices for FloatRgb.
            // An absent UV pool disables UV lookup; it does not invalidate
            // the independently consumed color index pointer.
            if (j != 2 || !uv.empty())
                require(std::llabs(a[i].get<std::int64_t>()) <= static_cast<long long>(sizes[j]),
                        "polyface index range");
            require((a[i] == 0) == (arrays[0][i] == 0), "polyface separators");
        }
    }
    const bool fixed = flags > 1;
    if (fixed)
        require(arrays[0].size() % flags == 0, "incomplete fixed-width polygon");
    else
        require(arrays[0].empty() || arrays[0].back() == 0, "unterminated polygon");
    Json polygons = Json::array();
    auto add_polygon = [&](std::size_t start, std::size_t end) {
        Json poly;
        for (unsigned j = 0; j < 3; ++j) {
            Json a = Json::array();
            if (!arrays[j].empty())
                for (auto k = start; k < end; ++k)
                    a.push_back(arrays[j][k]);
            poly[j == 0   ? "point_indices"
                 : j == 1 ? "normal_indices"
                          : "uv_indices"] = std::move(a);
        }
        polygons.push_back(std::move(poly));
    };
    if (fixed) {
        for (std::size_t start = 0; start < arrays[0].size(); start += flags) {
            auto end = start;
            while (end < start + flags && arrays[0][end] != 0)
                ++end;
            add_polygon(start, end);
        }
    } else {
        std::size_t start = 0;
        for (std::size_t i = 0; i < arrays[0].size(); ++i)
            if (arrays[0][i] == 0) {
                add_polygon(start, i);
                start = i + 1;
            }
    }
    auto channels = decode_mesh_channels(extra, arrays, polygons, flags);
    const bool color_target = !channels["float_colors"].empty();
    const char *param_usage = arrays[2].empty() ? "no_indices"
                              : !uv.empty()     ? (color_target ? "uv_and_color" : "uv")
                              : color_target    ? "color"
                                                : "unused";
    const auto channel_status = channels["status"] == "decoded"
                                    ? (extra == Bytes(24) ? "six empty channels" : "decoded")
                                    : "opaque";
    return {{"flags", flags},
            {"num_per_face", flags},
            {"polygon_layout", fixed ? "fixed_width" : "zero_terminated"},
            {"reported_indices", reported},
            {"reported_point_index_count", reported},
            {"point_index_count_matches_header", reported == arrays[0].size()},
            {"index_arrays", std::move(arrays)},
            {"param_index_usage", param_usage},
            {"points", std::move(p)},
            {"normals", std::move(normals)},
            {"uvs", std::move(uv)},
            {"polygons", std::move(polygons)},
            {"tail_hex", hex(slice(b, tail, b.size() - tail))},
            {"trailing_channels_hex", hex(extra)},
            {"trailing_channels_status", channel_status},
            {"mesh_channels", std::move(channels)},
            {"index_convention",
             fixed ? "signed one-based indices; fixed-width faces with zero padding; sign retained"
                   : "signed one-based indices; zero terminates face; sign retained"}};
}
Json command_fields(unsigned op, const Bytes &b) {
    static std::map<unsigned, std::string> names = {{1, "polyline"},
                                                    {3, "point_list"},
                                                    {5, "elliptical_arc"},
                                                    {7, "ellipse"},
                                                    {9, "polygon"},
                                                    {11, "cone"},
                                                    {13, "push_transform"},
                                                    {14, "pop_transform"},
                                                    {16, "ruled_loft_begin"},
                                                    {17, "extrusion_begin"},
                                                    {19, "solid_end"},
                                                    {20, "open_path_begin"},
                                                    {21, "region_begin"},
                                                    {22, "path_end"},
                                                    {23, "inner_ring_separator"},
                                                    {25, "polyface"},
                                                    {28, "symbology"},
                                                    {29, "identifier"},
                                                    {30, "box_frustum"},
                                                    {34, "instance_transform"},
                                                    {37, "text"},
                                                    {40, "metadata"},
                                                    {50, "sweep_spine_marker"},
                                                    {51, "sweep_profile_marker"},
                                                    {52, "sweep_begin"},
                                                    {53, "guided_loft_first_section"},
                                                    {54, "guided_loft_second_section"},
                                                    {55, "guided_loft_guides"},
                                                    {56, "guided_loft_begin"}};
    Json v = {{"name", names.count(op) ? names.at(op) : "unknown"}};
    Reader r(b);
    try {
        if (op == 1 || op == 3 || op == 9) {
            if (op == 9)
                v["flags"] = r.u8();
            auto n = r.u32();
            require(r.left() == std::uint64_t(n) * 24, "point-list payload length");
            v["points"] = Json::array();
            for (unsigned i = 0; i < n; ++i)
                v["points"].push_back(r.doubles(3));
        } else if (op == 5 || op == 7) {
            v["origin"] = r.doubles(3);
            v["quaternion"] = r.doubles(4);
            v["radii"] = r.doubles(2);
            v["quaternion_convention"] = "inverse of instance transform convention";
            if (op == 5) {
                v["start_angle"] = r.f64();
                v["sweep_angle"] = r.f64();
            } else
                v["flags"] = r.u8();
            r.finish();
        } else if (op == 11 || op == 30) {
            v["axis_u"] = r.doubles(3);
            v["axis_v"] = r.doubles(3);
            v["origin0"] = r.doubles(3);
            v["origin1"] = r.doubles(3);
            if (op == 11) {
                v["radius0"] = r.f64();
                v["radius1"] = r.f64();
            } else {
                for (auto k : {"x0", "y0", "x1", "y1"})
                    v[k] = r.f64();
                v["origin_convention"] = "face_centers";
            }
            v["capped"] = bool(r.u8());
            r.finish();
        } else if (op == 17) {
            v["vector"] = r.doubles(3);
            v["capped"] = bool(r.u8());
        } else if (op == 55) {
            auto groups = r.u32();
            require(groups <= (b.size() - 4) / 4, "guided loft group count exceeds payload");
            v["guide_group_count"] = groups;
            v["guide_counts"] = Json::array();
            for (std::uint32_t i = 0; i < groups; ++i)
                v["guide_counts"].push_back(r.u32());
            r.finish();
        } else if (op == 16 || op == 21 || op == 52 || op == 56)
            v["flags"] = r.u8();
        else if (op == 13) {
            v["matrix_3x4"] = r.doubles(12);
            r.finish();
        } else if (op == 34) {
            v["flags"] = r.u32();
            if (b.size() == 100)
                v["matrix_3x4"] = r.doubles(12);
            else {
                v["origin"] = r.doubles(3);
                if (b.size() == 60)
                    v["quaternion"] = r.doubles(4);
            }
            r.finish();
        } else if (op == 37) {
            auto ver = r.u8();
            auto h = r.u32(), n = r.u32();
            require(std::uint64_t(h) + n == b.size() && h >= 97, "text payload length");
            auto s = utf16(slice(b, h, n));
            while (!s.empty() && s.back() == 0)
                s.pop_back();
            v["text"] = s;
            v["origin"] = r.doubles(3);
            v["quaternion"] = r.doubles(4);
            v["width"] = r.f64();
            v["height"] = r.f64();
            v["style_words"] = r.uints(4);
            v["extra_style_hex"] = hex(r.take(h - 97));
            v["version"] = ver;
            v["header_bytes"] = h;
            v["text_bytes"] = n;
        } else if (op == 28)
            v.update(decode_symbology(b));
        else if (op == 25)
            v.update(decode_polyface(b));
        else if (op == 29)
            v.update(decode_curve_identifier(b));
        else if (op == 40)
            v.update(decode_symbology_extension(b));
    } catch (const std::exception &e) {
        v["field_decode_error"] = e.what();
    }
    return v;
}
Json parse_commands(const Bytes &b) {
    Reader r(b);
    auto ver = r.u8(), flags = r.u8();
    require(ver == 2, "graphics command version");
    Json cmds = Json::array();
    while (r.left()) {
        auto off = r.p;
        auto op = r.u16();
        auto size = r.u32();
        require(size >= 4 && size - 4 <= r.left(), "graphics command size");
        auto body = r.take(size - 4);
        cmds.push_back({{"op", op},
                        {"offset", off},
                        {"body", rawbytes(body)},
                        {"decoded", command_fields(op, body)}});
    }
    return {{"version", ver}, {"flags", flags}, {"commands", std::move(cmds)}};
}
static Bytes decompress_attribute(const Bytes &b, Json &metadata) {
    Reader r(b);
    auto ver = r.u32(), n = r.u32();
    require(ver >= 1 && ver <= 3 && n > 0 && n <= 64u * 1024 * 1024, "attribute envelope");
    if (ver != 3) {
        require(n == r.left(), "stored attribute size");
        metadata = {{"version", ver}, {"codec", "stored"}, {"decoded_bytes", n}};
        return r.take(n);
    }
    Bytes out(n);
    auto payload = r.take(r.left());
    int got = LZ4_decompress_safe((const char *)payload.data(), (char *)out.data(),
                                  int(payload.size()), int(n));
    std::string codec = "lz4";
    if (got != int(n)) {
        tinfl_decompressor inf;
        tinfl_init(&inf);
        std::size_t ni = payload.size(), no = out.size();
        auto status = tinfl_decompress(&inf, payload.data(), &ni, out.data(), out.data(), &no,
                                       TINFL_FLAG_PARSE_ZLIB_HEADER |
                                           TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
        require(status == TINFL_STATUS_DONE && no == n && ni == payload.size(),
                "attribute compressed size or framing mismatch");
        codec = "zlib";
    }
    metadata = {{"version", ver}, {"codec", codec}, {"decoded_bytes", n}};
    return out;
}
Json decode_attribute(unsigned group, unsigned key, const Bytes &b, unsigned index) {
    if (group == 2 && key == 10001) {
        auto name = utf16(b);
        Json(name).dump();
        return {{"encoding", "legacy_part_material_name"}, {"material_name", name}};
    }
    if (group == 4 && key == 10001)
        return decode_material_assignment(index, b);
    if (key == 4 && group <= 1 && index == 0)
        return decode_layer_group_attribute(group, b);
    auto display = decode_display_attribute(group, key, b);
    if (!display.is_null())
        return display;
    if (key == 22913) {
        Json result;
        auto payload = decompress_attribute(b, result);
        Reader r(payload);
        auto filename = r.take(256);
        std::size_t end = 0;
        while (end < filename.size() && (filename[end] || filename[end + 1]))
            end += 2;
        require(end < filename.size(), "embedded texture filename terminator");
        auto size = r.u32(), map_type = r.u32();
        require(size > 0 && size == r.left(), "embedded texture file size");
        result.update({{"encoding", "embedded_texture_file"},
                       {"filename", utf16(slice(filename, 0, end))},
                       {"filename_field", rawbytes(filename)},
                       {"native_map_type", map_type},
                       {"attribute_group_matches_map_type", group == (map_type & 0xffffu)},
                       {"file_offset", r.p},
                       {"file_bytes", size},
                       {"file_data", rawbytes(r.take(size))}});
        // Keep the full filename field, including unused capacity, without duplicating file_data.
        return result;
    }
    if (key == 23223 && (group == 300 || group == 313)) {
        Reader r(b);
        require(r.u16() == 1, "terrain attribute version");
        if (group == 300) {
            require(b.size() == 7 && r.u8() == 21, "terrain reference");
            return {{"encoding", "terrain_geometry_reference"},
                    {"version", 1},
                    {"kind", 21},
                    {"geometry_id", r.u32()}};
        }
        require(b.size() == 98, "terrain transform size");
        Json matrix = Json::array();
        for (int i = 0; i < 3; ++i) {
            auto row = r.doubles(4);
            for (auto v : row)
                require(std::isfinite(v.get<double>()), "nonfinite terrain transform");
            matrix.push_back(row);
        }
        matrix.push_back({0., 0., 0., 1.});
        return {{"encoding", "terrain_transform"}, {"version", 1}, {"matrix", matrix}};
    }
    if (group == 0 && key == 20031) {
        auto v = parse_commands(b);
        v["encoding"] = "geometry_commands";
        return v;
    }
    if (group == 1 && key == 10001) {
        auto s = utf8(b);
        while (!s.empty() && s.back() == 0)
            s.pop_back();
        return {{"encoding", "utf8_class_name"}, {"text", s}};
    }
    Json result;
    Bytes decoded;
    try {
        decoded = decompress_attribute(b, result);
    } catch (const std::exception &) {
        return {{"encoding", "opaque"}, {"bytes", b.size()}};
    }
    const bool stored = result["codec"] == "stored";
    result["encoding"] = stored ? "uncompressed_binary" : "compressed_binary";
    result["data"] = rawbytes(decoded);
    if (key == 2006) {
        const auto &raw = decoded;
        require(raw.size() >= 2 && raw.size() % 2 == 0, "UTF16 string attribute width");
        require(raw[raw.size() - 2] == 0 && raw.back() == 0, "UTF16 string attribute terminator");
        result["encoding"] = stored ? "uncompressed_utf16_string" : "compressed_utf16_string";
        result["text"] = utf16(slice(raw, 0, raw.size() - 2));
    }
    try {
        auto s = utf16(decoded);
        while (!s.empty() && s.back() == 0)
            s.pop_back();
        auto tree = xml_tree(s);
        result.update({{"encoding", stored ? "uncompressed_utf16_xml" : "compressed_utf16_xml"},
                       {"xml", s},
                       {"tree", tree}});
        if (tree["tag"] == "ExtendedColors") {
            Json colors = Json::array();
            for (auto &c : tree["children"]) {
                Json rgb = nullptr;
                try {
                    auto text = c["attributes"].value("Color", std::string());
                    require(text.size() >= 2, "RGB");
                    if (text.front() == '(')
                        text = text.substr(1);
                    if (text.back() == ')')
                        text.pop_back();
                    std::stringstream ss(text);
                    std::string tok;
                    Json arr = Json::array();
                    while (std::getline(ss, tok, ',')) {
                        int v = std::stoi(tok);
                        require(v >= 0 && v <= 255, "RGB range");
                        arr.push_back(v);
                    }
                    require(arr.size() == 3, "RGB channels");
                    rgb = arr;
                } catch (...) {
                }
                colors.push_back({{"ordinal", colors.size()},
                                  {"rgb", rgb},
                                  {"source_attributes", c["attributes"]}});
            }
            result["color_entries"] = colors;
        }
    } catch (const std::exception &) {
    }
    return result;
}
Json parse_graphics(const Bytes &b) {
    Reader r(b);
    Json out = Json::array();
    while (r.left()) {
        auto pos = r.p;
        require(r.u32() == 0xa11b, "graphics magic");
        auto len = r.u32();
        auto flags = r.u64(), id = r.u64();
        auto count = r.u32();
        auto end = pos + 28 + std::uint64_t(len);
        require(end <= b.size() && count <= len / 16, "graphics record length");
        Json attrs = Json::array();
        for (unsigned i = 0; i < count; ++i) {
            auto group = r.u16(), key = r.u16();
            auto ix = r.u32(), sf = r.u32(), res = r.u32();
            auto size = sf & 0x7fffffff;
            require(r.p <= end && size <= end - r.p, "graphics attribute length");
            auto off = r.p;
            auto raw = r.take(size);
            Json decoded;
            try {
                decoded = decode_attribute(group, key, raw, ix);
            } catch (const std::exception &e) {
                decoded = {{"encoding", "opaque"}, {"decode_error", e.what()}};
            }
            attrs.push_back({{"group", group},
                             {"key", key},
                             {"index", ix},
                             {"size_flags", sf},
                             {"reserved", res},
                             {"offset", off},
                             {"payload", rawbytes(raw)},
                             {"decoded", std::move(decoded)}});
        }
        require(r.p + 4 == end, "graphics trailer length");
        Json rec = {{"offset", pos},
                    {"length", end - pos},
                    {"id", id},
                    {"flags", flags},
                    {"attributes", std::move(attrs)},
                    {"trailer", r.u32()}};
        for (auto &a : rec["attributes"])
            if (a["group"] == 303 && a["key"] == 23223) {
                try {
                    rec["terrain"] = decode_terrain(rec["attributes"]);
                } catch (const std::exception &e) {
                    rec["terrain_decode_error"] = e.what();
                }
                break;
            }
        out.push_back(std::move(rec));
    }
    return out;
}
Json parse_relationships(const Bytes &b) {
    Reader r(b);
    Json rows = Json::array();
    while (r.left()) {
        auto pos = r.p;
        r.need(60);
        r.skip(8);
        auto len = r.u32();
        auto oid = r.u64();
        auto end = pos + 52 + std::uint64_t(len);
        require(pos + 60 <= end && end <= b.size(), "relationship size");
        auto sc = r.u64(), si = r.u64(), tc = r.u64(), ti = r.u64(), flags = r.u64();
        auto ns = r.zstring(), cl = r.zstring(), sn = r.zstring(), sname = r.zstring();
        require(r.u64() == si, "relationship source copy");
        auto tn = r.zstring(), tname = r.zstring();
        require(r.u64() == ti, "relationship target copy");
        auto xml = r.zstring();
        require(r.p == end, "relationship record end");
        auto tree = xml_tree(xml);
        Json props = Json::object();
        for (auto &c : tree["children"]) {
            auto name = c["tag"].get<std::string>();
            auto ix = name.find('}');
            props[ix == std::string::npos ? name : name.substr(ix + 1)] = c["text"];
        }
        rows.push_back({{"offset", pos},
                        {"length", end - pos},
                        {"object_id", oid},
                        {"source_class_id", sc},
                        {"source_object_id", si},
                        {"target_class_id", tc},
                        {"target_object_id", ti},
                        {"flags", flags},
                        {"namespace", ns},
                        {"class", cl},
                        {"source_namespace", sn},
                        {"source_class", sname},
                        {"target_namespace", tn},
                        {"target_class", tname},
                        {"xml", xml},
                        {"xml_root", tree["tag"]},
                        {"properties", props}});
    }
    return rows;
}
Json read_schemas(const Document &doc) {
    Json out = Json::array();
    std::map<StreamPath, std::vector<const Json *>> streams;
    for (auto &n : doc.native_records()) {
        auto s = n["stream"].get<StreamPath>();
        if (std::find(s.begin(), s.end(), doc.index().value("P3D-SSYS", std::string())) != s.end())
            streams[s].push_back(&n);
    }
    for (auto &entry : streams) {
        auto &records = entry.second;
        for (std::size_t i = 0; i < records.size(); ++i) {
            auto &n = *records[i];
            if (n["element_type"] != 31)
                continue;
            auto b = bytesof(n["data"]);
            Reader r(b);
            Json item = {{"id", n["id"]},
                         {"record_offset", n["offset"]},
                         {"continuation_ids", Json::array()},
                         {"header", rawbytes(slice(b, 0, std::min(std::size_t(86), b.size())))},
                         {"stream", entry.first}};
            try {
                require(b.size() >= 86 && hex(slice(b, 44, 4)) == "664c4d58", "schema envelope");
                auto count = r.at<std::uint32_t>(36), expected = r.at<std::uint32_t>(82);
                require(i + count < records.size(), "schema continuations");
                Bytes raw = slice(b, 86, b.size() - 86);
                for (unsigned j = 1; j <= count; ++j) {
                    auto &c = *records[i + j];
                    auto cb = bytesof(c["data"]);
                    Reader cr(cb);
                    require(c["element_type"] == 61 && cb.size() >= 44 &&
                                cr.at<std::uint32_t>(36) == cb.size() - 44,
                            "schema continuation");
                    item["continuation_ids"].push_back(c["id"]);
                    raw.insert(raw.end(), cb.begin() + 44, cb.end());
                }
                require(raw.size() == expected, "schema XML size");
                auto s = utf16(raw);
                while (!s.empty() && s.back() == 0)
                    s.pop_back();
                auto tree = xml_tree(s);
                item.update({{"xml", s},
                             {"tree", tree},
                             {"xml_bytes", expected},
                             {"schema_attributes", tree["attributes"]}});
            } catch (const std::exception &e) {
                item["decode_error"] = e.what();
            }
            out.push_back(std::move(item));
        }
    }
    return out;
}
Json read_models(const Document &doc) {
    std::map<std::string, std::uint64_t> ids;
    for (auto e = doc.index().begin(); e != doc.index().end(); ++e)
        if (e.key()[0] == '#' && e.key().find('%') == std::string::npos)
            ids[e.value()] = std::stoull(e.key().substr(1), nullptr, 16);
    Json out = Json::object();
    for (auto &s : doc.streams()) {
        if (s.path.size() != 3 || s.path.back() != doc.index().value("P3D~MMH", std::string()) ||
            !ids.count(s.path[1]))
            continue;
        const auto &b = *s.decoded;
        Reader r(b);
        Json info = {{"physical_storage", s.path[1]}};
        if (b.size() > 0x1104) {
            r.p = 0x10e4;
            auto fields = r.doubles(4);
            info["unit_fields_at_10e4"] = fields;
            double f = fields[1];
            if (f == 1 || f == 1000)
                info["length_scale_to_meters_inferred"] = 1 / f;
            info["unit_interpretation"] = "inferred from model header and engineering dimensions; "
                                          "verify against original application";
            info["coordinate_context"] = {
                {"storage", "source_float64"},
                {"unit_status", "inferred"},
                {"scale_to_meters", info.value("length_scale_to_meters_inferred", Json())},
                {"crs", nullptr},
                {"georeferencing_status", "unresolved"},
                {"source_header_offset", 0x10e4}};
        }
        try {
            auto records = parse_native(slice(b, 4096, b.size() - 4096));
            info["layer_group_references"] = Json::array();
            for (const auto &n : records)
                if (n.contains("model_layer_group_reference")) {
                    auto reference = n["model_layer_group_reference"];
                    reference["stream"] = s.path;
                    reference["record_offset"] = 4096 + n["offset"].get<std::uint64_t>();
                    info["layer_group_references"].push_back(std::move(reference));
                }
            for (auto &n : records)
                for (auto &l : n["links"])
                    if (l["app"] == 0x56d2) {
                        auto p = bytesof(l["payload"]);
                        if (p.size() < 8)
                            continue;
                        Reader lr(p);
                        auto key = lr.u32(), len = lr.u32();
                        try {
                            auto text =
                                native_string(lr.take(std::min<std::size_t>(len, lr.left())));
                            info["string_properties"][std::to_string(key)] = text;
                            if (key == 1)
                                info["name"] = text;
                        } catch (const std::exception &) {
                        }
                    }
        } catch (const std::exception &e) {
            info["parse_error"] = e.what();
        }
        out[std::to_string(ids[s.path[1]])] = info;
    }
    return out;
}
} // namespace p3d
