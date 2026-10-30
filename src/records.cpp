#include "internal.hpp"
#include "block_layer_inputs.hpp"
#include "line_style_source.hpp"
#include <p3d/csg.hpp>
#include "text_bytes.hpp"
#include "material_catalog_read.hpp"
#include <lz4/lz4.h>
#include <miniz/miniz_tinfl.h>
#include <p3d/native_metadata.hpp>
#include <p3d/material_assignment_table.hpp>
#include <p3d/color_resources.hpp>
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
Json parse_dex(const Bytes &b, std::size_t &offset, bool enrich,
               const std::vector<NarrowStringDecoder> &string_decoders) {
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
        enrich_tree(root, "", Json::object(), string_decoders);
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
        bool material_reference_seen = false;
        bool material_name_seen = false;
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
            if ((h & 0x1000) && app == 0x4f5a) {
                auto reference = decode_native_material_name(p);
                reference["reader_selection"] = material_name_seen ? "shadowed" : "first_match";
                material_name_seen = true;
                links.back()["decoded"] = std::move(reference);
            }
            if ((h & 0x1000) && app == 0x41 && p.size() >= 4 && Reader(p).u32() == 0x1000e) {
                Json reference = {
                    {"encoding", "native_element_material_reference"},
                    {"key", 0x1000e},
                    {"reader_selection", material_reference_seen ? "shadowed" : "first_match"},
                    {"material_id", nullptr},
                    {"material_id_offset", 8},
                    {"status", "invalid"}};
                // Selection precedes payload validation in the native getter. Keep a
                // malformed first match visible instead of substituting a later ID.
                material_reference_seen = true;
                if (p.size() >= 12) {
                    reference["material_id"] = Reader(p, 4).u64();
                    reference["status"] = p.size() == 12 ? "decoded" : "partial";
                    reference["unassigned_suffix_hex"] = hex(slice(p, 12, p.size() - 12));
                } else {
                    reference["decode_error"] =
                        "native element material reference requires 12 payload bytes";
                }
                links.back()["decoded"] = std::move(reference);
            }
            if ((h & 0x1000) && app == 0x56d0)
                links.back()["decoded"] = native_dependency_link(p);
            if ((h & 0x1000) && app == 0x56d0 && p.size() >= 8) {
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
        auto modification = decode_native_modification_time(slice(b, pos + 28, 8));
        modification["source_offset"] = 28;
        modification["source_offsets_include_stream_prefix"] = true;
        out.back()["modification_time"] = std::move(modification);
        if (type == 47 && attr - pos >= 38 && r.at<std::uint32_t>(pos + 16) == 20 &&
            r.at<std::uint16_t>(pos + 36) == 0x56e6)
            out.back()["owner_reference_path"] =
                native_reference_path(slice(b, pos, attr - pos), out.back()["links"]);
        if (type == 47 && attr - pos >= 38 && r.at<std::uint32_t>(pos + 16) == 20 &&
            (r.at<std::uint16_t>(pos + 36) == 0x56df || r.at<std::uint16_t>(pos + 36) == 0x5704))
            out.back()["application_record"] = native_application_record(slice(b, pos, attr - pos));
        if (type == 13) {
            out.back()["reference_input"] = native_reference_input(slice(b, pos, end - pos));
            out.back()["reference_input"]["palette_adjustment"] =
                read_native_reference_palette_adjustment(out.back());
            out.back()["reference_target"] =
                native_reference_target(out.back()["reference_input"], out.back()["links"]);
        }
        if (type == 62) {
            out.back()["block_layer_inputs"] = native_block_layer_inputs(slice(b, pos, attr - pos));
            try {
                out.back()["block_transform"] = native_block_transform(slice(b, pos, attr - pos));
            } catch (const std::exception &e) {
                out.back()["block_transform"] = {{"status", "invalid"}, {"error", e.what()}};
            }
        }
        static const std::set<unsigned> graphic_header_types = {19, 20, 21, 37, 40,
                                                                44, 52, 54, 62, 97};
        if (graphic_header_types.count(type)) {
            out.back()["native_graphic_header"] = {
                {"encoding", "stored_graphic_layer_header"},
                {"status", "decoded"},
                {"source_offset", 16},
                {"layer_id", r.at<std::uint32_t>(pos + 16)},
                {"entity_getter_policy", type == 21 ? "recursive_first_child" : "stored_header"},
                {"draw_style_selection", "not_evaluated"}};
            if (flags & 0x20) {
                Json priority = {{"status", "not_evaluated"}, {"source_offset", 44}};
                if (attr - pos >= 48)
                    priority.update(
                        {{"status", "decoded"}, {"value", r.at<std::int32_t>(pos + 44)}});
                else
                    priority["reason"] = "extended_graphic_priority_outside_saved_header";
                out.back()["native_graphic_header"]["display_priority"] = std::move(priority);
            }
        }
        if ((graphic_header_types.count(type) || type == 27 || type == 38 || type == 41 ||
             type == 63) &&
            std::any_of(
                out.back()["links"].begin(), out.back()["links"].end(), [](const Json &link) {
                    return (link.at("header").get<unsigned>() & 0x1000) && link.at("app") == 0x79f9;
                }))
            out.back()["native_line_style_parameters"] =
                native_graphic_line_style_input(slice(b, pos, attr - pos), out.back()["links"]);
        if (type == 54) {
            out.back()["native_text_style"] =
                decode_native_text_style(slice(b, pos, attr - pos), out.back()["links"]);
            try {
                out.back()["native_text"] = decode_native_text_record(slice(b, pos, attr - pos));
            } catch (const std::exception &e) {
                out.back()["native_text"] = {{"status", "invalid"}, {"error", e.what()}};
            }
        }
        if ((type == 49 || type == 10) && r.at<std::uint32_t>(pos + 16) == 2)
            out.back()[type == 49 ? "font_definition" : "font_table"] =
                decode_native_font_record(slice(b, pos, attr - pos));
        if ((type == 49 || type == 10) &&
            (r.at<std::uint32_t>(pos + 16) == 7 || r.at<std::uint32_t>(pos + 16) == 8))
            out.back()[type == 10 ? "line_style_table"
                       : r.at<std::uint32_t>(pos + 16) == 7 ? "line_style_name"
                                                          : "line_style_component"] =
                decode_native_line_style_resource(slice(b, pos, attr - pos));
        if (type == 49 && r.at<std::uint32_t>(pos + 16) == 1)
            out.back()["layer_definition"] =
                decode_native_layer(slice(b, pos, attr - pos), out.back()["links"]);
        if (type == 10 && r.at<std::uint32_t>(pos + 16) == 1)
            out.back()["layer_table"] =
                decode_native_layer_table(slice(b, pos, attr - pos), out.back()["links"]);
        if (type == 47 && r.at<std::uint32_t>(pos + 16) == 32) {
            out.back()["model_unit_state"] = decode_model_units(slice(b, pos, attr - pos));
            out.back()["model_coordinate_state"] =
                decode_model_coordinates(slice(b, pos, attr - pos));
            Json view = {{"status", "unsupported_header"},
                         {"scope", "model_header_view_sequence_position"},
                         {"source_offsets_include_stream_prefix", true}};
            if (attr - pos >= 500) {
                const auto flags = r.at<std::uint32_t>(pos + 72);
                view.update({{"status", "decoded"},
                             {"source_flags", flags},
                             {"flags_source_offset", 72},
                             {"current_model_last_bit", 11},
                             {"current_model_last", bool(flags & 0x800u)}});
            }
            out.back()["model_view_state"] = std::move(view);
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
    if (flags & 0x3f) {
        Json display = {{"scope", "command_display_overrides"},
                        {"final_display_status", "not_evaluated"}};
        // These update the display traits, not the source/by-layer symbology.
        // The reader processes packed color before indexed color, so the
        // indexed form wins when both flags occur in a single command.
        for (auto channel :
             {std::make_pair("primary_color", 0u), std::make_pair("fill_color", 2u)}) {
            const unsigned packed_bit = 1u << channel.second;
            const unsigned index_bit = 2u << channel.second;
            const auto packed_key = channel.second ? "field_0004" : "field_0001";
            const auto index_key = channel.second ? "field_0008" : "field_0002";
            if (flags & packed_bit) {
                const auto value = v.at(packed_key).get<std::uint32_t>();
                display[channel.first] = {
                    {"encoding", "packed_rgb"},
                    {"value", value},
                    {"source_flag", packed_bit},
                    {"rgb", {value & 255u, (value >> 8) & 255u, (value >> 16) & 255u}},
                    {"high_byte", value >> 24}};
            }
            if (flags & index_bit)
                display[channel.first] = {{"encoding", "context_color_index"},
                                          {"value", v.at(index_key)},
                                          {"source_flag", index_bit},
                                          {"rgb", nullptr}};
        }
        if (flags & 0x20)
            display["line_weight_code"] = v.at("field_0020");
        if (flags & 0x10)
            display["line_pattern_code"] = v.at("field_0010");
        v["display_traits"] = std::move(display);
    }
    if (v.contains("true_color_packed")) {
        auto packed = v["true_color_packed"].get<std::uint32_t>();
        v["true_color_rgb"] = {packed & 255u, (packed >> 8) & 255u, (packed >> 16) & 255u};
        v["true_color_unassigned_high_byte"] = packed >> 24;
    }
    if (flags & 0x8000) {
        const auto n = r.u16();
        v["line_style_modifiers"] = decode_line_style_modifier_source(r.take(n));
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
    Json index_bindings = Json::object();
    for (unsigned j = 0; j < 3; ++j) {
        auto &a = arrays[j];
        require(a.empty() || a.size() == arrays[0].size(), "polyface corner count");
        Json unavailable = Json::array();
        for (std::size_t i = 0; i < a.size(); ++i) {
            const bool in_range =
                std::llabs(a[i].get<std::int64_t>()) <= static_cast<long long>(sizes[j]);
            if (!j)
                require(in_range, "polyface index range");
            else if (sizes[j] && !in_range)
                unavailable.push_back(i);
            require((a[i] == 0) == (arrays[0][i] == 0), "polyface separators");
        }
        if (j)
            index_bindings[j == 1 ? "normal_indices" : "uv_indices"] = {
                {"status", a.empty()             ? "no_indices"
                           : !sizes[j]           ? "pool_absent"
                           : unavailable.empty() ? "mapped"
                                                 : "invalid_indices"},
                {"unavailable_read_positions", std::move(unavailable)}};
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
            {"index_bindings", std::move(index_bindings)},
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
                                                    {18, "arc_sweep_begin"},
                                                    {19, "solid_end"},
                                                    {20, "open_path_begin"},
                                                    {21, "region_begin"},
                                                    {22, "path_end"},
                                                    {23, "inner_ring_separator"},
                                                    {25, "polyface"},
                                                    {26, "quadratic_bspline_segment"},
                                                    {28, "symbology"},
                                                    {29, "identifier"},
                                                    {30, "box_frustum"},
                                                    {31, "sphere_surface"},
                                                    {34, "instance_transform"},
                                                    {37, "text"},
                                                    {40, "metadata"},
                                                    {50, "sweep_spine_marker"},
                                                    {51, "sweep_profile_marker"},
                                                    {52, "sweep_begin"},
                                                    {53, "guided_loft_first_section"},
                                                    {54, "guided_loft_second_section"},
                                                    {55, "guided_loft_guides"},
                                                    {56, "guided_loft_begin"},
                                                    {58, "csg_tree"}};
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
        } else if (op == 18) {
            const auto axis_x = r.doubles(3);
            const auto axis_y = r.doubles(3);
            v["path_axis_x"] = axis_x;
            v["path_axis_y"] = axis_y;
            v["path_center"] = r.doubles(3);
            const auto auxiliary = r.doubles(5);
            v["auxiliary_values"] = auxiliary;
            const std::array<double, 3> x{axis_x[0], axis_x[1], axis_x[2]},
                y{axis_y[0], axis_y[1], axis_y[2]},
                tail{auxiliary[2], auxiliary[3], auxiliary[4]};
            const std::array<double, 3> normal{x[1] * y[2] - x[2] * y[1],
                                               x[2] * y[0] - x[0] * y[2],
                                               x[0] * y[1] - x[1] * y[0]};
            const auto normal_sq = normal[0] * normal[0] + normal[1] * normal[1] +
                                   normal[2] * normal[2];
            const auto tail_length = std::hypot(tail[0], tail[1], tail[2]);
            if (std::isfinite(normal_sq) && normal_sq > 0 && std::isfinite(tail_length)) {
                const auto scale = (tail[0] * normal[0] + tail[1] * normal[1] +
                                    tail[2] * normal[2]) / normal_sq;
                const auto residual = std::hypot(tail[0] - scale * normal[0],
                                                 tail[1] - scale * normal[1],
                                                 tail[2] - scale * normal[2]);
                const auto normal_length = std::sqrt(normal_sq);
                if (std::isfinite(scale) && std::isfinite(residual) &&
                    residual <= 1e-10 * std::max({1., tail_length,
                                                  std::abs(scale) * normal_length}))
                    v["auxiliary_tail_relation"] = {
                        {"status", scale > 0 ? "positive_path_plane_normal_multiple"
                                             : scale < 0 ? "negative_path_plane_normal_multiple"
                                                         : "zero_path_plane_normal_multiple"},
                        {"scale_factor", scale}};
                else
                    v["auxiliary_tail_relation"] = {{"status", "not_parallel"}};
            } else
                v["auxiliary_tail_relation"] = {{"status", "degenerate_or_non_finite_axes"}};
            v["sweep_angle"] = r.f64();
            v["capped"] = bool(r.u8());
            r.finish();
        } else if (op == 26) {
            require(b.size() == 146, "quadratic spline segment length");
            auto header = r.take(26);
            require(hex(header) == "0000000000000001000300000000030000000000000000000000",
                    "quadratic spline segment header");
            v["header_hex"] = hex(header);
            v["control_points"] = Json::array();
            for (unsigned i = 0; i < 3; ++i)
                v["control_points"].push_back(r.doubles(3));
            v["knots"] = r.doubles(6);
            require(v["knots"] == Json({0., 0., 0., 1., 1., 1.}),
                    "quadratic spline segment knot vector");
            v["degree"] = 2;
            r.finish();
        } else if (op == 31) {
            v["axis_x"] = r.doubles(3);
            v["axis_y"] = r.doubles(3);
            v["origin"] = r.doubles(3);
            v["radius_xy"] = r.f64();
            v["radius_z"] = r.f64();
            v["latitude_start"] = r.f64();
            v["latitude_sweep"] = r.f64();
            r.finish();
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
            v.update(decode_text_bytes(b));
        } else if (op == 58) {
            v.update(decode_csg_bytes(b));
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
static Json decode_key_22909(const Bytes &b) {
    try {
        if (b.size() == 28) {
            Reader r(b);
            Json words = Json::array();
            while (r.left())
                words.push_back(r.u32());
            if (words[0] == 6)
                return {{"encoding", "key_22909_fixed_u32_words"},
                        {"words", std::move(words)},
                        {"business_meaning", "unresolved"}};
            return nullptr;
        }
        if (b.size() < 84)
            return nullptr;

        Reader r(b);
        const auto format_word = r.u32();
        const auto variant_word = r.u32();
        const auto flag_word = r.u32();
        const auto reserved_word = r.u32();
        if (format_word != 6 || flag_word != 1 || reserved_word != 0)
            return nullptr;
        auto label_bytes = r.take(10);
        auto label = utf16(label_bytes);
        if (label.empty() || label.back() != '\0')
            return nullptr;
        label.pop_back();
        auto guid_bytes = r.take(36);
        std::string guid(guid_bytes.begin(), guid_bytes.end());
        if (r.u16() != 0)
            return nullptr;
        const auto header_word_64 = r.u32();
        const auto header_word_68 = r.u32();

        Json groups = Json::array();
        std::size_t total_entries = 0;
        while (r.left() > 12) {
            require(r.left() >= 24, "key 22909 group header");
            const auto marker = r.u32();
            require(marker == 0x80000001u, "key 22909 group marker");
            const auto group_tag = r.u32();
            const auto reserved_u64 = r.u64();
            const auto count = r.u32();
            const auto reserved_u32 = r.u32();
            require(reserved_u64 == 0 && reserved_u32 == 0,
                    "key 22909 group reserved fields");
            require(count <= r.left() / 12, "key 22909 group entry count");
            Json entries = Json::array();
            for (std::uint32_t i = 0; i < count; ++i) {
                const auto object_id = r.u64();
                const auto entry_type = r.u32();
                entries.push_back({{"object_id", object_id}, {"entry_type", entry_type}});
            }
            total_entries += count;
            groups.push_back({{"group_tag_word", group_tag},
                              {"entry_count", count},
                              {"entries", std::move(entries)}});
        }
        require(r.left() == 12, "key 22909 footer size");
        const auto footer_u64 = r.u64();
        const auto footer_u32 = r.u32();
        require(footer_u64 == 0 && footer_u32 == 1, "key 22909 footer");
        const auto group_count = groups.size();
        return {{"encoding", "key_22909_tagged_object_groups"},
                {"format_word", format_word},
                {"variant_word", variant_word},
                {"flag_word", flag_word},
                {"label", label},
                {"guid", guid},
                {"header_word_64", header_word_64},
                {"header_word_68", header_word_68},
                {"groups", std::move(groups)},
                {"group_count", group_count},
                {"entry_count", total_entries},
                {"footer", {{"u64", footer_u64}, {"u32", footer_u32}}},
                {"tag_semantics", "unresolved"},
                {"entry_type_semantics", "unresolved"}};
    } catch (const std::exception &) {
        return nullptr;
    }
}
static Json decode_dgn_display_style_type_filter(const Bytes &b) {
    Json compression;
    const auto data = decompress_attribute(b, compression);
    Json out = compression;
    out["encoding"] = "dgn_display_style_type_filter";
    out["native_type_id"] = "0x58EA1000";
    out["data"] = rawbytes(data);

    if (data.size() < 12) {
        out["status"] = "partial";
        out["unparsed_data"] = rawbytes(data);
        out["reason"] = "type_filter_header_truncated";
        return out;
    }

    Reader r(data);
    out["serialized_header_word"] = r.u16();
    out["flags"] = r.u16();
    const auto apply_mode_word = r.u32();
    const auto apply_mode = apply_mode_word & 3u;
    static const char *const apply_mode_names[] = {"all_element_types", "selected_element_types",
                                                   "all_except_selected_element_types"};
    out["apply_to_mode_word"] = apply_mode_word;
    out["apply_to_mode"] = apply_mode;
    if (apply_mode < sizeof(apply_mode_names) / sizeof(apply_mode_names[0]))
        out["apply_to_mode_name"] = apply_mode_names[apply_mode];
    out["unassigned_apply_to_mode_bits"] = apply_mode_word & ~3u;
    out["discarded_word"] = r.u32();

    // These are the element-handler tokens accepted by the native enum mapper,
    // indexed by OdDgFilterElementType's serialized numeric value.
    static const char *const element_type_tokens[] = {
        "struct Bentley::Ustn::Element::BSplineSurfaceHandler",
        "struct Bentley::Ustn::Element::BSplineCurveHandler",
        "struct Bentley::Ustn::Element::MstnCellHeaderHandler",
        "struct Bentley::Ustn::Element::SpiralCurveHandler",
        "class Bentley::Ustn::Element::AssocRegionCellHeaderHandler",
        "struct Bentley::Ustn::Element::DimensionHandler",
        "class Bentley::Ustn::Element::BrepCellHeaderHandler",
        "struct Bentley::Ustn::Element::SharedCellHandler",
        "class Bentley::Ustn::Element::NoteCellHeaderHandler",
        "struct Bentley::Ustn::Element::MultilineHandler",
        "class Bentley::Ustn::Element::GroupedHoleHandler",
        "struct Bentley::Ustn::Element::AttributeHandler",
        "struct Bentley::Ustn::Element::LineStringHandler",
        "struct Bentley::Ustn::Element::RasterHdrHandler",
        "struct Bentley::Ustn::Element::TextNodeHandler",
        "class HIERasterFrameHandler",
        "struct Bentley::Ustn::Element::ComplexStringHandler",
        "struct Bentley::Ustn::Element::MeshHeaderHandler",
        "struct Bentley::Ustn::Element::EllipseHandler",
        "struct XGraphicsDisplayHandler",
        "struct Bentley::Ustn::Element::TextElemHandler",
        "struct NamedViewDisplayHandler",
        "struct Bentley::Ustn::Element::SolidHandler",
        "struct Bentley::Ustn::Element::SectionClipElementHandler",
        "struct Bentley::Ustn::Element::ConeHandler",
        "struct Bentley::Ustn::Element::SectionCalloutHandler",
        "class FeatureNodeHandler",
        "struct Bentley::Ustn::Element::DetailCallout3DViewHandler",
        "class Bentley::Ustn::Element::DigitalSignatureCellHeaderHandler",
        "struct Bentley::Ustn::Element::ElevationCalloutHandler",
        "struct Bentley::Ustn::Element::LineHandler",
        "struct Bentley::Ustn::Element::TitleTextHandler",
        "struct Bentley::Ustn::Element::CurveHandler",
        "struct Bentley::Ustn::Element::SurfaceHandler",
        "struct Bentley::Ustn::Element::PointStringHandler",
        "struct Bentley::Markup::ContentAreaHandler",
        "struct Bentley::Ustn::Element::OleCellHeaderHandler",
        "class Bentley::Ustn::Element::AnnotationCellHeaderHandler",
        "struct Bentley::Ustn::Element::ShapeHandler",
        "struct Bentley::Ustn::Element::ComplexShapeHandler",
        "struct Bentley::Ustn::Element::DrawingTitleHandler",
        "struct Bentley::Ustn::Element::UnderlineDrawingTitleHandler",
        "struct Bentley::Ustn::Element::DrawingTitleProxyClipElementHandler",
        "struct Bentley::Ustn::Element::PlanCalloutHandler",
        "struct Bentley::Ustn::Element::OrthogonalSectionCalloutHandler",
        "struct Bentley::Ustn::Element::InteriorElevationCalloutHandler",
        "struct Bentley::Ustn::Element::DetailCalloutHandler",
        "struct Bentley::Ustn::Element::ArcHandler"};

    Json selected = Json::array();
    bool complete = apply_mode < sizeof(apply_mode_names) / sizeof(apply_mode_names[0]);
    while (r.left()) {
        const auto entry_offset = r.p;
        if (r.left() < 4) {
            complete = false;
            out["unparsed_suffix"] = rawbytes(slice(data, entry_offset, r.left()));
            break;
        }
        const auto entry_size = r.u16();
        const auto control0 = r.u8();
        const auto control1 = r.u8();
        if (entry_size < 5 || static_cast<std::size_t>(entry_size - 4) > r.left()) {
            complete = false;
            out["unparsed_suffix"] = rawbytes(slice(data, entry_offset, data.size() - entry_offset));
            break;
        }
        const auto name_bytes = r.take(entry_size - 4);
        const bool terminated = !name_bytes.empty() && name_bytes.back() == 0;
        const auto name_size = terminated ? name_bytes.size() - 1 : name_bytes.size();
        std::string name(reinterpret_cast<const char *>(name_bytes.data()), name_size);
        const bool has_embedded_nul = name.find('\0') != std::string::npos;
        Json item = {{"record_offset", entry_offset},
                     {"record_byte_count", entry_size},
                     {"control_bytes", Json::array({control0, control1})},
                     {"serialized_name_bytes", rawbytes(name_bytes)},
                     {"name_terminated", terminated},
                     {"name_has_embedded_nul", has_embedded_nul}};
        if (terminated && !has_embedded_nul) {
            item["handler_name"] = name;
            Json type_value = nullptr;
            for (unsigned i = 0; i < sizeof(element_type_tokens) / sizeof(element_type_tokens[0]); ++i)
                if (name == element_type_tokens[i]) {
                    type_value = i;
                    break;
                }
            item["element_type_value"] = std::move(type_value);
            item["element_type_status"] = item["element_type_value"].is_null() ? "unknown_token"
                                                                                 : "decoded";
            if (item["element_type_value"].is_null())
                complete = false;
        }
        selected.push_back(std::move(item));
        if (!terminated || has_embedded_nul)
            complete = false;
    }
    out["selected_element_types"] = std::move(selected);
    out["status"] = complete ? "decoded" : "partial";
    return out;
}
Json native_material_texture_file(const Bytes &b) {
    Json out;
    const auto payload = decompress_attribute(b, out);
    require(payload.size() >= 260, "native embedded texture size field truncated");
    Reader r(payload, 256);
    const auto size = r.u32();
    out["file_bytes"] = size;
    // The material reader tests a signed 32-bit length before reading the
    // filename or exporting bytes. High-bit values also fail this guard.
    if (size == 0 || size >= 0x1400000u) {
        out["status"] = "skipped";
        out["reason"] = "native_file_size_guard";
        out["data"] = rawbytes(payload);
        return out;
    }
    const auto map_type = r.u32();
    require(size <= r.left(), "native embedded texture file exceeds payload");
    std::size_t end = 0;
    while (end + 1 < payload.size() && (payload[end] || payload[end + 1]))
        end += 2;
    require(end + 1 < payload.size(), "native embedded texture filename unterminated");
    out.update({{"status", "decoded"},
                {"encoding", "embedded_texture_file"},
                {"filename", utf16(slice(payload, 0, end))},
                {"filename_field", rawbytes(slice(payload, 0, 256))},
                {"filename_terminator_offset", end},
                {"filename_within_field", end < 256},
                {"native_map_type", map_type},
                {"file_offset", 264},
                {"file_data", rawbytes(r.take(size))}});
    out["ignored_suffix"] = rawbytes(r.take(r.left()));
    return out;
}

Json decode_sesm_gams_member_data(std::uint32_t type_code, const Bytes &b) {
    if (type_code != 1001 && type_code != 1002 && type_code != 1005 && type_code != 1006)
        return nullptr;
    try {
        Reader r(b);
        require(r.left() >= 16, "SESM member prefix");
        Json prefix = Json::array();
        for (int i = 0; i < 4; ++i)
            prefix.push_back(r.u32());
        Json out = {{"prefix_u32_values", std::move(prefix)},
                    {"semantic_status", "unresolved"}};
        if (type_code == 1006 && out["prefix_u32_values"][0].get<std::uint32_t>() < 1) {
            out["native_layout_enabled"] = false;
            out["unparsed_suffix"] = rawbytes(r.take(r.left()));
            out["status"] = "native_layout_not_applied";
            return out;
        }
        if (type_code == 1001) {
            require(r.left() >= 4, "SESM type 1001 value");
            const auto source = r.u32();
            out["source_u32_value"] = source;
            out["native_u32_value"] = source == 0x80000000u ? 0u : source;
        } else if (type_code == 1002) {
            // The installed native reader reads the first two u32 values,
            // skips the next eight bytes, then dispatches the remainder to
            // a virtual callback. The callback reads the exact sequence below;
            // domain meanings of its scalar/array fields remain unknown.
            out["native_reader_prefix"] = {
                {"first_u32", out["prefix_u32_values"][0]},
                {"second_u32", out["prefix_u32_values"][1]},
                {"skipped_bytes_8_to_15", rawbytes(slice(b, 8, 8))}};
            out["callback_input_offset"] = 16;
            out["callback_input"] = rawbytes(slice(b, 16, b.size() - 16));
            if (out["prefix_u32_values"][0].get<std::uint32_t>() < 4) {
                // The native callback returns without consuming its body for
                // these prefix values. Preserve the body without applying the
                // version-4 field layout.
                out["callback_layout_enabled"] = false;
                out["callback_consumed_bytes"] = 0;
                out["callback_unparsed_suffix"] = out["callback_input"];
            } else {
                out["callback_layout_enabled"] = true;
                Reader callback(b, 16);
                require(callback.left() >= 31, "SESM type 1002 callback header");
                out["callback_flag"] = callback.u8() != 0;
                Json values = Json::array();
                for (int i = 0; i < 3; ++i)
                    values.push_back(callback.f64());
                out["callback_f64_values"] = std::move(values);
                Json words = Json::array();
                for (int i = 0; i < 3; ++i)
                    words.push_back(callback.u16());
                const auto source_count = words[2].get<std::uint16_t>();
                const auto native_count = std::min<std::uint16_t>(source_count, 8);
                out["callback_u16_values"] = std::move(words);
                out["callback_source_array_count"] = source_count;
                out["callback_native_array_count"] = native_count;
                require(native_count <= callback.left() / 11,
                        "SESM type 1002 callback arrays");
                Json triplets = Json::array();
                for (std::uint16_t i = 0; i < native_count; ++i) {
                    Json entry = Json::array();
                    for (int k = 0; k < 3; ++k)
                        entry.push_back(callback.u8());
                    triplets.push_back(std::move(entry));
                }
                out["callback_triplet_byte_arrays"] = std::move(triplets);
                Json values64 = Json::array();
                for (std::uint16_t i = 0; i < native_count; ++i)
                    values64.push_back(callback.u64());
                out["callback_u64_array"] = std::move(values64);
                out["callback_consumed_bytes"] = callback.p - 16;
                out["callback_unparsed_suffix"] = rawbytes(callback.take(callback.left()));
                r.p = callback.p;
            }
        } else if (type_code == 1005) {
            require(r.left() >= 8, "SESM type 1005 value");
            out["native_u64_value"] = r.u64();
        } else {
            out["native_layout_enabled"] = true;
            require(r.left() >= 4, "SESM type 1006 flag");
            const auto flag = r.u32();
            out["source_flag_u32"] = flag;
            out["native_flag"] = flag != 0;
            Json strings = Json::array();
            for (int i = 0; i < 2; ++i) {
                require(r.left() >= 4, "SESM type 1006 string length");
                const auto signed_count = r.i32();
                require(signed_count >= 0, "SESM type 1006 negative string length");
                const auto count = static_cast<std::size_t>(signed_count);
                require(count <= r.left() / 2, "SESM type 1006 string bytes");
                auto source = r.take(count * 2);
                const bool terminated = source.size() >= 2 && source[source.size() - 2] == 0 &&
                                        source.back() == 0;
                auto text = utf16(source);
                if (terminated && !text.empty())
                    text.pop_back();
                strings.push_back({{"code_units", count},
                                   {"text", std::move(text)},
                                   {"source_terminated", terminated},
                                   {"source_data", rawbytes(source)}});
            }
            out["native_strings"] = std::move(strings);
        }
        out["unparsed_suffix"] = rawbytes(r.take(r.left()));
        out["status"] = "layout_decoded_semantics_unresolved";
        return out;
    } catch (const std::exception &e) {
        return {{"semantic_status", "unresolved"},
                {"status", "partial"},
                {"decode_error", e.what()},
                {"unparsed_data", rawbytes(b)}};
    }
}

Json decode_sesm_gams_stream(const Bytes &b) {
    if (b.size() < 8 || latin1(slice(b, 0, 4)) != "SESM")
        return nullptr;
    Json out;
    try {
        Reader r(b);
        require(latin1(r.take(4)) == "SESM", "SESM signature");
        const auto version = r.u32();
        require(version == 1, "SESM version");
        Json header_words = Json::array();
        for (int i = 0; i < 5; ++i)
            header_words.push_back(r.u32());
        require(header_words[0].get<std::uint32_t>() >= 2, "SESM header word 0");

        Json records = Json::array();
        Json terminator = nullptr;
        while (r.left() >= 4) {
            const auto record_offset = r.p;
            const auto marker = latin1(r.take(4));
            if (marker == "DOES") {
                terminator = { {"signature", marker}, {"offset", record_offset} };
                break;
            }
            require(marker == "GAMS", "SESM record marker");
            const auto data_size = r.u32();
            const auto type_code = r.u32();
            const auto data_offset = r.p;
            require(data_size <= r.left(), "SESM record data size");
            auto data = r.take(data_size);
            Json record = {{"signature", marker},
                           {"offset", record_offset},
                           {"data_bytes", data_size},
                           {"type_code", type_code},
                           {"data_offset", data_offset},
                           {"data", rawbytes(data)},
                           {"semantic_status", "unresolved"}};
            if (type_code == 1001 || type_code == 1002 || type_code == 1005 || type_code == 1006)
                record["native_reader_dispatch"] = "typed_member";
            else if (type_code == 1007)
                record["native_reader_dispatch"] = "skipped_without_typed_member";
            auto member = decode_sesm_gams_member_data(type_code, data);
            if (!member.is_null())
                record["native_member_layout"] = std::move(member);
            records.push_back(std::move(record));
        }
        require(!terminator.is_null(), "SESM terminator");
        out = {{"encoding", "sesm_gams_record_stream"},
               {"signature", "SESM"},
               {"version", version},
               {"header_u32_values", std::move(header_words)},
               {"records", std::move(records)},
               {"terminator", std::move(terminator)},
               {"trailing_bytes", rawbytes(r.take(r.left()))},
               {"semantic_status", "unresolved"},
               {"status", "structure_decoded_semantics_unresolved"}};
    } catch (const std::exception &e) {
        out = {{"encoding", "sesm_gams_record_stream"},
               {"signature", "SESM"},
               {"status", "partial"},
               {"semantic_status", "unresolved"},
               {"decode_error", e.what()},
               {"data", rawbytes(b)}};
    }
    return out;
}

Json decode_attribute(unsigned group, unsigned key, const Bytes &b, unsigned index,
                      int measurement_type) {
    if (group == 0 && key == 22909) {
        auto decoded = decode_key_22909(b);
        if (!decoded.is_null())
            return decoded;
    }
    auto reference_extension = decode_reference_extension(group, key, b, index);
    if (!reference_extension.is_null())
        return reference_extension;
    if (group == 0 && key == 22900 && index == 1) {
        Reader r(b);
        require(r.left() >= 8, "native processor descriptor requires eight bytes");
        const auto processor_key = r.u32();
        const auto selection_mask = r.u32();
        Json out = {{"encoding", "native_processor_dispatch_descriptor"},
                    {"processor_key", processor_key},
                    {"processor_key_enabled", (processor_key >> 16) != 0},
                    {"stored_selection_mask", selection_mask},
                    {"selection_rule", "(stored_selection_mask & requested_mask) == requested_mask"},
                    {"processor_resolution", "not_evaluated"},
                    {"consumed_bytes", 8}};
        if (r.left())
            out["ignored_suffix"] = rawbytes(r.take(r.left()));
        return out;
    }
    if (group == 3 && key == 20031 && index == 0) {
        Reader r(b);
        require(b.size() == 8, "native geometry pair key requires eight bytes");
        const auto first = r.u32();
        const auto second = r.u32();
        return {{"encoding", "native_geometry_pair_lookup_key"},
                {"key_words", {first, second}},
                {"comparison", "ordered_composite_u32_pair"},
                {"lookup_domain", "unresolved"},
                {"business_meaning", "unresolved"}};
    }
    if (group == 106 && key == 10001 && index == 0) {
        auto out = decode_graphics_bytes(b);
        out["role"] = "view_link_boundary_geometry";
        return out;
    }
    if (group == 2 && key == 10001) {
        auto name = utf16(b);
        Json(name).dump();
        return {{"encoding", "legacy_part_material_name"}, {"material_name", name}};
    }
    if (group == 4 && key == 10001)
        return decode_material_assignment(index, b);
    if (group == 307 && key == 23223) {
        if (b.size() < 2)
            return {{"encoding", "terrain_display_processor_state"},
                    {"status", "truncated_type_id"},
                    {"bytes", b.size()},
                    {"state_payload", rawbytes(b)}};
        Reader r(b);
        const auto type_id = r.u16();
        std::string type_name;
        switch (type_id) {
        case 401:
            type_name = "DTMEntityTrianglesDisplayProcessor";
            break;
        case 402:
            type_name = "DTMEntityContoursDisplayProcessor";
            break;
        case 403:
            type_name = "DTMEntityRegionsDisplayProcessor";
            break;
        case 404:
            type_name = "DTMEntitySpotsDisplayProcessor";
            break;
        case 405:
            type_name = "DTMEntityFeaturesDisplayProcessor";
            break;
        case 407:
            type_name = "DTMEntityFlowArrowsDisplayProcessor";
            break;
        case 408:
            type_name = "DTMEntityHighPointsDisplayProcessor";
            break;
        case 409:
            type_name = "DTMEntityLowPointsDisplayProcessor";
            break;
        case 413:
            type_name = "DTMEntityFeatureSpotsDisplayProcessor";
            break;
        case 414:
            type_name = "DTMEntityRasterDrapingDisplayProcessor";
            break;
        default:
            break;
        }
        auto state = r.take(r.left());
        Json out = {{"encoding", "terrain_display_processor_state"},
                    {"native_type_id", type_id},
                    {"state_bytes", state.size()},
                    {"state_payload", rawbytes(state)}};
        if (type_name.empty()) {
            out["status"] = "unknown_native_type";
        } else {
            out["status"] = "processor_type_identified";
            out["native_type_name"] = type_name;
        }
        if (!type_name.empty()) {
            auto decode_base_frame = [](const Bytes &bytes, Json &decoded) {
                Reader reader(bytes);
                if (reader.left() < 4) {
                    decoded["status"] = "truncated_base_frame";
                    decoded["unmapped_state"] = rawbytes(bytes);
                    return std::size_t(0);
                }
                const auto frame_length = reader.u16();
                const auto version = reader.u16();
                if (frame_length < 2 ||
                    std::size_t(frame_length - 2) > reader.left()) {
                    decoded["status"] = "invalid_base_frame_length";
                    decoded["unmapped_state"] = rawbytes(bytes);
                    return std::size_t(0);
                }
                auto raw_fields = reader.take(std::size_t(frame_length - 2));
                Json frame = {{"version", version},
                              {"length_after_length_word", frame_length},
                              {"raw_fields", rawbytes(raw_fields)}};
                Reader fields(raw_fields);
                const auto minimum = version == 1 ? 24u : 25u;
                if (version <= 2 && fields.left() >= minimum) {
                    const auto layer_id = fields.u32();
                    const auto color = fields.u32();
                    const auto style = fields.u32();
                    const auto weight = fields.u32();
                    const auto transparency = fields.f64();
                    const auto visible_raw = version == 1 ? std::uint8_t(1) : fields.u8();
                    auto sentinel = [](std::uint32_t value, std::uint32_t by_layer,
                                       std::uint32_t by_block) {
                        if (value == by_layer)
                            return "by_layer";
                        if (value == by_block)
                            return "by_block";
                        return "explicit";
                    };
                    frame["display_properties"] = {
                        {"layer_id", layer_id},
                        {"symbology",
                         {{"color", color},
                          {"color_source", sentinel(color, 0xffffffffu, 0xfffffffeu)},
                          {"style", style},
                          {"style_source", sentinel(style, 0x7fffffffu, 0x7ffffffeu)},
                          {"weight", weight},
                          {"weight_source", sentinel(weight, 0xffffffffu, 0xfffffffeu)}}},
                        {"transparency", transparency},
                        {"visible", visible_raw != 0},
                        {"visible_raw", visible_raw}};
                    frame["padding_bytes"] = rawbytes(fields.take(fields.left()));
                    frame["status"] = "decoded";
                } else {
                    frame["status"] = "unmapped_version_or_short_fields";
                }
                decoded["base_frame"] = std::move(frame);
                return reader.p;
            };

            Json decoded = {{"status", "partial"}};
            const auto derived_offset = decode_base_frame(state, decoded);
            if (type_id == 407 && derived_offset && derived_offset <= state.size()) {
                // DTMEntityFlowArrowsProcessor::Read follows the common frame
                // with its own versioned fields.
                Reader state_reader(state, derived_offset);
                if (state_reader.left() >= 2) {
                    const auto version = state_reader.u16();
                    Json fields = {{"version", version}};
                    if (state_reader.left() >= 2 + 24 + 4) {
                        fields["layer_id"] = state_reader.u16();
                        fields["scale_vector"] =
                            Json::array({state_reader.f64(), state_reader.f64(),
                                         state_reader.f64()});
                        const auto visible_raw = state_reader.u32();
                        fields["visible"] = visible_raw != 0;
                        fields["visible_raw"] = visible_raw;
                        bool complete = true;
                        if (version > 1) {
                            if (state_reader.left() >= 12) {
                                const auto arrow_type = state_reader.i32();
                                const auto scale = state_reader.f64();
                                static const char *arrow_names[] = {
                                    "hollow", "closed", "filled", "double"};
                                fields["arrow_type"] = arrow_type;
                                if (arrow_type >= 0 && arrow_type < 4)
                                    fields["arrow_type_name"] = arrow_names[arrow_type];
                                fields["scale"] = scale;
                                fields["flow_arrows_scale"] = scale;
                            } else {
                                complete = false;
                            }
                        } else {
                            fields["arrow_type"] = 1;
                            fields["arrow_type_name"] = "closed";
                            fields["scale"] = 10000.0;
                            fields["flow_arrows_scale"] = 10000.0;
                            fields["arrow_type_and_scale_source"] = "native_default_for_version_1";
                        }
                        decoded["status"] = version > 2
                                                 ? "known_fields_decoded_with_unknown_version_tail"
                                                 : complete ? "known_fields_decoded" : "truncated_known_fields";
                    }
                    fields["unmapped_suffix"] = rawbytes(state_reader.take(state_reader.left()));
                    if (decoded.value("status", "") == "known_fields_decoded" &&
                        fields["unmapped_suffix"]["bytes"] != 0)
                        decoded["status"] = "known_fields_decoded_with_unknown_suffix";
                    decoded["flow_arrows"] = std::move(fields);
                } else {
                    decoded["status"] = "truncated_flow_arrows_state";
                }
            }
            if ((type_id == 401 || type_id == 403 || type_id == 414) && derived_offset &&
                derived_offset <= state.size()) {
                // The native triangle-processor reader consumes a length-
                // delimited versioned integer: u32 for v1, u64 otherwise.
                // The property's business meaning is not established.
                Reader triangle_reader(state, derived_offset);
                Json triangle = Json::object();
                if (triangle_reader.left() >= 2) {
                    const auto block_length = triangle_reader.u16();
                    triangle["length_after_length_word"] = block_length;
                    if (block_length <= triangle_reader.left()) {
                        const auto field_bytes = triangle_reader.take(block_length);
                        Reader fields(field_bytes);
                        if (fields.left() >= 2) {
                            const auto version = fields.u16();
                            triangle["version"] = version;
                            if (version == 1 && fields.left() >= 4)
                                triangle["serialized_value_u32"] = fields.u32();
                            else if (version != 1 && fields.left() >= 8)
                                triangle["serialized_value_u64"] = fields.u64();
                            triangle["unmapped_block_suffix"] =
                                rawbytes(fields.take(fields.left()));
                            if (type_id == 401 || type_id == 414) {
                                // Both native processor vtables route their
                                // post-triangle reader through RVA 0x3e450,
                                // paired with writer RVA 0x3e420. The reader
                                // skips two bytes; the writer emits u16(1).
                                // Preserve the value without assigning it a
                                // business meaning.
                                if (triangle_reader.left() >= 2) {
                                    const auto field_offset = triangle_reader.p;
                                    const auto value = triangle_reader.u16();
                                    triangle["post_triangle_u16"] = {
                                        {"value", value},
                                        {"serialized_bytes",
                                         rawbytes(slice(state, field_offset, 2))},
                                        {"native_writer_value", 1},
                                        {"reader_behavior", "skip_two_bytes"},
                                        {"semantic_status", "unresolved"}};
                                } else if (triangle_reader.left() == 1) {
                                    triangle["post_triangle_u16_status"] = "truncated";
                                } else {
                                    triangle["post_triangle_u16_status"] = "absent";
                                }
                            }
                            triangle["unmapped_suffix"] =
                                rawbytes(triangle_reader.take(triangle_reader.left()));
                            const bool block_complete =
                                triangle["unmapped_block_suffix"]["bytes"] == 0;
                            if (type_id == 401 || type_id == 414) {
                                const bool field_complete =
                                    triangle.contains("post_triangle_u16");
                                if (!field_complete &&
                                    triangle.value("post_triangle_u16_status", "") == "truncated")
                                    decoded["status"] = "truncated_known_field";
                                else if (block_complete && field_complete &&
                                         triangle["unmapped_suffix"]["bytes"] == 0)
                                    decoded["status"] =
                                        "known_structure_with_unresolved_field_semantics";
                                else
                                    decoded["status"] =
                                        "known_field_decoded_with_unknown_suffix";
                            } else {
                                decoded["status"] =
                                    block_complete && triangle["unmapped_suffix"]["bytes"] == 0
                                        ? "known_fields_decoded"
                                        : "known_field_decoded_with_unknown_suffix";
                            }
                        } else {
                            triangle["unmapped_block"] = rawbytes(fields.take(fields.left()));
                        }
                    } else {
                        triangle["unmapped_block"] =
                            rawbytes(triangle_reader.take(triangle_reader.left()));
                    }
                } else {
                    triangle["unmapped_suffix"] =
                        rawbytes(triangle_reader.take(triangle_reader.left()));
                }
                if (type_id == 401)
                    decoded["triangle_processor"] = std::move(triangle);
                else if (type_id == 403) {
                    triangle["role"] = "regions_triangle_subobject";
                    decoded["regions_triangle_base"] = std::move(triangle);
                } else {
                    triangle["role"] = "raster_draping_triangle_subobject";
                    decoded["raster_draping_base"] = std::move(triangle);
                }
            }
            if (type_id == 403 && derived_offset && derived_offset <= state.size()) {
                // RegionsProcessor serializes its triangle subobject first,
                // then a version u16, a u64 member, and a UTF-16 zero-
                // terminated string. Locate the extension at the end of the
                // length-delimited triangle block.
                Json regions = Json::object();
                Reader block_reader(state, derived_offset);
                bool complete = false;
                std::size_t extension_offset = state.size();
                if (block_reader.left() >= 2) {
                    const auto block_length = block_reader.u16();
                    regions["triangle_block_length_after_length_word"] = block_length;
                    if (std::size_t(block_length) <= block_reader.left()) {
                        extension_offset = block_reader.p + block_length;
                        Reader region_reader(state, extension_offset);
                        if (region_reader.left() >= 10) {
                            const auto version = region_reader.u16();
                            regions["version"] = version;
                            regions["member_0x38_u64"] = region_reader.u64();
                            if (region_reader.left() >= 2) {
                                const auto string_start = region_reader.p;
                                bool terminated = false;
                                while (region_reader.left() >= 2) {
                                    if (region_reader.u16() == 0) {
                                        terminated = true;
                                        break;
                                    }
                                }
                                if (terminated) {
                                    const auto raw = slice(state, string_start,
                                                           region_reader.p - string_start);
                                    regions["member_0x40_utf16z"] =
                                        utf16(slice(raw, 0, raw.size() - 2));
                                    regions["member_0x40_utf16z_serialized"] =
                                        rawbytes(raw);
                                    regions["unmapped_suffix"] =
                                        rawbytes(region_reader.take(region_reader.left()));
                                    complete = version == 1 &&
                                               regions["unmapped_suffix"]["bytes"] == 0;
                                } else {
                                    regions["unmapped_suffix"] =
                                        rawbytes(region_reader.take(region_reader.left()));
                                }
                            } else {
                                regions["unmapped_suffix"] =
                                    rawbytes(region_reader.take(region_reader.left()));
                            }
                        } else {
                            regions["unmapped_suffix"] =
                                rawbytes(region_reader.take(region_reader.left()));
                        }
                    } else {
                        regions["unmapped_block"] =
                            rawbytes(block_reader.take(block_reader.left()));
                    }
                } else {
                    regions["unmapped_block"] = rawbytes(block_reader.take(block_reader.left()));
                }
                const auto triangle_base =
                    decoded.value("regions_triangle_base", Json::object());
                const bool triangle_complete =
                    triangle_base.contains("version") &&
                    triangle_base.contains("unmapped_block_suffix") &&
                    triangle_base["unmapped_block_suffix"].value("bytes", 1u) == 0 &&
                    (triangle_base.contains("serialized_value_u32") ||
                     triangle_base.contains("serialized_value_u64"));
                if (complete && triangle_complete &&
                    decoded.value("base_frame", Json::object())
                            .value("status", "") == "decoded")
                    decoded["status"] = "known_fields_decoded";
                regions["extension_offset"] = extension_offset;
                decoded["regions"] = std::move(regions);
            }
            if (type_id == 402 && derived_offset && derived_offset <= state.size()) {
                // ContoursProcessor::Read (TerrainEntityProcessor.dll RVA
                // 0x38f50) reads these scalar members in order. The native
                // offsets are stable evidence. SDK BPTerrainUtil contour
                // accessors take isMajor and expose contour distance in mm;
                // the matched bound BPTerrain object's main/secondary
                // distances confirm members +0x38 and +0x40 respectively.
                Reader contour_reader(state, derived_offset);
                Json contour = Json::object();
                if (contour_reader.left() >= 2) {
                    const auto version = contour_reader.u16();
                    contour["version"] = version;
                    bool complete = true;
                    auto take_u8 = [&](const char *name) {
                        if (!complete || contour_reader.left() < 1) {
                            complete = false;
                            return;
                        }
                        contour[std::string("member_") + name] = contour_reader.u8();
                    };
                    auto take_u16 = [&](const char *name) {
                        if (!complete || contour_reader.left() < 2) {
                            complete = false;
                            return;
                        }
                        contour[std::string("member_") + name] = contour_reader.u16();
                    };
                    auto take_u32 = [&](const char *name) {
                        if (!complete || contour_reader.left() < 4) {
                            complete = false;
                            return;
                        }
                        contour[std::string("member_") + name] = contour_reader.u32();
                    };
                    auto take_f64 = [&](const char *name) {
                        if (!complete || contour_reader.left() < 8) {
                            complete = false;
                            return;
                        }
                        contour[std::string("member_") + name] = contour_reader.f64();
                    };

                    take_u8("0x38_u8");
                    take_f64("0x40_f64");
                    take_u32("0x48_u32");
                    take_f64("0x50_f64");
                    take_u32("0x58_u32");
                    take_u8("0x5c_bool_raw");
                    if (contour.contains("member_0x5c_bool_raw"))
                        contour["member_0x5c_bool"] =
                            contour["member_0x5c_bool_raw"].get<std::uint8_t>() != 0;
                    take_u16("0x5e_u16");
                    take_u16("0x60_u16");
                    take_f64("0x68_f64");
                    take_u8("0x70_u8");
                    take_f64("0x78_f64");
                    take_u32("0x30_u32");
                    if (version > 1) {
                        take_u8("0x80_u8");
                        take_f64("0x88_f64");
                    }
                    take_u32("0x98_u32");
                    take_u32("0x90_u32");
                    take_u32("0x94_u32");
                    if (version > 3)
                        take_u32("0x4c_u32");

                    if (complete && version >= 1 && version <= 4) {
                        const auto major =
                            contour.at("member_0x38_u8").get<std::uint8_t>() != 0;
                        contour["is_major_contour"] = major;
                        contour["contour_distance_mm"] = contour.at("member_0x40_f64");
                    }

                    contour["unmapped_suffix"] =
                        rawbytes(contour_reader.take(contour_reader.left()));
                    if (!complete)
                        decoded["status"] = "truncated_known_fields";
                    else if (version >= 1 && version <= 4 &&
                             contour["unmapped_suffix"]["bytes"] == 0)
                        decoded["status"] = "known_fields_decoded";
                    else
                        decoded["status"] = "known_fields_decoded_with_unknown_extension";
                } else {
                    contour["unmapped_suffix"] =
                        rawbytes(contour_reader.take(contour_reader.left()));
                    decoded["status"] = "truncated_known_fields";
                }
                decoded["contours"] = std::move(contour);
            }
            if ((type_id == 404 || type_id == 413) && derived_offset &&
                derived_offset <= state.size()) {
                // The native SpotsProcessor reader is shared by type 404 and
                // FeatureSpotsProcessor (type 413). Its serialized members
                // are a fixed prefix followed by three UTF-16 strings and
                // fixed-width scalar/vector members.
                Reader spots_reader(state, derived_offset);
                Json spots = Json::object();
                if (spots_reader.left() >= 28) {
                    spots["version"] = spots_reader.u16();
                    spots["member_0x110_u8"] = spots_reader.u8();
                    spots["member_0x38_u8"] = spots_reader.u8();
                    spots["member_0x60_vector3"] =
                        Json::array({spots_reader.f64(), spots_reader.f64(),
                                     spots_reader.f64()});
                    bool complete = true;
                    auto take_utf16z = [&](const char *name) {
                        if (!complete || spots_reader.left() < 2) {
                            complete = false;
                            return;
                        }
                        const auto start = spots_reader.p;
                        bool terminated = false;
                        while (spots_reader.left() >= 2) {
                            if (spots_reader.u16() == 0) {
                                terminated = true;
                                break;
                            }
                        }
                        if (!terminated) {
                            complete = false;
                            return;
                        }
                        const auto raw = slice(state, start, spots_reader.p - start);
                        const auto text = slice(raw, 0, raw.size() - 2);
                        spots[std::string("member_") + name] = utf16(text);
                        spots[std::string("member_") + name + "_serialized"] =
                            rawbytes(raw);
                    };
                    auto take_u32 = [&](const char *name) {
                        if (!complete || spots_reader.left() < 4) {
                            complete = false;
                            return;
                        }
                        spots[std::string("member_") + name] = spots_reader.u32();
                    };
                    auto take_vector3 = [&](const char *name) {
                        if (!complete || spots_reader.left() < 24) {
                            complete = false;
                            return;
                        }
                        spots[std::string("member_") + name] =
                            Json::array({spots_reader.f64(), spots_reader.f64(),
                                         spots_reader.f64()});
                    };
                    auto take_u16 = [&](const char *name) {
                        if (!complete || spots_reader.left() < 2) {
                            complete = false;
                            return;
                        }
                        spots[std::string("member_") + name] = spots_reader.u16();
                    };
                    take_utf16z("0x40_utf16z");
                    take_u32("0x78_u32");
                    take_utf16z("0x80_utf16z");
                    take_utf16z("0xa0_utf16z");
                    take_u32("0x30_u32");
                    take_vector3("0xc0_vector3");
                    take_u16("0xd8_u16");
                    spots["unmapped_suffix"] =
                        rawbytes(spots_reader.take(spots_reader.left()));
                    decoded["status"] = !complete
                                             ? "truncated_known_fields"
                                             : spots["version"] == 1 &&
                                                       spots["unmapped_suffix"]["bytes"] == 0
                                                   ? "known_fields_decoded"
                                                   : "known_fields_decoded_with_unknown_suffix";
                } else {
                    spots["unmapped_suffix"] =
                        rawbytes(spots_reader.take(spots_reader.left()));
                    decoded["status"] = "truncated_known_prefix";
                }
                decoded["spots"] = std::move(spots);
            }
            if (type_id == 409 && derived_offset && derived_offset <= state.size()) {
                // LowPointsProcessor::Read (RVA 0x3d880) defines this exact
                // serialized order. The current sample ends in the middle of
                // member +0xc0, so expose only fully present fields.
                Reader low_reader(state, derived_offset);
                Json low = Json::object();
                bool complete = true;
                auto take_u8 = [&](const char *name) {
                    if (!complete || low_reader.left() < 1) {
                        complete = false;
                        return;
                    }
                    low[std::string("member_") + name] = low_reader.u8();
                };
                auto take_u16 = [&](const char *name) {
                    if (!complete || low_reader.left() < 2) {
                        complete = false;
                        return;
                    }
                    low[std::string("member_") + name] = low_reader.u16();
                };
                auto take_u32 = [&](const char *name) {
                    if (!complete || low_reader.left() < 4) {
                        complete = false;
                        return;
                    }
                    low[std::string("member_") + name] = low_reader.u32();
                };
                auto take_f64 = [&](const char *name) {
                    if (!complete || low_reader.left() < 8) {
                        complete = false;
                        return;
                    }
                    low[std::string("member_") + name] = low_reader.f64();
                };
                auto take_utf16z = [&](const char *name) {
                    if (!complete || low_reader.left() < 2) {
                        complete = false;
                        return;
                    }
                    const auto start = low_reader.p;
                    bool terminated = false;
                    while (low_reader.left() >= 2) {
                        if (low_reader.u16() == 0) {
                            terminated = true;
                            break;
                        }
                    }
                    if (!terminated) {
                        // Keep the whole incomplete string in the raw suffix.
                        low_reader.p = start;
                        complete = false;
                        return;
                    }
                    const auto raw = slice(state, start, low_reader.p - start);
                    low[std::string("member_") + name] =
                        utf16(slice(raw, 0, raw.size() - 2));
                    low[std::string("member_") + name + "_serialized"] =
                        rawbytes(raw);
                };
                auto take_vector3 = [&](const char *name) {
                    if (!complete || low_reader.left() < 24) {
                        complete = false;
                        return;
                    }
                    low[std::string("member_") + name] =
                        Json::array({low_reader.f64(), low_reader.f64(),
                                     low_reader.f64()});
                };

                if (low_reader.left() >= 2) {
                    const auto version = low_reader.u16();
                    low["version"] = version;
                    take_u8("0x38_u8");
                    take_utf16z("0x40_utf16z");
                    take_vector3("0x60_vector3");
                    take_u32("0x78_u32");
                    take_f64("0x110_f64");
                    take_u32("0x30_u32");
                    take_utf16z("0x80_utf16z");
                    take_utf16z("0xa0_utf16z");
                    const auto vector3_start = low_reader.p;
                    const auto vector3_available = low_reader.left();
                    take_vector3("0xc0_vector3");
                    if (vector3_available > 0 && vector3_available < 24) {
                        // Keep an incomplete native vector recoverable: decode
                        // complete doubles from a copy, while leaving the
                        // original cursor untouched so the entire truncated
                        // field remains in unmapped_suffix.
                        const auto partial_vector_bytes =
                            slice(state, vector3_start, vector3_available);
                        Reader partial_vector(partial_vector_bytes);
                        Json components = Json::array();
                        while (partial_vector.left() >= 8)
                            components.push_back(partial_vector.f64());
                        low["member_0xc0_vector3_partial"] = {
                            {"decoded_components", std::move(components)},
                            {"available_component_count", vector3_available / 8},
                            {"required_component_count", 3},
                            {"serialized_bytes",
                             rawbytes(slice(state, vector3_start, vector3_available))},
                            {"incomplete_component_bytes",
                             rawbytes(slice(state, vector3_start +
                                                       (vector3_available / 8) * 8,
                                           vector3_available % 8))}};
                    }
                    take_u16("0xd8_u16");
                    if (version > 1) {
                        take_u32("0xdc_u32");
                        take_f64("0xe0_f64");
                        take_u32("0xe8_u32");
                        take_utf16z("0xf0_utf16z");
                    } else {
                        low["member_0xdc_u32"] = 1;
                        low["member_0xdc_u32_source"] = "native_default_for_version_1";
                        low["member_0xe0_f64"] = 10000.0;
                        low["member_0xe0_f64_source"] = "native_default_for_version_1";
                        low["member_0xe8_u32"] = 2;
                        low["member_0xe8_u32_source"] = "native_default_for_version_1";
                        low["member_0xf0_utf16z"] = "";
                        low["member_0xf0_utf16z_source"] = "native_default_for_version_1";
                    }
                    Json style = {{"mapping_status", "candidate_sdk_member_mapping"}};
                    if (low.contains("member_0x38_u8")) {
                        const auto visible = low.at("member_0x38_u8").get<std::uint8_t>();
                        style["visible"] = visible != 0;
                        style["visible_raw"] = visible;
                    }
                    if (low.contains("member_0x30_u32"))
                        style["layer_id"] = low.at("member_0x30_u32");
                    if (low.contains("member_0x78_u32"))
                        style["color"] = low.at("member_0x78_u32");
                    if (low.contains("member_0xdc_u32")) {
                        const auto type = low.at("member_0xdc_u32").get<std::uint32_t>();
                        style["point_type"] = type;
                        static const char *type_names[] = {"square", "circle"};
                        if (type < 2)
                            style["point_type_name"] = type_names[type];
                    }
                    if (low.contains("member_0xe0_f64"))
                        style["size_mm"] = low.at("member_0xe0_f64");
                    if (low.contains("member_0xe8_u32"))
                        style["decimal_places"] = low.at("member_0xe8_u32");
                    if (low.contains("member_0xf0_utf16z"))
                        style["font_name"] = low.at("member_0xf0_utf16z");
                    low["display_style"] = std::move(style);
                    low["unmapped_suffix"] = rawbytes(low_reader.take(low_reader.left()));
                    decoded["status"] = !complete
                                             ? "truncated_known_fields"
                                             : version <= 2 &&
                                                       low["unmapped_suffix"]["bytes"] == 0
                                                   ? "known_fields_decoded"
                                                   : "known_fields_decoded_with_unknown_suffix";
                } else {
                    low["unmapped_suffix"] = rawbytes(low_reader.take(low_reader.left()));
                    decoded["status"] = "truncated_known_fields";
                }
                decoded["low_points"] = std::move(low);
            }
            if (type_id == 408 && derived_offset &&
                derived_offset <= state.size()) {
                // The high-points processor has no dedicated Read/Write
                // virtual pair. Its sample begins with the same 29-byte
                // prefix as LowPointsProcessor::Read; decode only that
                // cross-type-compatible prefix and leave the rest opaque.
                Reader point_reader(state, derived_offset);
                Json point = Json::object();
                bool prefix_decoded = false;
                std::uint16_t point_version = 0;
                if (point_reader.left() >= 2) {
                    point_version = point_reader.u16();
                    point["version"] = point_version;
                    if (point_reader.left() >= 1) {
                        point["member_0x38_u8"] = point_reader.u8();
                        if (point_reader.left() >= 2) {
                            const auto string_start = point_reader.p;
                            bool terminated = false;
                            while (point_reader.left() >= 2) {
                                if (point_reader.u16() == 0) {
                                    terminated = true;
                                    break;
                                }
                            }
                            if (terminated) {
                                const auto raw = slice(
                                    state, string_start,
                                    point_reader.p - string_start);
                                point["member_0x40_utf16z"] =
                                    utf16(slice(raw, 0, raw.size() - 2));
                                point["member_0x40_utf16z_serialized"] =
                                    rawbytes(raw);
                                const auto vector_start = point_reader.p;
                                if (point_reader.left() >= 24) {
                                    point["member_0x60_vector3"] = Json::array(
                                        {point_reader.f64(), point_reader.f64(),
                                         point_reader.f64()});
                                    prefix_decoded = true;
                                } else {
                                    point_reader.p = vector_start;
                                }
                            } else {
                                point_reader.p = string_start;
                            }
                        }
                    }
                }
                bool compatible_layout_complete = prefix_decoded;
                auto take_u32 = [&](const char *name) {
                    if (!compatible_layout_complete || point_reader.left() < 4) {
                        compatible_layout_complete = false;
                        return;
                    }
                    point[std::string("member_") + name] = point_reader.u32();
                };
                auto take_f64 = [&](const char *name) {
                    if (!compatible_layout_complete || point_reader.left() < 8) {
                        compatible_layout_complete = false;
                        return;
                    }
                    point[std::string("member_") + name] = point_reader.f64();
                };
                auto take_utf16z = [&](const char *name) {
                    if (!compatible_layout_complete || point_reader.left() < 2) {
                        compatible_layout_complete = false;
                        return;
                    }
                    const auto start = point_reader.p;
                    bool terminated = false;
                    while (point_reader.left() >= 2) {
                        if (point_reader.u16() == 0) {
                            terminated = true;
                            break;
                        }
                    }
                    if (!terminated) {
                        point_reader.p = start;
                        compatible_layout_complete = false;
                        return;
                    }
                    const auto raw = slice(state, start, point_reader.p - start);
                    point[std::string("member_") + name] =
                        utf16(slice(raw, 0, raw.size() - 2));
                    point[std::string("member_") + name + "_serialized"] =
                        rawbytes(raw);
                };
                auto take_vector3 = [&](const char *name) {
                    if (!compatible_layout_complete || point_reader.left() < 24) {
                        compatible_layout_complete = false;
                        return;
                    }
                    point[std::string("member_") + name] = Json::array(
                        {point_reader.f64(), point_reader.f64(), point_reader.f64()});
                };
                if (prefix_decoded) {
                    take_u32("0x78_u32");
                    take_f64("0x110_f64");
                    take_u32("0x30_u32");
                    take_utf16z("0x80_utf16z");
                    take_utf16z("0xa0_utf16z");
                    take_vector3("0xc0_vector3");
                    if (compatible_layout_complete && point_reader.left() >= 2)
                        point["member_0xd8_u16"] = point_reader.u16();
                    else
                        compatible_layout_complete = false;
                    if (point_version > 1) {
                        take_u32("0xdc_u32");
                        take_f64("0xe0_f64");
                        take_u32("0xe8_u32");
                        take_utf16z("0xf0_utf16z");
                    }
                }
                point["layout_evidence"] =
                    "matches_low_points_native_prefix_unconfirmed_for_high_points";
                point["unmapped_suffix"] =
                    rawbytes(point_reader.take(point_reader.left()));
                decoded["status"] = !prefix_decoded
                                         ? "truncated_known_prefix"
                                         : compatible_layout_complete &&
                                                   point["unmapped_suffix"]["bytes"] == 0
                                               ? "high_points_layout_compatible_fully_consumed_unconfirmed"
                                               : "high_points_prefix_layout_compatible_unconfirmed";
                decoded["high_low_points"] = std::move(point);
            }
            if (type_id == 405 && derived_offset && derived_offset <= state.size()) {
                // FeaturesProcessor stores an optional sentinel followed by
                // the feature category enum. The UI's native enum switch
                // supplies the labels for values 0 through 6.
                Reader feature_reader(state, derived_offset);
                Json feature = Json::object();
                if (feature_reader.left() >= 2) {
                    const auto marker = feature_reader.u16();
                    feature["record_marker"] = marker;
                    if (marker == 0xffff && feature_reader.left() >= 2) {
                        const auto value = feature_reader.u16();
                        static const char *feature_names[] = {
                            "break_line", "boundary", "contours", "hole",
                            "island", "spot", "void"};
                        static const char *feature_labels[] = {
                            "Break Line", "Bound", "Contours", "Hole",
                            "IsLand", "Spot", "Void"};
                        feature["feature_type"] = value;
                        if (value < 7) {
                            feature["feature_type_name"] = feature_names[value];
                            feature["feature_type_label"] = feature_labels[value];
                            decoded["status"] = "known_fields_decoded";
                        }
                        feature["unmapped_suffix"] = rawbytes(feature_reader.take(feature_reader.left()));
                        if (value >= 7 || feature["unmapped_suffix"]["bytes"] != 0)
                            decoded["status"] = "partial";
                    } else {
                        feature["unmapped_suffix"] = rawbytes(slice(state, derived_offset, state.size() - derived_offset));
                    }
                } else {
                    feature["unmapped_suffix"] = rawbytes(feature_reader.take(feature_reader.left()));
                }
                decoded["features"] = std::move(feature);
            }
            out["decoded_state"] = std::move(decoded);
            out["status"] = "processor_state_partially_decoded";
            if ((type_id == 402 || type_id == 403 || type_id == 404 || type_id == 405 ||
                 type_id == 407 || type_id == 413) &&
                out["decoded_state"].value("status", "") == "known_fields_decoded" &&
                out["decoded_state"].value("base_frame", Json::object()).value("status", "") == "decoded")
                out["status"] = "processor_state_decoded";
            if (type_id == 409 &&
                out["decoded_state"].value("status", "") == "known_fields_decoded" &&
                out["decoded_state"].value("base_frame", Json::object()).value("status", "") ==
                    "decoded")
                out["status"] = "processor_state_decoded";
        }
        return out;
    }
    if (group == 13 && key == 10001) {
        auto measure = decode_measure_info_attribute(index, b, measurement_type);
        if (!measure.is_null())
            return measure;
    }
    if (key == 4 && group <= 1 && index == 0)
        return decode_layer_group_attribute(group, b);
    auto display = decode_display_attribute(group, key, b);
    if (!display.is_null())
        return display;
    if (group == 0x1000 && key == 0x58EA)
        return decode_dgn_display_style_type_filter(b);
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
    if (group == 0 && key == 1 && !b.empty() && b.size() % 2 == 0) {
        try {
            auto text = utf16(b);
            if (text.find('\0') == std::string::npos)
                return {{"encoding", "raw_utf16_text"}, {"text", std::move(text)}};
        } catch (const std::exception &) {
        }
    }
    if (group == 1 && key == 22745 && b.size() == 1)
        return {{"encoding", "raw_u8"}, {"value", b[0]}};
    if (group == 13 && key == 10001 && index == 10 && b.size() == 4) {
        Reader r(b);
        return {{"encoding", "raw_i32"}, {"value", r.i32()}};
    }
    Json result;
    Bytes decoded;
    try {
        decoded = decompress_attribute(b, result);
    } catch (const std::exception &) {
        if (group == 13 && key == 10001) {
            try {
                const std::string text(b.begin(), b.end());
                auto value = Json::parse(text);
                if (value.is_object() || value.is_array())
                    return {{"encoding", "raw_json_text"},
                            {"text", text},
                            {"value", std::move(value)}};
            } catch (const std::exception &) {
            }
        }
        return {{"encoding", "opaque"}, {"bytes", b.size()}};
    }
    const bool stored = result["codec"] == "stored";
    result["encoding"] = stored ? "uncompressed_binary" : "compressed_binary";
    result["data"] = rawbytes(decoded);
    if (group == 0 && key == 22901) {
        auto sectioned = decode_sesm_gams_stream(decoded);
        if (!sectioned.is_null())
            result.update(sectioned);
    }
    if (group == 0 && key == 22906) {
        result["encoding"] = "material_auxiliary_records";
        result["native_input"] = decode_material_auxiliary_records(decoded);
        return result;
    }
    if (group == 0 && key == 20091) {
        result["encoding"] = "material_auxiliary_buffer";
        result["content_semantics"] = "unresolved";
        return result;
    }
    if (key == 2006) {
        const auto &raw = decoded;
        require(raw.size() >= 2 && raw.size() % 2 == 0, "UTF16 string attribute width");
        require(raw[raw.size() - 2] == 0 && raw.back() == 0, "UTF16 string attribute terminator");
        result["encoding"] = stored ? "uncompressed_utf16_string" : "compressed_utf16_string";
        result["text"] = utf16(slice(raw, 0, raw.size() - 2));
    }
    if (group == 0 && key == 20015) {
        try {
            std::size_t end = 0;
            while (end + 1 < decoded.size() && (decoded[end] || decoded[end + 1]))
                end += 2;
            require(end + 1 < decoded.size(), "assignment_xml_terminator_outside_payload");
            auto s = utf16(slice(decoded, 0, end));
            auto table = decode_native_material_assignment_table(s);
            result.update(
                {{"encoding", stored ? "uncompressed_utf16_xml" : "compressed_utf16_xml"},
                 {"xml", s},
                 {"material_assignment_table", table},
                 {"terminator_byte_offset", end},
                 {"ignored_suffix", rawbytes(slice(decoded, end + 2, decoded.size() - end - 2))}});
            if (table.contains("source_tree"))
                result["tree"] = table.at("source_tree");
        } catch (const std::exception &e) {
            result["material_assignment_table"] = {{"status", "unresolved"}, {"reason", e.what()}};
        }
        return result;
    }
    if (group == 0 && key == 22902) {
        try {
            auto text = utf16(decoded);
            while (!text.empty() && text.back() == 0)
                text.pop_back();
            NativeExtendedColorCache cache(text);
            result["native_extended_colors"] = cache.metadata();
            // The enclosing decoded attribute already retains the source XML.
            result["native_extended_colors"].erase("source_xml");
            if (cache.metadata().at("status") == "resolved") {
                result["color_entries"] = Json::array();
                for (const auto &entry : cache.metadata().at("entries"))
                    result["color_entries"].push_back({{"ordinal", entry.at("ordinal")},
                                                       {"rgb", entry.at("rgb")},
                                                       {"source_attributes", entry.at("source_attributes")}});
            }
        } catch (const std::exception &e) {
            result["native_extended_colors"] = {{"status", "unresolved"}, {"reason", e.what()}};
        }
    }
    try {
        auto s = utf16(decoded);
        while (!s.empty() && s.back() == 0)
            s.pop_back();
        auto tree = xml_tree(s);
        result.update({{"encoding", stored ? "uncompressed_utf16_xml" : "compressed_utf16_xml"},
                       {"xml", s},
                       {"tree", tree}});
    } catch (const std::exception &) {
    }
    return result;
}
Json parse_graphics(const Bytes &b) {
    Reader r(b);
    Json out = Json::array();
    struct GraphicAttribute {
        unsigned group, key, index, size_flags, reserved;
        std::size_t offset, size;
    };
    while (r.left()) {
        auto pos = r.p;
        require(r.u32() == 0xa11b, "graphics magic");
        auto len = r.u32();
        auto flags = r.u64(), id = r.u64();
        auto count = r.u32();
        auto end = pos + 28 + std::uint64_t(len);
        require(end <= b.size() && count <= len / 16, "graphics record length");
        std::vector<GraphicAttribute> source_attributes;
        source_attributes.reserve(count);
        for (unsigned i = 0; i < count; ++i) {
            auto group = r.u16(), key = r.u16();
            auto ix = r.u32(), sf = r.u32(), res = r.u32();
            auto size = sf & 0x7fffffff;
            require(r.p <= end && size <= end - r.p, "graphics attribute length");
            auto off = r.p;
            r.skip(size);
            source_attributes.push_back({group, key, ix, sf, res, off, size});
        }
        int measurement_type = -1;
        for (const auto &a : source_attributes)
            if (a.group == 13 && a.key == 10001 && a.index == 2 && a.size == 4) {
                Reader value(b, a.offset);
                measurement_type = value.i32();
                break;
            }
        Json attrs = Json::array();
        for (const auto &a : source_attributes) {
            auto raw = slice(b, a.offset, a.size);
            Json decoded;
            try {
                decoded = decode_attribute(a.group, a.key, raw, a.index, measurement_type);
            } catch (const std::exception &e) {
                decoded = {{"encoding", "opaque"}, {"decode_error", e.what()}};
            }
            attrs.push_back({{"group", a.group},
                             {"key", a.key},
                             {"index", a.index},
                             {"size_flags", a.size_flags},
                             {"reserved", a.reserved},
                             {"offset", a.offset},
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
        for (const auto &a : rec["attributes"])
            if (a["key"] == 20081 && (a["group"] == 0 || a["group"] == 20117)) {
                rec["reference_extension_input"] = reference_extension_input(rec["attributes"]);
                break;
            }
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
        Json info = {{"physical_storage", s.path[1]}};
        info["coordinate_context"] = {{"storage", "source_float64"},
                                      {"unit_status", "missing_model_unit_record"},
                                      {"scale_to_meters", nullptr},
                                      {"crs", nullptr},
                                      {"georeferencing_status", "unresolved"}};
        try {
            auto records = parse_native(slice(b, 4096, b.size() - 4096));
            info["layer_group_references"] = Json::array();
            info["unit_records"] = Json::array();
            info["coordinate_records"] = Json::array();
            info["view_records"] = Json::array();
            for (const auto &n : records)
                if (n.contains("model_view_state")) {
                    auto view = n.at("model_view_state");
                    view["stream"] = s.path;
                    view["record_offset"] = 4096 + n.at("offset").get<std::uint64_t>();
                    info["view_records"].push_back(std::move(view));
                }
            if (info["view_records"].size() == 1)
                info["view_state"] = info["view_records"][0];
            for (const auto &n : records)
                if (n.contains("model_coordinate_state")) {
                    auto coordinates = n.at("model_coordinate_state");
                    coordinates["stream"] = s.path;
                    coordinates["record_offset"] = 4096 + n.at("offset").get<std::uint64_t>();
                    info["coordinate_records"].push_back(std::move(coordinates));
                }
            if (info["coordinate_records"].size() == 1)
                info["coordinates"] = info["coordinate_records"][0];
            for (const auto &n : records)
                if (n.contains("model_unit_state")) {
                    auto units = n.at("model_unit_state");
                    units["stream"] = s.path;
                    units["record_offset"] = 4096 + n.at("offset").get<std::uint64_t>();
                    info["unit_records"].push_back(std::move(units));
                }
            if (info["unit_records"].size() == 1) {
                info["units"] = info["unit_records"][0];
                const auto &units = info.at("units");
                auto &context = info["coordinate_context"];
                context["unit_status"] = "unsupported_model_unit_record";
                if (units.at("status") == "decoded") {
                    if (units.at("record_offset") == 4096)
                        info["unit_fields_at_10e4"] =
                            Json::array({units.at("data_units_per_storage_unit").at("value"),
                                         units.at("storage_unit").at("numerator"),
                                         units.at("storage_unit").at("denominator"),
                                         units.at("unassigned_adjacent_value").at("value")});
                    const auto &meters = units.at("factors").at("meters_per_data_unit");
                    context["unit_status"] = "unusable_native_unit_factor";
                    if (meters.at("status") == "computed" && meters.at("positive") == true) {
                        info["length_scale_to_meters"] = meters.at("value");
                        context["unit_status"] = "decoded_native_unit_factor";
                        context["scale_to_meters"] = meters.at("value");
                    }
                    context["source_record_offset"] = units.at("record_offset");
                }
            } else if (info["unit_records"].size() > 1)
                info["coordinate_context"]["unit_status"] = "ambiguous_model_unit_records";
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
