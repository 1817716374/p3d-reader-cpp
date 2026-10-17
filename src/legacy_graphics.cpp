#include "internal.hpp"
#include "decode_budget.hpp"
#include <p3d/legacy_graphics.hpp>

namespace p3d {
namespace {
struct Limit : DecodeBudgetExceeded {
    using DecodeBudgetExceeded::DecodeBudgetExceeded;
};
struct Unsupported : std::runtime_error {
    using std::runtime_error::runtime_error;
};
constexpr std::size_t none = std::numeric_limits<std::size_t>::max();
struct Edge {
    std::size_t previous;
    Json value;
};
struct Geometry {
    Json value;
    std::size_t tail_offset = 0;
};
struct Entry {
    std::size_t previous, edges, end, start;
    std::shared_ptr<const Geometry> geometry;
    Json style;
    std::uint32_t kind;
};
struct Context {
    std::size_t previous, entry_count, start;
    std::shared_ptr<const Geometry> geometry;
    Json style;
    std::uint32_t kind;
};
struct Job {
    enum Kind { entries, edges, scan } kind = entries;
    std::size_t offset = 0, previous = none, count = 0;
    std::shared_ptr<const Context> context{};
    std::size_t edge_count = 0, edge_previous = none, next_size = 0;
};
class Decoder {
    const Bytes &bytes;
    const LegacyGraphicsLimits &limits;
    LegacyGraphicsProfile profile;
    std::size_t work = 0, values = 0;
    std::vector<Entry> entry_arena;
    std::vector<Edge> edge_arena;
    std::vector<Job> jobs;
    Json results = Json::array();
    std::string last_error;
    bool unsupported_branch = false;

    void tick(std::size_t n = 1) {
        if (n > limits.max_work - work)
            throw Limit("legacy graphics work limit");
        work += n;
    }
    void value_budget(std::size_t n) {
        if (n > limits.max_values - values)
            throw Limit("legacy graphics value limit");
        values += n;
        tick(n);
    }
    std::size_t count(Reader &r, std::size_t width) {
        auto n = r.u32();
        require(n <= r.left() / width, "legacy graphics collection overrun");
        value_budget(n);
        return n;
    }
    Json number(Reader &r, bool single = false) {
        const auto start = r.p;
        double v = single ? double(r.get<float>()) : r.f64();
        if (std::isfinite(v))
            return v;
        return {{"value", nullptr},
                {"ieee754_hex", hex(slice(bytes, start, single ? 4 : 8))},
                {"source_offset", start}};
    }
    Json doubles(Reader &r, std::size_t n) {
        value_budget(n);
        Json a = Json::array();
        while (n--)
            a.push_back(number(r));
        return a;
    }
    Json style(Reader &r, bool entry) {
        const auto start = r.p;
        value_budget(entry ? 10 : 9);
        Json j = {
            {"serialized_rgb", r.uints(3)}, {"line_style", r.i32()}, {"line_weight", r.u32()}};
        if (!entry)
            j["symbology_source"] = r.i32();
        j["transparency"] = number(r);
        j["line_style_scale"] = number(r);
        if (entry) {
            j["widths"] = doubles(r, 2);
            j["layer_id"] = r.u32();
        }
        j["source_offset"] = start;
        j["source_byte_count"] = r.p - start;
        return j;
    }
    Json vector(Reader &r, unsigned dimension, bool integer = false) {
        const auto start = r.p;
        const auto n = count(r, integer ? 4 : dimension * 8);
        Json a = Json::array();
        for (std::size_t i = 0; i < n; ++i)
            a.push_back(integer ? Json(r.i32()) : doubles(r, dimension));
        return {{"source_offset", start},
                {"source_byte_count", r.p - start},
                {"count", n},
                {"values", std::move(a)}};
    }
    Json uint_vector(Reader &r) {
        const auto start = r.p;
        auto n = count(r, 4);
        Json a = Json::array();
        for (std::size_t i = 0; i < n; ++i)
            a.push_back(r.u32());
        return {{"source_offset", start},
                {"source_byte_count", r.p - start},
                {"count", n},
                {"values", std::move(a)}};
    }
    Json string(Reader &r) {
        const auto start = r.p;
        bool wide = false;
        auto length = [&]() -> std::uint64_t {
            auto n = r.u8();
            if (n != 0xff)
                return n;
            auto n16 = r.u16();
            // 0xfffe is a Unicode marker, not an extended ANSI length.
            if (n16 == 0xfffe) {
                require(!wide, "repeated legacy CString Unicode marker");
                wide = true;
                n = r.u8();
                if (n != 0xff)
                    return n;
                n16 = r.u16();
                require(n16 != 0xfffe, "repeated legacy CString Unicode marker");
            }
            if (n16 != 0xffff)
                return n16;
            auto n32 = r.u32();
            return n32 != 0xffffffffu ? n32 : r.u64();
        }();
        if (length > limits.max_string_units)
            throw Limit("legacy graphics CString length limit");
        const auto width = wide ? 2u : 1u;
        require(length <= r.left() / width, "legacy graphics CString overrun");
        value_budget(static_cast<std::size_t>(length));
        const auto data = r.p;
        auto raw = r.take(static_cast<std::size_t>(length) * width);
        Json out = {{"encoding", wide ? "mfc_unicode_cstring" : "mfc_ansi_cstring"},
                    {"source_offset", start},
                    {"source_byte_count", r.p - start},
                    {"data_offset", data},
                    {"unit_count", length}};
        if (wide) {
            Json units = Json::array();
            for (std::size_t i = 0; i < raw.size(); i += 2)
                units.push_back(unsigned(raw[i]) | (unsigned(raw[i + 1]) << 8));
            out["code_units"] = units;
            // Validate before conversion: preserve malformed code units, without
            // replacement characters or loss of the original archive structure.
            bool valid = true;
            for (std::size_t i = 0; i < units.size(); ++i) {
                unsigned u = units[i];
                if (u >= 0xd800 && u <= 0xdbff) {
                    if (++i == units.size() || units[i].get<unsigned>() < 0xdc00 ||
                        units[i].get<unsigned>() > 0xdfff) {
                        valid = false;
                        break;
                    }
                } else if (u >= 0xdc00 && u <= 0xdfff) {
                    valid = false;
                    break;
                }
            }
            out["text"] = valid ? Json(utf16(raw)) : Json(nullptr);
            out["text_status"] = valid ? "decoded" : "invalid_utf16";
        } else {
            const bool ascii = std::all_of(raw.begin(), raw.end(), [](auto c) { return c < 128; });
            out["text"] = ascii ? Json(std::string(raw.begin(), raw.end())) : Json(nullptr);
            out["text_status"] = ascii ? "decoded_ascii" : "code_page_required";
        }
        return out;
    }
    Json primitive(Reader &r, std::size_t depth) {
        if (depth > limits.max_depth)
            throw Limit("legacy graphics curve depth limit");
        tick();
        const auto start = r.p;
        const auto type = r.u32();
        Json j = {{"type_code", type}};
        if (type == 1) {
            j["kind"] = "line_segment";
            j["start"] = doubles(r, 3);
            j["end"] = doubles(r, 3);
        } else if (type == 2 || type == 7) {
            j["kind"] = type == 2 ? "polyline" : "point_string";
            j["points"] = vector(r, 3).at("values");
        } else if (type == 3) {
            j["kind"] = "elliptical_arc";
            j["origin"] = doubles(r, 3);
            j["vector0"] = doubles(r, 3);
            j["vector90"] = doubles(r, 3);
            j["start_angle"] = number(r);
            j["sweep_angle"] = number(r);
        } else if (type == 4) {
            j["kind"] = "bspline_curve";
            value_budget(10);
            j["curve_type"] = r.i32();
            const auto rational = r.i32();
            j["rational_stored_int32"] = rational;
            j["pole_coordinates"] = rational ? "weighted_xyz" : "cartesian_xyz";
            j["curve_display"] = r.i32();
            j["polygon_display"] = r.i32();
            j["rules_by_length"] = r.i32();
            const auto closed = r.i32();
            j["closed_stored_int32"] = closed;
            j["declared_num_knots"] = r.i32();
            const auto poles = r.i32();
            j["num_poles"] = poles;
            j["num_rules"] = r.i32();
            const auto order = r.i32();
            j["order"] = order;
            // Native bspknot_numberKnots uses DWORD arithmetic and signed loops,
            // independently of the saved params.numKnots member.
            const std::uint32_t wrapped = std::uint32_t(poles) +
                                          std::uint32_t(order) * (closed ? 2u : 1u) -
                                          (closed ? 1u : 0u);
            std::int32_t knot_count;
            std::memcpy(&knot_count, &wrapped, 4);
            const auto np = poles > 0 ? std::size_t(poles) : 0;
            const auto nk = knot_count > 0 ? std::size_t(knot_count) : 0;
            require(np <= r.left() / 24, "legacy Bspline pole overrun");
            value_budget(np);
            j["poles"] = Json::array();
            for (std::size_t i = 0; i < np; ++i)
                j["poles"].push_back(doubles(r, 3));
            require(nk <= r.left() / 8, "legacy Bspline knot overrun");
            j["serialized_knot_count"] = nk;
            j["knots"] = doubles(r, nk);
            require(!rational || np <= r.left() / 8, "legacy Bspline weight overrun");
            j["weights"] = rational ? doubles(r, np) : Json::array();
        } else if (type == 5 || type == 6 || type == 9 || type == 10) {
            j["kind"] = "curve_payload_not_serialized";
            j["curve_kind"] = type == 5   ? "interpolation_curve"
                              : type == 6 ? "akima_curve"
                              : type == 9 ? "spiral"
                                          : "proport_curve";
            j["geometry_status"] = "not_serialized";
        } else if (type == 8) {
            j["kind"] = "child_curve_collection";
            j["collection"] = curves(r, depth + 1);
        } else {
            throw Unsupported("unsupported legacy curve type " + std::to_string(type));
        }
        j["source_offset"] = start;
        j["source_byte_count"] = r.p - start;
        return j;
    }
    Json curves(Reader &r, std::size_t depth) {
        if (depth > limits.max_depth)
            throw Limit("legacy graphics curve depth limit");
        Json j = {{"kind", "curve_collection"}, {"boundary_code", r.u32()}};
        const auto n = count(r, 4);
        j["curves"] = Json::array();
        for (std::size_t i = 0; i < n; ++i)
            j["curves"].push_back(primitive(r, depth));
        return j;
    }
    void byte_flag(Reader &r, Json &j, const std::string &name) {
        value_budget(1);
        const auto byte = r.u8();
        j[name + "_stored_byte"] = byte;
        j[name] = byte != 0;
    }
    Json solid(Reader &r) {
        const auto start = r.p;
        const auto type = r.u32();
        Json j = {{"kind", "solid"}, {"solid_type_code", type}};
        auto vectors = [&](std::initializer_list<const char *> names) {
            for (const auto *name : names)
                j[name] = doubles(r, 3);
        };
        auto scalars = [&](std::initializer_list<const char *> names) {
            value_budget(names.size());
            for (const auto *name : names)
                j[name] = number(r);
        };
        switch (type) {
        case 1:
            j["solid_kind"] = "torus_pipe";
            vectors({"center", "vector_x", "vector_y"});
            scalars({"major_radius", "minor_radius", "sweep_angle"});
            break;
        case 2:
            j["solid_kind"] = "cone";
            vectors({"center_a", "center_b", "vector0", "vector90"});
            // Runtime GeConeInfo stores B before A; this archive saves A first.
            scalars({"radius_a", "radius_b"});
            break;
        case 3:
            j["solid_kind"] = "box";
            vectors({"bottom_origin", "top_origin", "vector_x", "vector_y"});
            scalars({"bottom_x", "bottom_y", "top_x", "top_y"});
            break;
        case 4:
            j["solid_kind"] = "sphere";
            // Native 3x4 transform is rearranged as origin and three columns.
            vectors({"origin", "vector_x", "vector_y", "vector_z"});
            scalars({"start_latitude", "latitude_sweep"});
            break;
        case 5:
            j["solid_kind"] = "extrusion";
            j["base_curves"] = curves(r, 0);
            vectors({"extrusion_vector"});
            break;
        case 6:
            j["solid_kind"] = "rotational_sweep";
            j["base_curves"] = curves(r, 0);
            vectors({"axis_origin", "axis_direction"});
            scalars({"sweep_angle"});
            break;
        case 7: {
            j["solid_kind"] = "ruled_sweep";
            const auto n = count(r, 8);
            j["sections"] = Json::array();
            for (std::size_t i = 0; i < n; ++i)
                j["sections"].push_back(curves(r, 0));
            break;
        }
        case 8:
            j["solid_kind"] = "swept_body";
            j["profile_curves"] = curves(r, 0);
            j["path_curves"] = curves(r, 0);
            break;
        case 9:
            j["solid_kind"] = "lofted_body";
            j["bottom_curves"] = curves(r, 0);
            j["top_curves"] = curves(r, 0);
            // The native writer packages guide groups into a CurveVector;
            // preserve this saved grouping instead of guessing correspondence.
            j["guide_curves"] = curves(r, 0);
            break;
        default:
            throw Unsupported("unsupported legacy solid type " + std::to_string(type));
        }
        byte_flag(r, j, "capped");
        if (type == 6) {
            value_budget(1);
            j["num_v_rules_stored_uint32"] = r.u32();
        }
        j["source_offset"] = start;
        j["source_byte_count"] = r.p - start;
        return j;
    }
    Json text(Reader &r) {
        const auto start = r.p;
        Json j = {{"kind", "text"},
                  {"origin", doubles(r, 3)},
                  {"user_origin", doubles(r, 3)},
                  {"rotation_matrix_rows", Json::array()}};
        for (unsigned row = 0; row < 3; ++row)
            j["rotation_matrix_rows"].push_back(doubles(r, 3));
        j["string"] = string(r);
        j["unapplied_fields"] = Json::array();
        // These fields are consumed but not applied by the native reader.
        // Keep typed values and exact ranges without inventing business names.
        auto extra = [&](char type, unsigned n = 1) {
            const auto pos = r.p;
            Json value;
            value_budget(n);
            if (type == 'B')
                value = r.u8();
            else if (type == 'I')
                value = r.u32();
            else if (type == 'Q')
                value = r.u64();
            else if (type == 'd')
                value = doubles(r, n);
            else
                throw std::runtime_error("legacy text extra type");
            j["unapplied_fields"].push_back({{"type", std::string(1, type)},
                                             {"value", std::move(value)},
                                             {"source_offset", pos},
                                             {"source_byte_count", r.p - pos}});
        };
        auto symbology = [&] {
            const auto pos = r.p;
            value_budget(5);
            return Json{{"serialized_rgb", r.uints(3)},
                        {"line_style", r.i32()},
                        {"line_weight", r.u32()},
                        {"source_offset", pos},
                        {"source_byte_count", 20}};
        };
        byte_flag(r, j, "vertical");
        extra('B');
        extra('B');
        extra('B');
        value_budget(1);
        j["justification_code"] = r.i32();
        extra('I');
        extra('I');
        j["font_name"] = string(r);
        value_budget(3);
        j["font_type_code"] = r.i32();
        j["big_font_name"] = string(r);
        j["big_font_type_code"] = r.i32();
        byte_flag(r, j, "bold");
        byte_flag(r, j, "italic");
        j["custom_slant_angle"] = number(r);
        extra('B');
        extra('B');
        j["unapplied_symbologies"] = Json::array();
        j["unapplied_symbologies"].push_back(symbology());
        extra('Q');
        extra('B');
        extra('B');
        j["unapplied_symbologies"].push_back(symbology());
        extra('Q');
        extra('I');
        extra('Q');
        extra('B');
        j["unapplied_colors"] = Json::array();
        for (unsigned i = 0; i < 2; ++i) {
            const auto pos = r.p;
            value_budget(3);
            j["unapplied_colors"].push_back({{"serialized_rgb", r.uints(3)},
                                             {"source_offset", pos},
                                             {"source_byte_count", 12}});
        }
        extra('I');
        extra('I');
        extra('d', 2);
        extra('d', 2);
        extra('B');
        extra('B');
        extra('B');
        j["font_size"] = doubles(r, 2);
        j["font_lookup"] = "not_performed";
        j["unapplied_field_semantics"] = "unassigned";
        j["source_offset"] = start;
        j["source_byte_count"] = r.p - start;
        return j;
    }
    Geometry mesh(Reader &r) {
        Geometry g;
        auto &j = g.value;
        j = {{"kind", "indexed_mesh"},
             {"mesh_style", r.u32()},
             {"num_per_row", r.u32()},
             {"num_per_face", r.u32()},
             {"two_sided_stored_byte", r.u8()}};
        j["two_sided"] = j["two_sided_stored_byte"].get<unsigned>() != 0;
        j["points"] = vector(r, 3);
        j["parameters"] = vector(r, 2);
        j["normals"] = vector(r, 3);
        j["rgb_factors"] = vector(r, 3);
        j["rgb_uints"] = uint_vector(r);
        j["rgb_table"] = uint_vector(r);
        const bool writer = profile == LegacyGraphicsProfile::writer_2025;
        const auto rgb_start = r.p;
        const auto n = count(r, writer ? 20 : 12);
        Json colors = Json::array();
        for (std::size_t i = 0; i < n; ++i) {
            value_budget(3);
            Json c = {{"red_float32", number(r, true)}};
            if (writer) {
                c["factor_green_float64"] = number(r);
                c["factor_blue_float64"] = number(r);
            } else {
                c["green_float32"] = number(r, true);
                c["blue_float32"] = number(r, true);
            }
            colors.push_back(std::move(c));
        }
        j["rgb_floats"] = {{"source_offset", rgb_start},
                           {"source_byte_count", r.p - rgb_start},
                           {"count", n},
                           {"values", std::move(colors)}};
        j["native_reader_wire_compatible"] = !(writer && n);
        for (const auto *name :
             {"point_indices", "parameter_indices", "normal_indices", "rgb_indices"})
            j[name] = vector(r, 1, true);
        j["polygon_derivation_status"] = "unsupported_layout";
        if (j["mesh_style"] == 1 && j["num_per_row"] == 0 && j["num_per_face"] == 0) {
            Json polygons = Json::array(), face = Json::array();
            bool valid = true;
            for (const auto &value : j["point_indices"]["values"]) {
                tick();
                const auto index = value.get<std::int64_t>();
                if (std::llabs(index) > j["points"]["count"].get<std::int64_t>())
                    valid = false;
                if (index)
                    face.push_back(index);
                else if (!face.empty()) {
                    polygons.push_back(std::move(face));
                    face = Json::array();
                }
            }
            valid = valid && face.empty();
            j["polygon_derivation_status"] = valid ? "derived" : "invalid_point_indices";
            if (valid)
                j["polygons"] = std::move(polygons);
        }
        g.tail_offset = r.p;
        j["tag_name"] = string(r);
        j["face_indices"] = vector(r, 1, true);
        const auto face_start = r.p;
        const auto faces = count(r, 176);
        Json data = Json::array();
        for (std::size_t i = 0; i < faces; ++i) {
            const auto pos = r.p;
            Json f = {{"parameter_distance_range", doubles(r, 4)},
                      {"parameter_range", doubles(r, 4)},
                      {"xyz_range", doubles(r, 6)},
                      {"normal_range", doubles(r, 6)},
                      {"source_index_stored_uint32", r.u32()}};
            value_budget(4);
            f["face_ids_stored_int32"] = {r.i32(), r.i32(), r.i32()};
            f["source_offset"] = pos;
            f["source_byte_count"] = r.p - pos;
            data.push_back(std::move(f));
        }
        j["face_data"] = {{"source_offset", face_start},
                          {"source_byte_count", r.p - face_start},
                          {"count", faces},
                          {"values", std::move(data)}};
        return g;
    }
    void append_entry(const std::shared_ptr<const Context> &c, std::size_t edge, std::size_t end) {
        value_budget(1);
        entry_arena.push_back({c->previous, edge, end, c->start, c->geometry, c->style, c->kind});
        jobs.push_back({Job::entries, end, entry_arena.size() - 1, c->entry_count + 1});
    }
    Json materialize(std::size_t previous, Json footer) {
        std::vector<std::size_t> order;
        for (auto n = previous; n != none; n = entry_arena[n].previous) {
            tick();
            order.push_back(n);
        }
        Json entries = Json::array();
        for (auto it = order.rbegin(); it != order.rend(); ++it) {
            const auto &entry = entry_arena[*it];
            // Account for retained output copies before copying shared geometry.
            value_budget((entry.end - entry.start) / 4 + 1);
            auto geometry = entry.geometry->value;
            if (entry.kind == 3) {
                std::vector<std::size_t> edge_order;
                for (auto e = entry.edges; e != none; e = edge_arena[e].previous) {
                    tick();
                    edge_order.push_back(e);
                }
                geometry["edge_chains"] = Json::array();
                for (auto e = edge_order.rbegin(); e != edge_order.rend(); ++e) {
                    const auto &value = edge_arena[*e].value;
                    geometry["edge_chains"].push_back(value);
                    if (profile == LegacyGraphicsProfile::writer_2025 &&
                        value.at("declared_topology_id_count") != value.at("serialized_id_count"))
                        geometry["native_reader_wire_compatible"] = false;
                }
                geometry["tail_source_offset"] = entry.geometry->tail_offset;
                geometry["tail_source_byte_count"] = entry.end - entry.geometry->tail_offset;
            }
            entries.push_back({{"kind_code", entry.kind},
                               {"style", entry.style},
                               {"geometry", std::move(geometry)},
                               {"source_offset", entry.start},
                               {"source_byte_count", entry.end - entry.start}});
        }
        return {{"entries", std::move(entries)}, {"container_style", std::move(footer)}};
    }
    void entry_job(const Job &job, std::size_t total) {
        Reader r(bytes, job.offset);
        if (job.count == total) {
            require(r.left() == 40, "legacy graphics container footer size");
            auto footer = style(r, false);
            results.push_back(materialize(job.previous, std::move(footer)));
            return;
        }
        const auto start = r.p;
        const auto kind = r.u32();
        auto s = style(r, true);
        Geometry g;
        std::size_t edges = 0;
        if (kind == 3) {
            g = mesh(r);
            g.value["edge_chain_source_offset"] = r.p;
            edges = count(r, 12);
            g.value["edge_chain_count"] = edges;
        } else if (kind == 2) {
            g.value = curves(r, 0);
        } else if (kind == 1) {
            g.value = primitive(r, 0);
        } else if (kind == 6) {
            g.value = solid(r);
        } else if (kind == 7) {
            g.value = text(r);
        } else if (kind == 10) {
            const auto pos = r.p;
            const auto n = count(r, 1);
            const auto data_offset = r.p;
            const auto payload = r.take(n);
            g.value = {{"kind", "csg"},
                       {"source_offset", pos},
                       {"source_byte_count", r.p - pos},
                       {"payload_source_offset", data_offset},
                       {"payload_byte_count", n},
                       {"archive_decode_status", "not_parsed"}};
            try {
                g.value["archive"] = decode_csg_bytes_budgeted(
                    payload, std::min(std::size_t(100000), limits.max_values - values),
                    std::min(std::size_t(128), limits.max_depth),
                    [&](std::size_t cost) { value_budget(cost); });
                g.value["archive_decode_status"] = "decoded";
            } catch (const DecodeBudgetExceeded &) {
                throw;
            } catch (const std::exception &e) {
                g.value["archive_decode_error"] = e.what();
            }
        } else if (kind == 4 || kind == 5 || kind == 8 || kind == 9) {
            // These declared kinds fall through both native switch tables.
            // The writer stores their kind/style but no geometry payload.
            g.value = {{"kind", "entry_payload_not_serialized"},
                       {"geometry_status", "not_serialized"}};
        } else {
            throw Unsupported("unsupported legacy graphics entry kind " + std::to_string(kind));
        }
        auto c = std::make_shared<Context>();
        c->previous = job.previous;
        c->entry_count = job.count;
        c->start = start;
        c->geometry = std::make_shared<Geometry>(std::move(g));
        c->style = std::move(s);
        c->kind = kind;
        if (edges) {
            Job next;
            next.kind = Job::edges;
            next.offset = r.p;
            next.context = c;
            next.edge_count = edges;
            jobs.push_back(std::move(next));
        } else {
            append_entry(c, none, r.p);
        }
    }
    void edge_job(Job job) {
        if (!job.edge_count) {
            append_entry(job.context, job.edge_previous, job.offset);
            return;
        }
        Reader r(bytes, job.offset);
        auto type = r.u32(), declared = r.u32();
        std::size_t ids = declared, indices = 0;
        const bool writer = profile == LegacyGraphicsProfile::writer_2025;
        if (writer) {
            // The writer saves N, but loops over M IDs, followed by M and M
            // indices. Enumerate M lazily, and accept only full archives.
            const auto m = job.next_size;
            if (r.left() < 4 || m > (r.left() - 4) / 8)
                return;
            auto sibling = job;
            sibling.kind = Job::scan;
            sibling.next_size = m + 1;
            jobs.push_back(std::move(sibling));
            if (r.at<std::uint32_t>(r.p + 4 * m) != m)
                return;
            ids = indices = m;
        }
        require(ids <= r.left() / 4, "legacy topology ID count overrun");
        value_budget(ids);
        Json id_values = Json::array();
        for (std::size_t i = 0; i < ids; ++i)
            id_values.push_back(r.u32());
        const auto saved_indices = r.u32();
        if (!writer)
            indices = saved_indices;
        require(indices <= r.left() / 4, "legacy edge index count overrun");
        value_budget(indices);
        Json index_values = Json::array();
        for (std::size_t i = 0; i < indices; ++i)
            index_values.push_back(r.i32());
        value_budget(1);
        edge_arena.push_back({job.edge_previous,
                              {{"topology_type", type},
                               {"declared_topology_id_count", declared},
                               {"serialized_id_count", ids},
                               {"serialized_topology_ids", std::move(id_values)},
                               {"index_count", saved_indices},
                               {"indices", std::move(index_values)},
                               {"source_offset", job.offset},
                               {"source_byte_count", r.p - job.offset}}});
        job.kind = Job::edges;
        job.offset = r.p;
        job.edge_previous = edge_arena.size() - 1;
        --job.edge_count;
        job.next_size = 0;
        jobs.push_back(std::move(job));
    }

  public:
    Decoder(const Bytes &b, LegacyGraphicsProfile p, const LegacyGraphicsLimits &l)
        : bytes(b), limits(l), profile(p) {}
    Json decode() {
        Json out = {{"status", "not_parsed"},
                    {"candidate_set_complete", false},
                    {"profile",
                     profile == LegacyGraphicsProfile::writer_2025 ? "writer_2025" : "reader_2025"},
                    {"active_geometry_selection", "not_performed"},
                    {"material_binding", "not_performed"}};
        bool complete = false;
        try {
            require(profile == LegacyGraphicsProfile::writer_2025 ||
                        profile == LegacyGraphicsProfile::reader_2025,
                    "unknown legacy graphics profile");
            if (bytes.size() > limits.max_input_bytes)
                throw Limit("legacy graphics input byte limit");
            out["source_storage"] = rawbytes(bytes);
            if (!limits.max_interpretations)
                throw Limit("legacy graphics interpretation limit");
            Reader r(bytes);
            const auto version = r.f64();
            const auto total = r.u32();
            require(version == 3.1, "unsupported legacy graphics version");
            out["version"] = version;
            out["count"] = total;
            if (total > limits.max_entries)
                throw Limit("legacy graphics entry count limit");
            jobs.push_back({Job::entries, r.p});
            while (!jobs.empty()) {
                tick();
                auto job = std::move(jobs.back());
                jobs.pop_back();
                try {
                    if (job.kind == Job::entries)
                        entry_job(job, total);
                    else
                        edge_job(std::move(job));
                } catch (const Limit &) {
                    throw;
                } catch (const Unsupported &e) {
                    unsupported_branch = true;
                    last_error = e.what();
                } catch (const std::runtime_error &e) {
                    last_error = e.what();
                }
                if (results.size() >= limits.max_interpretations && !jobs.empty())
                    throw Limit("legacy graphics interpretation limit");
            }
            complete = !unsupported_branch;
        } catch (const std::runtime_error &e) {
            last_error = e.what();
        }
        out["candidate_set_complete"] = complete;
        out["unsupported_branch_encountered"] = unsupported_branch;
        if (complete && results.size() == 1) {
            out["status"] = "parsed";
            out.update(std::move(results[0]));
        } else {
            if (results.size() > 1)
                out["status"] = "ambiguous";
            out["hypotheses"] = std::move(results);
            out["reason"] =
                complete ? (out["status"] == "ambiguous" ? "multiple_full_buffer_interpretations"
                                                         : last_error)
                         : last_error;
        }
        out["work_used"] = work;
        out["values_used"] = values;
        return out;
    }
};
} // namespace
Json decode_legacy_graphics(const Bytes &bytes, LegacyGraphicsProfile profile,
                            const LegacyGraphicsLimits &limits) {
    return Decoder(bytes, profile, limits).decode();
}
} // namespace p3d
