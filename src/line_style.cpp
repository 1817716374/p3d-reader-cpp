#include "line_style_source.hpp"
#include <p3d/line_style.hpp>
namespace p3d {
namespace {
Json number(std::uint64_t bits) {
    double value;
    std::memcpy(&value, &bits, 8);
    if (std::isfinite(value))
        return value;
    std::string hex(16, '0');
    for (unsigned i = 0; i < 16; ++i)
        hex[15 - i] = "0123456789abcdef"[(bits >> (4 * i)) & 15];
    return {{"floating_point", std::isinf(value) ? "infinity" : "nan"},
            {"negative", bool(bits >> 63)},
            {"ieee754_hex", hex}};
}
Json read_number(Reader &r) {
    return number(r.u64());
}
Json read_values(Reader &r, unsigned n) {
    Json out = Json::array();
    while (n--)
        out.push_back(read_number(r));
    return out;
}
} // namespace
Json decode_line_style_modifier_source(const Bytes &raw) {
    const auto n = raw.size();
    require(n >= 4, "line modifier size");
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
            auto value = read_number(m);
            named[scalar_names[bit]] = value;
            fields.push_back({{"flag_mask", 1u << bit},
                              {"source_offset", offset},
                              {"native_member_offset", 8u * (bit + 1)},
                              {"name", scalar_names[bit]},
                              {"storage_uint64", storage},
                              {"float64_view", value}});
        }
    if (bits & 0x100)
        mod["orientation_vector"] = read_values(m, 3);
    if (bits & 0x200) {
        mod["orientation_data"] = read_values(m, 4);
        mod["orientation_encoding"] = "requires_dimension_context";
    }
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
    return mod;
}
namespace {
template <class T> void store(Bytes &b, std::size_t at, T value) {
    std::memcpy(b.data() + at, &value, sizeof(value));
}
Bytes defaults() {
    Bytes state(0xa8, 0);
    for (auto at : {8u, 16u, 24u, 88u, 96u, 128u, 160u})
        store(state, at, std::uint64_t(0x3ff0000000000000ull));
    return state;
}
std::size_t linkage_size(std::uint16_t header) {
    require(header & 0x1000, "line_style_user_linkage_required");
    const auto words = (header & 0x4000)
                           ? std::uint16_t((header & 255u) * (1u << ((header >> 8) & 15u)))
                           : std::uint16_t((header & 255u) + 1u);
    require(words >= 2, "line_style_linkage_length_invalid");
    return std::size_t(words) * 2;
}
void orientation(Bytes &state, Reader &r, bool dim3) {
    std::array<std::uint64_t, 4> raw{};
    for (auto &v : raw)
        v = r.u64();
    if (!dim3) {
        for (auto at : {96u, 104u, 112u, 120u, 128u, 136u, 144u, 152u})
            store(state, at, std::uint64_t(0));
        store(state, 96, raw[0]);
        store(state, 104, raw[1]);
        store(state, 120, raw[2]);
        store(state, 128, raw[3]);
        store(state, 160, std::uint64_t(0x3ff0000000000000ull));
    } else {
        std::array<double, 4> q{};
        for (unsigned i = 0; i < 4; ++i)
            std::memcpy(&q[i], &raw[i], 8);
        const auto w = q[0], x = q[1], y = q[2], z = q[3];
        const auto xx = x * x, yy = y * y, zz = z * z, ww = w * w;
        const auto sum = ((yy + xx) + zz) + ww;
        Matrix3 m{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        if (sum != 0) {
            const auto inv = 1. / sum, two = inv + inv;
            // Transposed native quaternion matrix; preserve operation order.
            m[0][0] = (((ww + xx) - yy) - zz) * inv;
            m[1][1] = (((ww - xx) + yy) - zz) * inv;
            m[2][2] = (((ww - xx) - yy) + zz) * inv;
            m[0][1] = (w * z + y * x) * two;
            m[0][2] = (z * x - w * y) * two;
            m[1][0] = (y * x - w * z) * two;
            m[1][2] = (z * y + w * x) * two;
            m[2][0] = (w * y + z * x) * two;
            m[2][1] = (z * y - w * x) * two;
        }
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j)
                store(state, 96 + 8 * (3 * i + j), m[i][j]);
    }
    // The matrix's third row overwrites a previously supplied source normal.
    for (unsigned i = 0; i < 3; ++i)
        store(state, 72 + 8 * i, Reader(state, 144 + 8 * i).u64());
}
Json state_json(const Bytes &b) {
    const auto flags = Reader(b).u32();
    Json scalars = Json::object();
    const char *names[] = {"scale",     "dash_scale", "gap_scale",     "start_width",
                           "end_width", "shift",      "fraction_phase"};
    for (unsigned i = 0; i < 7; ++i)
        scalars[names[i]] = number(Reader(b, 8 + 8 * i).u64());
    Json matrix = Json::array();
    for (unsigned i = 0; i < 3; ++i) {
        Json row = Json::array();
        for (unsigned j = 0; j < 3; ++j)
            row.push_back(number(Reader(b, 96 + 8 * (3 * i + j)).u64()));
        matrix.push_back(std::move(row));
    }
    return {
        {"flags", flags},
        {"scalars", std::move(scalars)},
        {"multiline_index", Reader(b, 64).u32()},
        {"multiline_data_type", Reader(b, 68).u32()},
        {"orientation_vector",
         {number(Reader(b, 72).u64()), number(Reader(b, 80).u64()), number(Reader(b, 88).u64())}},
        {"orientation_matrix", std::move(matrix)},
        {"centered_shift", bool(flags & 0x80)},
        {"true_scale", bool(flags & 0x2000)},
        {"break_at_corners", bool(flags & 0x40000000u)},
        {"run_through_corners", bool(flags & 0x80000000u)},
        {"storage", rawbytes(b)}};
}
} // namespace
Json native_default_line_style_parameter_state() {
    return state_json(defaults());
}
Json load_native_line_style_parameter_link(const Json &link, bool dim3) {
    Json out = {{"status", "not_evaluated"},
                {"scope", "fresh_native_line_style_linkage_parameters"},
                {"three_dimensional", dim3},
                {"line_style_resource_loading", "not_evaluated"}};
    try {
        require(link.at("app") == 0x79f9, "line_style_parameter_linkage_required");
        const auto header = link.at("header").get<std::uint16_t>();
        const auto raw = bytesof(link.at("payload"));
        const auto size = linkage_size(header);
        require(raw.size() == size - 4, "line_style_linkage_payload_length_mismatch");
        auto state = defaults();
        out["source_header"] = header;
        out["source_payload"] = rawbytes(raw);
        int code = 0;
        if (size == 8) {
            out["selection"] = "length_four_words_uses_defaults";
            out["consumed_payload_bytes"] = 0;
        } else {
            out["source"] = decode_line_style_modifier_source(raw);
            if (out["source"].contains("orientation_data"))
                out["source"]["orientation_encoding"] = dim3 ? "quaternion_wxyz" : "matrix_2x2";
            Reader r(raw);
            const auto flags = r.u32();
            store(state, 0, flags);
            for (unsigned i = 0; i < 7; ++i)
                if (flags & (1u << i))
                    store(state, 8 + 8 * i, r.u64());
            if (flags & 0x100)
                for (unsigned i = 0; i < 3; ++i)
                    store(state, 72 + 8 * i, r.u64());
            if (flags & 0x200)
                orientation(state, r, dim3);
            if (flags & 0x800)
                store(state, 64, r.u32());
            if (flags & 0x1000)
                store(state, 68, r.u32());
            out["consumed_payload_bytes"] = r.p;
            out["rejected_flag_bits"] = flags & 0x3fffc400u;
            if (flags & 0x3fffc400u) {
                state = defaults();
                code = 1;
                out["selection"] = "rejected_flags_reset_defaults";
            } else
                out["selection"] = "loaded";
        }
        out.update(
            {{"status", "decoded"}, {"reader_return_code", code}, {"state", state_json(state)}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json native_graphic_line_style_input(const Bytes &body, const Json &links) {
    Json out = {{"status", "not_evaluated"},
                {"scope", "graphic_native_line_style_parameter_reader"},
                {"state_write", "not_evaluated"},
                {"line_style_resource_loading", "not_evaluated"}};
    try {
        Reader r(body);
        if (!(r.at<std::uint16_t>(6) & 0x20)) {
            out.update({{"status", "not_found"},
                        {"reader_return_code", 1},
                        {"state_write", "not_written"},
                        {"selection", "nonextended_header"}});
            return out;
        }
        if (r.at<std::uint32_t>(48) <= 7) {
            out.update({{"status", "not_found"},
                        {"reader_return_code", 1},
                        {"state_write", "not_written"},
                        {"selection", "builtin_line_style"}});
            return out;
        }
        const auto type = r.at<std::uint16_t>(4);
        if (type == 63) {
            const bool dim3 = (r.at<std::uint16_t>(36) & 0x1000) != 0;
            out["three_dimensional"] = dim3;
            out["dimension_source_offset"] = 36;
            out["parameter_reader"] = "multiline_default";
            out["ignored_link_indices"] = Json::array();
            // Unlike the ordinary reader, this route clears the destination
            // before scanning. It ignores the inner loader's return code and
            // stops at the first converted parameter whose data type is zero.
            if (r.at<std::uint32_t>(8) > r.at<std::uint32_t>(12))
                for (std::size_t i = 0; i < links.size(); ++i) {
                    const auto &link = links.at(i);
                    if (!(link.at("header").get<unsigned>() & 0x1000) || link.at("app") != 0x79f9)
                        continue;
                    auto loaded = load_native_line_style_parameter_link(link, dim3);
                    if (loaded.at("status") != "decoded") {
                        out["source_link_index"] = i;
                        out["source_offset"] = link.at("offset");
                        out["linkage"] = std::move(loaded);
                        out["reason"] = out["linkage"].at("reason");
                        return out;
                    }
                    if (loaded.at("state").at("multiline_data_type") != 0) {
                        out["ignored_link_indices"].push_back(i);
                        continue;
                    }
                    out.update({{"status", "decoded"},
                                {"reader_return_code", 0},
                                {"state_write", "written"},
                                {"selection", "first_multiline_default_parameter"},
                                {"source_link_index", i},
                                {"source_offset", link.at("offset")},
                                {"state", loaded.at("state")}});
                    out["linkage"] = std::move(loaded);
                    return out;
                }
            out.update({{"status", "not_found"},
                        {"reader_return_code", 1},
                        {"state_write", "written"},
                        {"selection", "no_multiline_default_parameter"},
                        {"state", state_json(Bytes(0xa8, 0))}});
            return out;
        }
        const bool dim3 = (r.at<std::uint16_t>(36) & 0x1000) != 0;
        out["three_dimensional"] = dim3;
        out["dimension_source_offset"] = 36;
        for (std::size_t i = 0; i < links.size(); ++i) {
            const auto &link = links.at(i);
            if (!(link.at("header").get<unsigned>() & 0x1000) || link.at("app") != 0x79f9)
                continue;
            out["source_link_index"] = i;
            out["source_offset"] = link.at("offset");
            require(linkage_size(link.at("header").get<std::uint16_t>()) <= 176,
                    "native_line_style_copy_exceeds_scratch_buffer");
            auto loaded = load_native_line_style_parameter_link(link, dim3);
            out["linkage"] = loaded;
            if (loaded.at("status") != "decoded") {
                out["reason"] = loaded.at("reason");
                return out;
            }
            out.update({{"status", "decoded"},
                        {"reader_return_code", loaded.at("reader_return_code")},
                        {"state_write", "written"}});
            auto state = bytesof(loaded.at("state").at("storage"));
            if (loaded.at("reader_return_code") == 0 &&
                (type == 27 || type == 38 || type == 41 || type == 52))
                store(state, 0, Reader(state).u32() | 0x80000000u);
            out["state"] = state_json(state);
            return out;
        }
        out.update({{"status", "not_found"},
                    {"reader_return_code", 1},
                    {"state_write", "not_written"},
                    {"selection", "no_parameter_linkage"}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json read_native_graphic_line_style_parameters(const Json &record) {
    try {
        return native_graphic_line_style_input(bytesof(record.at("data")), record.at("links"));
    } catch (const std::exception &e) {
        return {{"status", "not_evaluated"}, {"reason", e.what()}};
    }
}
Json native_layer_line_style_input(const Json &links) {
    Json out = {{"status", "decoded"},
                {"scope", "initial_layer_line_style_parameter_slots"},
                {"three_dimensional", false},
                {"entries", Json::array()},
                {"selected_entries", {{"primary", nullptr}, {"by_layer", nullptr}}},
                {"runtime_changes", "not_evaluated"}};
    bool known = true;
    for (std::size_t i = 0; i < links.size(); ++i) {
        const auto &link = links.at(i);
        if (!(link.at("header").get<unsigned>() & 0x1000) || link.at("app") != 0x79f9)
            continue;
        auto decoded = load_native_line_style_parameter_link(link, false);
        Json entry = {{"source_link_index", i},
                      {"source_offset", link.at("offset")},
                      {"parameters", decoded}};
        if (decoded.at("status") != "decoded") {
            known = false;
            entry["selection"] = "unknown";
        } else {
            const auto role = decoded.at("state").at("multiline_index").get<std::uint32_t>();
            entry["slot_index"] = role;
            if (role == 1 || role == 2) {
                const auto name = role == 1 ? "primary" : "by_layer";
                entry["selection"] = "assigned";
                entry["slot"] = name;
                out["selected_entries"][name] = out["entries"].size();
            } else
                entry["selection"] = "ignored_slot_index";
        }
        out["entries"].push_back(std::move(entry));
    }
    if (!known) {
        out["status"] = "not_evaluated";
        out["reason"] = "layer_parameter_input_unknown";
        out["selected_entries"] = {{"primary", nullptr}, {"by_layer", nullptr}};
    }
    return out;
}
} // namespace p3d
