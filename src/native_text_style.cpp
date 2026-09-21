#include "internal.hpp"
#include "text_bytes.hpp"

namespace p3d {
namespace {
// Scratch state follows the reader's assignments, not a persisted ABI dump.
struct TextState {
    Bytes data = Bytes(0xe8);
    template <class T> T at(std::size_t offset) const {
        return Reader(data).at<T>(offset);
    }
    template <class T> void set(std::size_t offset, T value) {
        std::memcpy(data.data() + offset, &value, sizeof(value));
    }
};
void read_style_link(unsigned app, const Bytes &payload, TextState &state, Json &info) {
    Reader input(payload);
    std::uint32_t keep_flags = 0xff90fffeu;
    bool big_reference_known = payload.empty();
    info["fields"] = Json::array();
    auto read = [&](std::size_t target, std::size_t size, const char *encoding, const char *name) {
        const auto start = input.p;
        const auto bytes = input.take(size);
        if (target < state.data.size())
            std::copy(bytes.begin(), bytes.end(), state.data.begin() + target);
        Reader value(bytes);
        Json parsed;
        if (std::string(encoding) == "float64")
            parsed = value.f64();
        else if (std::string(encoding) == "uint16_pair")
            parsed = value.uints(2, 2);
        else if (std::string(encoding) == "uint16_quad")
            parsed = value.uints(4, 2);
        else
            parsed = size == 2 ? Json(value.u16()) : Json(value.u32());
        info["fields"].push_back({{"source_offset", start},
                                  {"source_bytes", size},
                                  {"encoding", encoding},
                                  {"name", name ? Json(name) : Json()},
                                  {"value", parsed},
                                  {"raw_hex", hex(bytes)}});
        if (name && std::string(name).find("line_style") != std::string::npos) {
            const auto bits = parsed.get<std::uint32_t>();
            info["fields"].back()["signed_value"] =
                bits <= INT32_MAX ? std::int64_t(bits) : std::int64_t(bits) - 0x100000000ll;
        }
        return parsed;
    };
    try {
        if (payload.empty()) {
            info["status"] = "empty";
        } else if (app == 0x80d4) {
            const auto flags = read(0x50, 2, "uint16", "flags16").get<unsigned>();
            big_reference_known = !(flags & 0x400);
            read(0xe8, 2, "uint16", "reserved_word");
            const auto extended = read(0x54, 4, "uint32", "flags32").get<std::uint32_t>();
            if ((flags & 3) || (extended & 0x400000u)) {
                read(0x38, 8, "float64", "inter_character_spacing");
                keep_flags = 0xffd0fffeu;
            }
            if (flags & 8) {
                read(0x28, 8, "float64", "custom_slant_angle");
                const auto angle = state.at<double>(0x28);
                if (angle > 3.141592653589793)
                    state.set(0x28, angle - 6.283185307179586);
            }
            if (flags & 4)
                read(0x40, 8, "float64", "underline_offset");
            if (flags & 0x100) {
                read(0x58, 8, "float64", "line_offset_x");
                read(0x60, 8, "float64", "line_offset_y");
            }
            if (flags & 0x200)
                read(0x10, 4, "uint32", nullptr);
            if (flags & 0x400) {
                read(0x18, 4, "uint32", "big_font_id");
                big_reference_known = true;
            }
            if (flags & 0x800) {
                read(0x80, 4, "uint32", "background_color");
                read(0x84, 4, "uint32", "background_line_style");
                read(0x88, 4, "uint32", "background_weight");
                read(0x68, 8, "float64", "background_border_x");
                read(0x70, 8, "float64", "background_border_y");
                read(0x1c, 4, "uint32", "background_fill_color");
            }
            if (extended & 1) {
                read(0xa8, 8, "float64", "overline_offset");
                keep_flags |= 1;
            }
            if (flags & 0x8000)
                read(0x20, 4, "uint32", nullptr);
            if (extended & 0x10000) {
                read(0x8c, 4, "uint32", "underline_color");
                read(0x90, 4, "uint32", "underline_line_style");
                read(0x94, 4, "uint32", "underline_weight");
                keep_flags |= 0x10000;
            }
            if (extended & 0x20000) {
                read(0x98, 4, "uint32", "overline_color");
                read(0x9c, 4, "uint32", "overline_line_style");
                read(0xa0, 4, "uint32", "overline_weight");
                keep_flags |= 0x20000;
            }
            if (extended & 0x80000) {
                read(0x48, 8, "float64", nullptr);
                keep_flags |= 0x80000;
            }
            if (extended & 0x40000) {
                read(0xb0, 8, "uint16_quad", "control_words");
                keep_flags |= 0x40000;
            }
            if (extended & 0x200000) {
                read(0x24, 4, "uint32", "color");
                keep_flags |= 0x200000;
            }
            if (flags & 0x4000) {
                read(0xcc, 4, "uint32", nullptr);
                read(0xd0, 4, "uint32", nullptr);
                read(0xd4, 4, "uint32", nullptr);
                // This final vector is copied atomically by the native reader.
                const auto start = input.p;
                const auto bytes = input.take(16);
                std::copy(bytes.begin(), bytes.end(), state.data.begin() + 0xd8);
                info["fields"].push_back({{"source_offset", start},
                                          {"source_bytes", 16},
                                           {"encoding", "bytes"},
                                          {"name", nullptr},
                                           {"value", rawbytes(bytes)},
                                          {"raw_hex", hex(bytes)}});
            }
            info["status"] = "decoded";
        } else if (app == 0x56f3) {
            read(0xe8, 2, "uint16", "reserved_word_0");
            read(0xe8, 2, "uint16", "reserved_word_1");
            read(0xc0, 8, "float64", nullptr);
            state.set(0x54, state.at<std::uint32_t>(0x54) | 0x10000000u);
            const auto packed = read(0xe8, 2, "uint16", "packed_control_flags").get<unsigned>();
            const auto low = ((packed & 0x3c) * 2 | (packed & 3)) << 2;
            state.set<std::uint16_t>(0xb0, (state.at<std::uint16_t>(0xb0) & 0xfe13) | low);
            state.set<std::uint16_t>(0xb2, (state.at<std::uint16_t>(0xb2) & 0xfdff) |
                                               ((packed & 0x80) << 2));
            state.set<std::uint16_t>(0xb4, (state.at<std::uint16_t>(0xb4) & 0xfffe) |
                                               ((packed >> 6) & 1));
            state.set<std::uint16_t>(0xb6, (state.at<std::uint16_t>(0xb6) & 0xffef) |
                                               ((packed >> 4) & 0x10));
            info["status"] = "decoded";
        } else {
            read(0xc8, 4, "uint32", "auxiliary_flags");
            info["status"] = "decoded";
        }
    } catch (const std::exception &e) {
        info["status"] = "truncated";
        info["error"] = e.what();
        if (app == 0x80d4) {
            state.set<std::uint16_t>(0x50, state.at<std::uint16_t>(0x50) & keep_flags);
            state.set(0x54, state.at<std::uint32_t>(0x54) & keep_flags);
            info["truncation_keep_mask"] = keep_flags;
        }
    }
    info["consumed_bytes"] = input.p;
    info["ignored_suffix_hex"] = hex(slice(payload, input.p, input.left()));
    if (app == 0x80d4)
        info["font_reference_status"] = big_reference_known ? "resolved" : "unresolved";
}
} // namespace

Json decode_native_text_style(const Bytes &base, const Json &links) {
    Json out = {{"status", "invalid"},
                {"scope", "initial_type54_style_after_source_links"},
                {"semantic_status", "partial"},
                {"font_reference_status", "unresolved"},
                {"links", Json::array()}};
    try {
        Reader source(base);
        require(base.size() >= 116 && source.at<std::uint16_t>(4) == 54,
                "native text style base header");
        TextState state;
        state.set(0, source.at<std::uint32_t>(108));
        state.set<std::uint32_t>(4, source.at<std::uint16_t>(112));
        state.set(0x24, source.at<std::uint32_t>(56));
        state.set<double>(0xc0, 1);
        bool complete = true, font_complete = true;
        // These getters execute in this order regardless of physical link order.
        for (unsigned app : {0x80d4u, 0x56f3u, 0x5817u}) {
            bool seen = false;
            for (std::size_t i = 0; i < links.size(); ++i) {
                const auto &link = links[i];
                if (!(link.at("header").get<unsigned>() & 0x1000) || link.at("app") != app)
                    continue;
                Json info = {{"app", app},
                             {"link_index", i},
                             {"link_offset", link.at("offset")},
                             {"reader_selection", seen ? "shadowed" : "first_match"}};
                if (seen) {
                    info["status"] = "not_selected";
                } else {
                    const auto payload = bytesof(link.at("payload"));
                    read_style_link(app, payload, state, info);
                    if (info.at("status") == "truncated")
                        complete = false;
                    if (app == 0x80d4 && info.at("font_reference_status") != "resolved")
                        font_complete = false;
                }
                out["links"].push_back(std::move(info));
                seen = true;
            }
        }
        const auto flags = state.at<std::uint16_t>(0x50);
        const auto extended = state.at<std::uint32_t>(0x54);
        const auto mode = flags & 2 ? 1u : extended & 0x400000 ? 2u : 0u;
        Json values = {
            {"font_id", state.at<std::uint32_t>(0)},
            {"font_lookup_id", state.at<std::uint32_t>(0) & 0xffffu},
            {"big_font_id", state.at<std::uint32_t>(0x18)},
            {"big_font_lookup_id", state.at<std::uint32_t>(0x18) & 0xffffu},
            {"justification_value", state.at<std::uint32_t>(4)},
            {"color", state.at<std::uint32_t>(0x24)},
            {"custom_slant_angle", state.at<double>(0x28)},
            {"inter_character_spacing_mode", mode},
            {"inter_character_spacing", mode || (flags & 1) ? state.at<double>(0x38) : 0.},
            {"line_offset", Reader(state.data, 0x58).doubles(2)},
            {"background_border", Reader(state.data, 0x68).doubles(2)}};
        for (const auto &[name, offset] :
             std::vector<std::pair<const char *, unsigned>>{{"background_fill_color", 0x1c},
                                                            {"background_color", 0x80},
                                                            {"background_line_style", 0x84},
                                                            {"background_weight", 0x88},
                                                            {"underline_color", 0x8c},
                                                            {"underline_line_style", 0x90},
                                                            {"underline_weight", 0x94},
                                                            {"overline_color", 0x98},
                                                            {"overline_line_style", 0x9c},
                                                            {"overline_weight", 0xa0}})
            values[name] = state.at<std::uint32_t>(offset);
        values["underline_offset"] = state.at<double>(0x40);
        values["overline_offset"] = state.at<double>(0xa8);
        out["initial_values"] = std::move(values);
        out["effective_flags16"] = flags;
        out["effective_flags32"] = extended;
        out["flags_decoded"] = {{"underline", bool(flags & 4)},
                                {"italic", bool(flags & 8)},
                                {"background_style", bool(flags & 0x800)},
                                {"subscript", bool(flags & 0x1000)},
                                {"superscript", bool(flags & 0x2000)},
                                {"overline", bool(extended & 1)},
                                {"bold", bool(extended & 0x8000)},
                                {"underline_style", bool(extended & 0x10000)},
                                {"overline_style", bool(extended & 0x20000)},
                                {"color_override", bool(extended & 0x200000)}};
        out["unassigned_state"] = {{"control_words", Reader(state.data, 0xb0).uints(4, 2)},
                                   {"auxiliary_scalar", state.at<double>(0xc0)},
                                   {"auxiliary_flags", state.at<std::uint32_t>(0xc8)}};
        out["status"] = complete ? "decoded" : "partial";
        out["font_reference_status"] = font_complete ? "resolved" : "unresolved";
    } catch (const std::exception &e) {
        out["error"] = e.what();
    }
    return out;
}
} // namespace p3d
