#include "internal.hpp"
namespace p3d {
static std::string layer_string(const Bytes &b) {
    // This linkage reader supplies native code page 1200: unmarked bytes are
    // widened individually, and both FF FE and FF FD wide forms are UTF-16LE.
    // Other native string users can supply a different code page.
    if (b.empty() || b[0] == 0)
        return "";
    std::string text;
    if (b.size() >= 2 && b[0] == 255 && (b[1] == 254 || b[1] == 253)) {
        bool narrow = b[1] == 254 && b.size() >= 4 && b[2] == 1 && b[3] == 0;
        auto body = slice(b, narrow ? 4 : 2, b.size() - (narrow ? 4 : 2));
        require(narrow || body.size() % 2 == 0, "layer UTF16 byte length");
        text = narrow ? latin1(body) : utf16(body);
    } else {
        require(b.size() < 2 || b[1] != 255 || (b[0] != 253 && b[0] != 254),
                "unsupported layer string byte order");
        text = latin1(b);
    }
    auto nul = text.find('\0');
    if (nul != std::string::npos)
        text.resize(nul);
    return text;
}
// Offsets below include the four-byte native stream prefix. A layer ID is
// scoped to its containing layer table, not to the whole document.
Json decode_native_layer(const Bytes &b, const Json &links) {
    Json out = {{"encoding", "layer_definition"}, {"view_visibility", "not_evaluated"}};
    Json strings = Json::array();
    for (const auto &link : links) {
        if (link["app"] != 0x56d2 || !(link["header"].get<unsigned>() & 0x1000))
            continue;
        Json text = {{"source_offset", link["offset"]}};
        try {
            auto raw = bytesof(link["payload"]);
            Reader r(raw);
            auto key = r.u16(), flags = r.u16();
            auto length = r.u32();
            text.update({{"key", key}, {"flags", flags}, {"byte_count", length}});
            auto value = layer_string(r.take(length));
            text["text"] = value;
            text["trailing_storage"] = rawbytes(r.take(r.left()));
            if (key == 1 || key == 2) {
                auto role = key == 1 ? "name" : "description";
                text["role"] = role;
                out[role] = value;
            }
        } catch (const std::exception &e) {
            text["decode_error"] = e.what();
        }
        strings.push_back(std::move(text));
    }
    out["strings"] = std::move(strings);
    try {
        Reader r(b, 36);
        auto id = r.u32();
        auto field28 = r.u32();
        auto version = r.u16();
        out.update({{"layer_id", id}, {"field_at_28", field28}, {"version", version}});
        // Version 0 has a different layout. Later versions are not presumed
        // compatible merely because the original reader has a default branch.
        if (version < 1 || version > 7) {
            out["status"] = "unsupported_version";
            return out;
        }
        r.need(62); // common fields through the 64-bit value at offset 100
        auto state = r.u16();
        auto extra = r.at<std::uint32_t>(92);
        unsigned access = (state >> 12) & 3;
        if (state & 0x10)
            access |= 1; // native reader restores this legacy access bit
        auto legacy = r.at<std::uint32_t>(76), extended = r.at<std::uint32_t>(80);
        auto color = legacy;
        if (extended && (extended <= 0xfffffffdu ? (extended & 255) : extended) == legacy)
            color = extended;
        float transparency = r.at<float>(84);
        out.update({{"status", "decoded"},
                    {"state_flags", state},
                    {"display", bool(state & 0x20)},
                    {"print", bool(state & 0x40)},
                    {"frozen", bool(state & 0x4000)},
                    {"access_mode", access},
                    {"locked", access == 1},
                    {"unassigned_state_bits", state & ~0x7070u},
                    {"by_layer_symbology",
                     {{"color_index", color},
                      {"legacy_color_index", legacy},
                      {"extended_color_index", extended},
                      {"line_style", r.at<std::int32_t>(68)},
                      {"line_weight", r.at<std::uint32_t>(72)}}},
                    {"transparency", std::isfinite(transparency) ? Json(transparency) : Json()},
                    {"transparency_source_bits", r.at<std::uint32_t>(84)},
                    {"extended_flags", extra},
                    {"business_code", version >= 4 ? Json((extra >> 3) & 0xffff) : Json()},
                    {"unassigned_extended_bits", extra & ~(version >= 4 ? 0x7fff8u : 0u)},
                    {"unassigned_ranges",
                     Json::array({{{"source_offset", 48}, {"data", rawbytes(slice(b, 48, 20))}},
                                  {{"source_offset", 88}, {"data", rawbytes(slice(b, 88, 4))}},
                                  {{"source_offset", 96},
                                   {"data", rawbytes(slice(b, 96, b.size() - 96))}}})}});
        if (!std::isfinite(transparency))
            out["transparency_error"] = "nonfinite source value";
    } catch (const std::exception &e) {
        out["decode_error"] = e.what();
    }
    return out;
}
} // namespace p3d
