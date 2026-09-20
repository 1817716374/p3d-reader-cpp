#include "internal.hpp"
#include "text_bytes.hpp"

namespace p3d {
Json decode_text_bytes(const Bytes &b) {
    Reader r(b);
    const auto version = r.u8();
    const auto header = r.u32(), text_size = r.u32();
    require(header >= 97 && std::uint64_t(header) + text_size == b.size(), "text payload length");
    auto text = utf16(slice(b, header, text_size));
    while (!text.empty() && text.back() == 0)
        text.pop_back();
    Json out = {
        {"text", text}, {"version", version}, {"header_bytes", header}, {"text_bytes", text_size}};
    out["origin"] = r.doubles(3);
    out["quaternion"] = r.doubles(4);
    out["width"] = r.f64();
    out["height"] = r.f64();
    const auto color = r.u32(), font = r.u32(), flags = r.u32(), presence = r.u32();
    out["style_words"] = Json::array({color, font, flags, presence});
    out["extra_style_hex"] = hex(slice(b, 97, header - 97));
    Json style = {{"status", "not_evaluated"},        {"color", color}, {"font_id", font},
                  {"font_lookup_id", font & 0xffffu}, {"flags", flags}, {"presence_mask", presence},
                  {"fields", Json::array()}};
    // Keep the old source view even if a new extension cannot be interpreted.
    if (version != 1) {
        style["reason"] = "unsupported_text_version";
        out["text_style"] = std::move(style);
        return out;
    }
    style["flags_decoded"] = {
        {"underline", bool(flags & (1u << 1))}, {"overline", bool(flags & (1u << 3))},
        {"bold", bool(flags & (1u << 5))},      {"italic", bool(flags & (1u << 6))},
        {"vertical", bool(flags & (1u << 10))}, {"is_3d", bool(flags & (1u << 14))}};
    static const char *justifications[] = {"LeftTop",
                                           "LeftMiddle",
                                           "LeftBaseline",
                                           "LeftMarginTop",
                                           "LeftMarginMiddle",
                                           "LeftMarginBaseline",
                                           "CenterTop",
                                           "CenterMiddle",
                                           "CenterBaseline",
                                           "RightMarginTop",
                                           "RightMarginMiddle",
                                           "RightMarginBaseline",
                                           "RightTop",
                                           "RightMiddle",
                                           "RightBaseline",
                                           "LeftCap",
                                           "LeftDescender",
                                           "LeftMarginCap",
                                           "LeftMarginDescender",
                                           "CenterCap",
                                           "CenterDescender",
                                           "RightMarginCap",
                                           "RightMarginDescender",
                                           "RightCap",
                                           "RightDescender"};
    const auto justification = (flags >> 20) & 0xffu;
    style["justification"] = {{"value", justification},
                              {"name", justification < 25     ? Json(justifications[justification])
                                       : justification == 127 ? Json("Invalid")
                                                              : Json()}};
    // These are the native reader's sequential optional fields, NOT a C++ ABI
    // dump. Some business names remain unassigned; source bit and position stay.
    static const unsigned widths[] = {4, 4,  4, 8, 4, 4, 4, 8,  4, 4, 4,
                                      4, 16, 1, 8, 4, 8, 4, 16, 4, 8};
    try {
        for (unsigned bit = 0; bit < 21; ++bit) {
            if (!(presence & (1u << bit)))
                continue;
            const auto start = r.p;
            require(start <= header && widths[bit] <= header - start,
                    "text optional field exceeds declared header");
            Json field = {{"bit", bit},
                          {"source_offset", start},
                          {"source_bytes", widths[bit]},
                          {"name", nullptr}};
            if (bit == 13) {
                field["value"] = r.u8();
                field["encoding"] = "uint8";
            } else if (bit == 3 || bit == 7 || bit == 14 || bit == 16 || bit == 20) {
                field["value"] = r.f64();
                field["encoding"] = "float64";
            } else if (bit == 12 || bit == 18) {
                field["value"] = r.doubles(2);
                field["encoding"] = "float64_pair";
            } else {
                field["value"] = r.u32();
                field["encoding"] = "uint32_bits";
            }
            if (bit == 15) {
                field["name"] = "big_font_id";
                const auto id = field.at("value").get<std::uint32_t>();
                field["native_font_id_in_lookup_range"] = id >= 512 && id <= 1023;
            } else if (bit == 16) {
                field["name"] = "custom_slant_angle";
            } else if (bit == 17) {
                field["native_usage"] = "skipped";
            } else if (bit == 20) {
                field["native_usage"] = "auxiliary_output_parameter";
            }
            field["raw_hex"] = hex(slice(b, start, widths[bit]));
            style["fields"].push_back(std::move(field));
        }
        if (presence & (1u << 21)) {
            require(text_size >= 2 && text_size % 2 == 0,
                    "text character mask requires UTF16 count");
            const std::size_t count = text_size / 2 - 1;
            const auto bytes = (count + 7) / 8;
            require(r.p <= header && bytes <= header - r.p, "text character mask exceeds header");
            style["character_mask"] = {{"source_offset", r.p},
                                       {"bit_count", count},
                                       {"bit_order", "least_significant_first"},
                                       {"packed_bits_hex", hex(slice(b, r.p, bytes))},
                                       {"remaining_header_hex", hex(slice(b, r.p, header - r.p))}};
            // Native storage uses 16-bit words; do not read beyond the source
            // header to fetch unused padding, or assign a purpose to marked text.
            r.p = header;
        }
        style["unassigned_presence_bits"] = presence & 0xffc00000u;
        if (r.p < header)
            style["unassigned_suffix_hex"] = hex(slice(b, r.p, header - r.p));
        style["status"] = (presence & 0xffc00000u) || r.p < header ? "partial" : "decoded";
    } catch (const std::exception &e) {
        style["status"] = "invalid";
        style["decode_error"] = e.what();
    }
    out["text_style"] = std::move(style);
    return out;
}
} // namespace p3d
