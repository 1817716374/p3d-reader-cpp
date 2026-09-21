#include "internal.hpp"
#include "text_bytes.hpp"

namespace p3d {
Json decode_native_text_record(const Bytes &b) {
    Reader r(b);
    require(b.size() >= 40 && r.at<std::uint16_t>(4) == 54, "native text record header");
    const auto flags = r.at<std::uint16_t>(6);
    const bool spatial = !(flags & 0x20u) || (r.at<std::uint16_t>(36) & 0x1000u);
    const std::size_t start = spatial ? 206 : 174;
    require(b.size() >= start, "native text fixed header");
    r.p = 108;
    const auto words = r.uints(4, 2);
    const auto count = words[3].get<unsigned>();
    require(count <= b.size() - start, "native text extent");
    r.p = 116;
    const auto scale = r.doubles(2);
    Json out = {{"status", "decoded"},
                {"source_layout", spatial ? "spatial" : "planar"},
                {"text_offset", start},
                {"text_bytes", count},
                {"font_id", r.at<std::uint32_t>(108)},
                {"font_lookup_id", r.at<std::uint32_t>(108) & 0xffffu},
                {"justification_value", words[2]},
                {"style_words", words},
                {"font_scale", scale},
                {"width", r.f64()},
                {"height", r.f64()}};
    // Source scale and stored extents are distinct. This getter's native zero
    // fallback and factor do not depend on the stored text bounding dimensions.
    out["native_font_size"] = Json::array();
    for (const auto &axis : scale) {
        const double value = axis.get<double>();
        out["native_font_size"].push_back((value == 0 ? 1. : value) * 0.006);
    }
    if (spatial) {
        out["quaternion"] = r.doubles(4);
        out["origin"] = r.doubles(3);
    } else {
        const double angle = r.f64();
        out["rotation_angle"] = angle;
        out["quaternion"] = Json::array({std::cos(angle / 2), 0., 0., std::sin(angle / 2)});
        out["origin"] = r.doubles(2);
        out["origin"].push_back(0.);
    }
    const auto range_count = r.u16();
    require(r.p == start, "native text coordinate layout");
    const auto source = r.take(count);
    out["text_source"] = rawbytes(source);
    out["source_encoding_marker"] = hex(slice(source, 0, std::min<std::size_t>(2, source.size())));
    out["trailing_hex"] = hex(slice(b, r.p, r.left()));
    out["text"] = nullptr;
    Json conversion = {{"status", "not_evaluated"}, {"font_mapping_status", "not_evaluated"}};
    try {
        std::size_t prefix = 0;
        bool wide = false;
        const auto first = source.size() >= 2 ? Reader(source).u16() : 0u;
        if (!source.empty() && source.front() != 0) {
            if (first == 0xfdff) {
                prefix = 2;
                wide = true;
                conversion["encoding"] = "font_encoded_16bit_units";
            } else if (first == 0xfeff) {
                require(source.size() >= 4, "native text marker lookahead");
                const bool bytes = Reader(source, 2).u16() == 1;
                prefix = bytes ? 4 : 2;
                wide = !bytes;
                conversion["encoding"] = bytes ? "unsigned_bytes" : "utf16le_units";
            } else {
                require(first != 0xfffd && first != 0xfffe, "unsupported native text marker");
                conversion["encoding"] = "unsigned_bytes";
            }
        } else {
            conversion["encoding"] = "empty";
            prefix = source.size();
        }
        require(!wide || (source.size() - prefix) % 2 == 0, "odd native text wide payload");
        conversion["prefix_bytes"] = prefix;
        conversion["source_units"] = Json::array();
        Bytes unicode;
        for (std::size_t i = prefix; i < source.size(); i += wide ? 2 : 1) {
            const unsigned unit = source[i] | (wide ? unsigned(source[i + 1]) << 8 : 0);
            conversion["source_units"].push_back(unit);
            unicode.push_back(std::uint8_t(unit));
            unicode.push_back(std::uint8_t(unit >> 8));
        }
        if (first == 0xfdff) {
            // Keep the existing string preview for API compatibility, while
            // exposing that the native font/codepage mapping is still required.
            out["text"] = native_string(source);
            conversion["text_interpretation"] = "gb18030_compatibility_preview";
        } else {
            auto text = utf16(unicode);
            while (!text.empty() && text.back() == 0)
                text.pop_back();
            out["text"] = text;
            conversion["text_interpretation"] = "source_units_before_font_mapping";
        }
        conversion["status"] = "decoded";
    } catch (const std::exception &e) {
        conversion["status"] = "invalid";
        conversion["error"] = e.what();
    }
    out["text_conversion"] = std::move(conversion);
    out["declared_text_range_count"] = range_count;
    out["native_text_range_count"] = std::min<unsigned>(range_count, 20);
    out["text_ranges"] = Json::array();
    try {
        for (unsigned i = 0; i < std::min<unsigned>(range_count, 20); ++i) {
            const auto offset = r.p;
            const unsigned begin = r.u8(), length = r.u8(), alignment = r.u8();
            out["text_ranges"].push_back({{"source_offset", offset},
                                          {"source_index", i},
                                          {"start_1_based", begin},
                                          {"length", length},
                                          {"control_byte", alignment},
                                          {"raw_hex", hex(slice(b, offset, 3))}});
        }
        out["text_range_status"] = "decoded";
        if (r.left())
            out["unassigned_suffix_hex"] = hex(slice(b, r.p, r.left()));
    } catch (const std::exception &e) {
        out["text_range_status"] = "invalid";
        out["text_range_error"] = e.what();
    }
    if (out["text_conversion"]["status"] == "invalid" || out["text_range_status"] == "invalid")
        out["status"] = "partial";
    return out;
}
} // namespace p3d
