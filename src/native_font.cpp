#include "internal.hpp"
#include "text_bytes.hpp"

namespace p3d {
Json decode_native_font_record(const Bytes &b) {
    Json out = {{"status", "invalid"}, {"font_resolution_status", "not_evaluated"}};
    try {
        Reader r(b);
        require(b.size() >= 36 && r.at<std::uint32_t>(16) == 2, "native font record class");
        const auto type = r.at<std::uint16_t>(4);
        require(type == 10 || type == 49, "native font record type");
        if (type == 10) {
            r.p = (r.at<std::uint16_t>(6) & 0x20u) ? 108 : 36;
            out["encoding"] = "native_font_table";
            out["descendant_count_source_offset"] = r.p;
            out["unassigned_header_extension"] = rawbytes(slice(b, 36, r.p - 36));
            out["declared_descendant_count"] = r.u32();
            out["unassigned_suffix"] = rawbytes(r.take(r.left()));
            out["status"] = "decoded";
            return out;
        }
        out["encoding"] = "native_font_definition";
        r.p = 44;
        const auto id = r.u32();
        out["font_id"] = id;
        out["initial_loader_action"] = id < 512 ? "skip_font_id" : "request_font";
        out["requested_font_family"] = id < 512 ? Json() : Json(id < 1024 ? "Shx" : "TrueType");
        out["unassigned_header_extension"] = rawbytes(slice(b, 36, 8));
        const auto count = r.u16();
        out["name_byte_count"] = count;
        out["name_source_offset"] = r.p;
        const auto source = r.take(count);
        out["name_source"] = rawbytes(source);
        out["unassigned_suffix"] = rawbytes(r.take(r.left()));
        out["name_source_units"] = Json::array();
        for (std::size_t i = 0; i + 1 < source.size(); i += 2)
            out["name_source_units"].push_back(unsigned(source[i]) | unsigned(source[i + 1]) << 8);
        // The native copy floors the byte count to wchar_t units. Its fixed
        // destination has 512 units, including the terminator it appends.
        out["ignored_name_suffix_hex"] = count % 2 ? hex(slice(source, count - 1, 1)) : "";
        out["name"] = nullptr;
        out["status"] = "decoded";
        try {
            require(count / 2 < 512, "native font name exceeds 512-unit buffer");
            Bytes name;
            for (std::size_t i = 0; i + 1 < source.size(); i += 2) {
                if (source[i] == 0 && source[i + 1] == 0)
                    break;
                name.push_back(source[i]);
                name.push_back(source[i + 1]);
            }
            // Keep malformed UTF-16 as source units instead of emitting an
            // invalid UTF-8 string into the public JSON representation.
            for (std::size_t i = 0; i < name.size(); i += 2) {
                const auto unit = unsigned(name[i]) | unsigned(name[i + 1]) << 8;
                require(unit < 0xdc00 || unit > 0xdfff, "native font name lone low surrogate");
                if (unit >= 0xd800 && unit <= 0xdbff) {
                    require(i + 3 < name.size(), "native font name missing low surrogate");
                    const auto low = unsigned(name[i + 2]) | unsigned(name[i + 3]) << 8;
                    require(low >= 0xdc00 && low <= 0xdfff, "native font name surrogate pair");
                    i += 2;
                }
            }
            out["name"] = utf16(name);
            out["name_read_status"] = "decoded";
        } catch (const std::exception &e) {
            out["name_read_status"] = "invalid";
            out["name_read_error"] = e.what();
            out["status"] = "partial";
        }
    } catch (const std::exception &e) {
        out["error"] = e.what();
    }
    return out;
}
} // namespace p3d
