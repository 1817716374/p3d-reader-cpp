#include "internal.hpp"

namespace p3d {
Json decode_material_auxiliary_records(const Bytes &b) {
    Json out = {{"reader_profile", "bimbase_2025_material_auxiliary_records"},
                {"status", "resolved"},
                {"records", Json::array()},
                {"selected_record_indices", Json::array()}};
    // The input reader promotes byte 19 to a nonnegative int and orders it
    // before unsigned word 16. Reusing a key resets and replaces its value.
    std::map<std::pair<unsigned, unsigned>, std::size_t> selected;
    std::size_t pos = 0;
    while (pos < b.size()) {
        Json record = {{"offset", pos}};
        try {
            require(b.size() - pos >= 8, "material auxiliary record header truncated");
            Reader r(b, pos);
            const auto size = r.u32(), version = r.u32();
            record.update({{"record_bytes", size}, {"version", version}});
            require(size != 0, "material auxiliary record does not advance");
            if (version != 2) {
                record["action"] = "skipped_version";
            } else {
                require(size <= b.size() - pos, "material auxiliary record exceeds buffer");
                require(size >= 64, "material auxiliary version 2 record shorter than header");
                const auto a = r.u32(), c = r.u32();
                const auto key_word = r.u16();
                const auto value_byte = r.u8(), key_byte = r.u8();
                const auto value_word = r.u16(), unused_word = r.u16();
                const auto x = r.f64(), y = r.f64();
                const auto value_dword = r.u32();
                record["key"] = {{"byte_19", key_byte}, {"word_16", key_word}};
                record["unassigned_fields"] = {{"u32_8", a},
                                               {"u32_12", c},
                                               {"u8_18", value_byte},
                                               {"u16_20", value_word},
                                               {"f64_24", std::isfinite(x) ? Json(x) : Json()},
                                               {"f64_32", std::isfinite(y) ? Json(y) : Json()},
                                               {"u32_40", value_dword}};
                record["header"] = rawbytes(slice(b, pos, 64));
                record["ignored_header_fields"] = {{"u16_22", unused_word},
                                                   {"bytes_44_63", rawbytes(r.take(20))}};
                record["data"] = rawbytes(slice(b, pos + 64, size - 64));
                const auto key = std::make_pair(unsigned(key_byte), unsigned(key_word));
                const auto previous = selected.find(key);
                record["action"] = previous == selected.end() ? "inserted" : "replaced";
                if (previous != selected.end())
                    record["replaced_record_index"] = previous->second;
                selected[key] = out["records"].size();
            }
            out["records"].push_back(std::move(record));
            pos += size;
        } catch (const std::exception &e) {
            record.update({{"action", "unresolved"}, {"reason", e.what()}});
            out["records"].push_back(std::move(record));
            out["status"] = "partial";
            break;
        }
    }
    for (const auto &entry : selected)
        out["selected_record_indices"].push_back(entry.second);
    // A skipped version only reads the eight-byte prefix. Its advance may
    // pass the buffer end, which ends the native loop without reading the body.
    out["native_cursor"] = pos;
    out["consumed_bytes"] = std::min(pos, b.size());
    out["remaining_bytes"] = b.size() - std::min(pos, b.size());
    return out;
}
} // namespace p3d
