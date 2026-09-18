#include "internal.hpp"

namespace p3d {
namespace {
Json directory_text(Reader &reader, unsigned declared) {
    Json out = {{"declared_bytes", declared}, {"source_offset", reader.p}};
    // The native wide read consumes floor(length/2) units, including for an
    // odd declared byte count; there is no extra byte skip afterwards.
    require((declared & ~1u) < 0x208, "directory_string_exceeds_native_buffer");
    const auto bytes = reader.take(declared & ~1u);
    out["source_storage"] = rawbytes(bytes);
    std::vector<std::uint16_t> units;
    for (std::size_t at = 0; at < bytes.size(); at += 2) {
        const auto unit = Reader(bytes, at).u16();
        if (!unit)
            break;
        units.push_back(unit);
    }
    out["utf16_code_units"] = units;
    out["status"] = "decoded";
    try {
        auto text = utf16(slice(bytes, 0, units.size() * 2));
        (void)Json(text).dump();
        out["text"] = std::move(text);
    } catch (const std::exception &) {
        out["text_status"] = "invalid_utf16";
    }
    return out;
}

Json directory_mask(const Bytes &bytes) {
    Json out = {{"source_storage", rawbytes(bytes)}, {"role", "unresolved"}};
    if (bytes.empty()) {
        out["status"] = "absent";
        return out;
    }
    if (bytes.size() < 8) {
        out["status"] = "not_evaluated";
        out["reason"] = "native_bitmap_header_outside_payload";
        return out;
    }
    Reader reader(bytes);
    const auto bits = reader.u32(), fill = reader.u32();
    const auto words = std::uint32_t(bits + 15u) >> 4;
    out["source_bit_count"] = bits;
    out["source_default_word"] = fill;
    if (std::uint64_t(words) * 2 != reader.left()) {
        out["status"] = "rejected";
        out["native_result"] = "null_bitmap";
        return out; // The directory reader ignores the bitmap decoder's error.
    }
    out["status"] = "decoded";
    out["effective_bit_count"] = std::uint64_t(words) * 16;
    out["default_bit"] = fill != 0;
    out["packed_words"] = Json::array();
    while (reader.left())
        out["packed_words"].push_back(reader.u16());
    return out;
}
} // namespace

Json parse_native_model_directory(const Bytes &bytes, std::optional<std::uint32_t> default_id) {
    Json out = {{"scope", "persisted_model_directory_before_runtime_refresh"},
                {"status", "not_evaluated"},
                {"native_input_status", "not_evaluated"},
                {"entries", Json::array()}};
    Reader reader(bytes);
    try {
        require(reader.u32() == 0xaa00ba11u, "invalid_model_directory_signature");
        const auto version = reader.u32(), count = reader.u32(), extension_bytes = reader.u32();
        out["version"] = version;
        out["declared_count"] = count;
        require(version >= 4, "model_directory_requires_header_rebuild_for_older_version");
        require(extension_bytes <= 0xfe0, "directory_header_exceeds_native_stack_buffer");
        out["header_extension"] = rawbytes(reader.take(extension_bytes));
        require(count <= reader.left() / 32, "truncated_model_directory_entry_headers");
        bool native_known = true;
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto at = reader.p;
            const auto header = reader.take(32);
            Reader h(header);
            const auto kind = h.u16(), flags = h.u16();
            const auto id = h.u32();
            const auto value = h.f64();
            Json entry = {
                {"source_offset", at},
                {"source_header", rawbytes(header)},
                {"model_id", id},
                {"source_kind", kind},
                {"kind", (flags & 0x20) ? 2u : unsigned(kind)},
                {"source_flags", flags},
                {"excluded_from_name_lookup", bool(flags & 0x8000)},
                {"unassigned_flags", unsigned(flags & ~0x803du)},
                {"unassigned_flag_values",
                 {{"bit_0", bool(flags & 1)},
                  {"bit_2", bool(flags & 4)},
                  {"bit_3", bool(flags & 8)},
                  {"bit_4", bool(flags & 16)}}},
                {"unassigned_float64", std::isfinite(value) ? Json(value) : Json()},
                {"unassigned_u16", h.at<std::uint16_t>(26)},
                {"unassigned_header_ranges",
                 Json::array({{{"offset", 16}, {"data", rawbytes(slice(header, 16, 2))}},
                              {{"offset", 22}, {"data", rawbytes(slice(header, 22, 2))}},
                              {{"offset", 28}, {"data", rawbytes(slice(header, 28, 4))}}})}};
            const auto name_bytes = h.at<std::uint16_t>(18);
            entry["name"] = directory_text(reader, name_bytes);
            if (!name_bytes) {
                auto &name = entry["name"];
                if (default_id && id == *default_id) {
                    name["text"] = "Default";
                    name["utf16_code_units"] =
                        std::vector<std::uint16_t>{'D', 'e', 'f', 'a', 'u', 'l', 't'};
                    name["source"] = "default_model_name_fallback";
                } else {
                    name["status"] = "not_evaluated";
                    name["reason"] = "default_id_or_file_name_fallback_context_required";
                    name.erase("utf16_code_units");
                    name.erase("text");
                }
            }
            entry["secondary_string"] = directory_text(reader, h.at<std::uint16_t>(20));
            entry["secondary_string"]["role"] = "unresolved";
            entry["bitmap"] = directory_mask(reader.take(h.at<std::uint16_t>(24)));
            if (entry["bitmap"]["status"] == "not_evaluated")
                native_known = false;
            entry["extensions"] = Json::array();
            if (version == 5) {
                const auto extensions = reader.u32();
                require(extensions <= reader.left() / 12, "truncated_directory_extensions");
                std::set<std::uint32_t> keys;
                for (std::uint32_t j = 0; j < extensions; ++j) {
                    const auto offset = reader.p;
                    const auto key = reader.u32();
                    const auto size = reader.u64();
                    require(size <= reader.left(), "truncated_directory_extension_payload");
                    const auto unique = keys.insert(key).second;
                    entry["extensions"].push_back(
                        {{"source_offset", offset},
                         {"key", key},
                         {"size", size},
                         {"duplicate_key", !unique},
                         {"data", rawbytes(reader.take(std::size_t(size)))}});
                }
                // The installed version's nonempty map input has an indirect
                // buffer operation not yet confirmed safe. Preserve the wire
                // records without pretending its runtime map is reconstructed.
                entry["extension_map_status"] = extensions ? "not_evaluated" : "empty";
                if (extensions)
                    native_known = false;
            }
            entry["end_offset"] = reader.p;
            out["entries"].push_back(std::move(entry));
        }
        out["status"] = "decoded";
        out["native_input_status"] = native_known ? "decoded" : "not_evaluated";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
        out["header_rebuild_required"] = true;
    }
    out["consumed_bytes"] = reader.p;
    out["trailing_storage"] = rawbytes(reader.take(reader.left()));
    return out;
}

Json Document::native_model_directory() const {
    Json out = {{"status", "not_evaluated"}};
    if (!index().contains("P3D~MMIx")) {
        out["reason"] = "model_directory_alias_missing";
        return out;
    }
    const StreamPath path{index().at("P3D~MMIx").get<std::string>()};
    const auto found = std::find_if(streams().begin(), streams().end(),
                                    [&](const Stream &s) { return s.path == path; });
    if (found == streams().end()) {
        out["reason"] = "model_directory_stream_missing_header_rebuild_required";
    } else {
        std::optional<std::uint32_t> id;
        const auto &header = file_header();
        if (header.value("status", Json()) == "resolved")
            id = header.at("initial_probe").at("default_model_id").get<std::uint32_t>();
        out = parse_native_model_directory(*found->decoded, id);
    }
    out["stream"] = path;
    return out;
}

Json lookup_native_model_directory(const Json &directory, const std::u16string &name,
                                   const NativeModelNameEqual &equal) {
    Json out = {{"scope", "ordered_lookup_in_supplied_model_directory"},
                {"status", "not_evaluated"}};
    try {
        require(directory.at("status") == "decoded" &&
                    directory.at("native_input_status") == "decoded",
                "complete_native_model_directory_required");
        const auto query = name.substr(0, name.find(u'\0'));
        const auto &entries = directory.at("entries");
        for (std::size_t i = 0; i < entries.size(); ++i) {
            const auto &entry = entries[i];
            if (entry.at("excluded_from_name_lookup") == true)
                continue;
            out["candidate_index"] = i;
            const auto &candidate = entry.at("name");
            require(candidate.at("status") == "decoded", "candidate_model_name_unresolved");
            const auto units = candidate.at("utf16_code_units").get<std::vector<std::uint16_t>>();
            const std::u16string text(units.begin(), units.end());
            bool matches = text == query;
            if (!matches) {
                require(bool(equal), "native_wide_comparison_context_required");
                matches = equal(text, query);
            }
            if (matches) {
                out["status"] = "matched";
                out["entry_index"] = i;
                out["model_id"] = entry.at("model_id");
                return out;
            }
        }
        out["status"] = "missing";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
