#include "internal.hpp"

namespace p3d {
Json decode_native_material_name(const Bytes &payload,
                                 const std::function<std::string(const Bytes &)> &ansi_decoder) {
    Json out = {{"encoding", "native_element_material_name_reference"},
                {"status", "invalid"},
                {"name_encoding", "windows_ansi"},
                {"name_offset", 6},
                {"material_name", nullptr},
                {"source_bytes", rawbytes(payload)},
                {"lookup_status", "not_performed"}};
    try {
        require(payload.size() >= 2, "material name length field is truncated");
        const auto declared = Reader(payload).u16();
        out["declared_byte_length"] = declared;
        // Native getters pass linkage+6 to a NUL-terminated ANSI constructor.
        // They do not use the writer's stored byte count as the read boundary.
        const auto end = std::find(payload.begin() + 2, payload.end(), std::uint8_t(0));
        require(end != payload.end(), "material name terminator is outside the linkage");
        const Bytes name(payload.begin() + 2, end);
        out["name_bytes"] = rawbytes(name);
        out["actual_byte_length"] = name.size();
        out["declared_length_matches"] = declared == name.size();
        out["terminator_offset"] = name.size() + 6;
        out["ignored_suffix"] = rawbytes(Bytes(end + 1, payload.end()));
        std::string text;
        if (ansi_decoder) {
            text = ansi_decoder(name);
            out["text_decoder"] = "caller_supplied";
        } else if (std::all_of(name.begin(), name.end(), [](auto c) { return c < 128; })) {
            text.assign(name.begin(), name.end());
            out["text_decoder"] = "ascii_subset";
        } else {
            out["status"] = "requires_ansi_decoder";
            return out;
        }
        // Validate the callback's UTF-8 before publishing a usable name.
        Json(text).dump();
        out["material_name"] = text;
        out["status"] = "decoded";
        out["lookup_request"] = {{"name", text},
                                 {"comparison", "native_case_insensitive_locale_dependent"},
                                 {"empty_name_matches", false},
                                 {"candidate_order", "native_material_catalog_order"},
                                 {"result_selection", "first_successfully_loaded_candidate"}};
    } catch (const std::exception &e) {
        out["decode_error"] = e.what();
    }
    return out;
}
} // namespace p3d
