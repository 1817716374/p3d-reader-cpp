#include "internal.hpp"

namespace p3d {
namespace {
Json bounded_reference_string(const Bytes &bytes, std::size_t count, std::size_t capacity,
                              std::size_t post_limit) {
    Json out = {{"utf16_code_units", Json::array()}, {"native_conversion_status", 0}};
    std::vector<std::uint16_t> units;
    unsigned conversion_status = 0;
    if (count != 0 && bytes[0] != 0) {
        // The native temporary copies the complete linkage and appends a wide
        // zero. Marker lookahead can therefore include linkage padding.
        auto peek = [&](std::size_t i) { return i < bytes.size() ? bytes[i] : std::uint8_t(0); };
        bool wide = false;
        std::size_t prefix = 0;
        if (bytes[0] == 255 && peek(1) == 253) {
            wide = true;
            prefix = 2;
        } else if (bytes[0] == 255 && peek(1) == 254) {
            const bool narrow = peek(2) == 1 && peek(3) == 0;
            wide = !narrow;
            prefix = narrow ? 4 : 2;
        } else if (peek(1) == 255 && (bytes[0] == 253 || bytes[0] == 254)) {
            // The native decoder rejects swapped markers after clearing the
            // destination. Its caller ignores the return code and sees empty.
            conversion_status = 2;
            prefix = count;
            out["encoding"] = "unsupported_byte_order";
        }
        require(prefix <= count, "reference_string_marker_exceeds_declared_length");
        const auto body_size = count - prefix;
        const auto available = wide ? body_size / 2 : body_size;
        require(body_size == 0 || available != 0, "incomplete_reference_wide_character");
        const auto copied = std::min(available, capacity);
        for (std::size_t i = 0; i < copied; ++i)
            units.push_back(wide ? Reader(bytes, prefix + 2 * i).u16() : bytes[prefix + i]);
        if (copied) {
            if (units.back() != 0 && copied >= capacity - 1) {
                // Native replaces the last copied unit even at capacity-1;
                // this differs from ordinary max_length truncation.
                units.back() = 0;
                conversion_status = 1;
            } else if (!wide && available > capacity) {
                conversion_status = 1;
            }
        }
        if (!out.contains("encoding"))
            out["encoding"] = wide ? "utf16le" : "byte_widening";
        out["prefix_bytes"] = prefix;
        out["discarded_odd_tail_byte"] = wide && (body_size % 2 != 0);
    }
    const auto zero = std::find(units.begin(), units.end(), 0);
    units.erase(zero, units.end());
    if (units.size() > post_limit)
        units.resize(post_limit);
    out["utf16_code_units"] = units;
    out["native_conversion_status"] = conversion_status;
    Bytes utf16_bytes;
    for (auto unit : units) {
        utf16_bytes.push_back(static_cast<std::uint8_t>(unit));
        utf16_bytes.push_back(static_cast<std::uint8_t>(unit >> 8));
    }
    try {
        const auto text = utf16(utf16_bytes);
        (void)Json(text).dump(); // A lone low surrogate is not valid UTF-8 JSON.
        out["text"] = text;
        out["text_status"] = "decoded";
    } catch (const std::exception &) {
        out["text_status"] = "invalid_utf16";
    }
    return out;
}

Json string_slot(const Json &links, unsigned key, std::size_t capacity,
                 std::size_t post_limit = SIZE_MAX) {
    Json out = {{"key", key},
                {"status", "absent"},
                {"capacity_code_units", capacity},
                {"utf16_code_units", Json::array()},
                {"text", ""}};
    if (post_limit != SIZE_MAX)
        out["post_limit_code_units"] = post_limit;
    try {
        for (std::size_t i = 0; i < links.size(); ++i) {
            const auto &link = links[i];
            if (link.at("app") != 0x56d2 || !(link.at("header").get<unsigned>() & 0x1000))
                continue;
            const auto payload = bytesof(link.at("payload"));
            require(payload.size() >= 2, "incomplete_string_key_before_selection");
            if (Reader(payload).u16() != key)
                continue;
            out["selected_linkage_index"] = i;
            out["source_offset"] = link.value("offset", Json());
            require(payload.size() >= 8, "truncated_selected_reference_string");
            const auto count = Reader(payload, 4).u32();
            out["byte_count"] = count;
            out["reserved_word"] = Reader(payload, 2).u16();
            require(count <= payload.size() - 8, "reference_string_length_outside_linkage");
            out.erase("text");
            out.update(bounded_reference_string(slice(payload, 8, payload.size() - 8), count,
                                                capacity, post_limit));
            out["status"] = "decoded";
            return out; // First matching key, including malformed/truncated text.
        }
    } catch (const std::exception &e) {
        out["status"] = "not_evaluated";
        out["reason"] = e.what();
        out.erase("text");
        out.erase("utf16_code_units");
    }
    return out;
}

Json model_id_link(const Json &links) {
    Json out = {{"status", "absent"}, {"app", 0x5717}, {"skipped_versions", Json::array()}};
    try {
        for (std::size_t i = 0; i < links.size(); ++i) {
            const auto &link = links[i];
            if (link.at("app") != 0x5717 || !(link.at("header").get<unsigned>() & 0x1000))
                continue;
            const auto payload = bytesof(link.at("payload"));
            require(payload.size() >= 2, "incomplete_model_id_link_version");
            const auto version = Reader(payload).u16();
            if (version != 0) {
                out["skipped_versions"].push_back({{"linkage_index", i}, {"version", version}});
                continue;
            }
            out["selected_linkage_index"] = i;
            out["source_offset"] = link.value("offset", Json());
            require(payload.size() >= 8, "truncated_selected_model_id_link");
            const auto id = Reader(payload, 4).u32();
            out.update({{"status", "selected"},
                        {"model_id", id},
                        {"reserved_word", Reader(payload, 2).u16()},
                        {"trailing_storage", rawbytes(slice(payload, 8, payload.size() - 8))}});
            return out;
        }
    } catch (const std::exception &e) {
        out["status"] = "not_evaluated";
        out["reason"] = e.what();
    }
    return out;
}

Json file_specification(const Json &links) {
    Json out = {{"scope", "persisted_file_specification_before_host_fallback"},
                {"status", "not_evaluated"},
                {"file_location", "not_evaluated"},
                {"host_search_context", "not_evaluated"},
                {"strings", Json::array()}};
    for (const auto key : {3u, 31u, 64u}) {
        // This getter sizes its destination from the complete linkage area.
        // For a structurally bounded payload it cannot truncate the string.
        auto slot = string_slot(links, key, SIZE_MAX);
        slot.erase("capacity_code_units");
        slot["capacity_policy"] = "record_linkage_sized";
        slot["role"] = key == 3    ? "source_file_reference"
                       : key == 31 ? "lookup_reference_override"
                                   : "resource_service_parameter";
        if (slot.at("status") != "not_evaluated") {
            const bool success =
                slot.at("status") == "decoded" && slot.at("native_conversion_status") == 0;
            slot["getter_status"] = success ? "success" : "failure";
            if (!success) {
                slot["text"] = "";
                slot["utf16_code_units"] = Json::array();
            }
        }
        out["strings"].push_back(std::move(slot));
    }
    const auto &primary = out["strings"][0];
    if (primary.at("status") == "not_evaluated") {
        out["reason"] = "primary_file_reference_unresolved";
        return out;
    }
    if (primary.at("utf16_code_units").empty()) {
        out["status"] = "host_file_specification_required";
        out["reason"] = "primary_file_reference_absent_empty_or_rejected";
        out["resource_service_called"] = false;
        return out;
    }
    out["resource_service_called"] = true;
    for (const auto &slot : out.at("strings"))
        if (slot.at("status") == "not_evaluated" || !slot.contains("text")) {
            out["reason"] = "file_specification_string_unresolved";
            return out;
        }
    const auto &alternate = out["strings"][1];
    const auto &parameter = out["strings"][2];
    out["service_parameter"] = parameter.at("text");
    out["service_option"] = parameter.at("utf16_code_units").empty()
                                ? "runtime_service_state_required"
                                : "enabled_by_nonempty_parameter";
    out["default_service_reference"] =
        native_file_resource_reference(primary.at("text"), alternate.at("text"));
    out["status"] = out["default_service_reference"]["status"];
    return out;
}
} // namespace

Json native_reference_target(const Json &input, const Json &links) {
    Json out = {{"profile", "bimbase_2025_persisted_reference_target_input"},
                {"scope", "initial_model_selector_after_persisted_linkages"},
                {"status", "decoded"},
                {"target_resolution", "not_evaluated"},
                {"file_location", "not_evaluated"},
                {"strings", Json::array()}};
    for (const auto key : {4u, 2u, 21u, 36u, 35u, 42u, 46u, 48u}) {
        auto value = string_slot(links, key, key == 2 ? 256 : 512, key == 4 ? 95 : SIZE_MAX);
        value["role"] = key == 21   ? "model_name"
                        : key == 36 ? "alternate_model_name"
                                    : "unresolved";
        if (value.at("status") == "not_evaluated")
            out["status"] = "partial";
        out["strings"].push_back(std::move(value));
    }
    const auto id_link = model_id_link(links);
    out["model_id_link"] = id_link;
    if (id_link.at("status") == "not_evaluated")
        out["status"] = "partial";
    Json selection = {{"status", "not_evaluated"}, {"model_directory_lookup", "not_evaluated"}};
    try {
        require(input.at("status") == "decoded", "decoded_reference_input_required");
        const auto flags = input.at("origin_inputs").at("secondary_flags").get<std::uint32_t>();
        selection["source_secondary_flags"] = flags;
        selection["clear_requested_load_mask"] = bool(flags & 0x8000);
        if (flags & 0x8000) {
            selection.update({{"kind", "file_default_model"}, {"source", "secondary_flag_0x8000"}});
        } else if (id_link.at("status") == "selected") {
            selection.update({{"kind", "model_id"},
                              {"model_id", id_link.at("model_id")},
                              {"source", "model_id_link"}});
        } else {
            require(id_link.at("status") == "absent", "model_id_override_state_unknown");
            const auto &name = out["strings"][flags & 0x20000 ? 3 : 2];
            require(name.at("status") != "not_evaluated", "selected_model_name_unresolved");
            selection["name_key"] = name.at("key");
            if (name.at("utf16_code_units").empty()) {
                selection.update({{"kind", "file_default_model"}, {"source", "empty_model_name"}});
            } else {
                selection.update({{"kind", "model_name"},
                                  {"source", "selected_model_name"},
                                  {"utf16_code_units", name.at("utf16_code_units")},
                                  {"comparison", "native_case_insensitive_wide"}});
                if (name.contains("text"))
                    selection["name"] = name.at("text");
            }
        }
        selection["status"] = "resolved";
    } catch (const std::exception &e) {
        selection["reason"] = e.what();
        out["status"] = "partial";
    }
    out["model_selection"] = std::move(selection);
    out["file_specification"] = file_specification(links);
    if (out["file_specification"]["status"] == "not_evaluated")
        out["status"] = "partial";
    return out;
}
} // namespace p3d
