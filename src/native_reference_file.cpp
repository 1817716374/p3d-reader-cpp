#include "internal.hpp"
#include <codecvt>
#include <locale>

namespace p3d {
NativePathParts native_path_parts(std::u16string path) {
    NativePathParts out;
    path.resize(path.find(u'\0') == std::u16string::npos ? path.size() : path.find(u'\0'));
    auto colon = path.find(u':'), slash = path.find_first_of(u"/\\");
    if (colon != std::u16string::npos && colon > 1 &&
        path.find(u':', colon + 1) == std::u16string::npos &&
        (slash == std::u16string::npos || colon < slash)) {
        out.logical_prefix = path.substr(0, colon);
        path.erase(0, colon + 1);
    }
    if (path.size() > 1 && path[1] == u':')
        path.erase(0, 2);
    slash = path.find_last_of(u"/\\");
    const auto basename = slash == std::u16string::npos ? path : path.substr(slash + 1);
    const auto dot = basename.find_last_of(u'.');
    out.stem = basename.substr(0, dot);
    out.extension = dot == std::u16string::npos ? std::u16string() : basename.substr(dot);
    out.buffer_limit = (slash != std::u16string::npos && slash + 1 >= 260) ||
                       out.stem.size() >= 260 || out.extension.size() >= 260;
    if (out.buffer_limit) {
        out.stem.clear();
        out.extension.clear();
    }
    return out;
}

namespace {
std::u16string c_string(const std::u16string &value) {
    return value.substr(0, value.find(u'\0'));
}
std::u16string from_utf8(const Json &value) {
    const auto text = value.get<std::string>();
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> codec;
    auto result = codec.from_bytes(text);
    require(codec.converted() == text.size(), "invalid_file_reference_utf8");
    return c_string(result);
}
Json wide_value(const std::u16string &value) {
    Json out = {{"utf16_code_units", Json::array()}};
    Bytes bytes;
    for (const auto unit : value) {
        out["utf16_code_units"].push_back(std::uint16_t(unit));
        bytes.push_back(std::uint8_t(unit));
        bytes.push_back(std::uint8_t(unit >> 8));
    }
    try {
        auto text = utf16(bytes);
        (void)Json(text).dump();
        out["text"] = std::move(text);
    } catch (const std::exception &) {
        out["text_status"] = "invalid_utf16";
    }
    return out;
}
std::u16string slot_value(const Json &target, unsigned key) {
    for (const auto &slot : target.at("strings")) {
        if (slot.at("key") != key)
            continue;
        require(slot.at("status") != "not_evaluated", "reference_search_string_unresolved");
        std::u16string value;
        for (const auto &unit : slot.at("utf16_code_units")) {
            const auto v = unit.get<unsigned>();
            require(v <= 0xffff, "invalid_reference_utf16_code_unit");
            if (!v)
                break;
            value.push_back(char16_t(v));
        }
        return value;
    }
    throw std::runtime_error("reference_search_string_slot_required");
}
} // namespace

Json initial_reference_file_query(const Json &record, const ReferenceFileQueryContext &context) {
    Json out = {{"scope", "initial_default_service_file_query_before_external_search"},
                {"status", "not_evaluated"},
                {"target_resolution", "not_evaluated"},
                {"model_loading", "not_evaluated"}};
    try {
        require(record.at("element_type") == 13, "type_13_reference_record_required");
        const auto &target = record.at("reference_target");
        std::vector<Json> chain = {target};
        chain.insert(chain.end(), context.host_reference_targets.begin(),
                     context.host_reference_targets.end());
        out["query_gate"] = initial_reference_file_query_gate(chain, context.complete_host_chain);
        if (out["query_gate"]["status"] == "blocked") {
            out["status"] = "blocked";
            out["native_error_code"] = 0;
            return out;
        }
        require(out["query_gate"]["status"] == "allowed", "reference_query_gate_unresolved");
        const auto &input = record.at("reference_input");
        require(input.at("status") == "decoded", "decoded_reference_input_required");
        const auto flags = input.at("origin_inputs").at("primary_flags").get<std::uint32_t>();
        out["source_primary_flags"] = flags;

        const auto &spec = target.at("file_specification");
        std::u16string stored, lookup;
        if (spec.at("status") == "host_file_specification_required") {
            require(!(context.host_file && context.host_file_known_absent),
                    "contradictory_host_file_context");
            require(bool(context.host_file), context.host_file_known_absent
                                                 ? "host_file_unavailable_for_specification"
                                                 : "host_file_specification_required");
            const auto source = c_string(context.host_file->stored_reference);
            const auto parts = native_path_parts(source);
            stored = parts.stem + parts.extension;
            lookup = c_string(context.host_file->lookup_reference);
            out["reference_strings_source"] = "host_specification_clone";
            out["host_stored_reference"] = wide_value(source);
            out["host_path_buffer_limit"] = parts.buffer_limit;
        } else {
            require(spec.at("status") == "decoded", "persisted_file_specification_unresolved");
            const auto &service = spec.at("default_service_reference");
            require(service.at("status") == "decoded", "default_file_service_reference_unresolved");
            stored = from_utf8(service.at("stored_reference"));
            lookup = from_utf8(service.at("lookup_reference"));
            out["reference_strings_source"] = "persisted_default_service_specification";
        }
        out["stored_reference"] = wide_value(stored);
        out["lookup_reference"] = wide_value(lookup);
        // This view exposes the strings needed by this query, not a complete
        // replacement for the specification's other inherited service state.
        if (flags & 0x200) {
            if (!stored.empty()) {
                out["status"] = "rejected";
                out["native_error_code"] = 0x13011;
                out["reason"] = "host_file_branch_requires_empty_stored_reference";
                return out;
            }
            require(!(context.host_file && context.host_file_known_absent),
                    "contradictory_host_file_context");
            require(context.host_file || context.host_file_known_absent,
                    "host_file_availability_required");
            out["status"] = context.host_file ? "reuse_host_file" : "no_file";
            out["native_error_code"] = 0;
            return out;
        }

        // The initial loader clears runtime alternate-file text (+218). Key35
        // is a different member (+210), even when the alternate-name flag is on.
        out["lookup_source"] = "initial_specification_lookup_reference";
        auto search = slot_value(target, 48);
        if (search.empty()) {
            require(context.inherited_search_context.has_value(),
                    "prepared_host_search_context_required");
            search = c_string(*context.inherited_search_context);
            out["search_context_source"] = "explicit_host_preparation";
        } else {
            out["search_context_source"] = "persisted_key_48";
        }
        out["search_context"] = wide_value(search);
        if (!search.empty()) {
            out["status"] = "external_search_required";
            out["reason"] = "nonempty_search_context_bypasses_current_file_reuse";
            return out;
        }
        require(!(context.current_file && context.current_file_known_absent),
                "contradictory_current_file_context");
        require(context.current_file || context.current_file_known_absent,
                "current_file_availability_required");
        if (!context.current_file) {
            out["status"] = "external_search_required";
            out["reason"] = "current_file_unavailable";
            return out;
        }
        auto current = c_string(context.current_file->lookup_reference);
        if (current.empty())
            current = c_string(context.current_file->stored_reference);
        out["current_file_reference"] = wide_value(current);
        bool equal = current == lookup;
        if (!equal) {
            require(bool(context.equal), "native_wide_case_comparison_required");
            equal = context.equal(current, lookup);
        }
        out["current_file_matches"] = equal;
        out["status"] = equal ? "reuse_current_file" : "external_search_required";
        if (equal)
            out["native_error_code"] = 0;
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
