#include "internal.hpp"
#include "tree_resources.hpp"
namespace p3d {
bool native_tree_resource_text(const Json &value, std::string &text) {
    if (value.is_string()) {
        text = value.get<std::string>();
        return true;
    }
    if (value.is_object() && value.size() == 1 && value.contains("null_string_index") &&
        value["null_string_index"].is_number_integer() &&
        (value["null_string_index"].is_number_unsigned() ||
         value["null_string_index"].get<std::int64_t>() >= 0)) {
        const auto index = value["null_string_index"].get<std::uint64_t>();
        if (index == UINT8_MAX || index == UINT16_MAX || index == UINT32_MAX ||
            index == UINT64_MAX) {
            text.clear();
            return true;
        }
    }
    return false;
}
namespace {
std::vector<std::uint16_t> wide_prefix(const std::string &source) {
    const auto text = source.substr(0, source.find('\0'));
    (void)Json(text).dump(); // Validate before decoding UTF-8.
    std::vector<std::uint16_t> units;
    for (std::size_t i = 0; i < text.size();) {
        const auto first = static_cast<unsigned char>(text[i++]);
        unsigned c = first;
        if (first >= 0x80) {
            const unsigned count = first < 0xe0 ? 1 : first < 0xf0 ? 2 : 3;
            c = first & (0x7f >> (count + 1));
            for (unsigned j = 0; j < count; ++j)
                c = (c << 6) | (static_cast<unsigned char>(text[i++]) & 0x3f);
        }
        if (c < 0x10000)
            units.push_back(static_cast<std::uint16_t>(c));
        else {
            units.push_back(static_cast<std::uint16_t>(0xd800 + ((c - 0x10000) >> 10)));
            units.push_back(static_cast<std::uint16_t>(0xdc00 + ((c - 0x10000) & 0x3ff)));
        }
    }
    return units;
}
int sextet(unsigned c) {
    return c >= 'A' && c <= 'Z'   ? int(c - 'A')
           : c >= 'a' && c <= 'z' ? int(c - 'a' + 26)
           : c >= '0' && c <= '9' ? int(c - '0' + 52)
           : c == '+'             ? 62
           : c == '/'             ? 63
                                  : -1;
}
} // namespace
Json native_tree_property_bytes(const Json &resources) {
    Json out = {{"query_profile", "native_tree_property_byte_getter"},
                {"status", "not_evaluated"},
                {"source_slot", 1},
                {"payload_semantics", "application_defined"}};
    std::string text;
    if (!resources.is_array() ||
        (resources.size() > 1 && !native_tree_resource_text(resources[1], text))) {
        out["reason"] = "resource_slot_text_required";
        return out;
    }
    std::vector<std::uint16_t> input;
    try {
        input = wide_prefix(text);
    } catch (const std::exception &e) {
        out["reason"] = e.what();
        return out;
    }
    out["input_utf16_code_unit_count"] = input.size();
    out["native_byte_count"] = 0;
    if (input.empty()) {
        out["status"] = "empty";
        return out;
    }
    out["native_decoder_return_value"] = false;
    out["status"] = "rejected";
    if (input.size() % 4) {
        out["reason"] = "encoded_length_not_multiple_of_four";
        return out;
    }
    Bytes bytes;
    for (std::size_t i = 0; i < input.size(); i += 4) {
        unsigned value = 0, count = 3;
        for (unsigned j = 0; j < 4; ++j) {
            const auto digit = sextet(input[i + j]);
            if (digit >= 0)
                value |= unsigned(digit) << (6 * j);
            else {
                if (input[i + j] != '=' || input.size() - i - j > 2) {
                    out["reason"] = "invalid_encoded_character_or_padding";
                    return out;
                }
                count = j - 1;
                break; // Native input ignores the characters after first padding.
            }
        }
        for (unsigned j = 0; j < count; ++j)
            bytes.push_back(static_cast<std::uint8_t>(value >> (8 * j)));
    }
    out.update({{"status", "decoded"},
                {"native_decoder_return_value", true},
                {"native_byte_count", bytes.size()},
                {"payload", rawbytes(bytes)},
                {"encoding", "native_low_sextet_first_base64"}});
    return out;
}
namespace {
Json boolean_resource(const Json &resources, const std::string &prefix, const char *profile) {
    Json out = {{"query_profile", profile},
                {"scope", "node_resource_query_before_view_composition"},
                {"numeric_reader", "windows_ucrt_wcstol_one_utf16_code_unit"},
                {"numeric_locale", "C"},
                {"status", "not_evaluated"},
                {"matches", Json::array()}};
    if (!resources.is_array()) {
        out["reason"] = "resource_array_required";
        return out;
    }
    const std::vector<std::uint16_t> marker(prefix.begin(), prefix.end());
    bool visible = true;
    for (std::size_t slot = 0; slot < resources.size(); ++slot) {
        std::string text;
        if (!native_tree_resource_text(resources[slot], text)) {
            out["reason"] = "resource_slot_text_required";
            out["source_slot"] = slot;
            return out;
        }
        std::vector<std::uint16_t> units;
        try {
            units = wide_prefix(text);
        } catch (const std::exception &e) {
            out["reason"] = e.what();
            out["source_slot"] = slot;
            return out;
        }
        const auto at = std::search(units.begin(), units.end(), marker.begin(), marker.end());
        if (at == units.end())
            continue;
        const auto offset = std::size_t(at - units.begin()), next = offset + marker.size();
        const unsigned code = next < units.size() ? units[next] : 0;
        const auto number = native_tree_decimal_digit(code);
        Json match = {
            {"source_slot", slot}, {"marker_utf16_offset", offset}, {"value_code_unit", code}};
        if (number < 0) {
            match["status"] = "invalid_argument";
            out["matches"].push_back(match);
            out["status"] = "rejected";
            out["native_exception"] = "std::invalid_argument";
            return out;
        }
        visible = number != 0;
        match.update({{"status", "decoded"}, {"value", visible}});
        out["matches"].push_back(match);
    }
    out.update(
        {{"status", "decoded"},
         {"value", visible},
         {"basis", out["matches"].empty() ? "native_default_true" : "last_matching_resource"}});
    return out;
}
} // namespace
Json native_tree_visibility(const Json &resources) {
    return boolean_resource(resources, "TreeNode@Visible:", "native_public_get_node_visibility");
}
Json native_tree_fold_input(const Json &resources) {
    return boolean_resource(resources,
                            "TreeNode@Fold:", "native_ui_fold_resource_input_if_stage_reached");
}
Json native_tree_ui_parameters(const Json &resources) {
    Json out = {{"query_profile", "native_ui_icon_and_visibility_resource_input"},
                {"scope", "resource_parameters_before_ui_creation_and_view_composition"},
                {"numeric_reader", "windows_ucrt_wcstol_int32_suffix"},
                {"numeric_locale", "C"},
                {"status", "not_evaluated"},
                {"matches", Json::array()}};
    if (!resources.is_array()) {
        out["reason"] = "resource_array_required";
        return out;
    }
    std::int64_t image = -1;
    bool visible = true;
    for (std::size_t slot = 0; slot < resources.size(); ++slot) {
        std::string text;
        if (!native_tree_resource_text(resources[slot], text)) {
            out["reason"] = "resource_slot_text_required";
            out["source_slot"] = slot;
            return out;
        }
        text = text.substr(0, text.find('\0'));
        try {
            (void)Json(text).dump();
        } catch (const std::exception &e) {
            out["reason"] = e.what();
            out["source_slot"] = slot;
            return out;
        }
        // The native helper parses the image first, then visibility, for
        // each resource, regardless of the marker positions in that text.
        for (const auto *prefix : {"TreeNode@ImgIdx:", "TreeNode@Visible:"}) {
            const auto at = text.find(prefix);
            if (at == std::string::npos)
                continue;
            const auto suffix = at + std::char_traits<char>::length(prefix);
            const auto number = native_tree_integer32(text.substr(suffix));
            Json match = {{"source_slot", slot},
                          {"marker", prefix},
                          {"marker_byte_offset", at},
                          {"suffix_byte_offset", suffix}};
            if (!number.digits || number.saturated) {
                match["status"] = number.saturated ? "out_of_range" : "invalid_argument";
                out["matches"].push_back(match);
                out["status"] = "rejected";
                out["native_exception"] =
                    number.saturated ? "std::out_of_range" : "std::invalid_argument";
                return out;
            }
            const bool is_image = prefix[9] == 'I';
            if (is_image)
                image = number.value;
            else
                visible = number.value != 0;
            match.update({{"status", "decoded"}, {"parsed_integer", number.value}});
            out["matches"].push_back(match);
        }
    }
    out.update({{"status", "decoded"},
                {"image_index", image},
                {"visibility", visible},
                {"basis", out["matches"].empty() ? "native_defaults" : "stored_resource_order"}});
    return out;
}
} // namespace p3d
