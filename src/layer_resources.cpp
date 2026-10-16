#include "layer_name_key.hpp"
#include "layer_resource_source.hpp"
#include <p3d/layer_resources.hpp>

namespace p3d {
namespace {
Json units(const std::u16string &text) {
    return std::vector<std::uint16_t>(text.begin(), text.end());
}
std::u16string wide(const Json &value) {
    const auto words = value.get<std::vector<std::uint16_t>>();
    return {words.begin(), words.end()};
}
std::pair<std::u16string, std::size_t> quoted_end(const std::u16string &text, std::size_t start,
                                                  std::size_t capacity) {
    require(start < text.size(), "layer_resource_name_read_outside_text");
    const auto end = text.find(u'"', start);
    require(end != std::u16string::npos, "layer_resource_closing_quote_missing");
    // Native copy includes the closing quote before replacing it by NUL.
    require(end - start < capacity, "layer_resource_name_exceeds_native_buffer");
    return {text.substr(start, end - start), end};
}
bool space(char16_t c) {
    return c == u' ' || c == u'\t' || c == u'\r' || c == u'\n' || c == u'\v' || c == u'\f';
}
std::int32_t decimal(const std::u16string &text, std::size_t &position) {
    while (position < text.size() && space(text[position]))
        ++position;
    bool negative = false;
    if (position < text.size() && (text[position] == u'-' || text[position] == u'+'))
        negative = text[position++] == u'-';
    const auto first = position;
    std::uint64_t magnitude = 0;
    const std::uint64_t limit = negative ? 0x80000000ull : 0x7fffffffull;
    while (position < text.size() && text[position] >= u'0' && text[position] <= u'9') {
        const auto digit = unsigned(text[position++] - u'0');
        require(magnitude <= (limit - digit) / 10, "layer_color_decimal_overflow");
        magnitude = magnitude * 10 + digit;
    }
    require(position > first, "layer_color_decimal_conversion_unknown");
    return std::int32_t(negative ? -std::int64_t(magnitude) : std::int64_t(magnitude));
}
const char *resource_role(unsigned key) {
    static const char *roles[] = {"primary_color",       "by_layer_color",   "primary_line_style",
                                  "by_layer_line_style", "primary_material", "by_layer_material"};
    return roles[key - 0x43];
}
} // namespace
Json decode_native_layer_material_name(const std::u16string &input) {
    const auto text = detail::layer_name_prefix(input);
    Json out = {{"status", "unresolved"}, {"name_utf16", nullptr}};
    try {
        if (text.empty()) {
            out.update({{"status", "decoded"},
                        {"name_utf16", Json::array()},
                        {"selection", "empty_input"}});
            return out;
        }
        const auto first = text.find(u'"', 1);
        require(first != std::u16string::npos, "layer_material_first_quote_missing");
        const auto [name, end] = quoted_end(text, first + 3, 512);
        out.update({{"status", "decoded"},
                    {"name_utf16", units(name)},
                    {"name_start_utf16", first + 3},
                    {"ignored_prefix_utf16", units(text.substr(0, first + 1))},
                    {"skipped_separator_utf16", units(text.substr(first + 1, 2))},
                    {"ignored_suffix_utf16", units(text.substr(end + 1))},
                    {"selection", "native_quote_scan"}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json decode_native_layer_color_name(const std::u16string &input) {
    const auto text = detail::layer_name_prefix(input);
    Json out = {{"status", "unresolved"},
                {"rgb_bytes", nullptr},
                {"first_name_utf16", nullptr},
                {"second_name_utf16", nullptr}};
    try {
        if (text.empty()) {
            out.update({{"status", "decoded"}, {"selection", "empty_input"}});
            return out;
        }
        std::size_t position = 0;
        std::array<std::int32_t, 3> values;
        std::array<unsigned, 3> rgb;
        for (std::size_t i = 0; i < values.size(); ++i) {
            values[i] = decimal(text, position);
            rgb[i] = std::uint32_t(values[i]) & 255u;
        }
        require(text.size() >= 11, "layer_color_fixed_name_offset_outside_text");
        std::u16string first, second;
        std::size_t after = 11;
        if (text.size() != 11) {
            const auto a = quoted_end(text, 13, 1024);
            const auto b = quoted_end(text, a.second + 3, 1024);
            first = a.first;
            second = b.first;
            after = b.second + 1;
        }
        out.update({{"status", "decoded"},
                    {"selection", "native_decimal_and_fixed_name_scan"},
                    {"decimal_values", values},
                    {"rgb_bytes", rgb},
                    {"decimal_consumed_utf16", position},
                    {"first_name_utf16", units(first)},
                    {"second_name_utf16", units(second)},
                    {"ignored_suffix_utf16", units(text.substr(after))}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json lookup_native_line_style_name(
    const std::u16string &input,
    const std::function<std::optional<std::int32_t>(const std::u16string &)> &lookup) {
    const auto name = detail::layer_name_prefix(input);
    Json out = {{"status", "unresolved"},
                {"line_style", nullptr},
                {"source_profile", "prepared_owner_line_style_name_without_import"}};
    try {
        std::optional<std::int32_t> value;
        if (name == u"STYLE_ByLayer")
            value = 0x7fffffff;
        else if (name == u"STYLE_ByBlock")
            value = 0x7ffffffe;
        else if (name == u"STYLE_Invalid")
            value = 0x7fffff00;
        else if (name.size() == 1 && name[0] >= u'0' && name[0] <= u'7')
            value = name[0] - u'0';
        if (value) {
            out["selection"] = "builtin_exact_name";
        } else if (name.empty()) {
            value = 0x7fffff00;
            out["selection"] = "empty_name";
        } else {
            require(bool(lookup), "prepared_owner_line_style_lookup_required");
            value = lookup(name);
            require(value.has_value(), "prepared_owner_line_style_lookup_unknown");
            if (*value == 0)
                value = 0x7fffff00;
            out["selection"] = "prepared_owner_lookup";
        }
        out.update({{"status", "resolved"},
                    {"line_style", *value},
                    {"assign_to_layer", *value != 0x7fffff00}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json native_layer_resource_name_inputs(const Json &strings) {
    Json out = {{"status", "decoded"},
                {"entries", Json::array()},
                {"requests", Json::array()},
                {"resolution_order", "ascending_u16_key"}};
    std::map<unsigned, std::size_t> selected;
    bool bounded = true;
    for (std::size_t i = 0; i < strings.size(); ++i) {
        const auto &text = strings[i];
        if (text.contains("decode_error"))
            bounded = false;
        if (!text.contains("key"))
            continue;
        const auto key = text.at("key").get<unsigned>();
        if (key < 0x43 || key > 0x48)
            continue;
        Json entry = {{"key", key},
                      {"role", resource_role(key)},
                      {"source_string_index", i},
                      {"source_offset", text.at("source_offset")},
                      {"selected", true}};
        if (const auto previous = selected.find(key); previous != selected.end())
            out["entries"][previous->second]["selected"] = false;
        selected[key] = out["entries"].size();
        if (text.contains("decode_error"))
            entry.update({{"status", "unresolved"}, {"reason", text.at("decode_error")}});
        else {
            entry.update({{"status", "decoded"}, {"text", text.at("text")}});
            const auto name = detail::layer_name_wide(text.at("text").get<std::string>());
            if (key <= 0x44)
                entry["color_input"] = decode_native_layer_color_name(name);
            else if (key >= 0x47)
                entry["material_name_input"] = decode_native_layer_material_name(name);
        }
        out["entries"].push_back(std::move(entry));
    }
    if (!bounded) {
        out.update({{"status", "unresolved"},
                    {"requests", nullptr},
                    {"reason", "layer_string_loading_outside_supported_bounds"}});
    } else
        for (const auto &[key, index] : selected)
            out["requests"].push_back(out["entries"][index]);
    return out;
}
Json resolve_native_layer_material_names(const Json &records, std::size_t record_index,
                                         const Json &catalog,
                                         const NativeMaterialReferenceNameContext &context) {
    Json out = {{"status", "unresolved"},
                {"native_record_index", record_index},
                {"source_profile", "prepared_owner_layer_material_name_assignments"},
                {"primary", nullptr},
                {"by_layer", nullptr},
                {"material_loading", "not_performed"}};
    try {
        const auto &record = records.at(record_index);
        const auto bytes = bytesof(record.at("data"));
        Reader r(bytes);
        require(r.at<std::uint16_t>(4) == 49 && r.at<std::uint32_t>(16) == 1,
                "native_layer_record_required");
        const auto layer = decode_native_layer(bytes, record.at("links"));
        require(layer.value("status", "") == "decoded" && !layer.contains("decode_error"),
                "saved_layer_decode_unknown");
        const auto id = layer.at("layer_id").get<std::uint32_t>();
        require(id != 0 && id != UINT32_MAX, "fresh_layer_id_requires_repair_or_is_rejected");
        const auto &inputs = layer.at("resource_name_inputs");
        require(inputs.at("status") == "decoded", "layer_resource_name_inputs_unresolved");
        bool complete = true;
        for (unsigned channel = 0; channel < 2; ++channel) {
            Json result = {{"status", "unresolved"}, {"material_id", nullptr}};
            try {
                const auto &saved =
                    layer.at(channel ? "material_reference" : "primary_material_reference");
                require(saved.at("status") == "decoded", "saved_layer_material_id_unresolved");
                const auto saved_id = saved.at("material_id").get<std::uint64_t>();
                result.update({{"status", "resolved"},
                               {"material_id", saved_id},
                               {"saved_material_id", saved_id},
                               {"saved_source_link_index", saved.at("source_link_index")},
                               {"selection", "saved_reference"}});
                for (const auto &request : inputs.at("requests")) {
                    if (request.at("key") != 0x47 + channel)
                        continue;
                    result["source_string_index"] = request.at("source_string_index");
                    const auto &name = request.at("material_name_input");
                    require(name.at("status") == "decoded", "layer_material_name_input_unresolved");
                    const auto query = wide(name.at("name_utf16"));
                    auto lookup = lookup_native_material_reference_name(query, catalog, context);
                    result["name_lookup"] = lookup;
                    require(lookup.at("status") != "unresolved",
                            "layer_material_name_lookup_unresolved");
                    if (lookup.at("status") == "matched" &&
                        lookup.at("material_id") != UINT64_MAX) {
                        result["material_id"] = lookup.at("material_id");
                        result["selection"] = "named_catalog_reference";
                    }
                }
            } catch (const std::exception &e) {
                result.update({{"status", "unresolved"},
                               {"material_id", nullptr},
                               {"selection", "unresolved_name_assignment"},
                               {"reason", e.what()}});
                complete = false;
            }
            out[channel ? "by_layer" : "primary"] = std::move(result);
        }
        out["status"] = complete ? "evaluated" : "partially_evaluated";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
