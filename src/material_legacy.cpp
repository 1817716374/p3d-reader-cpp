#include "internal.hpp"
#include <charconv>
namespace p3d {
namespace {
enum class Kind { Number, Integer, Boolean, Color, Bond };
struct LegacyField {
    unsigned type;
    const char *name;
    Kind kind;
    double scale;
};
#include "material_legacy_fields.inc"
std::string content(const Json &node) {
    auto text = node.value("text", Json());
    std::string out = text.is_string() ? text.get<std::string>() : std::string();
    for (const auto &child : node["children"]) {
        out += content(child);
        if (child.value("tail", Json()).is_string())
            out += child["tail"].get<std::string>();
    }
    return out;
}
bool whitespace(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\v' || c == '\f';
}
struct Numeric {
    Json value = nullptr;
    std::size_t end = 0;
    bool matched = false;
    const char *status = "decoded";
};
Numeric prefix_number(const std::string &s, std::size_t pos, bool integer, bool scan = false) {
    Numeric out;
    out.end = pos;
    while (pos < s.size() && whitespace(s[pos]))
        ++pos;
    if (pos < s.size() && static_cast<unsigned char>(s[pos]) >= 128) {
        out.status = "locale_dependent_numeric_prefix";
        return out;
    }
    bool negative = pos < s.size() && s[pos] == '-';
    if (pos < s.size() && (s[pos] == '+' || s[pos] == '-'))
        ++pos;
    auto start = pos;
    if (integer) {
        std::uint64_t v = 0, limit = std::uint64_t(INT32_MAX) + unsigned(negative);
        bool overflow = false;
        while (pos < s.size() && s[pos] >= '0' && s[pos] <= '9') {
            v = v * 10 + unsigned(s[pos++] - '0');
            if (v > limit) {
                v = limit;
                overflow = true;
            }
        }
        out.value = negative ? -std::int64_t(v) : std::int64_t(v);
        out.matched = pos != start;
        out.end = pos;
        out.status = overflow      ? "integer_clamped"
                     : out.matched ? "decoded"
                                   : "no_numeric_prefix_uses_zero";
        return out;
    }
    const bool hex =
        pos + 1 < s.size() && s[pos] == '0' && (s[pos + 1] == 'x' || s[pos + 1] == 'X');
    if (hex)
        pos += 2;
    if (pos < s.size() && (s[pos] == '+' || s[pos] == '-')) {
        out.value = scan ? Json() : Json(0.0);
        out.status = scan ? "scan_failed" : "no_numeric_prefix_uses_zero";
        return out;
    }
    double value = 0;
    const auto first = s.data() + pos, last = s.data() + s.size();
    auto result = std::from_chars(first, last, value,
                                  hex ? std::chars_format::hex : std::chars_format::general);
    if (result.ec == std::errc::invalid_argument) {
        out.value = scan ? Json() : Json(0.0);
        out.status = scan ? "scan_failed" : "no_numeric_prefix_uses_zero";
        return out;
    }
    out.end = std::size_t(result.ptr - s.data());
    // Unlike _wtof, the native scanf reader rejects an unfinished exponent.
    if (scan && result.ptr != last &&
        (hex ? (*result.ptr == 'p' || *result.ptr == 'P')
             : (*result.ptr == 'e' || *result.ptr == 'E'))) {
        out.status = "scan_failed";
        return out;
    }
    out.matched = true;
    if (result.ec == std::errc::result_out_of_range) {
        out.status = "out_of_range";
        return out;
    }
    value = negative ? -value : value;
    if (!std::isfinite(value)) {
        out.status = "non_finite";
        return out;
    }
    out.value = value;
    return out;
}
Json numeric_view(const Numeric &n) {
    return {{"value", n.value},
            {"conversion", n.status},
            {"numeric_prefix", n.matched},
            {"end_byte_offset", n.end}};
}
// The gradient reader advances by scanf's %n plus exactly one wchar after
// every scalar. This is not splitting on arbitrary whitespace or commas.
Json gradient_values(const std::string &body) {
    Json out = {{"status", "decoded"}, {"values", Json::array()}, {"tokens", Json::array()}};
    std::size_t pos = 0;
    while (pos < body.size()) {
        auto n = prefix_number(body, pos, false, true);
        auto token = numeric_view(n);
        token["start_byte_offset"] = pos;
        out["tokens"].push_back(std::move(token));
        if (!n.matched || n.value.is_null()) {
            out["status"] = n.status;
            break;
        }
        out["values"].push_back(n.value);
        pos = n.end;
        if (pos < body.size()) {
            // Unknown non-ASCII separators may occupy two UTF16 units; do not
            // silently replace the native single-unit advance with UTF8 bytes.
            if (static_cast<unsigned char>(body[pos]) >= 128) {
                out["status"] = "non_ascii_gradient_separator";
                break;
            }
            ++pos;
        }
    }
    return out;
}
Json read_field(const LegacyField &field, const std::string &body) {
    Json out = {{"value", nullptr}, {"reader_value", nullptr}, {"reader_applies", true}};
    if (field.kind == Kind::Bond) {
        static const char *names[] = {"Running", "Stack", "One-third", "Flemish",
                                      "Common",  "Dutch", "English"};
        out["source_type"] = "bond_name";
        out["value"] = body;
        for (unsigned i = 0; i < 7; ++i)
            if (body == names[i]) {
                out["reader_value"] = i;
                out["status"] = "decoded";
                return out;
            }
        out["status"] = "ignored_unknown_enum";
        out["reader_applies"] = false;
        return out;
    }
    if (field.kind == Kind::Color) {
        out["source_type"] = "rgb_float64";
        out["components"] = Json::array();
        out["value"] = Json::array();
        std::size_t pos = 0;
        bool scanning = true;
        for (unsigned c = 0; c < 3; ++c) {
            Numeric n;
            n.status = "not_scanned";
            if (scanning)
                n = prefix_number(body, pos, false, true);
            auto component = numeric_view(n);
            component["reader_applies"] = scanning && n.matched;
            out["components"].push_back(std::move(component));
            out["value"].push_back(n.value);
            scanning &= n.matched;
            pos = n.end;
        }
        out["reader_value"] = out["value"];
        out["status"] = scanning && std::none_of(out["value"].begin(), out["value"].end(),
                                                 [](const Json &v) { return v.is_null(); })
                            ? "decoded"
                            : "partial";
        return out;
    }
    auto n = prefix_number(body, 0, field.kind != Kind::Number);
    out.update(numeric_view(n));
    out["source_type"] = field.kind == Kind::Number ? "float64" : "int32";
    out["status"] = n.value.is_null() ? "unresolved" : "decoded";
    out["reader_value"] = n.value;
    if (field.kind == Kind::Boolean && !n.value.is_null())
        out["reader_value"] = n.value != 0;
    if (field.kind == Kind::Number) {
        out["reader_scale"] = field.scale;
        if (!n.value.is_null()) {
            const double scaled = n.value.get<double>() * field.scale;
            out["reader_value"] = std::isfinite(scaled) ? Json(scaled) : Json();
            if (!std::isfinite(scaled))
                out["status"] = "unresolved";
        }
    }
    return out;
}
} // namespace
Json material_legacy_parameters(const Json &owner, const Json &dispatch) {
    Json out = {{"status", "not_applicable"},
                {"entries", Json::array()},
                {"parameters", Json::object()},
                {"order", "direct_element_content_in_source_order"},
                {"value_policy", "explicit_content_writes_in_invariant_numeric_locale"},
                {"evaluation_status", "not_evaluated"}};
    if (dispatch.value("status", std::string()) == "unresolved") {
        out["status"] = "unresolved_dispatch";
        return out;
    }
    const auto branch = dispatch.value("branch", Json());
    if (branch != "pma_preset" && branch != "pma_user_data")
        return out;
    out["status"] = "decoded";
    unsigned type =
        branch == "pma_preset" ? dispatch["preset"]["procedure_type"].get<unsigned>() : 0;
    const unsigned schema_type = type == 15 ? 14 : type >= 27 && type <= 29 ? 25 : type;
    std::map<std::string, const LegacyField *> fields;
    for (const auto &field : legacy_fields)
        if (field.type == schema_type) {
            fields[field.name] = &field;
            out["parameters"][field.name] = {
                {"status", "missing"}, {"value", nullptr}, {"source_entry_indices", Json::array()}};
        }
    Json channels = nullptr;
    bool gradient_known = type == 31;
    if (type == 31) {
        channels = Json::array();
        for (unsigned c = 0; c < 5; ++c) {
            Json points = Json::array();
            for (unsigned p = 0; p < 2; ++p)
                points.push_back({{"x", p},
                                  {"y", p == 0 || c == 4 ? 1 : 0},
                                  {"position_source_entry_index", nullptr},
                                  {"value_source_entry_index", nullptr},
                                  {"value_source", "preset_constructor"}});
            channels.push_back({{"channel_index", c}, {"reader_selector", 1}, {"points", points}});
        }
    }
    for (std::size_t i = 0; i < owner["children"].size(); ++i) {
        const auto &node = owner["children"][i];
        const auto text = content(node);
        Json entry = {{"child_index", i}, {"source_tag", node["tag"]}, {"source_content", text}};
        const auto entry_index = out["entries"].size();
        if (!type)
            entry["status"] = "user_data_string";
        else if (type == 13 || type == 17 || type == 21)
            entry["status"] = "ignored_no_content_reader";
        else if (text.find('\0') != std::string::npos)
            entry["status"] = "invalid_embedded_nul";
        else if (const auto underscore = text.find('_'); underscore == std::string::npos)
            entry["status"] = "ignored_missing_prefix_separator";
        else {
            const auto tail = text.substr(underscore + 1);
            const auto split = tail.find(' ');
            const auto key = tail.substr(0, split);
            // Native npos + 1 wraps to zero: a missing space reuses the key as its value.
            const auto body = split == std::string::npos ? tail : tail.substr(split + 1);
            entry["prefix"] = text.substr(0, underscore);
            entry["key"] = key;
            entry["source_value"] = body;
            entry["has_value_separator"] = split != std::string::npos;
            auto found = fields.find(key);
            if (type == 31 && (key == "input" || key == "u_x_offsets" || key == "x_colors")) {
                if (key == "input") {
                    entry.update(read_field({31, "input", Kind::Integer, 1}, body));
                    out["parameters"][key] = {{"status", entry["status"]},
                                              {"value", entry["reader_value"]},
                                              {"source_entry_indices", {entry_index}}};
                } else {
                    auto array = gradient_values(body);
                    entry["array"] = array;
                    entry["status"] = array["status"];
                    out["parameters"][key] = {{"status", array["status"]},
                                              {"value", array["values"]},
                                              {"source_entry_indices", {entry_index}}};
                    if (key == "u_x_offsets") {
                        gradient_known = array["status"] == "decoded";
                        channels = Json::array();
                        for (unsigned c = 0; c < 5; ++c) {
                            Json points = Json::array();
                            for (const auto &x : array["values"])
                                points.push_back(
                                    {{"x", x},
                                     {"y", c == 4 ? 1 : 0},
                                     {"position_source_entry_index", entry_index},
                                     {"value_source_entry_index", nullptr},
                                     {"value_source", c == 4 ? "offset_reader_sets_one"
                                                             : "control_constructor_zero"}});
                            channels.push_back(
                                {{"channel_index", c}, {"reader_selector", 1}, {"points", points}});
                        }
                        entry["reader_effect"] = "replace_all_five_control_lists";
                    } else {
                        entry["reader_effect"] = "assign_existing_points_in_order";
                        if (array["status"] != "decoded" || array["values"].size() % 3) {
                            entry["status"] = "incomplete_color_triplets";
                            gradient_known = false;
                        } else if (!gradient_known)
                            entry["status"] = "requires_existing_control_lists";
                        else if (array["values"].size() / 3 > channels[0]["points"].size()) {
                            entry["status"] = "color_count_exceeds_control_count";
                            gradient_known = false;
                        } else
                            for (std::size_t p = 0; p < array["values"].size() / 3; ++p)
                                for (unsigned c = 0; c < 4; ++c) {
                                    auto &point = channels[c]["points"][p];
                                    point["y"] = array["values"][p * 3 + (c == 0 ? 0 : c - 1)];
                                    point["value_source_entry_index"] = entry_index;
                                    point["value_source"] = "color_record";
                                }
                    }
                }
            } else if (found == fields.end())
                entry["status"] = "ignored_unknown_key";
            else {
                const auto &field = *found->second;
                entry.update(read_field(field, body));
                auto &param = out["parameters"][key];
                if (field.kind == Kind::Color) {
                    if (param["value"].is_null()) {
                        param["value"] = Json::array({nullptr, nullptr, nullptr});
                        param["source_entry_indices"] = Json::array({nullptr, nullptr, nullptr});
                    }
                    for (unsigned c = 0; c < 3; ++c)
                        if (entry["components"][c]["reader_applies"] == true) {
                            param["value"][c] = entry["reader_value"][c];
                            param["source_entry_indices"][c] = entry_index;
                        }
                    param["status"] = std::any_of(param["value"].begin(), param["value"].end(),
                                                  [](const Json &v) { return v.is_null(); })
                                          ? "partial"
                                          : "decoded";
                } else if (entry["reader_applies"] == true) {
                    param = {{"status", entry["status"]},
                             {"value", entry["reader_value"]},
                             {"source_entry_indices", {entry_index}}};
                }
            }
        }
        const auto status = entry["status"].get<std::string>();
        if (status != "decoded" && status != "user_data_string" && status.rfind("ignored_", 0) != 0)
            out["status"] = "partial";
        out["entries"].push_back(std::move(entry));
    }
    if (type == 31)
        out["gradient_controls"] = {
            {"status", gradient_known ? "decoded" : "invalid_controls"},
            {"channels", channels},
            {"channel_names_status", "unassigned"},
            {"input_selector",
             out["parameters"].contains("input") ? out["parameters"]["input"]["value"] : Json(0)},
            {"input_selector_source",
             out["parameters"].contains("input") ? "source_record" : "preset_constructor"},
            {"value_policy", "native_constructor_then_source_ordered_writes"}};
    return out;
}
} // namespace p3d
