#include "internal.hpp"
#include <charconv>
#include <cmath>
#include <limits>
#include <string_view>

namespace p3d {
namespace {

std::optional<std::int64_t> integer_field(const Json &attributes, const char *name) {
    if (!attributes.is_object() || !attributes.contains(name) || !attributes[name].is_string())
        return std::nullopt;
    const auto value = attributes[name].get<std::string>();
    std::int64_t parsed = 0;
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (value.empty() || result.ec != std::errc{} || result.ptr != value.data() + value.size())
        return std::nullopt;
    return parsed;
}

std::optional<unsigned> fixed_decimal(std::string_view text, std::size_t offset,
                                      std::size_t width) {
    if (offset + width > text.size())
        return std::nullopt;
    unsigned value = 0;
    for (std::size_t i = offset; i < offset + width; ++i) {
        const char c = text[i];
        if (c < '0' || c > '9')
            return std::nullopt;
        value = value * 10 + static_cast<unsigned>(c - '0');
    }
    return value;
}

bool valid_calendar_date(unsigned year, unsigned month, unsigned day) {
    if (year == 0 || year > 9999 || month == 0 || month > 12 || day == 0)
        return false;
    constexpr unsigned month_days[] = {31, 28, 31, 30, 31, 30,
                                       31, 31, 30, 31, 30, 31};
    unsigned max_day = month_days[month - 1];
    if (month == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)))
        max_day = 29;
    return day <= max_day;
}

Json datetime_value(std::string_view text) {
    Json out = { {"status", "unrecognized"}, {"type", "datetime"} };
    const bool date_only = text.size() == 10 && text[4] == '-' && text[7] == '-';
    const bool date_time = (text.size() == 21 || text.size() == 22) &&
                           text[8] == ' ' && text[11] == ':' && text[14] == ':' &&
                           text[17] == '.' && (text.size() == 21 || text[21] == 'Z');
    if (!date_only && !date_time)
        return out;

    const auto year = fixed_decimal(text, 0, 4);
    const auto month = fixed_decimal(text, date_only ? 5 : 4, 2);
    const auto day = fixed_decimal(text, date_only ? 8 : 6, 2);
    if (!year || !month || !day || !valid_calendar_date(*year, *month, *day))
        return out;

    out["status"] = "decoded";
    out["component"] = date_only ? "date" : "date_and_time";
    out["year"] = *year;
    out["month"] = *month;
    out["day"] = *day;
    out["source_format"] = date_only ? "YYYY-MM-DD" : "YYYYMMDD HH:MM:SS.sss[Z]";
    if (date_time) {
        const auto hour = fixed_decimal(text, 9, 2);
        const auto minute = fixed_decimal(text, 12, 2);
        const auto second = fixed_decimal(text, 15, 2);
        const auto millisecond = fixed_decimal(text, 18, 3);
        if (!hour || !minute || !second || !millisecond || *hour > 23 ||
            *minute > 59 || *second > 59)
            return { {"status", "unrecognized"}, {"type", "datetime"} };
        out["hour"] = *hour;
        out["minute"] = *minute;
        out["second"] = *second;
        out["millisecond"] = *millisecond;
        out["timezone"] = text.size() == 22 ? "utc" : "unspecified_or_local";
    }
    return out;
}

Json double_field(const Json &attributes, const char *name);

Json point_value(std::string_view text, std::size_t dimensions) {
    Json out = { {"status", "unrecognized"},
                 {"type", dimensions == 2 ? "point2d" : "point3d"} };
    Json coordinates = Json::array();
    std::size_t begin = 0;
    while (begin <= text.size()) {
        const auto end = text.find(',', begin);
        const auto token_end = end == std::string_view::npos ? text.size() : end;
        const auto token = std::string(text.substr(begin, token_end - begin));
        const auto decoded = double_field(Json{{"value", token}}, "value");
        if (decoded.value("status", std::string{}) != "decoded")
            return out;
        coordinates.push_back(decoded["value"]);
        if (end == std::string_view::npos)
            break;
        begin = end + 1;
    }
    if (coordinates.size() != dimensions)
        return out;
    out["status"] = "decoded";
    out["coordinates"] = std::move(coordinates);
    out["source_format"] = dimensions == 2 ? "%.17g,%.17g" : "%.17g,%.17g,%.17g";
    return out;
}

Json base64_value(std::string_view text, std::string type) {
    Json out = { {"status", "invalid_base64"}, {"type", std::move(type)},
                 {"encoding", "base64"} };
    Bytes bytes;
    try {
        bytes = unbase64(std::string(text));
    } catch (const std::exception &) {
        return out;
    }
    if (base64(bytes) != std::string(text))
        return out;
    out["status"] = "decoded";
    out["data"] = rawbytes(bytes);
    return out;
}

Json boolean_field(const Json &attributes, const char *name) {
    if (!attributes.is_object() || !attributes.contains(name))
        return { {"status", "missing"} };
    const auto &value = attributes[name];
    Json out = {{"source_value", value}, {"status", "unrecognized"}};
    if (!value.is_string())
        return out;
    const auto text = value.get<std::string>();
    if (text == "true" || text == "1") {
        out["status"] = "decoded";
        out["value"] = true;
    } else if (text == "false" || text == "0") {
        out["status"] = "decoded";
        out["value"] = false;
    }
    return out;
}

Json double_field(const Json &attributes, const char *name) {
    if (!attributes.is_object() || !attributes.contains(name))
        return {{"status", "missing"}};
    const auto &source = attributes[name];
    Json out = {{"source_value", source}, {"status", "unrecognized"}};
    if (!source.is_string())
        return out;
    const auto text = source.get<std::string>();
    double parsed = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed,
                                        std::chars_format::general);
    if (text.empty() || result.ec != std::errc{} || result.ptr != text.data() + text.size() ||
        !std::isfinite(parsed))
        return out;
    out["status"] = "decoded";
    out["value"] = parsed;
    return out;
}

Json integer_width_field(const Json &attributes, const char *name, bool is_signed) {
    if (!attributes.is_object() || !attributes.contains(name))
        return {{"status", "missing"}};
    Json out = {{"source_value", attributes[name]}, {"status", "unrecognized"}};
    const auto parsed = integer_field(attributes, name);
    if (!parsed)
        return out;
    const std::int64_t min_value = is_signed
                                       ? std::numeric_limits<std::int32_t>::min()
                                       : 0;
    const std::int64_t max_value = is_signed
                                       ? std::numeric_limits<std::int32_t>::max()
                                       : std::numeric_limits<std::uint32_t>::max();
    if (*parsed < min_value || *parsed > max_value)
        return out;
    out["status"] = "decoded";
    out["value"] = *parsed;
    out["width_bits"] = 32;
    out["signed"] = is_signed;
    return out;
}

Json integer_code_field(const Json &attributes, const char *name) {
    if (!attributes.is_object() || !attributes.contains(name))
        return {{"status", "missing"}};
    Json out = {{"source_value", attributes[name]}, {"status", "unrecognized"}};
    const auto parsed = integer_field(attributes, name);
    if (!parsed || *parsed < std::numeric_limits<std::int32_t>::min() ||
        *parsed > std::numeric_limits<std::int32_t>::max())
        return out;
    out["status"] = "decoded";
    out["code"] = *parsed;
    out["width_bits"] = 32;
    return out;
}

Json string_field(const Json &attributes, const char *name) {
    if (!attributes.is_object() || !attributes.contains(name))
        return {{"status", "missing"}};
    const auto &source = attributes[name];
    Json out = {{"source_value", source}, {"status", "unrecognized"}};
    if (!source.is_string())
        return out;
    out["status"] = "decoded";
    out["value"] = source;
    return out;
}

Json hyphen_string_list_field(const Json &attributes, const char *name) {
    if (!attributes.is_object() || !attributes.contains(name))
        return {{"status", "missing"}, {"items", Json::array()}};
    const auto &source = attributes[name];
    Json out = {{"source_value", source}, {"separator", "-"},
                {"status", "unrecognized"}, {"items", Json::array()}};
    if (!source.is_string())
        return out;
    const auto text = source.get<std::string>();
    if (text.empty()) {
        out["status"] = "decoded";
        return out;
    }
    std::size_t start = 0;
    while (start <= text.size()) {
        const auto end = text.find('-', start);
        const auto length = (end == std::string::npos ? text.size() : end) - start;
        out["items"].push_back(text.substr(start, length));
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    out["status"] = "decoded";
    return out;
}

Json signed_integer_list_field(const Json &attributes, const char *name,
                               std::int64_t min_value, std::int64_t max_value) {
    const auto split = hyphen_string_list_field(attributes, name);
    Json out = split;
    if (split.value("status", std::string{}) != "decoded")
        return out;
    Json values = Json::array();
    bool all_decoded = true;
    for (const auto &item : split["items"]) {
        const auto text = item.get<std::string>();
        std::int64_t parsed = 0;
        const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
        Json value = {{"source_token", item}, {"status", "unrecognized"}};
        if (!text.empty() && result.ec == std::errc{} &&
            result.ptr == text.data() + text.size() && parsed >= min_value &&
            parsed <= max_value) {
            value["status"] = "decoded";
            value["value"] = parsed;
        } else {
            all_decoded = false;
        }
        values.push_back(std::move(value));
    }
    out["items"] = std::move(values);
    out["status"] = all_decoded ? "decoded" : "partially_decoded";
    return out;
}

Json packed_entity_id_list_field(const Json &attributes) {
    auto out = signed_integer_list_field(attributes, "EntityIds",
                                         std::numeric_limits<std::int64_t>::min(),
                                         std::numeric_limits<std::int64_t>::max());
    if (out.value("status", std::string{}) == "missing" ||
        out.value("status", std::string{}) == "unrecognized")
        return out;
    for (auto &item : out["items"]) {
        if (item.value("status", std::string{}) != "decoded")
            continue;
        const auto packed = item["value"].get<std::int64_t>();
        const auto bits = static_cast<std::uint64_t>(packed);
        const auto model_bits = static_cast<std::uint32_t>(bits >> 32);
        const auto entity_bits = static_cast<std::uint32_t>(bits);
        const auto model_id = model_bits <= static_cast<std::uint32_t>(
                                          std::numeric_limits<std::int32_t>::max())
                                  ? static_cast<std::int64_t>(model_bits)
                                  : static_cast<std::int64_t>(model_bits) -
                                        (std::int64_t{1} << 32);
        item["packed_value"] = packed;
        item["model_id"] = model_id;
        item["model_id_bits"] = model_bits;
        item["entity_id"] = entity_bits;
        item["entity_id_bits"] = entity_bits;
        item["valid"] = packed != -1 && model_id > -2;
    }
    return out;
}

Json enum_field(const Json &attributes, const char *name,
                std::initializer_list<std::pair<std::int64_t, const char *>> values) {
    Json out = {{"source_value", attributes.is_object() && attributes.contains(name)
                                     ? attributes[name]
                                     : Json()},
                {"status", "missing_or_unrecognized"}};
    const auto parsed = integer_field(attributes, name);
    if (!parsed)
        return out;
    out["code"] = *parsed;
    for (const auto &[code, label] : values)
        if (*parsed == code) {
            out["status"] = "decoded";
            out["name"] = label;
            break;
        }
    return out;
}

Json bp_value_type_field(const Json &attributes) {
    Json out = enum_field(attributes, "dexDataType",
                          {{0x000, "unknown"},
                           {0x101, "double"},
                           {0x201, "integer"},
                           {0x301, "boolean"},
                           {0x401, "long"},
                           {0x501, "point2d"},
                           {0x601, "point3d"},
                           {0x701, "geometry"},
                           {0x801, "string"},
                           {0x901, "binary"},
                           {0xa01, "datetime"},
                           {0xb01, "json"}});
    out["source_width_bits"] = 16;
    out["source_enum"] = "BIMBase::BPValueType";
    return out;
}

Json bp_value_text(const Json &attributes) {
    const Json source = attributes.is_object() && attributes.contains("dexDataValue")
                            ? attributes["dexDataValue"]
                            : Json();
    Json out = {{"source_value", source},
                {"encoding", "type-directed BPValue text; original lexical value retained"},
                {"typed_value", {{"status", "not_decoded"}}}};
    const auto type = integer_field(attributes, "dexDataType");
    if (!type || !source.is_string())
        return out;
    const auto text = source.get<std::string>();
    if (*type == 0x101) {
        const auto decoded = double_field(Json{{"value", text}}, "value");
        if (decoded.value("status", std::string{}) == "decoded")
            out["typed_value"] = {{"status", "decoded"}, {"type", "double"},
                                  {"value", decoded["value"]}};
    } else if (*type == 0x201 || *type == 0x401) {
        std::int64_t parsed = 0;
        const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
        const bool in_range = *type == 0x401 ||
                              (parsed >= std::numeric_limits<std::int32_t>::min() &&
                               parsed <= std::numeric_limits<std::int32_t>::max());
        if (!text.empty() && result.ec == std::errc{} && result.ptr == text.data() + text.size() &&
            in_range)
            out["typed_value"] = {{"status", "decoded"},
                                  {"type", *type == 0x201 ? "integer" : "long"},
                                  {"value", parsed}};
    } else if (*type == 0x301) {
        if (text == "true" || text == "1")
            out["typed_value"] = {{"status", "decoded"}, {"type", "boolean"},
                                  {"value", true}};
        else if (text == "false" || text == "0")
            out["typed_value"] = {{"status", "decoded"}, {"type", "boolean"},
                                  {"value", false}};
    } else if (*type == 0x801) {
        out["typed_value"] = {{"status", "decoded"}, {"type", "string"}, {"value", text}};
    } else if (*type == 0xb01) {
        const auto parsed = Json::parse(text, nullptr, false);
        if (parsed.is_discarded()) {
            out["typed_value"] = {{"status", "invalid_json"}, {"type", "json"}};
        } else {
            out["typed_value"] = {{"status", "decoded"}, {"type", "json"},
                                  {"value", parsed}};
        }
    } else if (*type == 0xa01) {
        out["typed_value"] = datetime_value(text);
    } else if (*type == 0x501 || *type == 0x601) {
        out["typed_value"] = point_value(text, *type == 0x501 ? 2 : 3);
    } else if (*type == 0x701 || *type == 0x901) {
        out["typed_value"] = base64_value(text, *type == 0x701 ? "geometry" : "binary");
    }
    return out;
}

const char *condition_kind(std::string_view tag) {
    if (tag == "P3DEntityRangeShowCondition")
        return "entity_range";
    if (tag == "P3DEntityVolumeShowCondition")
        return "entity_volume";
    if (tag == "P3DEntityDexDataShowCondition")
        return "entity_property";
    return nullptr;
}

Json condition_details(const std::string &tag, const Json &attributes) {
    const auto kind = condition_kind(tag);
    Json out = {{"kind", kind ? kind : "unknown"}, {"tag", tag}, {"fields", attributes}};
    if (tag == "P3DEntityRangeShowCondition") {
        out["range"] = {{"low_x", double_field(attributes, "RangeConditionLowX")},
                        {"low_y", double_field(attributes, "RangeConditionLowY")},
                        {"low_z", double_field(attributes, "RangeConditionLowZ")},
                        {"high_x", double_field(attributes, "RangeConditionHighX")},
                        {"high_y", double_field(attributes, "RangeConditionHighY")},
                        {"high_z", double_field(attributes, "RangeConditionHighZ")}};
        out["operator"] = enum_field(attributes, "RangeConditionOperatorType",
                                     {{0, "unknown"}, {1, "inner"}, {2, "outer"},
                                      {4, "intersect"}});
        out["exclude"] = boolean_field(attributes, "RangeConditionExclude");
    } else if (tag == "P3DEntityVolumeShowCondition") {
        out["volume"] = double_field(attributes, "VolumeCondition");
        out["operator"] = enum_field(attributes, "VolumeCompareOperatorType",
                                     {{1, "equal"}, {2, "not_equal"}, {3, "less"},
                                      {4, "greater"}});
        out["exclude"] = boolean_field(attributes, "VolumeCompareExclude");
    } else if (tag == "P3DEntityDexDataShowCondition") {
        out["property_name"] = string_field(attributes, "DexDataPropertyName");
        out["value_type"] = bp_value_type_field(attributes);
        out["value"] = bp_value_text(attributes);
        out["operator"] = enum_field(attributes, "dexDataValueCompareOperator",
                                     {{0, "equal"}, {1, "not_equal"}, {2, "greater"},
                                      {3, "greater_or_equal"}, {4, "less"},
                                      {5, "less_or_equal"}, {6, "in"}, {7, "not_in"}});
        out["exclude"] = boolean_field(attributes, "dexDataValueExclude");
    }
    return out;
}

Json action_details(const std::string &tag, const Json &attributes) {
    Json out = {{"tag", tag}, {"fields", attributes}, {"action_id", nullptr}};
    if (tag == "EntityDisplayAction")
        out["action_id"] = 0;
    else if (tag == "ColorOverrideAction")
        out["action_id"] = 1;
    else if (tag == "FillColorOverrideAction")
        out["action_id"] = 2;
    else if (tag == "StyleOverrideAction")
        out["action_id"] = 3;
    else if (tag == "WeightOverrideAction")
        out["action_id"] = 4;
    else if (tag == "TransparencyOverrideAction")
        out["action_id"] = 5;
    else if (tag == "PDEntityPriorityAction")
        out["action_id"] = 6;
    else if (tag == "PresentationFormAction")
        out["action_id"] = 7;
    else if (tag == "AreaHatchAction")
        out["action_id"] = 8;
    else if (tag == "AreaPatternAction")
        out["action_id"] = 9;
    else if (tag == "LayerOfDetailAction")
        out["action_id"] = 11;
    else if (tag == "ShowStyleOverrideAction")
        out["action_id"] = 12;
    out["action_id_status"] = out["action_id"].is_number_integer()
                                   ? "resolved_native_action_id"
                                   : (tag == "MaterialOverrideAction"
                                          ? "native_material_action_alias_unresolved"
                                          : "unmapped_native_action_id");
    if (tag == "EntityDisplayAction")
        out["entity_hidden"] = boolean_field(attributes, "ElementDisplayOff");
    else if (tag == "ColorOverrideAction") {
        out["entity_color"] = integer_width_field(attributes, "ElementColor", false);
        out["edge_color"] = integer_width_field(attributes, "ElementEdgeColor", false);
    } else if (tag == "FillColorOverrideAction") {
        out["fill_color"] = integer_width_field(attributes, "ElementFillColor", false);
    } else if (tag == "StyleOverrideAction") {
        out["line_style"] = integer_width_field(attributes, "LineStyle", true);
    } else if (tag == "ShowStyleOverrideAction") {
        out["display_style"] = integer_code_field(attributes, "DisplayStyle");
    } else if (tag == "WeightOverrideAction") {
        out["line_weight"] = integer_width_field(attributes, "LineWeight", false);
    } else if (tag == "TransparencyOverrideAction") {
        out["transparency"] = double_field(attributes, "Transparency");
    } else if (tag == "PDEntityPriorityAction") {
        out["entity_priority"] = integer_width_field(attributes, "ElementPriority", true);
    } else if (tag == "PresentationFormAction") {
        out["presentation_form"] = string_field(attributes, "PresentationForm");
    } else if (tag == "LayerOfDetailAction") {
        out["level_of_detail"] = double_field(attributes, "LevelOfDetail");
    } else if (tag == "AreaHatchAction") {
        out["distance"] = double_field(attributes, "Distance");
        out["angle"] = double_field(attributes, "Angle");
        out["distance2"] = double_field(attributes, "Distance2");
        out["angle2"] = double_field(attributes, "Angle2");
        out["element_color"] = integer_width_field(attributes, "ElementColor", false);
        out["line_style"] = integer_width_field(attributes, "LineStyle", true);
        out["line_weight"] = integer_width_field(attributes, "LineWeight", false);
        out["annotation_scale"] = boolean_field(attributes, "AnnotationScale");
    } else if (tag == "AreaPatternAction") {
        out["cell_id"] = string_field(attributes, "CellId");
        out["scale"] = double_field(attributes, "Scale");
        out["angle"] = double_field(attributes, "Angle");
        out["element_color"] = integer_width_field(attributes, "ElementColor", false);
        out["line_style"] = integer_width_field(attributes, "LineStyle", true);
        out["line_weight"] = integer_width_field(attributes, "LineWeight", false);
        out["annotation_scale"] = boolean_field(attributes, "AnnotationScale");
    } else if (tag == "MaterialOverrideAction") {
        out["material_id"] = string_field(attributes, "MaterialId");
        out["material_name"] = string_field(attributes, "MaterialName");
    }
    return out;
}

bool is_known_action_tag(const std::string &tag) {
    return tag == "EntityDisplayAction" || tag == "ColorOverrideAction" ||
           tag == "FillColorOverrideAction" || tag == "StyleOverrideAction" ||
           tag == "ShowStyleOverrideAction" || tag == "WeightOverrideAction" ||
           tag == "TransparencyOverrideAction" || tag == "PDEntityPriorityAction" ||
           tag == "PresentationFormAction" || tag == "AreaHatchAction" ||
           tag == "AreaPatternAction" || tag == "MaterialOverrideAction" ||
           tag == "LayerOfDetailAction";
}

Json condition_entry(const Json &node) {
    const auto tag = node.value("tag", std::string{});
    const auto attributes = node.value("attributes", Json::object());
    Json out = {{"tag", tag}, {"fields", attributes}, {"source_tree", node}};
    if (condition_kind(tag)) {
        out["entry_kind"] = "condition";
        out["condition"] = condition_details(tag, attributes);
    } else if (is_known_action_tag(tag)) {
        out["entry_kind"] = "action";
        out["action"] = action_details(tag, attributes);
    } else {
        out["entry_kind"] = "unclassified";
    }
    return out;
}

Json condition_record(const Json &node) {
    const auto attributes = node.value("attributes", Json::object());
    Json entries = Json::array();
    if (node.contains("children") && node["children"].is_array())
        for (const auto &child : node["children"])
            entries.push_back(condition_entry(child));
    Json out = {{"fields", attributes},
                {"condition_name", attributes.value("ConditionName", Json())},
                {"description", attributes.value("ConditionDescription", Json())},
                {"enabled", boolean_field(attributes, "Enabled")},
                {"stop_if_true", boolean_field(attributes, "StopIfTrue")},
                {"entity_ids", attributes.value("EntityIds", Json())},
                {"entity_id_values", packed_entity_id_list_field(attributes)},
                {"entity_ids_exclude", boolean_field(attributes, "EntityIdsExclude")},
                {"model_link_ids", attributes.value("ModelLinkIds", Json())},
                {"model_link_id_values",
                 signed_integer_list_field(attributes, "ModelLinkIds",
                                           std::numeric_limits<std::int32_t>::min(),
                                           std::numeric_limits<std::int32_t>::max())},
                {"model_link_ids_exclude", boolean_field(attributes, "ModelLinkIdsExclude")},
                {"full_name_list", attributes.value("fullNameList", Json())},
                {"full_name_values", hyphen_string_list_field(attributes, "fullNameList")},
                {"full_name_list_exclude", boolean_field(attributes, "fullNameListExclude")},
                {"entries", std::move(entries)},
                {"source_tree", node}};
    return out;
}

Json parse_condition_set(const Json &tree) {
    const auto attributes = tree.value("attributes", Json::object());
    Json conditions = Json::array();
    if (tree.contains("children") && tree["children"].is_array())
        for (const auto &child : tree["children"])
            if (child.value("tag", std::string{}) == "P3DShowCondition")
                conditions.push_back(condition_record(child));
    return {{"condition_set_name", attributes.value("ConditionSetName", Json())},
            {"fields", attributes},
            {"conditions", std::move(conditions)},
            {"source_tree", tree}};
}

Json condition_view_binding(const Json &name, const Json &models) {
    if (!name.is_string())
        return {{"status", "condition_set_name_unavailable"}};
    const auto text = name.get<std::string>();
    constexpr std::string_view prefix = "bimbase_conditionset_";
    if (text.rfind(prefix, 0) != 0)
        return {{"status", "unrecognized_condition_set_name"}, {"source_name", name}};
    const auto separator = text.find("_view", prefix.size());
    if (separator == std::string::npos || separator == prefix.size() ||
        separator + 5 == text.size())
        return {{"status", "unrecognized_condition_set_name"}, {"source_name", name}};
    std::uint64_t model_id = 0;
    unsigned view_index = 0;
    const auto model_text = std::string_view(text).substr(prefix.size(), separator - prefix.size());
    const auto view_text = std::string_view(text).substr(separator + 5);
    const auto model_parse = std::from_chars(model_text.data(), model_text.data() + model_text.size(),
                                             model_id);
    const auto view_parse = std::from_chars(view_text.data(), view_text.data() + view_text.size(),
                                            view_index);
    if (model_parse.ec != std::errc{} || model_parse.ptr != model_text.data() + model_text.size() ||
        view_parse.ec != std::errc{} || view_parse.ptr != view_text.data() + view_text.size())
        return {{"status", "unrecognized_condition_set_name"}, {"source_name", name}};

    auto expected_model_text = std::to_string(model_id);
    if (expected_model_text.size() < 9)
        expected_model_text.insert(0, 9 - expected_model_text.size(), '0');
    if (model_text != expected_model_text || view_text != std::to_string(view_index))
        return {{"status", "unrecognized_condition_set_name"}, {"source_name", name}};

    const auto key = std::to_string(model_id);
    const Json *model = models.is_object() && models.contains(key) ? &models[key] : nullptr;
    Json reference = {{"status", model ? "resolved" : "missing"},
                      {"match", "exact_decimal_model_id"}, {"model_id", model_id}};
    if (model) {
        reference["name"] = model->value("name", Json());
        reference["physical_storage"] = model->value("physical_storage", Json());
    }
    // Native exports create/find condition sets by (modelId, nViewNum), and
    // the DLL formats their persisted key with this exact zero-padded model ID.
    // This resolves the set identity, not whether that view is currently active.
    return {{"status", "resolved_model_view_identity"},
            {"match", "native_model_id_and_view_number_key"},
            {"active_view_selection", "not_evaluated"},
            {"source_name", name},
            {"model_id", model_id},
            {"view_index", view_index},
            {"model_reference", std::move(reference)}};
}

} // namespace

Json build_native_view_condition_catalog(const Json &graphics_records, const Json &models) {
    Json records = Json::array();
    if (!graphics_records.is_array())
        return {{"status", "graphics_records_unavailable"}, {"records", std::move(records)}};

    for (std::size_t record_index = 0; record_index < graphics_records.size(); ++record_index) {
        const auto &graphic = graphics_records[record_index];
        if (!graphic.is_object() || !graphic.contains("attributes") || !graphic["attributes"].is_array())
            continue;
        for (const auto &attribute : graphic["attributes"]) {
            if (attribute.value("group", 0) != 0 || attribute.value("key", 0) != 22840)
                continue;
            const Json decoded = attribute.value("decoded", Json::object());
            const Json tree = decoded.value("tree", Json::object());
            Json record = {{"source_graphics_record_index", record_index},
                           {"source_graphics_record_id", graphic.value("id", Json())},
                           {"source_graphics_record_offset", graphic.value("offset", Json())},
                           {"source_attribute_group", 0},
                           {"source_attribute_key", 22840},
                           {"source_attribute_index", attribute.value("index", Json())},
                           {"source_attribute_offset", attribute.value("offset", Json())},
                           {"source_attribute", attribute},
                           {"source_encoding", decoded.value("encoding", Json())}};
            if (!tree.is_object() || tree.value("tag", std::string{}) != "P3DShowConditionSet") {
                record["status"] = tree.is_object() ? "unexpected_xml_root" : "xml_tree_unavailable";
                record["decoded"] = decoded;
            } else {
                record["status"] = "decoded";
                record.update(parse_condition_set(tree));
                record["view_binding"] = condition_view_binding(record["condition_set_name"], models);
            }
            records.push_back(std::move(record));
        }
    }
    return {{"status", records.empty() ? "no_records" : "decoded"},
            {"source_attribute", {{"group", 0}, {"key", 22840}}},
            {"records", std::move(records)}};
}

} // namespace p3d
