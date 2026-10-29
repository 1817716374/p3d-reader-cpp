#include "internal.hpp"
#include <charconv>
#include <cmath>
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
        out["value_type"] = bp_value_type_field(attributes);
        out["value"] = {{"source_value", attributes.is_object() && attributes.contains("dexDataValue")
                                             ? attributes["dexDataValue"]
                                             : Json()},
                         {"encoding", "type-directed BPValue text; original lexical value retained"}};
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
    else if (tag == "StyleOverrideAction" || tag == "ShowStyleOverrideAction")
        out["action_id"] = 3;
    else if (tag == "WeightOverrideAction")
        out["action_id"] = 4;
    else if (tag == "TransparencyOverrideAction")
        out["action_id"] = 5;
    if (tag == "EntityDisplayAction")
        out["entity_hidden"] = boolean_field(attributes, "ElementDisplayOff");
    return out;
}

Json condition_entry(const Json &node) {
    const auto tag = node.value("tag", std::string{});
    const auto attributes = node.value("attributes", Json::object());
    Json out = {{"tag", tag}, {"fields", attributes}, {"source_tree", node}};
    if (condition_kind(tag)) {
        out["entry_kind"] = "condition";
        out["condition"] = condition_details(tag, attributes);
    } else if (tag == "EntityDisplayAction" || tag == "ColorOverrideAction" ||
               tag == "FillColorOverrideAction" || tag == "StyleOverrideAction" ||
               tag == "ShowStyleOverrideAction" || tag == "WeightOverrideAction" ||
               tag == "TransparencyOverrideAction") {
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
                {"entity_ids_exclude", boolean_field(attributes, "EntityIdsExclude")},
                {"model_link_ids", attributes.value("ModelLinkIds", Json())},
                {"model_link_ids_exclude", boolean_field(attributes, "ModelLinkIdsExclude")},
                {"full_name_list", attributes.value("fullNameList", Json())},
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

} // namespace

Json build_native_view_condition_catalog(const Json &graphics_records) {
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
            }
            records.push_back(std::move(record));
        }
    }
    return {{"status", records.empty() ? "no_records" : "decoded"},
            {"source_attribute", {{"group", 0}, {"key", 22840}}},
            {"records", std::move(records)}};
}

} // namespace p3d
