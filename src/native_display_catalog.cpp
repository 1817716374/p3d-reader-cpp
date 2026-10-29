#include "internal.hpp"
#include <charconv>
#include <string_view>

namespace p3d {
namespace {

Json parse_location_index(const Json &value) {
    Json out = {{"source_value", value}, {"status", "unavailable"}};
    if (!value.is_string()) {
        out["status"] = "not_string";
        return out;
    }

    const auto text = value.get<std::string>();
    Json tokens = Json::array();
    Json numbers = Json::array();
    std::size_t start = 0;
    bool valid = !text.empty();
    while (valid) {
        const auto end = text.find('-', start);
        const auto stop = end == std::string::npos ? text.size() : end;
        const std::string_view token(text.data() + start, stop - start);
        std::uint64_t number = 0;
        const auto parsed = std::from_chars(token.data(), token.data() + token.size(), number);
        if (token.empty() || parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size()) {
            valid = false;
            break;
        }
        tokens.push_back(std::string(token));
        numbers.push_back(number);
        if (end == std::string::npos)
            break;
        start = end + 1;
        if (start == text.size()) {
            valid = false;
            break;
        }
    }
    if (!valid) {
        out["status"] = "not_numeric_hyphen_sequence";
        return out;
    }
    out["status"] = "decoded";
    out["tokens"] = std::move(tokens);
    out["unsigned_components"] = std::move(numbers);
    return out;
}

const Json *unique_field(const Json &tree, const char *name) {
    if (!tree.is_object() || !tree.contains("children") || !tree["children"].is_array())
        return nullptr;
    const Json *found = nullptr;
    for (const auto &child : tree["children"]) {
        if (child.value("name", std::string{}) != name)
            continue;
        if (found)
            return nullptr;
        found = &child;
    }
    return found;
}

Json direct_fields(const Json &tree) {
    Json fields = Json::object();
    if (!tree.is_object() || !tree.contains("children") || !tree["children"].is_array())
        return fields;
    for (const auto &child : tree["children"]) {
        if (!child.is_object() || !child.contains("name") || !child.contains("value"))
            continue;
        const auto name = child["name"].get<std::string>();
        if (!fields.contains(name))
            fields[name] = child["value"];
        else if (fields[name].is_array())
            fields[name].push_back(child["value"]);
        else
            fields[name] = Json::array({fields[name], child["value"]});
    }
    return fields;
}

std::optional<std::uint64_t> unsigned_integer(const Json *value) {
    if (!value || !value->is_number_integer())
        return std::nullopt;
    if (value->is_number_unsigned())
        return value->get<std::uint64_t>();
    const auto signed_value = value->get<std::int64_t>();
    if (signed_value < 0)
        return std::nullopt;
    return static_cast<std::uint64_t>(signed_value);
}

Json display_statuses(const Json &tree) {
    Json out = Json::array();
    const auto *info = unique_field(tree, "DisplayInfo");
    if (!info || !info->is_object() || !info->contains("children") || !(*info)["children"].is_array())
        return out;
    for (const auto &item : (*info)["children"]) {
        if (item.value("name", std::string{}) != "PBBimDisplayStatus")
            continue;
        Json status = {{"property_tree", item}, {"fields", direct_fields(item)}};
        if (const auto *path = unique_field(item, "SubCategory"); path && path->contains("value"))
            status["subcategory_location_index"] = parse_location_index((*path)["value"]);
        out.push_back(std::move(status));
    }
    return out;
}

Json nested_records(const Json &tree, const char *field_name, const char *record_name,
                   bool with_location_index) {
    Json out = Json::array();
    const auto *group = unique_field(tree, field_name);
    if (!group || !group->is_object() || !group->contains("children") || !(*group)["children"].is_array())
        return out;
    for (const auto &item : (*group)["children"]) {
        if (item.value("name", std::string{}) != record_name)
            continue;
        Json row = {{"property_tree", item}, {"fields", direct_fields(item)}};
        if (with_location_index) {
            if (const auto *path = unique_field(item, "PixType"); path && path->contains("value"))
                row["pix_type_location_index"] = parse_location_index((*path)["value"]);
        } else if (record_name == std::string("PBBimDisplayStatus")) {
            if (const auto *path = unique_field(item, "SubCategory"); path && path->contains("value"))
                row["subcategory_location_index"] = parse_location_index((*path)["value"]);
        }
        out.push_back(std::move(row));
    }
    return out;
}

} // namespace

Json build_native_display_catalog(const Json &graph) {
    Json records = Json::array();
    if (!graph.is_object() || !graph.contains("objects") || !graph["objects"].is_array())
        return {{"status", "object_graph_unavailable"}, {"records", records}};

    for (const auto &object : graph["objects"]) {
        if (!object.is_object() || !object.contains("class") || !object.contains("property_tree"))
            continue;
        const auto class_name = object["class"].get<std::string>();
        const bool display_cache = class_name == "PBBimDisplayCacheInfo";
        const bool line_style = class_name == "PBBimStyleLineInfo";
        const bool hatch_style = class_name == "PBBimStyleHatchInfo";
        const bool style_definition = class_name == "PBBimStyleInfo";
        const bool style_cache = class_name == "PBBimStyleCacheInfo";
        const bool style_graphic = class_name == "PBBimStyleGraphicInfo";
        const bool model_view = class_name == "PBStoreyModelViewInfo";
        const bool view_data = class_name == "BCCBBViewData";
        const bool view_config = class_name == "BCCBBViewConfigData";
        const bool view_config_list = class_name == "BCCBBViewConfigDataList";
        const bool feature_style = class_name == "FeatureStyleDataElement";
        if (!display_cache && !line_style && !hatch_style && !style_definition && !style_cache &&
            !style_graphic && !model_view && !view_data && !view_config && !view_config_list &&
            !feature_style)
            continue;

        const auto &tree = object["property_tree"];
        Json record = {{"record_key", object.value("record_key", std::string{})},
                       {"object_key", object.value("object_key", std::string{})},
                       {"class", class_name},
                       {"class_id", object.value("class_id", Json())},
                       {"object_id", object.value("object_id", Json())},
                       {"fields", direct_fields(tree)},
                       {"property_tree", tree}};
        if (display_cache) {
            record["record_kind"] = "display_cache";
            record["display_statuses"] = display_statuses(tree);
        } else if (style_definition) {
            record["record_kind"] = "style_definition";
            record["line_components"] = nested_records(tree, "Line", "PBBimLineStyle", true);
            record["hatch_components"] = nested_records(tree, "Hatch", "PBBimHatchStyle", true);
            record["visibility_statuses"] = nested_records(tree, "Visble", "PBBimDisplayStatus", false);
        } else if (view_data) {
            record["record_kind"] = "view_data";
            const auto fields = record["fields"];
            const auto class_id = unsigned_integer(fields.contains("ModelElementClassID")
                                                       ? &fields["ModelElementClassID"]
                                                       : nullptr);
            const auto object_id = unsigned_integer(fields.contains("ModelElementDataID")
                                                        ? &fields["ModelElementDataID"]
                                                        : nullptr);
            if (class_id && object_id) {
                record["model_element_reference"] =
                    {{"class_id", *class_id},
                     {"object_id", *object_id},
                     {"target", get_object(graph, *class_id, *object_id)}};
            } else {
                record["model_element_reference"] = {{"status", "source_id_pair_unavailable"}};
            }
        } else {
            record["record_kind"] = line_style ? "line_style"
                                      : hatch_style ? "hatch_style"
                                      : style_cache ? "style_cache"
                                      : style_graphic ? "style_graphic"
                                      : model_view ? "model_view"
                                      : view_data ? "view_data"
                                      : view_config ? "view_config"
                                      : view_config_list ? "view_config_list"
                                      : feature_style ? "feature_style"
                                                      : "unknown";
            if (const auto *pix_type = unique_field(tree, "PixType"); pix_type && pix_type->contains("value"))
                record["pix_type_location_index"] = parse_location_index((*pix_type)["value"]);
        }
        records.push_back(std::move(record));
    }
    return {{"status", "decoded"},
            {"records", std::move(records)},
            {"relationship_policy", "records remain independent; no display or style linkage is inferred"},
            {"path_policy", "hyphen-separated unsigned numeric components are exposed lexically without assigning component meanings"}};
}

} // namespace p3d
