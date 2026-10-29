#include "internal.hpp"
#include <charconv>
#include <map>
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

std::vector<std::uint64_t> path_components(const Json &parsed) {
    std::vector<std::uint64_t> out;
    if (!parsed.is_object() || parsed.value("status", "") != "decoded" ||
        !parsed.contains("unsigned_components"))
        return out;
    for (const auto &component : parsed["unsigned_components"])
        out.push_back(component.get<std::uint64_t>());
    return out;
}

bool is_prefix(const std::vector<std::uint64_t> &prefix,
               const std::vector<std::uint64_t> &path) {
    return !prefix.empty() && prefix.size() <= path.size() &&
           std::equal(prefix.begin(), prefix.end(), path.begin());
}

Json record_reference(const Json &record) {
    Json out = {{"record_key", record.value("record_key", std::string{})},
                {"object_key", record.value("object_key", std::string{})},
                {"class", record.value("class", std::string{})},
                {"class_id", record.value("class_id", Json())},
                {"object_id", record.value("object_id", Json())},
                {"fields", record.value("fields", Json::object())}};
    if (record.contains("native_layer_state"))
        out["native_layer_state"] = record["native_layer_state"];
    return out;
}

Json model_reference(const Json &models, const Json &source_id) {
    const auto id = unsigned_integer(&source_id);
    Json out = {{"source_model_id", source_id}, {"match", "exact_decimal_model_id_key"}};
    if (!id) {
        out["status"] = "source_id_unavailable";
        return out;
    }
    const auto key = std::to_string(*id);
    if (!models.is_object() || !models.contains(key)) {
        out["status"] = "missing";
        out["model_id"] = *id;
        return out;
    }
    const auto &model = models[key];
    out["status"] = "resolved";
    out["model_id"] = *id;
    out["model"] = {{"model_id", *id},
                     {"name", model.value("name", Json())},
                     {"physical_storage", model.value("physical_storage", Json())}};
    return out;
}

Json model_reference_list(const Json &models, const Json &fields, const char *field_name) {
    Json out = {{"source_field", field_name},
                {"match", "exact_decimal_model_id_key"},
                {"references", Json::array()}};
    if (!fields.contains(field_name)) {
        out["status"] = "source_field_unavailable";
        return out;
    }
    const auto &source = fields[field_name];
    if (!source.is_array()) {
        out["status"] = "source_value_not_array";
        out["source_value"] = source;
        return out;
    }
    out["status"] = "decoded";
    out["source_value"] = source;
    for (std::size_t i = 0; i < source.size(); ++i) {
        auto reference = model_reference(models, source[i]);
        reference["source_ordinal"] = i;
        out["references"].push_back(std::move(reference));
    }
    return out;
}

void add_path_context(Json &item, const Json &parsed,
                      const std::map<std::string, std::vector<Json>> &subcategories,
                      const std::map<std::string, std::vector<Json>> &nodes,
                      bool allow_subcategory_prefix) {
    const auto path = path_components(parsed);
    Json subcategory_matches = Json::array(), hierarchy = Json::array();
    if (!path.empty()) {
        for (const auto &[key, rows] : subcategories) {
            const auto index = path_components(parse_location_index(key));
            const bool match = allow_subcategory_prefix ? is_prefix(index, path) : index == path;
            if (!match)
                continue;
            for (const auto &row : rows)
                subcategory_matches.push_back(
                    {{"match", allow_subcategory_prefix && index != path ? "path_prefix" : "exact"},
                     {"location_index", key}, {"record", record_reference(row)}});
        }
        std::vector<std::pair<std::size_t, Json>> ordered_nodes;
        for (const auto &[key, rows] : nodes) {
            const auto index = path_components(parse_location_index(key));
            if (!is_prefix(index, path))
                continue;
            for (const auto &row : rows)
                ordered_nodes.emplace_back(index.size(),
                    Json{{"location_index", key}, {"record", record_reference(row)}});
        }
        std::stable_sort(ordered_nodes.begin(), ordered_nodes.end(),
                         [](const auto &a, const auto &b) { return a.first < b.first; });
        for (auto &entry : ordered_nodes)
            hierarchy.push_back(std::move(entry.second));
    }
    item["location_index_context"] = {
        {"status", path.empty() ? "unresolved_path" : "decoded"},
        {"subcategory_match_rule", allow_subcategory_prefix ? "source_index_is_prefix_of_path"
                                                             : "exact_source_index"},
        {"subcategories", std::move(subcategory_matches)},
        {"hierarchy_nodes", std::move(hierarchy)}};
}

void add_layer_pix_references(Json &item, const Json &path,
                              const std::map<std::string, std::vector<Json>> &layer_pix) {
    if (!path.is_string()) {
        item["layer_pix_references"] = {{"status", "source_path_unavailable"},
                                        {"records", Json::array()}};
        return;
    }
    const auto found = layer_pix.find(path.get<std::string>());
    Json refs = Json::array();
    if (found != layer_pix.end())
        for (const auto &record : found->second)
            refs.push_back(record_reference(record));
    item["layer_pix_references"] = {
        {"status", refs.empty() ? "missing" : refs.size() == 1 ? "resolved" : "ambiguous"},
        {"match", "exact_PixType"}, {"records", std::move(refs)}};
}

} // namespace

Json build_native_display_catalog(const Json &graph, const Json &layer_tables,
                                 const Json &native_records, const Json &models) {
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
        const bool category_manager_pre = class_name == "BPEditEnvCategoryManagerPreData";
        const bool category_manager_post = class_name == "BPEditEnvCategoryManagerPostData";
        const bool feature_style = class_name == "FeatureStyleDataElement";
        const bool group_relation = class_name == "PBBimGroupRelInfo";
        const bool layer_pix_relation = class_name == "PBBimLayerPixRelInfo";
        const bool subcategory_info = class_name == "PBBimSubCategoryInfo";
        const bool subcategory_node = class_name == "PBBimSubCategoryNodeName";
        if (!display_cache && !line_style && !hatch_style && !style_definition && !style_cache &&
            !style_graphic && !model_view && !view_data && !view_config && !view_config_list &&
            !category_manager_pre && !category_manager_post && !feature_style && !group_relation &&
            !layer_pix_relation && !subcategory_info && !subcategory_node)
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
        } else if (view_config) {
            record["record_kind"] = "view_config";
            record["filters"] = nested_records(tree, "Filters", "BCCBBViewFilter", false);
        } else if (category_manager_pre || category_manager_post) {
            record["record_kind"] = category_manager_pre ? "category_manager_pre_data"
                                                          : "category_manager_post_data";
        } else if (group_relation) {
            record["record_kind"] = "group_relation";
        } else if (layer_pix_relation) {
            record["record_kind"] = "layer_pix_relation";
            if (const auto *path = unique_field(tree, "PixType"); path && path->contains("value"))
                record["pix_type_location_index"] = parse_location_index((*path)["value"]);
        } else if (subcategory_info) {
            record["record_kind"] = "subcategory_info";
            if (const auto *path = unique_field(tree, "LocationIndex"); path && path->contains("value"))
                record["location_index"] = parse_location_index((*path)["value"]);
        } else if (subcategory_node) {
            record["record_kind"] = "subcategory_node_name";
            if (const auto *path = unique_field(tree, "LocationIndex"); path && path->contains("value"))
                record["location_index"] = parse_location_index((*path)["value"]);
        } else {
            record["record_kind"] = line_style ? "line_style"
                                      : hatch_style ? "hatch_style"
                                      : style_cache ? "style_cache"
                                      : style_graphic ? "style_graphic"
                                      : model_view ? "model_view"
                                      : view_data ? "view_data"
                                      : view_config ? "view_config"
                                      : view_config_list ? "view_config_list"
                                      : category_manager_pre ? "category_manager_pre_data"
                                      : category_manager_post ? "category_manager_post_data"
                                      : feature_style ? "feature_style"
                                                      : "unknown";
            if (const auto *pix_type = unique_field(tree, "PixType"); pix_type && pix_type->contains("value"))
                record["pix_type_location_index"] = parse_location_index((*pix_type)["value"]);
        }
        records.push_back(std::move(record));
    }

    std::map<std::string, std::vector<Json>> subcategory_index, node_index, layer_pix_index;
    std::map<std::uint64_t, std::vector<Json>> native_layer_index;
    std::map<std::uint64_t, std::vector<Json>> style_definition_index,
        style_cache_index, display_cache_index;
    std::map<std::string, std::vector<Json>> view_config_index,
        config_list_membership_index, active_config_list_index;
    if (layer_tables.is_object() && layer_tables.contains("group_states") &&
        layer_tables["group_states"].is_array()) {
        auto layer_source_records = [&](const Json &indices) {
            Json sources = Json::array();
            if (!indices.is_array())
                return sources;
            for (const auto &index : indices) {
                const auto row = unsigned_integer(&index);
                if (!row || *row >= native_records.size())
                    continue;
                const auto &native = native_records[static_cast<std::size_t>(*row)];
                if (!native.contains("layer_definition"))
                    continue;
                const auto &definition = native["layer_definition"];
                sources.push_back({{"native_record_index", *row},
                                   {"stream", native.value("stream", Json::array())},
                                   {"offset", native.value("offset", Json())},
                                   {"layer_id", definition.value("layer_id", Json())},
                                   {"name", definition.value("name", Json())},
                                   {"initial_name", definition.value("initial_name", Json())},
                                   {"description", definition.value("description", Json())}});
            }
            return sources;
        };
        for (const auto &group : layer_tables["group_states"]) {
            if (!group.contains("layers") || !group["layers"].is_array())
                continue;
            for (const auto &layer : group["layers"]) {
                const auto id = unsigned_integer(layer.contains("layer_id") ? &layer["layer_id"] : nullptr);
                if (!id)
                    continue;
                Json state = {{"layer_id", *id},
                              {"group_table_index", group.value("group_table_index", Json())},
                              {"file_table_indices", group.value("file_table_indices", Json::array())},
                              {"file_record_indices", layer.value("file_record_indices", Json::array())},
                              {"group_record_indices", layer.value("group_record_indices", Json::array())},
                              {"properties", layer.value("properties", Json::object())},
                              {"status", layer.value("status", std::string("unavailable"))}};
                state["file_layer_records"] = layer_source_records(state["file_record_indices"]);
                state["group_layer_records"] = layer_source_records(state["group_record_indices"]);
                native_layer_index[*id].push_back(std::move(state));
            }
        }
    }

    for (const auto &record : records) {
        const auto &fields = record["fields"];
        if (record["record_kind"] == "subcategory_info" && fields.contains("LocationIndex") &&
            fields["LocationIndex"].is_string())
            subcategory_index[fields["LocationIndex"].get<std::string>()].push_back(record);
        else if (record["record_kind"] == "subcategory_node_name" &&
                 fields.contains("LocationIndex") && fields["LocationIndex"].is_string())
            node_index[fields["LocationIndex"].get<std::string>()].push_back(record);
        else if (record["record_kind"] == "view_config" && fields.contains("Name") &&
                 fields["Name"].is_string())
            view_config_index[fields["Name"].get<std::string>()].push_back(record);
        else if (record["record_kind"] == "view_config_list") {
            if (fields.contains("ConfigNames") && fields["ConfigNames"].is_array())
                for (const auto &name : fields["ConfigNames"])
                    if (name.is_string())
                        config_list_membership_index[name.get<std::string>()].push_back(record);
            if (fields.contains("ActiveConfigName") && fields["ActiveConfigName"].is_string())
                active_config_list_index[fields["ActiveConfigName"].get<std::string>()]
                    .push_back(record);
        }
    }

    auto name_reference = [&](const Json &source_name) {
        Json matches = Json::array();
        if (source_name.is_string()) {
            const auto found = view_config_index.find(source_name.get<std::string>());
            if (found != view_config_index.end())
                for (const auto &config : found->second)
                    matches.push_back(record_reference(config));
        }
        const auto status = !source_name.is_string() ? "source_name_unavailable"
                           : matches.empty() ? "missing"
                           : matches.size() == 1 ? "resolved"
                                                 : "ambiguous";
        return Json{{"status", status}, {"match", "exact_config_name"},
                    {"source_name", source_name}, {"records", std::move(matches)}};
    };

    for (auto &record : records) {
        if (record["record_kind"] != "layer_pix_relation")
            continue;
        const auto id = unsigned_integer(record["fields"].contains("LayerId")
                                             ? &record["fields"]["LayerId"]
                                             : nullptr);
        Json candidates = Json::array();
        if (id) {
            const auto found = native_layer_index.find(*id);
            if (found != native_layer_index.end())
                for (const auto &state : found->second)
                    candidates.push_back(state);
        }
        record["native_layer_state"] = {
            {"status", !id ? "layer_id_unavailable"
                            : candidates.empty() ? "missing"
                                                 : candidates.size() == 1 ? "resolved"
                                                                          : "ambiguous"},
            {"match", "exact_layer_id"}, {"layer_id", id ? Json(*id) : Json()},
            {"records", std::move(candidates)}};
    }

    for (const auto &record : records)
        if (record["record_kind"] == "layer_pix_relation" && record["fields"].contains("PixType") &&
            record["fields"]["PixType"].is_string())
            layer_pix_index[record["fields"]["PixType"].get<std::string>()].push_back(record);

    for (const auto &record : records) {
        const auto &fields = record["fields"];
        if (record["record_kind"] == "style_definition") {
            const auto id = unsigned_integer(fields.contains("StyleId") ? &fields["StyleId"] : nullptr);
            if (id)
                style_definition_index[*id].push_back(record);
        } else if (record["record_kind"] == "style_cache") {
            const auto id = unsigned_integer(fields.contains("StyleId") ? &fields["StyleId"] : nullptr);
            if (id)
                style_cache_index[*id].push_back(record);
        } else if (record["record_kind"] == "display_cache") {
            const auto id = unsigned_integer(fields.contains("DisplayId") ? &fields["DisplayId"] : nullptr);
            if (id)
                display_cache_index[*id].push_back(record);
        }
    }

    auto id_candidates = [](const std::map<std::uint64_t, std::vector<Json>> &index,
                            const Json *id_value, const Json *domain_value,
                            const std::string &id_name, const std::string &domain_name) {
        const auto id = unsigned_integer(id_value);
        Json candidates = Json::array();
        if (id) {
            const auto found = index.find(*id);
            if (found != index.end())
                for (const auto &record : found->second)
                    candidates.push_back(record_reference(record));
        }
        return Json{{"status", !id ? "source_id_unavailable"
                                    : candidates.empty() ? "missing"
                                                         : "candidate_by_id"},
                    {"match", "exact_numeric_id"},
                    {"id_field", id_name},
                    {"id", id ? Json(*id) : Json()},
                    {"domain_field", domain_name},
                    {"domain", domain_value ? *domain_value : Json()},
                    {"domain_comparison", "not_evaluated"},
                    {"records", std::move(candidates)}};
    };

    for (auto &record : records) {
        const auto kind = record.value("record_kind", std::string{});
        if (kind == "model_view") {
            const auto &fields = record["fields"];
            record["model_id_reference"] = model_reference(
                models, fields.contains("ModelId") ? fields["ModelId"] : Json());
            record["associated_model_references"] =
                model_reference_list(models, fields, "AssociatedModels");
            record["visible_model_references"] =
                model_reference_list(models, fields, "VisibleModels");
        } else if (kind == "view_config_list") {
            const auto &fields = record["fields"];
            Json references = Json::array();
            if (fields.contains("ConfigNames") && fields["ConfigNames"].is_array())
                for (const auto &name : fields["ConfigNames"])
                    references.push_back(name_reference(name));
            record["config_name_references"] = std::move(references);
            record["active_config_reference"] = name_reference(
                fields.contains("ActiveConfigName") ? fields["ActiveConfigName"] : Json());
        } else if (kind == "view_config") {
            const auto &fields = record["fields"];
            const auto name = fields.contains("Name") && fields["Name"].is_string()
                                  ? fields["Name"].get<std::string>()
                                  : std::string{};
            Json listed_by = Json::array(), active_in = Json::array();
            if (const auto found = config_list_membership_index.find(name);
                found != config_list_membership_index.end())
                for (const auto &candidate : found->second)
                    listed_by.push_back(record_reference(candidate));
            if (const auto found = active_config_list_index.find(name);
                found != active_config_list_index.end())
                for (const auto &candidate : found->second)
                    active_in.push_back(record_reference(candidate));
            record["listed_by_view_config_lists"] = std::move(listed_by);
            record["active_in_view_config_lists"] = std::move(active_in);
        } else if (kind == "line_style" || kind == "hatch_style" || kind == "layer_pix_relation") {
            const auto &fields = record["fields"];
            if (fields.contains("PixType")) {
                const auto parsed = parse_location_index(fields["PixType"]);
                add_path_context(record, parsed, subcategory_index, node_index, true);
                if (kind != "layer_pix_relation")
                    add_layer_pix_references(record, fields["PixType"], layer_pix_index);
            }
        } else if (kind == "subcategory_info" || kind == "subcategory_node_name") {
            const auto &fields = record["fields"];
            if (fields.contains("LocationIndex"))
                add_path_context(record, parse_location_index(fields["LocationIndex"]),
                                 subcategory_index, node_index, false);
        } else if (kind == "display_cache") {
            for (auto &status : record["display_statuses"])
                if (status.contains("subcategory_location_index"))
                    add_path_context(status, status["subcategory_location_index"],
                                     subcategory_index, node_index, false);
        } else if (kind == "style_definition") {
            for (const char *name : {"line_components", "hatch_components"})
                for (auto &component : record[name]) {
                    const auto &fields = component["fields"];
                    if (!fields.contains("PixType"))
                        continue;
                    add_path_context(component, parse_location_index(fields["PixType"]),
                                     subcategory_index, node_index, true);
                    add_layer_pix_references(component, fields["PixType"], layer_pix_index);
                }
            for (auto &status : record["visibility_statuses"])
                if (status.contains("subcategory_location_index"))
                    add_path_context(status, status["subcategory_location_index"],
                                     subcategory_index, node_index, false);
        } else if (kind == "style_cache") {
            const auto &fields = record["fields"];
            const auto id = unsigned_integer(fields.contains("StyleId") ? &fields["StyleId"] : nullptr);
            Json matches = Json::array();
            if (id) {
                const auto found = style_definition_index.find(*id);
                if (found != style_definition_index.end())
                    for (const auto &definition : found->second)
                        matches.push_back(record_reference(definition));
            }
            record["style_definition_group"] = {
                {"status", !id ? "source_id_unavailable" : matches.empty() ? "missing" : "matched"},
                {"match", "exact_StyleId"}, {"style_id", id ? Json(*id) : Json()},
                {"records", std::move(matches)}};
        } else if (kind == "group_relation") {
            const auto &fields = record["fields"];
            record["display_cache_candidates"] = id_candidates(
                display_cache_index,
                fields.contains("DisplayId") ? &fields["DisplayId"] : nullptr,
                fields.contains("DisplayDomain") ? &fields["DisplayDomain"] : nullptr,
                "DisplayId", "DisplayDomain");
            record["style_cache_candidates"] = id_candidates(
                style_cache_index,
                fields.contains("StyleId") ? &fields["StyleId"] : nullptr,
                fields.contains("StyleDomain") ? &fields["StyleDomain"] : nullptr,
                "StyleId", "StyleDomain");
        }
    }
    return {{"status", "decoded"},
            {"records", std::move(records)},
            {"relationship_policy", "PixType, LayerId, category locations, StyleId, and group relation IDs use exact keys or explicit path-prefix matches; unresolved domains are retained without merging"},
            {"path_policy", "hyphen-separated unsigned numeric components are exposed lexically without assigning component meanings"}};
}

} // namespace p3d
