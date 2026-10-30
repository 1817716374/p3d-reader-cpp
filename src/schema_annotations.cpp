#include "internal.hpp"

namespace p3d {
namespace {
using EntityKey = std::pair<std::string, std::string>;

std::string local_name(const Json &node) {
    auto tag = node.value("tag", std::string());
    auto end = tag.find('}');
    return end == std::string::npos ? tag : tag.substr(end + 1);
}

struct EntityDefinition {
    Json attributes = Json::object();
    std::vector<std::string> bases;
    std::map<std::string, std::vector<Json>> properties;
};

class SchemaIndex {
  public:
    explicit SchemaIndex(const Json &schemas) {
        for (const auto &schema : schemas) {
            const auto attrs = schema.value("schema_attributes", Json::object());
            const auto name = attrs.value("schemaname", std::string());
            if (name.empty() || !schema.contains("tree"))
                continue;
            const auto version = attrs.value("version", std::string());
            const auto key = version.empty() ? name : name + "." + version;
            schema_attributes_[key] = attrs;
            const auto prefix = attrs.value("namespaceprefix", std::string());
            if (!prefix.empty())
                prefixes_[prefix].insert(key);
            collect(schema.at("tree"), key);
        }
    }

    void annotate_objects(Json &objects) const {
        if (!objects.is_array())
            return;
        for (auto &object : objects) {
            if (!object.contains("root") || !object["root"].is_object())
                continue;
            auto &root = object["root"];
            const auto xml_namespace =
                root.value("attributes", Json::object()).value("xmlns", std::string());
            if (xml_namespace.empty())
                continue;
            annotate_instance(root, {xml_namespace, root.value("name", std::string())}, 0, {});
        }
    }

  private:
    std::map<std::string, Json> schema_attributes_;
    std::map<std::string, std::set<std::string>> prefixes_;
    std::map<EntityKey, std::vector<EntityDefinition>> entities_;

    void collect(const Json &node, const std::string &schema_key) {
        if (!node.is_object())
            return;
        const auto tag = local_name(node);
        if ((tag == "PKPMEntity" || tag == "PKPMRelationshipEntity") &&
            node.contains("attributes")) {
            const auto attrs = node.at("attributes");
            const auto name = attrs.value("entityname", std::string());
            if (!name.empty()) {
                EntityDefinition entity;
                entity.attributes = attrs;
                if (node.contains("children"))
                    for (const auto &child : node.at("children")) {
                        const auto child_tag = local_name(child);
                        if (child_tag == "BaseClass") {
                            const auto base = child.value("text", std::string());
                            if (!base.empty())
                                entity.bases.push_back(base);
                        } else if ((child_tag == "PKPMProperty" ||
                                    child_tag == "PKPMStructProperty" ||
                                    child_tag == "PKPMArrayProperty") &&
                                   child.contains("attributes")) {
                            auto property = child.at("attributes");
                            const auto property_name =
                                property.value("propertyname", std::string());
                            if (!property_name.empty()) {
                                property["declaration_tag"] = child_tag;
                                property["owner_entity"] = name;
                                property["schema_name"] = schema_key;
                                if (child.contains("children") && !child.at("children").empty())
                                    property["declaration_children"] = child.at("children");
                                entity.properties[property_name].push_back(std::move(property));
                            }
                        }
                    }
                entities_[{schema_key, name}].push_back(std::move(entity));
            }
        }
        if (node.contains("children"))
            for (const auto &child : node.at("children"))
                collect(child, schema_key);
    }

    std::optional<EntityKey> resolve_reference(const std::string &current_schema,
                                               const std::string &reference) const {
        if (reference.empty())
            return {};
        const auto separator = reference.find(':');
        if (separator != std::string::npos) {
            const auto prefix = reference.substr(0, separator);
            auto found = prefixes_.find(prefix);
            if (found == prefixes_.end() || found->second.size() != 1)
                return {};
            return EntityKey{*found->second.begin(), reference.substr(separator + 1)};
        }
        EntityKey exact{current_schema, reference};
        if (entities_.count(exact))
            return exact;
        std::optional<EntityKey> candidate;
        for (const auto &entry : entities_)
            if (entry.first.second == reference) {
                if (candidate)
                    return {};
                candidate = entry.first;
            }
        return candidate;
    }

    const EntityDefinition *unique_entity(const EntityKey &key) const {
        auto found = entities_.find(key);
        if (found == entities_.end() || found->second.empty())
            return nullptr;
        const auto &first = found->second.front();
        for (std::size_t i = 1; i < found->second.size(); ++i) {
            const auto &other = found->second[i];
            if (other.attributes != first.attributes || other.bases != first.bases ||
                other.properties != first.properties)
                return nullptr;
        }
        return &first;
    }

    using PropertyMap = std::map<std::string, std::vector<Json>>;

    static void append_unique(std::vector<Json> &target, const std::vector<Json> &source) {
        for (const auto &candidate : source)
            if (std::find(target.begin(), target.end(), candidate) == target.end())
                target.push_back(candidate);
    }

    PropertyMap resolve_properties(const EntityKey &key, unsigned depth,
                                   std::set<EntityKey> active) const {
        if (depth > 64 || active.count(key))
            return {};
        active.insert(key);
        const auto *entity = unique_entity(key);
        if (!entity)
            return {};
        PropertyMap result;
        for (const auto &base : entity->bases) {
            auto base_key = resolve_reference(key.first, base);
            if (!base_key)
                continue;
            auto inherited = resolve_properties(*base_key, depth + 1, active);
            for (const auto &property : inherited)
                append_unique(result[property.first], property.second);
        }
        for (const auto &property : entity->properties)
            result[property.first] = property.second;
        return result;
    }

    Json entity_annotation(const EntityKey &key, const EntityDefinition &entity) const {
        const auto schema = schema_attributes_.find(key.first);
        Json bases = Json::array();
        for (const auto &base : entity.bases)
            bases.push_back(base);
        Json out = {{"schema_name", key.first},
                    {"entity_name", key.second},
                    {"entity_attributes", entity.attributes},
                    {"base_classes", std::move(bases)}};
        if (schema != schema_attributes_.end())
            out["schema_attributes"] = schema->second;
        return out;
    }

    std::optional<EntityKey> property_type(const EntityKey &owner,
                                          const Json &property) const {
        const auto type = property.value("propertytype", std::string());
        if (type.empty())
            return {};
        return resolve_reference(owner.first, type);
    }

    void annotate_instance(Json &node, const EntityKey &key, unsigned depth,
                           std::set<EntityKey> active) const {
        if (depth > 64 || active.count(key))
            return;
        const auto *entity = unique_entity(key);
        if (!entity)
            return;
        node["schema_entity"] = entity_annotation(key, *entity);
        active.insert(key);
        const auto properties = resolve_properties(key, 0, {});
        if (!node.contains("children") || !node["children"].is_array())
            return;
        for (auto &child : node["children"]) {
            const auto name = child.value("name", std::string());
            auto found = properties.find(name);
            if (found == properties.end())
                continue;
            if (found->second.size() != 1) {
                Json candidates = Json::array();
                for (const auto &candidate : found->second)
                    candidates.push_back(candidate);
                child["schema_property_resolution"] =
                    {{"status", "ambiguous"}, {"candidates", std::move(candidates)}};
                continue;
            }
            const auto &property = found->second.front();
            child["schema_property"] = property;
            const auto declaration = property.value("declaration_tag", std::string());
            auto type_key = property_type(key, property);
            if (!type_key)
                continue;
            if (declaration == "PKPMStructProperty") {
                annotate_instance(child, *type_key, depth + 1, active);
            } else if (declaration == "PKPMArrayProperty" && child.contains("children") &&
                       child["children"].is_array()) {
                const auto target_name = type_key->second;
                for (auto &item : child["children"])
                    if (item.is_object() &&
                        (item.value("name", std::string()) == target_name ||
                         item.contains("children")))
                        annotate_instance(item, *type_key, depth + 1, active);
            }
        }
    }
};
} // namespace

void annotate_objects_from_schemas(Json &objects, const Json &schemas) {
    SchemaIndex(schemas).annotate_objects(objects);
}
} // namespace p3d
