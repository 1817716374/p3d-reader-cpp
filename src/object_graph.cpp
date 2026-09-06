#include "internal.hpp"
#include <deque>
namespace p3d {
static std::string key(const Json &a, const Json &b) {
    return a.dump() + ":" + b.dump();
}
static Json fields(const Json &root, bool full = true) {
    Json out = Json::object();
    std::map<std::string, unsigned> count;
    if (root.contains("children")) {
        for (auto &c : root["children"])
            count[c["name"]]++;
        for (auto &c : root["children"]) {
            auto name = c["name"].get<std::string>();
            if (!full && name != "Guid" && name != "ModelId" && name != "StoreyGuid" &&
                name != "StoreyListGuid" && name.rfind("RelData", 0) != 0 &&
                name.rfind("RelModelData", 0) != 0)
                continue;
            if (count[name] == 1 && c.contains("value"))
                out[name] = c["value"];
        }
    }
    return out;
}
static Json resolution(const Json &candidates) {
    return {{"status", candidates.size() == 1 ? "resolved"
                       : candidates.empty()   ? "missing"
                                              : "ambiguous"},
            {"candidates", candidates}};
}
static void index_add(Json &index, const std::string &key, const Json &value) {
    if (!index.contains(key))
        index[key] = Json::array();
    index[key].push_back(value);
}
static std::string uppercase(std::string s) {
    for (auto &c : s)
        if (c >= 'a' && c <= 'z')
            c -= 32;
    return s;
}
static Json cycles(const Json &nodes, const Json &edges) {
    std::map<std::string, std::size_t> indegree;
    std::map<std::string, std::vector<std::string>> children;
    std::vector<std::string> order;
    for (auto &n : nodes) {
        auto k = n["key"].get<std::string>();
        indegree[k] = 0;
        order.push_back(k);
    }
    for (auto &e : edges)
        if (e["status"] == "resolved") {
            auto parent = e["candidates"][0].get<std::string>(),
                 child = e["child"].get<std::string>();
            children[parent].push_back(child);
            indegree[child]++;
        }
    std::deque<std::string> queue;
    for (auto &k : order)
        if (!indegree[k])
            queue.push_back(k);
    while (!queue.empty()) {
        auto p = queue.front();
        queue.pop_front();
        for (auto &c : children[p])
            if (--indegree[c] == 0)
                queue.push_back(c);
    }
    Json out = Json::array();
    for (auto &k : order)
        if (indegree[k])
            out.push_back(k);
    return out;
}
Json build_graph_records(const Json &objects, const Json &bindings, const Json &relationships,
                         const std::set<std::uint64_t> &models, const std::set<std::string> &live) {
    Json catalog = Json::array(), index = Json::object(), record_index = Json::object();
    std::map<std::string, std::vector<std::size_t>> grouped;
    std::vector<std::string> order, canonical;
    std::map<std::string, std::size_t> first;
    for (auto &obj : objects) {
        auto k = key(obj["class_id"], obj["object_id"]);
        auto &group = grouped[k];
        if (group.empty())
            order.push_back(k);
        auto rk = k + "@" + std::to_string(group.size());
        Json source = {{"stream", obj["stream"]},
                       {"offset", obj["offset"]},
                       {"length", obj["length"]},
                       {"prefix_hex", hex(bytesof(obj["prefix"]))}};
        Json row = {{"record_key", rk},
                    {"object_key", k},
                    {"class_id", obj["class_id"]},
                    {"object_id", obj["object_id"]},
                    {"class", obj["root"]["name"]},
                    {"source", source}};
        row["property_tree"] = obj["root"];
        record_index[rk] = catalog.size();
        group.push_back(catalog.size());
        catalog.push_back(std::move(row));
    }
    for (auto &k : order) {
        auto &rows = grouped[k];
        bool same = true;
        Json copies = Json::array();
        for (auto i : rows) {
            if (i != rows[0])
                same &= catalog[i]["property_tree"] == catalog[rows[0]]["property_tree"];
            copies.push_back(catalog[i]["record_key"]);
        }
        index[k] = {{"status", rows.size() == 1 ? "resolved"
                               : same           ? "identical_copies"
                                                : "ambiguous"},
                    {"records", copies}};
        if (same) {
            canonical.push_back(k);
            first[k] = rows[0];
        }
    }
    auto object_ref = [&](const std::string &k) {
        auto o = index.value(k, Json{{"status", "missing"}, {"records", Json::array()}});
        o["object_key"] = k;
        return o;
    };
    Json binding_rows = Json::array(), binding_index = Json::object(),
         relation_rows = Json::array();
    for (std::size_t i = 0; i < bindings.size(); ++i) {
        auto b = bindings[i];
        auto ek = key(b["model_id"], b["element_id"]), rk = "binding:" + std::to_string(i);
        b.update({{"key", rk},
                  {"element_key", ek},
                  {"object", object_ref(key(b["schema_id"], b["object_id"]))},
                  {"element_status", live.count(ek) ? "resolved" : "missing"}});
        binding_rows.push_back(b);
        index_add(binding_index, ek, rk);
    }
    for (std::size_t i = 0; i < relationships.size(); ++i) {
        auto row = relationships[i];
        row["key"] = "relationship:" + std::to_string(i);
        for (std::string side : {"source", "target"})
            row[side + "_object"] =
                object_ref(key(row[side + "_class_id"], row[side + "_object_id"]));
        relation_rows.push_back(row);
    }
    Json tree_nodes = Json::array(), model_nodes = Json::array(), incomplete = Json::array(),
         tree_index = Json::object(), names = Json::object(), guid_index = Json::object(),
         qualified = Json::object(), reference_rows = Json::array();
    std::map<std::string, Json> fs;
    for (auto &k : canonical) {
        auto &row = catalog[first[k]];
        auto cls = row["class"].get<std::string>();
        auto f =
            fields(row["property_tree"], cls == "BimBaseTreeNodeData" || cls == "BPModelTreeSet" ||
                                             cls == "BPModelTreeItem");
        fs[k] = fields(row["property_tree"], false);
        auto guid = f.value("Guid", Json());
        if (guid.is_string() && !guid.get<std::string>().empty())
            index_add(guid_index, uppercase(guid.get<std::string>()), k);
        auto ns = row["property_tree"].value("attributes", Json::object()).value("xmlns", Json());
        index_add(qualified, Json::array({row["object_id"], cls, ns}).dump(), k);
        if (cls == "BimBaseTreeNodeData") {
            Json node = {{"key", k},
                         {"object", object_ref(k)},
                         {"fields", f},
                         {"name", f.value("NodeName", Json())},
                         {"tree_id", f.value("TreeId", Json())}};
            if (!f.value("TreeId", Json()).is_number_integer() ||
                !f.value("ParentNodeId", Json()).is_number_integer()) {
                node["status"] = "missing_identity_fields";
                incomplete.push_back(k);
            } else {
                node["node_id"] = f.value("NodeId", row["object_id"]);
                if (!node["node_id"].is_number_integer()) {
                    node["status"] = "invalid_node_id";
                    incomplete.push_back(k);
                } else {
                    node["identity_basis"] =
                        f.contains("NodeId") ? "NodeId" : "object_id_observed_legacy_variant";
                    node["status"] = "identified";
                    index_add(tree_index,
                              Json::array({row["class_id"], f["TreeId"], node["node_id"]}).dump(),
                              k);
                }
            }
            tree_nodes.push_back(node);
        } else if (cls == "BPModelTreeSet" || cls == "BPModelTreeItem") {
            Json node = {{"key", k},
                         {"object", object_ref(k)},
                         {"class", cls},
                         {"fields", f},
                         {"name", f.value("DispalyName", f.value("Name", Json()))}};
            if (f.contains("ModelId"))
                node["model_reference"] = {
                    {"model_id", f["ModelId"]},
                    {"status", f["ModelId"].is_number_integer() &&
                                       models.count(f["ModelId"].get<std::uint64_t>())
                                   ? "resolved"
                                   : "missing"}};
            model_nodes.push_back(node);
            if (f.value("Name", Json()).is_string())
                index_add(names, Json::array({cls, f["Name"]}).dump(), k);
        }
    }
    Json edges = Json::array(), roots = Json::array(), element_refs = Json::array();
    for (auto &node : tree_nodes) {
        if (node["status"] != "identified")
            continue;
        auto k = node["key"].get<std::string>();
        auto &row = catalog[first[k]];
        auto &f = node["fields"];
        auto parent = f["ParentNodeId"];
        auto candidates = tree_index.value(
            Json::array({row["class_id"], f["TreeId"], parent}).dump(), Json::array());
        if (candidates.empty() && parent == 65534)
            roots.push_back({{"node", k}, {"basis", "observed_parent_sentinel_65534"}});
        else {
            auto edge = resolution(candidates);
            edge.update({{"kind", "source_tree_parent"},
                         {"child", k},
                         {"parent_id", parent},
                         {"tree_id", f["TreeId"]},
                         {"basis", node["identity_basis"]}});
            edges.push_back(edge);
        }
        auto resource = f.value("ResourceRefers", Json::array());
        if (f.value("NodeType", Json()) == 11 && resource.is_array() && resource.size() > 3 &&
            resource[3].is_string()) {
            std::stringstream ss(resource[3].get<std::string>());
            std::string token;
            while (std::getline(ss, token, ',')) {
                if (token.empty())
                    continue;
                bool valid = std::all_of(token.begin(), token.end(),
                                         [](auto c) { return c >= '0' && c <= '9'; });
                std::uint64_t packed = 0;
                try {
                    require(valid, "packed token");
                    packed = std::stoull(token);
                } catch (...) {
                    element_refs.push_back({{"node", k},
                                            {"raw_value", token},
                                            {"status", "unsupported_reference"},
                                            {"source_slot", 3}});
                    continue;
                }
                auto ek = key(packed >> 32, packed & 0xffffffff);
                element_refs.push_back({{"node", k},
                                        {"raw_value", token},
                                        {"source_slot", 3},
                                        {"basis", "observed_packed_model_element_id"},
                                        {"model_id", packed >> 32},
                                        {"element_id", packed & 0xffffffff},
                                        {"element_key", ek},
                                        {"status", live.count(ek) ? "resolved" : "missing"},
                                        {"bindings", binding_index.value(ek, Json::array())}});
            }
        }
    }
    Json model_edges = Json::array();
    for (auto &node : model_nodes)
        for (auto pair : std::vector<std::pair<std::string, std::string>>{
                 {"ModelSetChilds", "BPModelTreeSet"}, {"ModelItems", "BPModelTreeItem"}}) {
            auto children = node["fields"].value(pair.first, Json::array());
            if (!children.is_array())
                continue;
            for (std::size_t i = 0; i < children.size(); ++i) {
                auto &name = children[i];
                auto e = resolution(
                    name.is_string()
                        ? names.value(Json::array({pair.second, name}).dump(), Json::array())
                        : Json::array());
                e.update({{"kind", "model_tree_child"},
                          {"parent", node["key"]},
                          {"field", pair.first},
                          {"target_class", pair.second},
                          {"ordinal", i},
                          {"source_name", name}});
                model_edges.push_back(e);
            }
        }
    for (auto &k : canonical) {
        auto &row = catalog[first[k]];
        auto &f = fs[k];
        auto cls = row["class"].get<std::string>();
        if (cls == "PBModelInfo" || cls == "PBStoreyModelViewInfo" || cls == "PBSheetModelInfo") {
            for (auto field : {"StoreyGuid", "StoreyListGuid"}) {
                auto guid = f.value(field, Json());
                if (guid.is_string() && !guid.get<std::string>().empty()) {
                    auto rr = resolution(
                        guid_index.value(uppercase(guid.get<std::string>()), Json::array()));
                    rr.update({{"source", k},
                               {"field", field},
                               {"raw_value", guid},
                               {"kind", "guid_reference"}});
                    reference_rows.push_back(rr);
                }
            }
            if (f.value("ModelId", Json()).is_number_integer())
                reference_rows.push_back(
                    {{"source", k},
                     {"field", "ModelId"},
                     {"raw_value", f["ModelId"]},
                     {"kind", "model_reference"},
                     {"status",
                      models.count(f["ModelId"].get<std::uint64_t>()) ? "resolved" : "missing"},
                     {"model_id", f["ModelId"]}});
        }
        if (cls == "BimBaseTreeNodeData")
            for (std::string prefix : {"RelData", "RelModelData"}) {
                auto oid = f.value(prefix + "Id", Json()),
                     cl = f.value(prefix + "ClassName", Json()),
                     ns = f.value(prefix + "SchemaName", Json());
                if (!oid.is_number_integer() || !cl.is_string() || cl.get<std::string>().empty() ||
                    !ns.is_string() || ns.get<std::string>().empty())
                    continue;
                auto pg = f.value(prefix + "ProjectGUID", Json());
                bool local = pg.is_null() || (pg.is_string() && pg.get<std::string>().empty());
                auto rr = local ? resolution(qualified.value(Json::array({oid, cl, ns}).dump(),
                                                             Json::array()))
                                : Json{{"status", "project_scope_unverified"},
                                       {"candidates", Json::array()}};
                rr.update({{"source", k},
                           {"field", prefix},
                           {"kind", "qualified_object_reference"},
                           {"object_id", oid},
                           {"class_name", cl},
                           {"schema_name", ns},
                           {"project_guid", pg}});
                reference_rows.push_back(rr);
            }
    }
    Json model_cycle_edges = Json::array(), model_roots = Json::array(), ambiguous = Json::array();
    std::set<std::string> children;
    for (auto &e : model_edges)
        if (e["status"] == "resolved") {
            model_cycle_edges.push_back({{"child", e["candidates"][0]},
                                         {"candidates", Json::array({e["parent"]})},
                                         {"status", "resolved"}});
            children.insert(e["candidates"][0].get<std::string>());
        }
    for (auto &n : model_nodes)
        if (!children.count(n["key"].get<std::string>()))
            model_roots.push_back(n["key"]);
    for (auto &k : order)
        if (index[k]["status"] == "ambiguous")
            ambiguous.push_back(k);
    Json hierarchy = {{"nodes", tree_nodes},
                      {"parent_edges", edges},
                      {"roots", roots},
                      {"element_references", element_refs},
                      {"model_nodes", model_nodes},
                      {"model_edges", model_edges},
                      {"incomplete_nodes", incomplete},
                      {"model_roots", model_roots},
                      {"cycle_affected_nodes", cycles(tree_nodes, edges)},
                      {"model_cycle_affected_nodes", cycles(model_nodes, model_cycle_edges)},
                      {"ambiguous_objects", ambiguous},
                      {"policy", "Separate source trees and model trees; preserve ambiguous, "
                                 "missing and incomplete nodes. No visibility filtering."}};
    Json model_props = Json::object();
    auto add_unique = [&](const std::string &model, const std::string &k) {
        if (!model_props.contains(model))
            model_props[model] = Json::array();
        auto &a = model_props[model];
        if (std::find(a.begin(), a.end(), Json(k)) == a.end())
            a.push_back(k);
    };
    for (auto &b : binding_rows) {
        auto k = b["object"]["object_key"].get<std::string>();
        if (!first.count(k))
            continue;
        auto cl = catalog[first[k]]["class"].get<std::string>();
        if (cl == "PBModelInfo" || cl == "PBSheetModelInfo" || cl == "PBStoreyModelViewInfo")
            add_unique(b["model_id"].dump(), k);
    }
    for (auto &k : canonical)
        if (catalog[first[k]]["class"] == "PBStoreyModelViewInfo" &&
            fs[k].value("ModelId", Json()).is_number_integer())
            add_unique(fs[k]["ModelId"].dump(), k);
    Json element_index = Json::object(), model_node_index = Json::object(),
         relation_index = Json::object(), reference_index = Json::object();
    auto element_entry = [&](const std::string &k) -> Json & {
        if (!element_index.contains(k))
            element_index[k] = {{"bindings", Json::array()}, {"tree_nodes", Json::array()}};
        return element_index[k];
    };
    for (std::size_t i = 0; i < binding_rows.size(); ++i)
        element_entry(binding_rows[i]["element_key"].get<std::string>())["bindings"].push_back(i);
    for (auto &r : element_refs)
        if (r.contains("element_key"))
            element_entry(r["element_key"].get<std::string>())["tree_nodes"].push_back(r["node"]);
    for (auto &n : model_nodes)
        if (n["fields"].contains("ModelId"))
            index_add(model_node_index, n["fields"]["ModelId"].dump(), n["key"]);
    for (std::size_t i = 0; i < relation_rows.size(); ++i) {
        std::set<std::string> ks = {
            relation_rows[i]["source_object"]["object_key"].get<std::string>(),
            relation_rows[i]["target_object"]["object_key"].get<std::string>()};
        for (auto &k : ks)
            index_add(relation_index, k, i);
    }
    for (std::size_t i = 0; i < reference_rows.size(); ++i)
        index_add(reference_index, reference_rows[i]["source"].get<std::string>(), i);
    Json result = Json::object();
    result["objects"] = std::move(catalog);
    result["object_index"] = std::move(index);
    result["bindings"] = std::move(binding_rows);
    result["record_index"] = std::move(record_index);
    result["binding_index"] = std::move(binding_index);
    result["relationships"] = std::move(relation_rows);
    result["element_index"] = std::move(element_index);
    result["model_node_index"] = std::move(model_node_index);
    result["relationship_index"] = std::move(relation_index);
    result["reference_index"] = std::move(reference_index);
    result["model_property_objects"] = std::move(model_props);
    result["object_references"] = std::move(reference_rows);
    result["hierarchy"] = std::move(hierarchy);
    result["identity_scope"] =
        "Keys are local to this document; namespace by source document when combining files.";
    result["property_policy"] =
        "Full source trees retained, including unbound objects and identical/conflicting copies. "
        "Related properties are separate objects, never silently inherited.";
    return result;
}
Json build_object_graph(const Document &doc) {
    std::set<std::string> live;
    std::set<std::uint64_t> models;
    std::map<std::string, std::uint64_t> model_storage;
    for (auto it = doc.index().begin(); it != doc.index().end(); ++it)
        if (it.key()[0] == '#' && it.key().find('%') == std::string::npos) {
            auto id = std::stoull(it.key().substr(1), nullptr, 16);
            models.insert(id);
            model_storage[it.value()] = id;
        }
    for (auto &n : doc.native_records()) {
        auto s = n["stream"].get<StreamPath>();
        if (s.size() == 4 && s[0] == doc.index().value("P3D-SM", std::string()) &&
            s[2] == doc.index().value("P3D-SMG", std::string()) && model_storage.count(s[1]))
            live.insert(key(model_storage[s[1]], n["id"]));
    }
    return build_graph_records(doc.objects(), doc.bindings(), doc.relationships(), models, live);
}
} // namespace p3d
