#include "internal.hpp"
namespace p3d {
Json Document::binary_fields() const {
    Json out = Json::array();
    for (auto &obj : objects()) {
        std::function<void(const Json &, std::string)> walk = [&](const Json &n, std::string path) {
            auto v = n.value("value", Json());
            if (v.is_object() && v.contains("binary_base64"))
                out.push_back({{"class_id", obj["class_id"]},
                               {"object_id", obj["object_id"]},
                               {"class_name", obj["root"]["name"]},
                               {"stream", obj["stream"]},
                               {"object_offset", obj["offset"]},
                               {"path", path},
                               {"name", n["name"]},
                               {"value", v}});
            if (n.contains("children"))
                for (std::size_t i = 0; i < n["children"].size(); ++i) {
                    auto &c = n["children"][i];
                    walk(c,
                         path + "/" + c["name"].get<std::string>() + "[" + std::to_string(i) + "]");
                }
        };
        walk(obj["root"], "/" + obj["root"]["name"].get<std::string>());
    }
    return out;
}
Json Document::inline_materials() const {
    Json out = Json::array();
    for (auto &obj : objects()) {
        std::function<void(const Json &, std::string)> walk = [&](const Json &v, std::string path) {
            if (v.is_object()) {
                if (v.value("encoding", Json()) == "inline_material_v1")
                    out.push_back({{"class_id", obj["class_id"]},
                                   {"object_id", obj["object_id"]},
                                   {"stream", obj["stream"]},
                                   {"object_offset", obj["offset"]},
                                   {"decoded_path", path},
                                   {"material", v}});
                for (auto it = v.begin(); it != v.end(); ++it)
                    if (it.value().is_structured())
                        walk(it.value(), path + "." + it.key());
            } else if (v.is_array())
                for (std::size_t i = 0; i < v.size(); ++i)
                    if (v[i].is_structured())
                        walk(v[i], path + "[" + std::to_string(i) + "]");
        };
        walk(obj["root"], "$");
    }
    return out;
}
Json element_context(const Json &graph, std::uint64_t model, std::uint64_t element) {
    auto ek = std::to_string(model) + ":" + std::to_string(element);
    auto indexed = graph.at("element_index")
                       .value(ek, Json{{"bindings", Json::array()}, {"tree_nodes", Json::array()}});
    Json bindings = Json::array();
    std::set<std::string> objects, associated;
    for (auto &i : indexed["bindings"]) {
        auto b = graph.at("bindings").at(i.get<std::size_t>());
        objects.insert(b["object"]["object_key"].get<std::string>());
        bindings.push_back(b);
    }
    auto model_objects =
        graph.at("model_property_objects").value(std::to_string(model), Json::array());
    associated = objects;
    for (auto &k : model_objects)
        associated.insert(k.get<std::string>());
    std::set<std::size_t> refs, relations;
    for (auto &k : associated)
        for (auto &i : graph.at("reference_index").value(k, Json::array()))
            refs.insert(i.get<std::size_t>());
    for (auto &k : objects)
        for (auto &i : graph.at("relationship_index").value(k, Json::array()))
            relations.insert(i.get<std::size_t>());
    Json rr = Json::array(), rs = Json::array();
    for (auto i : refs)
        rr.push_back(graph.at("object_references").at(i));
    for (auto i : relations)
        rs.push_back(graph.at("relationships").at(i));
    return {{"element_key", ek},
            {"bindings", bindings},
            {"tree_nodes", indexed["tree_nodes"]},
            {"model_tree_nodes",
             graph.at("model_node_index").value(std::to_string(model), Json::array())},
            {"model_property_objects", model_objects},
            {"object_references", rr},
            {"relationships", rs}};
}
Json get_object(const Json &graph, std::uint64_t cls, std::uint64_t id) {
    auto k = std::to_string(cls) + ":" + std::to_string(id);
    auto e =
        graph.at("object_index").value(k, Json{{"status", "missing"}, {"records", Json::array()}});
    Json records = Json::array();
    for (auto &rk : e["records"])
        records.push_back(graph.at("objects").at(
            graph.at("record_index").at(rk.get<std::string>()).get<std::size_t>()));
    return {{"object_key", k}, {"status", e["status"]}, {"records", records}};
}
} // namespace p3d
