#include <p3d/csg.hpp>
#include "internal.hpp"

namespace p3d {
namespace {
Json identity_rows() {
    return {{1., 0., 0., 0.}, {0., 1., 0., 0.}, {0., 0., 1., 0.}};
}
bool known_node(const Json &node) {
    const auto status = node.value("status", std::string());
    return status == "decoded" || status == "decoded_with_trailing_bytes";
}
Json leaf_state(const Json &node, const Json &nodes) {
    bool unknown = false;
    for (const auto *side : {"left_index", "right_index"}) {
        const auto &index = node.at(side);
        if (index.is_null())
            continue;
        const auto &child = nodes.at(index.get<std::size_t>());
        if (known_node(child))
            return false;
        unknown = true;
    }
    return unknown ? Json() : Json(true);
}
Json node_matrix(const Json &node, const Json &archive) {
    const auto &indices = node.at("matrix_indices");
    Json out = {{"status", "identity_fallback"},
                {"matrix_3x4_rows", identity_rows()},
                {"selected_transform_index", nullptr},
                {"source_index", nullptr},
                {"ignored_later_indices", indices.empty() ? 0u : indices.size() - 1}};
    if (indices.empty()) {
        out["reason"] = "empty_index_list";
        return out;
    }
    const auto index = indices.at(0).get<std::int32_t>();
    out["source_index"] = index;
    const auto &transforms = archive.at("transforms");
    if (index < 0 || std::size_t(index) >= transforms.size()) {
        out["reason"] = "first_index_out_of_range";
        return out;
    }
    out["status"] = "selected";
    out["selected_transform_index"] = index;
    out["matrix_3x4_rows"] = transforms.at(index).at("matrix_3x4_rows");
    out["numeric_validation"] = "not_performed_by_native_accessor";
    return out;
}
void select(const Json &indices, const char *list, const Json &archive, std::size_t node_index,
            Json &out) {
    const auto count = archive.at(list).size();
    for (std::size_t position = 0; position < indices.size(); ++position) {
        const auto index = indices[position].get<std::int32_t>();
        Json item = {{"list", list},
                     {"source_index", index},
                     {"index_position", position},
                     {"node_index", node_index}};
        if (index < 0 || std::size_t(index) >= count) {
            item["reason"] = "source_index_out_of_range";
            out["skipped_sources"].push_back(std::move(item));
        } else {
            item["target_index"] = index;
            out["selected_sources"].push_back(std::move(item));
        }
    }
}
} // namespace
Json csg_node_geometry_input(const Json &archive, std::size_t node_index) {
    const auto &nodes = archive.at("tree").at("nodes");
    const auto &node = nodes.at(node_index);
    Json out = {{"scope", "stored_node_before_update"},
                {"node_index", node_index},
                {"status", "not_evaluated"},
                {"selected_sources", Json::array()},
                {"skipped_sources", Json::array()},
                {"geometry_included", nullptr},
                {"is_leaf", nullptr},
                {"node_transform", nullptr},
                {"geometry_object_restore_status", "not_evaluated"},
                {"cache_validity_status", "not_evaluated"}};
    if (!known_node(node)) {
        out["reason"] = "node_layout_not_available";
        return out;
    }
    out["node_transform"] = node_matrix(node, archive);
    out["is_leaf"] = leaf_state(node, nodes);
    // getGeometriesFinal appends caches first, independently of isOld.
    select(node.at("cache_indices"), "node_caches", archive, node_index, out);
    const auto operation = node.at("operation").get<std::int32_t>();
    const auto old = node.at("is_old_value").get<unsigned>();
    if (std::uint32_t(operation) > 1 || old != 0 || out["is_leaf"] == false) {
        out["geometry_included"] = true;
    } else if (out["is_leaf"] == true) {
        out["geometry_included"] = false;
    } else {
        out["reason"] = "child_presence_not_established";
        out["status"] = "partial";
        return out;
    }
    if (out["geometry_included"] == true)
        select(node.at("geometry_indices"), "geometries", archive, node_index, out);
    out["status"] = "source_selection_known";
    return out;
}
Json csg_mesh_input(const Json &archive) {
    Json out = {{"scope", "stored_root_mesh_conversion_before_update"},
                {"status", "source_selection_known"},
                {"root_index", archive.at("tree").at("root_index")},
                {"selected_sources", Json::array()},
                {"skipped_sources", Json::array()},
                {"conversion_matrix_3x4_rows", identity_rows()},
                {"angle_tolerance", archive.at("angle_tolerance")},
                {"geometry_object_restore_status", "not_evaluated"},
                {"mesh_evaluation_status", "not_evaluated"},
                {"cache_validity_status", "not_evaluated"}};
    if (out.at("root_index").is_null())
        return out;
    auto node = csg_node_geometry_input(archive, out.at("root_index").get<std::size_t>());
    for (auto key : {"status", "selected_sources", "skipped_sources"})
        out[key] = node.at(key);
    if (node.contains("reason"))
        out["reason"] = node.at("reason");
    out["node_input"] = std::move(node);
    return out;
}
} // namespace p3d
