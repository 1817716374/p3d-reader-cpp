#pragma once
#include <p3d/reader.hpp>

namespace p3d {
struct PolyfaceMeshOptions {
    std::size_t max_points = 3000000;
    std::size_t max_corners = 9000000;
    std::size_t max_triangles = 3000000;
    std::size_t max_polygon_edge_tests = 10000000;
};
struct PolyfaceMeshResult {
    std::string status = "not_evaluated";
    Geometry geometry;
    // Original BGFB table, including unused channels and scalar tails.
    Json source;
    Json report = Json::object();
    // For indexed faces these are positions in pointIndex; for implicit lists
    // they are source point positions. No deduplication of points or corners.
    std::vector<std::array<std::size_t, 3>> face_source_corners;
    std::vector<std::optional<Triangle>> face_double_color_indices;
    std::vector<std::optional<Triangle>> face_int_color_indices;
    std::vector<std::optional<Triangle>> face_color_table_indices;
};
// Derived triangles for a decoded BGFB Polyface table (not VariantGeometry).
// Supports signed indexed loops/fixed blocks and implicit triangle/quad lists.
// A meshed result describes geometry, not complete material or color selection:
// report.attribute_bindings records unresolved/missing/invalid source bindings.
// On geometry failure no partial Geometry or triangle mappings are returned.
PolyfaceMeshResult mesh_bgfb_polyface(const Json &table, const PolyfaceMeshOptions &options = {});
} // namespace p3d
