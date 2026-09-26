#pragma once
#include <p3d/reader.hpp>

namespace p3d {
struct PolyfaceMeshOptions {
    std::size_t max_points = 3000000;
    std::size_t max_corners = 9000000;
    std::size_t max_triangles = 3000000;
    std::size_t max_polygon_edge_tests = 10000000;
    // Used by curve-based solid reconstruction: denominator proofs, knot
    // inspection and B-spline evaluation work. Shared across CSG snapshots.
    std::size_t max_curve_work = 1000000;
};
struct PolyfaceSourceEdge {
    std::size_t source_polygon = 0;
    std::size_t start_corner = 0, end_corner = 0;
    std::array<std::uint32_t, 2> points{};
    // Signed pointIndex at the start corner: positive means visible.
    // This is the stored edge flag, not final scene/view visibility.
    bool visible = false;
};
struct PolyfaceTriangleEdgeSource {
    std::size_t source_edge = 0;
    bool reversed = false;
};
struct PolyfaceMeshResult {
    std::string status = "not_evaluated";
    Geometry geometry;
    // Original BGFB table, including unused channels and scalar tails.
    Json source;
    Json report = Json::object();
    // For indexed faces these are positions in pointIndex; for implicit lists/grids
    // they are source point positions. No deduplication of points or corners.
    std::vector<std::array<std::size_t, 3>> face_source_corners;
    // Original edges include those affected by derived boundary reduction.
    std::vector<PolyfaceSourceEdge> source_edges;
    // Entry k describes the triangle edge from corner k to (k+1)%3.
    // Null means no exact original corner adjacency, including new diagonals
    // and shortened boundaries. Visibility is not guessed for these edges.
    std::vector<std::array<std::optional<PolyfaceTriangleEdgeSource>, 3>> face_source_edges;
    std::vector<std::optional<Triangle>> face_double_color_indices;
    std::vector<std::optional<Triangle>> face_int_color_indices;
    std::vector<std::optional<Triangle>> face_color_table_indices;
};
// Derived triangles for a decoded BGFB Polyface table (not VariantGeometry).
// Supports signed indexed loops/fixed blocks and implicit triangle/quad lists/grids.
// A meshed result describes geometry, not complete material or color selection:
// report.attribute_bindings records unresolved/missing/invalid source bindings.
// On geometry failure no partial Geometry or triangle mappings are returned.
PolyfaceMeshResult mesh_bgfb_polyface(const Json &table, const PolyfaceMeshOptions &options = {});
} // namespace p3d
