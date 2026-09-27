#pragma once
#include "native_tube_mesh_edges.hpp"
#include "native_polygon_projection.hpp"
#include "native_vu_graph.hpp"
#include <optional>
namespace p3d::swept_detail {
struct NativeFacetIndexPlan {
    enum class Route { passthrough, quad, projected_loops_required, projected_loops };
    Route route = Route::projected_loops_required;
    bool completed = false;
    // Local signed 1-based indices, zero-terminated. Large-face indices refer
    // to the visitor's wrapped point list (input plus its automatic closure).
    // Empty on unresolved large-face output; inspect input_graph reports.
    std::vector<std::int32_t> indices;
    // Large-face preparation includes the visitor's extra closing point.
    std::optional<NativePolygonProjection> projection;
    std::optional<NativeVuInput> input_graph;
};
NativeFacetIndexPlan native_facet_index_plan(const std::vector<Point3> &, TubeBudget &);
struct TubeMeshVisibilityLayout {
    std::size_t u_count = 0, v_count = 0, coordinate_count = 0;
    std::int32_t first_column_count = 0, last_column_count = 0;
    bool trimmed = false, first_column_visible = false, last_column_visible = false;
};
struct TubeMeshVisibleIndices {
    std::vector<std::int32_t> indices;
    Json report;
};
// 88981..88c9f: explicit native caller flags, not inferred from closedness.
// Only point-index signs are changed; prior normal/parameter index copies
// retain their earlier values. Inputs are not modified, including on failure.
TubeMeshVisibleIndices apply_tube_mesh_edge_visibility(const std::vector<std::int32_t> &,
                                                       const TubeMeshVisibilityLayout &,
                                                       TubeBudget &);
} // namespace p3d::swept_detail
