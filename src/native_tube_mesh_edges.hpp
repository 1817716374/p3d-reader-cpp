#pragma once
#include "native_tube_mesh_vertices.hpp"
namespace p3d::swept_detail {
struct TubeMeshEdgeState {
    std::size_t patch_count = 0, strip_count = 0, next_patch = 0, next_strip = 0;
    bool profile_closed = false, path_closed = false;
    std::vector<std::vector<Point3>> rows, profile_seam, path_seam;
    std::vector<Point3> column, start_points, end_points;
};
TubeMeshEdgeState make_tube_mesh_edge_state(std::size_t patch_count, std::size_t strip_count,
                                            bool profile_closed, bool path_closed, TubeBudget &);
struct TubeMeshRegularMesh {
    std::vector<Point3> points, normals;
    std::vector<Point2> parameters;
    std::vector<std::int32_t> point_indices, normal_indices, parameter_indices;
    bool point_index_active = false;
    Json report;
};
struct TubeMeshEdgeOptions {
    bool normals = true, parameters = true;
    // Exact optional collection pointers supplied by the caller, not inferred
    // cap flags. Collecting edge points does not construct a cap.
    bool collect_start = false, collect_end = false;
};
// 85dc0 rectangular branch: sequential original patch/strip state, point
// correspondence, native triangle-grid indices. No distance welding, cap
// construction, material association or final combineCoordinate operation.
// Exceptions leave the caller's state unchanged; work already used is charged.
TubeMeshRegularMesh connect_tube_mesh_regular_vertices(const TubeMeshRegularVertices &,
                                                       TubeMeshEdgeState &,
                                                       const TubeMeshEdgeOptions &, TubeBudget &);
} // namespace p3d::swept_detail
