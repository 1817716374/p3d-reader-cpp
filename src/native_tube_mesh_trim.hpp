#pragma once
#include "native_tube_mesh_vertices.hpp"
namespace p3d::swept_detail {
struct TubeMeshTrimColumn {
    double local_u = 0, source_u = 0, lower = 0, upper = 1;
    std::int32_t first_interior = 0, last_interior = 0;
    bool lower_snapped = false;
    std::size_t offset = 0, count = 0;
};
struct TubeMeshTrimPlan {
    bool success = false;
    std::vector<TubeMeshTrimColumn> columns;
    std::size_t vertex_count = 0;
    Json report;
};
// Native 86edf..875df: scalar bounds and signed interior-row ranges.
// No polygon parity/clipping or lower/upper reordering is performed.
TubeMeshTrimPlan prepare_tube_mesh_trim_columns(const TubeMeshBoundaryCurves &,
                                                const std::vector<double> &u,
                                                const std::vector<double> &v,
                                                std::array<double, 2> source_u, TubeBudget &);
struct TubeMeshTrimVertex {
    std::int32_t row = 0;
    double geometry_v = 0;
    Point3 point{}, normal{};
    bool normal_fallback = false;
};
struct TubeMeshTrimVertices {
    TubeMeshTrimPlan plan;
    // U outside, each column's variable V range inside. Geometry parameters
    // are not final texture parameters; no boundary correspondence yet.
    std::vector<TubeMeshTrimVertex> vertices;
};
TubeMeshTrimVertices evaluate_tube_mesh_trim_vertices(const TubeMeshSampledPatch &,
                                                      const TubeSectionMeshSamples &,
                                                      std::size_t strip_index, TubeBudget &);
} // namespace p3d::swept_detail
