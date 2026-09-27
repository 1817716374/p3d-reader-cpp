#pragma once
#include "native_tube_mesh_path.hpp"
namespace p3d::swept_detail {
struct TubeMeshVertex {
    Point3 point{}, normal{};
    Point2 parameter{};
    bool normal_fallback = false;
};
TubeMeshVertex evaluate_tube_mesh_vertex(const BsplineSurface &, double u, double v,
                                         std::array<double, 2> source_u_interval,
                                         std::size_t patch_index, std::size_t patch_count,
                                         TubeBudget &);
struct TubeMeshRegularVertices {
    std::size_t u_count = 0, v_count = 0;
    // Native evaluation order: V outside, U inside. These are evaluated
    // attributes BEFORE boundary correspondence and shared-edge replacement.
    std::vector<TubeMeshVertex> vertices;
};
// 85dc0's rectangular branch, only when BOTH boundary functions are absent.
// This does not triangulate, stitch edges, weld coordinates or assign materials.
TubeMeshRegularVertices evaluate_tube_mesh_regular_vertices(const TubeMeshSampledPatch &,
                                                            const TubeSectionMeshSamples &,
                                                            std::size_t strip_index,
                                                            std::size_t patch_index,
                                                            std::size_t patch_count, TubeBudget &);
} // namespace p3d::swept_detail
