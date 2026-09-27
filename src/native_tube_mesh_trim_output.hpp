#pragma once
#include "native_tube_mesh_trim_edges.hpp"
#include "native_polyface_triangulate.hpp"
namespace p3d::swept_detail {
struct TubeMeshTrimOutput {
    TubeMeshRegularMesh mesh;
    bool complete = false;
    Json report;
};
// 888eb..88c9f after original strip coordinate connections. Point-only facets
// are triangulated first; normal/parameter indices copy that result before
// point-edge visibility is applied. No caps or final coordinate combination.
// Native partial face output remains available with complete=false.
TubeMeshTrimOutput emit_tube_mesh_trimmed_strip(const TubeMeshTrimConnected &,
                                                const TubeMeshEdgeOptions &,
                                                const TubeMeshVisibilityLayout &, TubeBudget &);
} // namespace p3d::swept_detail
