#pragma once
#include "native_tube_mesh_edges.hpp"
#include "native_tube_mesh_trim_facets.hpp"
namespace p3d::swept_detail {
struct TubeMeshTrimConnected {
    std::vector<Point3> points, normals;
    std::vector<Point2> parameters;
    TubeMeshTrimFacetPlan facets;
    Json report;
};
// Original U-major traversal with column-relative row correspondence. Facets
// are still the native pre-triangulation arguments, not a final solid mesh.
// Regular and trimmed strips share the same original edge state. No geometric
// equality merging is performed, and failures restore that state.
TubeMeshTrimConnected connect_tube_mesh_trim_vertices(const TubeMeshTrimVertices &,
                                                      const std::vector<double> &v_samples,
                                                      TubeMeshEdgeState &,
                                                      const TubeMeshEdgeOptions &, TubeBudget &);
} // namespace p3d::swept_detail
