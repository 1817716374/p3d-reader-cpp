#pragma once
#include "native_vu_graph.hpp"
namespace p3d::swept_detail {
struct NativeVuIndices {
    bool succeeded = false;
    // Native output, including the terminated partial face on an unresolved
    // vertex. Callers must check succeeded before using this as geometry.
    std::vector<std::int32_t> indices;
    Json report;
};
// Projected-loop caller: signed one-based indices, no new vertex creation,
// no optional boundary output. Resolves the first numbered sector of each
// vertex in native traversal order, not by coordinates or nearest point.
NativeVuIndices collect_native_vu_source_indices(NativeVuGraph &, TubeBudget &);
struct NativeVuMeshIndices {
    std::vector<Point3> points;
    NativeVuIndices faces;
};
// General XY-polygon route with coordinate output: propagate existing labels
// around vertices, then append unnumbered crossings in original face order.
// Coordinates retain the complete source prefix, including disconnects.
// This differs from the source-only facet route above; no coordinate welding.
NativeVuMeshIndices collect_native_vu_mesh_indices(NativeVuGraph &,
                                                   const std::vector<Point3> &, TubeBudget &);
} // namespace p3d::swept_detail
