#pragma once
#include "native_polyface_visitor.hpp"

namespace p3d::swept_detail {
struct NativePolyfaceMeshAttributes {
    NativePolyfaceMesh output;
    bool native_succeeded = false, complete = false;
    Json report;
};
// Original per-face generators with the complete typed query and raw mesh
// style. Preserve metadata, replace only the selected pool/indices, and read
// each subsequent face after the previous face's updates. No layout conversion
// or missing index-position synthesis. Unsafe native reads are rejected.
NativePolyfaceMeshAttributes build_native_polyface_mesh_normals(const NativePolyfaceMesh &,
                                                                TubeBudget &);
NativePolyfaceMeshAttributes build_native_polyface_mesh_parameters(const NativePolyfaceMesh &,
                                                                   int coordinate_selector,
                                                                   TubeBudget &);
NativePolyfaceMeshAttributes build_native_polyface_mesh_approximate_normals(
    const NativePolyfaceMesh &, double max_single_edge_angle, double max_accumulated_angle,
    bool mark_transitions_visible, TubeBudget &);
} // namespace p3d::swept_detail
