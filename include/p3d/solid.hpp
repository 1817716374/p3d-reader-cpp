#pragma once
#include <p3d/polyface.hpp>

namespace p3d {
struct SolidMeshResult {
    Json source; // Original BGFB solid table, never a replacement Polyface.
    // An explicitly derived Polyface and its triangles/corner/edge mappings.
    PolyfaceMeshResult derived;
    // Native GeFaceIndices, indexed by derived.geometry.face_source_polygons.
    std::vector<std::array<std::int64_t, 3>> face_indices;
};
// Derived planar faces for DgnBox, including skew/tapered and uncapped boxes.
// Requires a nondegenerate frame and nonzero widths with consistent signs
// between the two ends. Other solids and collapsed/crossing boxes are reported
// as not meshed. This does not reproduce native tessellator subdivision or UVs.
SolidMeshResult mesh_bgfb_solid(const Json &table, const PolyfaceMeshOptions &options = {});
} // namespace p3d
