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
// Derived faces for DgnBox, DgnCone and DgnSphere, including uncapped solids.
// Requires a nondegenerate frame. Boxes need nonzero widths with consistent
// signs between the two ends. Other solids and collapsed/crossing boxes are reported
// as not meshed. Cones use an explicit uniform angular segment count (>=3),
// not the native adaptive tessellator. Either radius may be zero (an apex),
// but both zero or opposite nonzero signs are unsupported. Original analytic
// parameters remain in source; native subdivision and UVs are not reproduced.
// Spheres use the same longitude segment count, with uniform latitude bands no
// larger than 2*pi/count. Raw latitudes must be distinct and within [-pi/2,pi/2].
// The original affine frame is retained, including anisotropy, shear and reflection.
SolidMeshResult mesh_bgfb_solid(const Json &table, const PolyfaceMeshOptions &options = {},
                                unsigned circle_segments = 64);
struct ConeTransformResult {
    std::string status = "not_evaluated";
    Json transformed;
    Matrix4 geometry_transform{};
    Json report = Json::object();
};
// Native GeConeInfo placement: normalize both transformed section vectors,
// then scale BOTH radii by the first vector's length. geometry_transform maps
// the original cone's points to those new parameters; it need not equal matrix.
// Source is unchanged; singular/nonfinite/unrepresentable transforms fail.
ConeTransformResult transform_bgfb_cone(const Json &table, const Matrix4 &matrix);
// Native sphere parameter queries: cap selection, getSweepInfo and isClosedSolid.
// The latter is a native flag/tolerance decision, not a manifold validity proof.
// Only getSweepInfo clamps latitudes to the polar interval; raw angles remain
// in source_latitudes. force_sweep_north orders only the reported clamped sweep.
// Finite singular frames may be inspected even though meshing rejects them.
Json native_bgfb_sphere_parameters(const Json &table, bool force_sweep_north = false);
} // namespace p3d
