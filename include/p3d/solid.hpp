#pragma once
#include <p3d/polyface.hpp>

namespace p3d {
struct SolidMeshResult {
    Json source; // Original BGFB solid table, never a replacement Polyface.
    // An explicitly derived Polyface and its triangles/corner/edge mappings.
    PolyfaceMeshResult derived;
    // Native GeFaceIndices, indexed by derived.geometry.face_source_polygons.
    std::vector<std::array<std::int64_t, 3>> face_indices;
    // Optional analytic surface coordinates per OUTPUT triangle corner. Empty
    // when unavailable; a null triangle means no coordinates for that face.
    // These are not material texture UVs. Currently populated for loft sides.
    std::vector<std::optional<std::array<Point2, 3>>> surface_parameters;
};
// Derived faces for DgnBox, DgnCone, DgnSphere, DgnTorusPipe,
// DgnExtrusion, DgnRuledSweep and P3DSectionLoft.
// Requires a nondegenerate frame. Boxes need nonzero widths with consistent
// signs between the two ends. Other solids and collapsed/crossing boxes are reported
// as not meshed. Cones use an explicit uniform angular segment count (>=3),
// not the native adaptive tessellator. Either radius may be zero (an apex),
// but both zero or opposite nonzero signs are unsupported. Original analytic
// parameters remain in source; native subdivision and UVs are not reproduced.
// Spheres use the same longitude segment count, with uniform latitude bands no
// larger than 2*pi/count. Raw latitudes must be distinct and within [-pi/2,pi/2].
// The original affine frame is retained, including anisotropy, shear and reflection.
// Torus meshing rotates the source start section; |majorRadius| must exceed
// nonzero |minorRadius|. Native rotational sweeps are limited to one revolution.
// Extrusion/ruled profiles support line segments, line strings and elliptic arcs,
// open chains without caps, closed loops, parity regions and union children.
// Corresponding source primitive/component counts must agree. Cap topology is
// triangulated in a derived reference plane while retaining original 3D samples,
// including nonplanar caps. No missing source boundary segment is invented.
// circle_segments controls arc sampling and V bands for nonplanar ruled patches.
// These derived meshes do not claim a world-space error bound or native UVs.
// Section lofts use SectionLoft::mesh with max_uv_edge=1/circle_segments;
// its source-curve, denominator, join and planar-cap requirements still apply.
// Side surface parameters and original native face IDs are retained separately
// from material UVs. CSG rebuilds transformed curve-based sources into snapshots.
SolidMeshResult mesh_bgfb_solid(const Json &table, const PolyfaceMeshOptions &options = {},
                                unsigned circle_segments = 64);
struct SolidTransformResult {
    std::string status = "not_evaluated";
    Json transformed;
    Matrix4 geometry_transform{};
    Json report = Json::object();
};
using ConeTransformResult = SolidTransformResult;
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
// Torus native parameter queries distinguish enumerated faces from actual caps.
// Sweep meshing rotates the original start section about an orthogonal axis;
// it does not treat the two stored directions as arbitrary affine torus axes.
Json native_bgfb_torus_parameters(const Json &table);
// Native placement normalizes both directions and scales both radii by the
// first transformed direction's length. The returned matrix maps the derived
// rotational-sweep geometry, including its recomputed normal direction.
SolidTransformResult transform_bgfb_torus(const Json &table, const Matrix4 &matrix);
struct LoftSourceTransformOptions {
    std::size_t max_points = 3000000;
    std::size_t max_curve_nodes = 100000;
    unsigned max_depth = 80; // Hard ceiling 80.
};
struct LoftSourceTransformResult {
    std::string status = "not_evaluated";
    Json transformed;
    Json report = Json::object();
};
// Transform original sections and guide groups before reconstructing a loft.
// Supports line segments/strings, B-splines and elliptic arcs,
// including nested curve arrays. Rational poles receive weight*translation.
// Native near-identity suppression applies to B-splines only. Arc axes shorter
// than 1e-5 trigger native replacement by an extremal line segment, including
// first-axis priority when both are short. report.arc_replacements retains the
// transformed arc and its source path. Singular source transforms are allowed;
// successful transformation does not prove that the new loft can be meshed.
// This is a derived parameter table, not an edited/serializable source file.
// No single matrix maps the old loft surface to the reconstructed new one.
LoftSourceTransformResult
transform_bgfb_section_loft(const Json &table, const Matrix4 &matrix,
                            const LoftSourceTransformOptions &options = {});
// The same source-curve rules for a section loft, extrusion or ruled sweep.
// Extrusion vectors receive the linear transform before the base curve;
// ruled sections are visited in their stored order. Rebuild from transformed.
using CurveSolidTransformOptions = LoftSourceTransformOptions;
using CurveSolidTransformResult = LoftSourceTransformResult;
CurveSolidTransformResult
transform_bgfb_curve_solid(const Json &table, const Matrix4 &matrix,
                           const CurveSolidTransformOptions &options = {});
} // namespace p3d
