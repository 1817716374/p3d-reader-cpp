#pragma once
#include <p3d/material_parametric.hpp>

namespace p3d {
struct MaterialPlanarRenderContext {
    // Native preparation can clear the planar flag. Known false selects the
    // parametric rules using the already registered mode-2 UV matrix.
    std::optional<bool> enabled;
    MaterialParametricRenderContext parametric;
    std::optional<std::int32_t> geometry_kind;
    // Float render matrix R. Its first column supplies the fallback U axis;
    // its third column supplies the reference axis (not normalized).
    std::optional<Matrix3> vertex_linear_transform;
    // Double matrix C and translation a are needed by both kinds. For nonzero
    // kind, also provide A and float t: origin=C*(float(A*t)+a).
    // Kind zero uses origin=C*a, ignoring A and t.
    std::optional<Matrix3> origin_basis_transform;
    std::optional<Point3> reference_translation;
    std::optional<Matrix3> reference_linear_transform;
    std::optional<Point3> vertex_translation;
    // Nonzero kind transforms vertex normals by this float matrix. The bool
    // explicitly selects normalization AFTER transformation. Kind zero uses
    // source float normals without this transformation or normalization.
    std::optional<Matrix3> normal_transform;
    std::optional<bool> normalize_transformed_normal;
};
Json prepare_material_planar_sampling(const Json &uv_transform,
                                      const MaterialPlanarRenderContext &context);
// Pure function returning a next_state snapshot on success. Feed that snapshot
// to the next query in the same native sampling sequence: dynamic planar UV
// offsets persist and may be read by a later per-face query. Independent
// streams can keep independent states. The input is never modified on failure.
// A present face frame has priority; native_uv is used only when planar was
// explicitly disabled. nullopt means confirmed absence, not unknown data.
Json sample_material_planar(
    const Json &state, const Point3 &point, const std::optional<Point3> &normal = std::nullopt,
    const std::optional<Point2> &native_uv = std::nullopt,
    const std::optional<MaterialParametricFrame> &face_frame = std::nullopt);
} // namespace p3d
