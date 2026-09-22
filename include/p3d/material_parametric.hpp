#pragma once
#include <p3d/material_elevation.hpp>

namespace p3d {
// The native per-face frame representation and projection kernel are shared
// with ElevationDrape; the two names denote the same C++ type.
using MaterialParametricFrame = MaterialElevationFrame;
struct MaterialParametricRenderContext {
    // Required for absolute scale modes. false means a known absent frame;
    // nullopt means unknown. Relative modes do not read this context.
    std::optional<bool> preparation_frame_present;
    std::optional<MaterialParametricFrame> preparation_frame;
    std::optional<double> geometry_scale;
    // Only used in absolute mode with no preparation frame. False selects
    // factors (1,1); true requires the native two factors, before geometry scale.
    std::optional<bool> use_parameter_factors;
    std::optional<Point2> parameter_factors;
};
// Consumes successful build_material_uv_transform() output for mode 0. Both
// native geometry-kind branches use the same parametric preparation rules.
Json prepare_material_parametric_sampling(const Json &uv_transform,
                                          const MaterialParametricRenderContext &context);
// Per-face frame first, then native UV if present, then render vertex XY.
// nullopt explicitly means the corresponding native pointer is absent.
// An absolute prepared frame overrides the supplied face frame ONLY when a
// face frame is present at sampling time. No normal is needed.
Json sample_material_parametric(
    const Json &prepared, const Point3 &point,
    const std::optional<Point2> &native_uv = std::nullopt,
    const std::optional<MaterialParametricFrame> &face_frame = std::nullopt);
} // namespace p3d
