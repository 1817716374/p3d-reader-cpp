#pragma once
#include <p3d/reader.hpp>

namespace p3d {
// Render-space origin and two projection axes, stored by row. All nine
// components are rounded to float, as in the native render buffers.
struct MaterialElevationFrame {
    Point3 origin{};
    Matrix2x3 axes{};
};

struct MaterialElevationRenderContext {
    std::optional<std::int32_t> geometry_kind;
    // XY of the native render reference point, BEFORE any reference transform.
    std::optional<Point2> reference_xy;
    // Known absence differs from unknown. With a texture present, each native
    // axis flag controls offset reduction: zero keeps it, nonzero subtracts
    // floor. These flags are not a public texture-addressing enum.
    std::optional<bool> texture_present;
    std::optional<std::array<std::int32_t, 2>> texture_axis_flags;
    // Required only for kind zero. True retains the registered UV translations
    // without adding reference_xy or applying texture-axis offset reduction.
    std::optional<bool> preserve_registered_offset;
    // Kind zero scales both UV linear rows by float(1 / float(scale * scale)).
    std::optional<double> geometry_scale;
    std::optional<MaterialElevationFrame> vertex_frame;
};

// Consumes a successful build_material_uv_transform() result for mode 1.
// Does not discover the active render/texture context or infer its flags.
Json prepare_material_elevation_sampling(const Json &uv_transform,
                                         const MaterialElevationRenderContext &context);
// A supplied per-face frame takes priority over both geometry-kind branches.
// nullopt explicitly selects the native path with no per-face frame. No normal
// is needed; the nonzero-kind path without a frame reads only point X and Y.
// Prepared state can be reused concurrently without modification.
Json sample_material_elevation(
    const Json &prepared, const Point3 &point,
    const std::optional<MaterialElevationFrame> &face_frame = std::nullopt);
} // namespace p3d
