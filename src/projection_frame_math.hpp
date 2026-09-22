#pragma once
#include <p3d/material_elevation.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace p3d::projection_detail {
inline float f32(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::range_error("nonfinite_or_unrepresentable_float_arithmetic");
    return static_cast<float>(value);
}
inline MaterialElevationFrame rounded_frame(MaterialElevationFrame frame) {
    for (auto &v : frame.origin)
        v = f32(v);
    for (auto &row : frame.axes)
        for (auto &v : row)
            v = f32(v);
    return frame;
}
// Shared native per-face path: float subtraction and products, (Y+X)+Z.
inline Point2 project(const MaterialElevationFrame &source, const Point3 &point) {
    const auto frame = rounded_frame(source);
    std::array<float, 3> d{};
    for (unsigned i = 0; i < 3; ++i)
        d[i] = f32(f32(point[i]) - static_cast<float>(frame.origin[i]));
    Point2 out{};
    for (unsigned i = 0; i < 2; ++i) {
        const auto &a = frame.axes[i];
        const float x = f32(d[0] * static_cast<float>(a[0]));
        const float y = f32(d[1] * static_cast<float>(a[1]));
        const float z = f32(d[2] * static_cast<float>(a[2]));
        out[i] = f32(f32(y + x) + z);
    }
    return out;
}
} // namespace p3d::projection_detail
