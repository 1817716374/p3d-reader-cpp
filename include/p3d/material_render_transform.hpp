#pragma once
#include <p3d/reader.hpp>

namespace p3d {
// Reconstruct the derived state written by native active-transform setup.
// Inputs are the already resolved DOUBLE render matrix C and translation t,
// not a source object transform before reference/view composition.
// Later primitive-specific render overrides are outside this operation.
// No geometry_scale is selected: the native host chooses between the two
// returned candidates, and some render paths explicitly replace both.
Json derive_material_render_transform(const Matrix3 &linear_transform, const Point3 &translation);
} // namespace p3d
