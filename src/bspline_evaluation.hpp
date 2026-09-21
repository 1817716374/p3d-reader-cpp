#pragma once
#include <p3d/reader.hpp>

namespace p3d {
// Internal mesh endpoint evaluation in the original knot domains. An exact
// knot and an explicit side avoid epsilon offsets across a discontinuity.
Point3 bspline_surface_point_at_knots(const BsplineSurface &surface, double u, double v,
                                      bool left_u, bool left_v);
} // namespace p3d
