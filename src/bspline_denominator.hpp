#pragma once
#include <p3d/reader.hpp>

namespace p3d {
Json certify_surface_denominator(const BsplineSurface &surface, unsigned max_steps);
Json certify_curve_plane(const BsplineCurve &curve, Point3 origin, Point3 normal, double tolerance,
                         unsigned max_steps);
} // namespace p3d
