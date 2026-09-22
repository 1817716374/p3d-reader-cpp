#pragma once
#include <p3d/reader.hpp>

namespace p3d {
Json certify_surface_denominator(const BsplineSurface &surface, unsigned max_steps);
}
