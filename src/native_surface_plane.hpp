#pragma once
#include "native_tube.hpp"
namespace p3d::swept_detail {
// Native control-net principal extents and isPlane predicate. This is not a
// certified surface-error or mesh-flatness test.
Json native_surface_plane(const BsplineSurface &, TubeBudget &);
} // namespace p3d::swept_detail
