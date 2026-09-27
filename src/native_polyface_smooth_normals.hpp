#pragma once
#include "native_polyface_attributes.hpp"
namespace p3d::swept_detail {
// Original BuildApproximateNormals: per-face normals, native connectivity,
// barrier/sector averaging, then optional point-index visibility replacement.
// Angles are in radians and use original strict thresholds. Native failure may
// retain newly generated per-face normals; source input is always immutable.
NativePolyfaceAttributes build_native_polyface_approximate_normals(
    const NativePolyfaceFaceDataState &, double max_single_edge_angle, double max_accumulated_angle,
    bool mark_transitions_visible, TubeBudget &);
} // namespace p3d::swept_detail
