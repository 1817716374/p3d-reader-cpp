#pragma once
#include "internal.hpp"
namespace p3d::detail {
struct NativeSurfaceSample {
    Point3 point{}, du{}, dv{};
    double weight = 1, weight_du = 0, weight_dv = 0;
};
// 115dc0: k=(1-f)*firstActiveKnot+f, upper clamp 1, raw-knot
// derivatives. This intentionally differs from full-domain curve evaluation.
// No trim classification, periodic fraction wrapping or zero-weight fallback.
NativeSurfaceSample native_surface_sample(const BsplineSurface &, double u, double v);
} // namespace p3d::detail
