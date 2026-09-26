#pragma once
#include "native_bezier.hpp"
namespace p3d::detail {
struct NativeIsoCurve {
    BsplineCurve curve;
    std::size_t zero_weight_fallbacks = 0;
};
// Complete constant-V isocurve, ignoring trim boundaries. V is a fraction of
// the full native knot domain and clamps at its endpoints, even when closed.
// Output retains U order, closed state and full knots. Weighted XYZ undergoes
// the native point division and multiplication by the original evaluated W.
NativeIsoCurve native_iso_v_curve(const BsplineSurface &, double fraction, curve_detail::BezierWork,
                                  std::size_t max_control_points = 1000000);
} // namespace p3d::detail
