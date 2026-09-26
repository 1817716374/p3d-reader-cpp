#pragma once
#include "native_tube.hpp"
namespace p3d::swept_detail {
// Native reversal for untrimmed tube surfaces. A failed tiny-domain knot
// normalization is reported and leaves reversed (descending) knots in JSON,
// just as the native caller does; such output is not a valid BsplineSurface.
TubeAssembly reverse_tube_surface(const BsplineSurface &, bool reverse_u, TubeBudget &);
struct TubeOrientation {
    std::vector<Json> surfaces;
    Json report;
};
// Native multi-ring orientation with an explicitly supplied native start
// tangent. This does not compute that tangent or validate source curve groups.
// Area failure exits the whole loop, returning true only for an order-2 iso
// curve. Earlier reversals persist and remaining surfaces are not visited.
TubeOrientation orient_tube_surfaces(const std::vector<Json> &, Point3 tangent, TubeBudget &);
} // namespace p3d::swept_detail
