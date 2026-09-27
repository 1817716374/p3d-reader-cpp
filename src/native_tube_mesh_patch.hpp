#pragma once
#include "native_tube_mesh_sampling.hpp"
namespace p3d::swept_detail {
struct TubeMeshBoundaryCurves {
    bool success = false;
    // Scalar V(U) is stored in X; Y/Z are zero. Null means native default.
    std::optional<BsplineCurve> lower, upper;
    Json report;
};
// Native 78380 uses only the first runtime UV point record, in its saved order.
// Does not consume pcurves, reorder loops or check the final closure point.
TubeMeshBoundaryCurves tube_mesh_boundary_curves(const std::vector<std::vector<Point2>> &,
                                                 TubeBudget &);
struct TubeMeshPatchPreparation {
    bool success = false;
    TubeMeshBoundaryCurves boundaries;
    curve_detail::NativeKnotData knots;
    std::vector<BsplineSurface> strips;
    Json report;
};
// Native mesh-patch preparation: boundary V(U) graphs followed by U Bezier
// strips (48480 direction 0). No triangulation or material assignment.
TubeMeshPatchPreparation prepare_tube_mesh_patch(const BsplineSurface &,
                                                 const std::vector<std::vector<Point2>> &,
                                                 TubeBudget &);
} // namespace p3d::swept_detail
