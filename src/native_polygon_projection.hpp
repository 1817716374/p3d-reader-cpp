#pragma once
#include "native_tube.hpp"
namespace p3d::swept_detail {
struct NativePolygonProjection {
    bool frame_succeeded = false;
    Matrix4 local_to_world{}, world_to_local{};
    // All source entries, including unchanged disconnect markers. Empty when
    // coordinateFrame fails. Success is not a planar/valid-face certificate.
    std::vector<Point3> points;
    Json report;
};
// coordinateFrame selector 0 (unit axes at the first point), followed by the
// projected-loop input transform. No triangulation or vertex insertion here.
NativePolygonProjection prepare_native_polygon_projection(const std::vector<Point3> &,
                                                          TubeBudget &);
} // namespace p3d::swept_detail
