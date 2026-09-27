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
// coordinateFrame selectors 0 (unit axes at start), 1 (unit axes at lower
// left), 2 (unit XY ranges), 3 (unit larger XY range), followed by projection.
// Other selector integers follow the native lower-left/unit-axis branch.
// project_points=false requests only the frame (used by face normals).
// No triangulation or vertex insertion here.
NativePolygonProjection prepare_native_polygon_projection(const std::vector<Point3> &, TubeBudget &,
                                                          int coordinate_selector = 0,
                                                          bool project_points = true);
} // namespace p3d::swept_detail
