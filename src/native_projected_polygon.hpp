#pragma once
#include "native_vu_indices.hpp"
namespace p3d::swept_detail {
struct NativeProjectedPolygon {
    NativeVuInput input_graph;
    std::vector<Point3> points;
    std::vector<std::int32_t> indices;
    bool native_succeeded = false, complete = false;
    Json report;
};
// Original projected polygon route with coordinate output, signed one-based
// indices and edge limit three. The caller supplies the original frame pair.
// No additional frame fitting, ring reordering, or adaptive edge subdivision.
// Keep source slots (including markers) as a prefix; append intersections
// in original face order. Success does not certify a planar or closed surface.
NativeProjectedPolygon triangulate_native_projected_polygon(const std::vector<Point3> &,
                                                            const Matrix4 &local_to_world,
                                                            const Matrix4 &world_to_local,
                                                            double xy_tolerance, TubeBudget &);
} // namespace p3d::swept_detail
