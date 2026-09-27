#pragma once
#include "native_polygon_projection.hpp"
namespace p3d::swept_detail {
struct NativeTubeMeshCapInput {
    // Original ring order: the first ring is outer, subsequent rings inner.
    // Each ring has undergone the native closure operation, not region repair.
    std::vector<std::vector<Point3>> rings;
    std::vector<std::uint32_t> ring_types;
    std::uint32_t region_type = 0;
    // Stroked input retains native disconnects, including the trailing one.
    std::vector<Point3> stroked_points, points;
    NativePolygonProjection projection;
    bool preparation_succeeded = false;
    Json report;
};
// Polyline cap route: reverse each ring if requested, build native outer/inner
// regions without enforcing XY orientation, stroke segments, then prepare the
// original addTriangulation input and lower-left frame. max_edge_length is the
// caller's resolved length setting (including any source scale factor).
// This does not generate triangles, builder attributes, or a complete cap.
NativeTubeMeshCapInput prepare_native_tube_mesh_cap_input(const std::vector<std::vector<Point3>> &,
                                                          bool reverse, double max_edge_length,
                                                          TubeBudget &);
} // namespace p3d::swept_detail
