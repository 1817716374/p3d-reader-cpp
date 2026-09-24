#pragma once
#include "internal.hpp"
namespace p3d {
// Triangulates already sampled, planar boundaries. Does not establish planarity
// of analytic curves between samples. Preserves every input perimeter segment.
std::vector<Triangle>
triangulate_planar_sample_rings(const std::vector<std::vector<std::uint32_t>> &rings,
                                const std::vector<Point3> &vertices, unsigned triangle_budget);
struct ProjectedRingMesh {
    std::vector<Triangle> faces;
    Json report;
};
// Only the topology working copy is projected. Indices still address original
// 3D samples, including their nonzero distances from the projection plane.
ProjectedRingMesh
triangulate_projected_sample_rings(const std::vector<std::vector<std::uint32_t>> &rings,
                                   const std::vector<Point3> &vertices, unsigned triangle_budget);
} // namespace p3d
