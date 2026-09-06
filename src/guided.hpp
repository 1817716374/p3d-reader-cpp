#pragma once
#include "geometry.hpp"
namespace p3d {
// Temporary source boundary data; identities and sharing remain in the command stream.
struct GuidedBoundary {
    bool ellipse = false;
    std::vector<Point3> points;
    std::vector<GuidedBoundary> parts;
    Point3 center{}, axis_x{}, axis_y{};
    double start = 0, sweep = 0;
};
struct GuidedMesh {
    std::vector<std::vector<Point3>> rings;
    Json note;
};
GuidedMesh guided_surface(const std::vector<GuidedBoundary> &bottom,
                          const std::vector<GuidedBoundary> &top,
                          const std::vector<GuidedBoundary> &guides, const Tessellation &,
                          bool closed = true);
} // namespace p3d
