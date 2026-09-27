#pragma once
#include "native_tube_facet_sample.hpp"
#include "native_tube_facet_boundaries.hpp"
#include <functional>
namespace p3d::swept_detail {
using TubeFacetUvQuery = std::function<std::optional<Point2>(double)>;
struct TubeFacetUvNode {
    double fraction = 0;
    Point2 parameters{};
    std::optional<std::size_t> left, right;
    unsigned depth = 0;
};
struct TubeFacetUvTree {
    bool success = false;
    std::vector<TubeFacetUvNode> tree;
    // Every node, including quarter-point acceptance probes, in native inorder.
    // No externally queried interval endpoints are inserted here.
    std::vector<Point3> first, second;
    Json report;
};
// f6760 + f6f70. A missing query result is native failure; exceptions propagate.
// Query order and repeated boundary queries are significant. No deduplication.
TubeFacetUvTree sample_tube_facet_uv_tree(const TubeFacetUvQuery &,
                                          const std::array<double, 2> &interval, double tolerance,
                                          TubeBudget &);
struct TubeFacetUvSegments {
    bool success = false;
    TubeFacetBoundarySegments first, second;
    Json report;
};
// f7050, using the profile's compressed active knot intervals, raw knot values
// as U fractions and fixed .001 tolerance. A final endpoint is appended only
// at an interval high exactly equal to 1, where iteration terminates.
TubeFacetUvSegments sample_tube_facet_uv_segments(const BsplineCurve &profile,
                                                  const TubeFacetUvQuery &, TubeBudget &);
// Actual surface route: every query uses the complete plane/pair sampler.
// This generates two UV segment lists, not the final face-chain boundaries.
TubeFacetUvSegments sample_tube_facet_uv_seam(const BsplineSurface &, const BsplineSurface &,
                                              const BsplineCurve &profile,
                                              const std::array<Point3, 2> &plane,
                                              bool require_same_point, TubeBudget &);
} // namespace p3d::swept_detail
