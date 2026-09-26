#pragma once
#include "native_tube_path_branches.hpp"
namespace p3d::swept_detail {
bool native_path_points_equal(const Point3 &, const Point3 &);
bool native_path_vectors_parallel(const Point3 &, const Point3 &);
struct TubeFacetPathPlacement {
    TubeFacetPathBranches branches;
    std::optional<Matrix4> prefix_transform, suffix_transform;
    bool success = false;
    Json report;
};
// Consumes private working branches, preserving member aliases and ordered
// frame side effects. On success the prefix is reversed for traversal.
// Path placement only; no face patches or complete getFacets result.
TubeFacetPathPlacement place_tube_facet_paths(TubeFacetPathBranches, TubeBudget &);
TubeFacetPathPlacement prepare_tube_facet_path_placement(const Json &, const Json &, TubeBudget &);
} // namespace p3d::swept_detail
