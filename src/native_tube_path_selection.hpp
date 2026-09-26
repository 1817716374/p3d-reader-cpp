#pragma once
#include "native_tube_path_source.hpp"
namespace p3d::swept_detail {
struct TubePathSelection {
    std::vector<BsplineCurve> curves;
    std::size_t index = 0;
    double fraction = 0;
    // Winning candidate before end-of-member remapping. On a disconnected path
    // this need not equal curves[index].point_at(fraction).
    Point3 candidate_point{};
    double candidate_squared_distance = 0;
    Json report;
};
// Native face-path selection over prepared sources, preserving member order.
// Closest query when reference area failed; otherwise plane intersections only.
// Converts every member before returning. Unsupported/undefined states fail.
TubePathSelection select_tube_path_candidate(const TubeFacetSources &, TubeBudget &);
struct TubeFacetPath {
    TubeFacetSources sources;
    TubePathSelection selection;
    Json selected_member_planarity;
};
// Reference and path copying followed by candidate selection, sharing budget.
// Includes original selected member frame/range planarity. Actual point/tangent,
// splitting and surface assembly are subsequent.
TubeFacetPath prepare_tube_facet_path(const Json &profile, const Json &path, TubeBudget &);
} // namespace p3d::swept_detail
