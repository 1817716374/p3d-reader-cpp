#pragma once
#include "native_tube_mesh_trim.hpp"
namespace p3d::swept_detail {
enum class TubeMeshTrimFacetKind { common_rows, lower_boundary, upper_boundary };
struct TubeMeshTrimFacet {
    std::size_t column_pair = 0;
    TubeMeshTrimFacetKind kind = TubeMeshTrimFacetKind::common_rows;
    // Exact addFacetIndices arguments, before termination and triangulation.
    // Invalid native indices remain diagnostic data, never dereferenced.
    std::vector<std::int32_t> indices;
    bool indices_in_range = true, column_correspondence_valid = true;
};
struct TubeMeshTrimFacetPlan {
    std::vector<TubeMeshTrimFacet> facets;
    bool indices_in_range = true, column_correspondence_valid = true;
    Json report;
};
TubeMeshTrimFacetPlan prepare_tube_mesh_trim_facets(const TubeMeshTrimPlan &, TubeBudget &);
// 88285 / 88714: lower uses its evaluated/snapped boundary value; every
// other row, including the upper boundary, uses the original shared V table.
std::vector<Point2> evaluate_tube_mesh_trim_parameters(const TubeMeshTrimPlan &,
                                                       const std::vector<double> &v,
                                                       std::size_t patch_index,
                                                       std::size_t patch_count, TubeBudget &);
} // namespace p3d::swept_detail
