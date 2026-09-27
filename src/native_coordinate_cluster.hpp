#pragma once
#include "native_polyface_triangulate.hpp"
namespace p3d::swept_detail {
template <std::size_t N> struct NativeCoordinateCluster {
    std::vector<std::array<double, N>> coordinates;
    std::vector<std::size_t> source_to_packed;
    Json report;
};
NativeCoordinateCluster<2> cluster_native_coordinates(const std::vector<Point2> &, double abs_tol,
                                                      double rel_tol, TubeBudget &);
NativeCoordinateCluster<3> cluster_native_coordinates(const std::vector<Point3> &, double abs_tol,
                                                      double rel_tol, TubeBudget &);
struct NativePolyfaceCoordinateState {
    std::int32_t mesh_style = 1;
    bool parameter_pool_active = false, normal_pool_active = false;
    std::vector<Point3> points, normals;
    std::vector<Point2> parameters;
    NativePolyfaceIndexState indices;
};
struct NativePolyfaceCoordinateCombination {
    NativePolyfaceCoordinateState output;
    std::vector<std::size_t> point_map, parameter_map, normal_map;
    bool applied = false;
    Json report;
};
// PolyfaceHandle::combineCoordinate on explicitly prepared pools and flags.
// This is a native derived-mesh operation, not implicit file-data deduplication.
// Every enabled channel clusters independently and retains its index signs.
NativePolyfaceCoordinateCombination
combine_native_polyface_coordinates(const NativePolyfaceCoordinateState &, TubeBudget &);
} // namespace p3d::swept_detail
