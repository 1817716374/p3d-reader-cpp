#pragma once
#include "native_vu_graph.hpp"
namespace p3d::swept_detail {
// XY coordinate consolidation only: no edge splitting, vertex twist, source
// relabeling, or intersection solving. Nonzero scratch mask is caller-owned.
// Invalid input, arithmetic, or work budget leaves the graph unchanged.
Json consolidate_native_vu_coordinates(NativeVuGraph &, double tolerance, std::uint32_t visit_mask,
                                       TubeBudget &,
                                       std::vector<std::size_t> *cluster_nodes = nullptr);
double native_vu_merge_tolerance(const NativeVuGraph &, double absolute_tolerance,
                                 double relative_tolerance, TubeBudget &);
// Fresh indexed triangulation graph: original absolute/relative defaults and
// first consolidation, near-vertex splits, second consolidation, intersections.
// Completes native merge2002; regularization and triangulation are separate.
void prepare_native_vu_merge(NativeVuInput &, TubeBudget &);
void prepare_native_vu_merge(NativeVuInput &, TubeBudget &, double absolute_tolerance,
                             double relative_tolerance);
} // namespace p3d::swept_detail
