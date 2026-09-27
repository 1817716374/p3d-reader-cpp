#pragma once
#include "native_vu_graph.hpp"
namespace p3d::swept_detail {
std::vector<std::size_t> validate_native_vu_split_graph(const NativeVuGraph &, TubeBudget &);
std::vector<std::size_t> collect_native_vu_up_edges(const NativeVuGraph &,
                                                    const std::vector<std::size_t> &, TubeBudget &);
void sort_native_vu_nodes(const NativeVuGraph &, std::vector<std::size_t> &, TubeBudget &);
// Shared primitive for an already validated transaction graph. The caller owns
// rollback; each split uses the unchanged endpoints of the original edge.
std::size_t split_native_vu_at_fractions(NativeVuGraph &, std::size_t edge,
                                         std::vector<double> fractions, double vertex_tolerance,
                                         double along_edge_tolerance, TubeBudget &);
} // namespace p3d::swept_detail
