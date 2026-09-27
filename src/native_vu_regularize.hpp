#pragma once
#include "native_vu_graph.hpp"
namespace p3d::swept_detail {
// Native strict sector test, including null faces and collinear sectors.
bool native_vu_node_in_sector(const NativeVuGraph &, std::size_t node, std::size_t sector,
                              TubeBudget &);
// Two native lexical sweeps, with XY sign reversal between them. Works on the
// merged graph; does not classify exterior faces or generate triangle indices.
// Failed validation or exhausted budgets leave the graph unchanged.
Json regularize_native_vu_graph(NativeVuGraph &, TubeBudget &);
} // namespace p3d::swept_detail
