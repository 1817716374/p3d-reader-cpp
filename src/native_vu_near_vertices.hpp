#pragma once
#include "native_vu_graph.hpp"
namespace p3d::swept_detail {
std::vector<double> collapse_native_vu_fractions(std::vector<double> fractions, double tolerance,
                                                 double minimum, double maximum, TubeBudget &);
// Graph is committed only after successful completion. New split nodes keep
// native default source labels/masks; this does not connect nearby vertices.
Json split_native_vu_edges_near_vertices(NativeVuGraph &, double perpendicular_tolerance,
                                         double vertex_tolerance, double along_edge_tolerance,
                                         std::uint32_t visit_mask, TubeBudget &);
} // namespace p3d::swept_detail
