#pragma once
#include "native_vu_graph.hpp"
namespace p3d::swept_detail {
struct NativeVuLinearSolution {
    bool solved = false;
    Point2 value{};
};
NativeVuLinearSolution solve_native_vu_2x2(double a00, double a01, double a10, double a11,
                                           double b0, double b1, TubeBudget &);
// Original transverse interior intersections. Contact/parallel edges do not
// create records. Vertex-angle connection is a separate merge operation.
Json split_native_vu_edges_at_intersections(NativeVuGraph &, double vertex_tolerance,
                                            double along_edge_tolerance, TubeBudget &);
} // namespace p3d::swept_detail
