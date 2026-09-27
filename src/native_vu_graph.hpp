#pragma once
#include "native_tube.hpp"
namespace p3d::swept_detail {
inline constexpr std::size_t native_vu_null = std::numeric_limits<std::size_t>::max();
inline constexpr std::uint32_t native_vu_numbered_mask = 0x80000000u;
inline constexpr std::uint32_t native_vu_boundary_mask = 1;
struct NativeVuNode {
    std::size_t all_next = native_vu_null, face_next = native_vu_null, vertex_next = native_vu_null;
    std::uint32_t mask = 0;
    std::int32_t source_index = 0;
    Point3 point{};
};
// Stable indices express the original node identity, not geometric equality.
// The list tail stays at the first allocated node; new nodes are inserted
// after it, preserving the original all-node traversal order.
struct NativeVuGraph {
    std::vector<NativeVuNode> nodes;
    std::size_t tail = native_vu_null;
};
// Fresh triangulation graph defaults: new-loop masks 1/1, split-copy 0x54ef,
// no callbacks or user-data inheritance. New coordinates/labels start at zero.
std::pair<std::size_t, std::size_t> split_native_vu_edge(NativeVuGraph &, std::size_t,
                                                         TubeBudget &);
void twist_native_vu_vertices(NativeVuGraph &, std::size_t, std::size_t, TubeBudget &);
struct NativeVuInputLoop {
    std::size_t begin = 0, end = 0; // Source half-open interval before markers.
    std::size_t trimmed_end = 0, base = native_vu_null, inserted_vertices = 0;
};
struct NativeVuInput {
    NativeVuGraph graph;
    std::vector<NativeVuInputLoop> loops;
    std::size_t first_loop = native_vu_null;
    Json report;
};
// Builds the original indexed input loops only. Source points are immutable;
// XY suppression changes graph nodes, not the input array or its indexing.
NativeVuInput build_native_vu_input(const std::vector<Point3> &, double xy_tolerance, TubeBudget &);
} // namespace p3d::swept_detail
