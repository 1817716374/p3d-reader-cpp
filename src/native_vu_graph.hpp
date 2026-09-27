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
    bool active = true;
    std::int32_t internal_data = 0;
};
// Stable indices express the original node identity, not geometric equality.
// New nodes are inserted after the current tail. Deletion preserves survivor
// order and selects the last survivor as tail; freed slots can be reused.
struct NativeVuGraph {
    std::vector<NativeVuNode> nodes;
    std::size_t tail = native_vu_null;
    std::size_t free_head = native_vu_null;
};
// Validated active traversal; inactive slots must occur exactly once in the
// original free chain. Slots are never compacted or geometrically deduplicated.
std::vector<std::size_t> native_vu_all_nodes(const NativeVuGraph &, TubeBudget &);
// Removes both sides of marked edges. Preserves active traversal and native
// free-list reuse order. Failure leaves the graph unchanged.
std::size_t free_marked_native_vu_edges(NativeVuGraph &, std::uint32_t mask, TubeBudget &);
// Fresh triangulation graph defaults: new-loop masks 1/1, split-copy 0x54ef,
// no callbacks or user-data inheritance. New coordinates/labels start at zero.
std::pair<std::size_t, std::size_t> split_native_vu_edge(NativeVuGraph &, std::size_t,
                                                         TubeBudget &);
void twist_native_vu_vertices(NativeVuGraph &, std::size_t, std::size_t, TubeBudget &);
// Isolated two-node edge: face successors paired, vertex successors self.
// Both node payloads start at zero, unlike the boundary-masked split sling.
std::pair<std::size_t, std::size_t> make_native_vu_pair(NativeVuGraph &, TubeBudget &);
// Inserts an edge between two existing face sectors. New nodes copy endpoint
// XYZ only: the fresh graph has zero join-copy mask and no label inheritance.
std::pair<std::size_t, std::size_t> join_native_vu_sectors(NativeVuGraph &, std::size_t,
                                                       std::size_t, TubeBudget &);
struct NativeVuInputLoop {
    std::size_t begin = 0, end = 0; // Source half-open interval before markers.
    std::size_t trimmed_end = 0, base = native_vu_null, inserted_vertices = 0;
};
struct NativeVuInput {
    NativeVuGraph graph;
    std::vector<NativeVuInputLoop> loops;
    std::size_t first_loop = native_vu_null; // Input seed; may be deleted by a later merge.
    Json report;
};
// Builds the original indexed input loops only. Source points are immutable;
// XY suppression changes graph nodes, not the input array or its indexing.
NativeVuInput build_native_vu_input(const std::vector<Point3> &, double xy_tolerance, TubeBudget &);
// General XY polygon constructor: only X/Y disconnects, signed loop bounds
// allow an initial marker. Otherwise shares original point and mask insertion.
NativeVuInput build_native_vu_polygon_input(const std::vector<Point3> &, double xy_tolerance,
                                           TubeBudget &);
} // namespace p3d::swept_detail
