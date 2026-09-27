#pragma once
#include "native_polyface_face_data.hpp"
namespace p3d::swept_detail {
inline constexpr std::uint32_t native_mtg_exterior = 1, native_mtg_boundary = 2,
                               native_mtg_primary = 0x200, native_mtg_polar_loop = 0x1000;
struct NativeMtgNode {
    std::size_t vertex_successor = 0, face_successor = 0;
    std::uint32_t mask = 0;
    std::int32_t vertex_index = -1, read_index = -1;
};
struct NativePolyfaceHalfEdge {
    std::size_t vertex0 = 0, vertex1 = 0; // Positive one-based original vertex identities.
    std::size_t read_index = 0, successor_read_index = 0, node = 0;
    bool visible = false;
};
struct NativePolyfaceConnectivity {
    std::vector<Point3> points;
    std::vector<NativeMtgNode> nodes;
    std::vector<NativePolyfaceHalfEdge> half_edges; // Native sorted order.
    std::vector<std::size_t> read_to_half_edge;     // SIZE_MAX means no source corner.
    bool native_succeeded = false, complete = false;
    Json report;
};
// Original indexed-query route, including all visible/invisible source edges.
// Uses original vertex identities; equal coordinates are never merged here.
// The optional native degeneracy filter packs repeated vertex identities and
// skips two-vertex faces. This does not compute normals or certify a manifold.
NativePolyfaceConnectivity build_native_polyface_connectivity(const NativePolyfaceFaceDataState &,
                                                              bool ignore_degeneracies,
                                                              TubeBudget &);
} // namespace p3d::swept_detail
