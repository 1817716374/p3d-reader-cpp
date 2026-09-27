#pragma once
#include "native_polyface_visitor.hpp"
namespace p3d::swept_detail {
struct NativePolyfaceEdgeVisibility {
    NativePolyfaceIndexState output;
    Json report;
};
// Original post-combination point-index scan. Undirected edges use absolute
// vertex IDs, never positions. Later visible occurrences promote only the first
// remembered hidden occurrence; hidden duplicates in between are not rewritten.
// Other index channels/flags and an unterminated final token remain untouched.
NativePolyfaceEdgeVisibility
resolve_native_polyface_edge_visibility(const NativePolyfaceIndexState &, TubeBudget &);
struct NativePolyfaceFinalization {
    NativePolyfaceMesh output;
    std::vector<std::size_t> point_map, normal_map, parameter_map;
    bool combination_applied = false;
    Json report;
};
// Original sweep's final combineCoordinate followed by edge-visibility scan.
// Preserve all other metadata, including original unremapped edge chains and
// face ranges. Completion of this operation is not a mesh-validity certificate.
NativePolyfaceFinalization finalize_native_polyface_mesh(const NativePolyfaceMesh &, TubeBudget &);
} // namespace p3d::swept_detail
