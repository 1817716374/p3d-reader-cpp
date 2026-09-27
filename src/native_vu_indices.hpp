#pragma once
#include "native_vu_graph.hpp"
namespace p3d::swept_detail {
struct NativeVuIndices {
    bool succeeded = false;
    // Native output, including the terminated partial face on an unresolved
    // vertex. Callers must check succeeded before using this as geometry.
    std::vector<std::int32_t> indices;
    Json report;
};
// Projected-loop caller: signed one-based indices, no new vertex creation,
// no optional boundary output. Resolves the first numbered sector of each
// vertex in native traversal order, not by coordinates or nearest point.
NativeVuIndices collect_native_vu_source_indices(NativeVuGraph &, TubeBudget &);
} // namespace p3d::swept_detail
