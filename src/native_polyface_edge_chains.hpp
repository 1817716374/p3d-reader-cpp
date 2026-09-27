#pragma once
#include "native_polyface_face_data.hpp"
namespace p3d::swept_detail {
struct NativePolyfaceEdgeChains {
    NativePolyfaceFaceDataState output;
    // Native P3DStatus: zero is success, one means chains already existed.
    std::uint32_t native_status = 1;
    bool native_succeeded = false, complete = false;
    Json report;
};
// Original AddEdgeChains for prepared indexed-query data. The native draw
// method argument is unused. Emits individual two-vertex records, preserving
// original topology IDs, doubled facet numbering and second-pass orientation.
NativePolyfaceEdgeChains build_native_polyface_edge_chains(const NativePolyfaceFaceDataState &,
                                                           std::size_t draw_method_index,
                                                           TubeBudget &);
} // namespace p3d::swept_detail
