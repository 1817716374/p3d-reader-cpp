#pragma once
#include "native_tube_mesh_index_rules.hpp"
namespace p3d::swept_detail {
enum NativePolyfaceChannel : std::size_t {
    point_channel,
    parameter_channel,
    normal_channel,
    color_channel,
    face_channel,
    polyface_channel_count
};
struct NativePolyfaceIndexState {
    // Explicit flags after native available-data activation. Inactive original
    // buffers are retained unless the native consumer explicitly clears them.
    std::array<bool, polyface_channel_count> active{true, false, false, false, false};
    std::array<std::vector<std::int32_t>, polyface_channel_count> indices;
};
struct NativePolyfaceVisitorFacet {
    // Points exclude the visitor's wrap-one closing point. Client indices and
    // visibility are the prepared visitor arrays, including their actual wrap.
    std::vector<Point3> points;
    std::array<std::vector<std::int32_t>, polyface_channel_count> client_indices;
    std::vector<std::uint8_t> visible;
};
struct NativePolyfaceTriangulation {
    NativePolyfaceIndexState output;
    bool native_succeeded = false, complete = false;
    Json report;
};
// Native edge-limit-three consumer for explicitly prepared visitor facets.
// No raw mesh-style conversion or visitor extraction is implied. Failed faces
// are skipped; successful faces remain in output even if native_succeeded=false.
// complete also requires all requested channels and triangle generation.
NativePolyfaceTriangulation
triangulate_native_polyface_facets(const std::vector<NativePolyfaceVisitorFacet> &,
                                   const NativePolyfaceIndexState &, TubeBudget &);
} // namespace p3d::swept_detail
