#pragma once
#include "native_builder_polyface.hpp"
#include "native_polyface_layout.hpp"

namespace p3d::swept_detail {
// Typed runtime pools and their independent layout metadata. Counts are always
// derived from actual arrays; this is not an on-disk object layout.
struct NativePolyfaceMesh {
    NativeBuilderPolyface data;
    std::vector<Point3> double_colors;
    std::vector<std::array<float, 3>> float_colors;
    std::vector<std::uint32_t> integer_colors, color_table;
    std::array<bool, native_polyface_pool_count> pool_active{};
    std::array<std::uint32_t, native_polyface_pool_count> pool_rows{};
    std::array<std::uint32_t, polyface_channel_count> index_rows{};
    std::uint32_t mesh_style = 1, num_per_row = 0;
};
struct NativePolyfaceVisit {
    std::vector<NativePolyfaceVisitorFacet> facets;
    std::vector<std::size_t> read_indices;
    bool complete = false;
    Json report;
};
struct NativePolyfaceMeshTriangulation {
    NativePolyfaceMesh output;
    bool native_succeeded = false, complete = false;
    Json report;
};
// Indexed style only. A failed advance stops traversal; wrap is independent of
// all_data. Facet points exclude the actual appended wrap, while client indices
// and visibility retain it. Pool values remain in the input without conversion.
NativePolyfaceVisit visit_native_polyface(const NativePolyfaceMesh &, TubeBudget &,
                                          bool all_data = true, std::uint32_t wrap = 1);
// Convert layout, activate available data, extract the original independent
// client indices and run the edge-limit consumer. Existing pools are preserved.
// Unsupported mesh styles are rejected before the native null-visitor access.
NativePolyfaceMeshTriangulation triangulate_native_polyface_mesh(const NativePolyfaceMesh &,
                                                                 TubeBudget &,
                                                                 std::size_t max_edges = 3);
} // namespace p3d::swept_detail
