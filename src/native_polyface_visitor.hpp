#pragma once
#include "native_builder_polyface.hpp"
#include "native_polyface_layout.hpp"

namespace p3d::swept_detail {
struct NativePolyfaceVectorTags {
    std::uint32_t num_per_struct = 1, tag = 0, index_family = 0, indexed_by = 0;
};
// Typed runtime pools and their independent layout metadata. Counts are always
// derived from actual arrays; this is not an on-disk object layout.
struct NativePolyfaceMesh {
    NativeBuilderPolyface data;
    std::vector<Point3> double_colors;
    std::vector<std::array<float, 3>> float_colors;
    std::vector<std::uint32_t> integer_colors, color_table;
    // Independent native extensions. Do not infer engineering semantics or
    // per-corner correspondence from their lengths alone.
    std::vector<Point2> face_uv_points;
    std::vector<std::uint64_t> face_material_ids;
    std::vector<std::int32_t> face_smooth_groups;
    std::uint64_t texture_id = 0;
    // Exact UTF-16 storage; native query-copy may include a terminating zero
    // in the string's logical contents. Empty storage yields a null query CP.
    std::u16string illumination_name;
    std::array<bool, native_polyface_pool_count> pool_active{};
    std::array<std::uint32_t, native_polyface_pool_count> pool_rows{};
    std::array<std::uint32_t, polyface_channel_count> index_rows{};
    std::array<NativePolyfaceVectorTags, native_polyface_pool_count> pool_tags{};
    std::array<NativePolyfaceVectorTags, polyface_channel_count> index_tags{};
    NativePolyfaceVectorTags edge_chain_tags;
    std::uint32_t edge_chain_rows = 0;
    bool edge_chain_active = false;
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
