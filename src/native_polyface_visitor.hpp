#pragma once
#include "native_builder_polyface.hpp"
#include "native_polyface_layout.hpp"
#include <functional>

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
struct NativePolyfaceVisitedData {
    // Values include the visitor's actual wrap. Some original visitors produce
    // values without client indices, or read multiple color representations.
    std::vector<Point3> normals, double_colors;
    std::vector<Point2> parameters;
    std::vector<std::array<float, 3>> float_colors;
    std::vector<std::uint32_t> integer_colors, color_table;
    std::vector<NativeBuilderFaceData> face_data;
    std::vector<std::size_t> index_positions;
};
struct NativePolyfaceVisit {
    std::vector<NativePolyfaceVisitorFacet> facets;
    std::vector<NativePolyfaceVisitedData> data;
    std::vector<std::size_t> read_indices;
    bool complete = false;
    Json report;
};
struct NativePolyfaceMeshTriangulation {
    NativePolyfaceMesh output;
    bool native_succeeded = false, complete = false;
    Json report;
};
// Original styles 1/3/4/5/6, without conversion. Facet points exclude the actual
// appended wrap; values and client indices retain it. Grid visitors ignore
// all_data and clamp wrap to ten. Source read indices are offsets for style 1,
// facet ordinals otherwise. Unsafe original array reads are explicitly rejected.
NativePolyfaceVisit visit_native_polyface(const NativePolyfaceMesh &, TubeBudget &,
                                          bool all_data = true, std::uint32_t wrap = 1);
using NativePolyfaceFacetConsumer = std::function<void(
    std::size_t, const NativePolyfaceVisitorFacet &, const NativePolyfaceVisitedData &)>;
// Read-only incremental traversal. Retains read positions and diagnostics,
// releases each consumed facet's values. The consumer must not mutate source.
NativePolyfaceVisit consume_native_polyface(const NativePolyfaceMesh &, TubeBudget &,
                                            const NativePolyfaceFacetConsumer &,
                                            bool all_data = true, std::uint32_t wrap = 0);
// Internal generation traversal: all data, wrap zero. The consumer runs between
// native advances and may update only the selected normal/UV pool, its index
// values and parameter ranges. Do not change source points or layout. Missing
// reads of the generated channel are not truncation errors; unsafe reads still
// fail. Later faces query the current attribute arrays, not a frozen snapshot.
// Returns read positions and diagnostics; consumed facet/value arrays stay empty.
NativePolyfaceVisit visit_native_polyface_attribute_updates(NativePolyfaceMesh &, TubeBudget &,
                                                            std::size_t selected_channel,
                                                            const NativePolyfaceFacetConsumer &);
// Convert layout, activate available data, extract the original independent
// client indices and run the edge-limit consumer. Existing pools are preserved.
// Unsupported mesh styles are rejected before the native null-visitor access.
NativePolyfaceMeshTriangulation triangulate_native_polyface_mesh(const NativePolyfaceMesh &,
                                                                 TubeBudget &,
                                                                 std::size_t max_edges = 3);
} // namespace p3d::swept_detail
