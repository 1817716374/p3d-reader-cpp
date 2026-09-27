#pragma once
#include "native_builder_coordinates.hpp"
#include "native_polyface_triangulate.hpp"

namespace p3d::swept_detail {
// Prepared runtime face data, not a file-layout struct or range constructor.
struct NativeBuilderFaceData {
    std::array<Point2, 2> parameter_distance_range{}, parameter_range{};
    std::array<Point3, 2> point_range{}, normal_range{};
    std::uint64_t source_index = 0;
    std::array<std::int64_t, 3> face_indices{0, 0, -1};
};
struct NativeBuilderEdgeChain {
    std::uint32_t topology_type = 0;
    std::vector<std::uint32_t> topology_ids;
    std::vector<std::int32_t> point_indices;
};
struct NativeBuilderPolyface {
    NativeBuilderCoordinateBatch coordinates;
    // Source pointer availability follows buffer presence, not these activity
    // flags. This is the native query view; it is not a raw mesh-style decoder.
    NativePolyfaceIndexState indices;
    std::vector<NativeBuilderFaceData> face_data;
    std::vector<NativeBuilderEdgeChain> edge_chains;
    std::uint32_t num_per_face = 0;
    bool two_sided = false;
};
struct NativeBuilderPolyfaceOptions {
    NativeBuilderCoordinateOptions coordinates;
    bool normals_required = true, parameters_required = true;
};
struct NativeBuilderPolyfaceOutput {
    NativeBuilderCoordinates coordinates;
    NativePolyfaceIndexState indices;
    std::vector<NativeBuilderFaceData> face_data;
    std::vector<NativeBuilderEdgeChain> edge_chains;
    std::uint32_t num_per_face = 0;
    bool two_sided = false, parameter_pool_active = false, normal_pool_active = false;
    bool native_succeeded = false, complete = false;
    Json report;
};
// Fresh builder's AddPolyface_matched sequence, using already placed query
// coordinates and already prepared face data. Failed layout matches leave the
// builder unchanged and later sources are still attempted. This does not run
// AddPolyface's missing-attribute, face-data or triangulation preparation.
// Native invalid remap keys select slot zero; this is retained and diagnosed,
// never presented as complete output. Source color indices are not transferred
// by this native path and their presence also prevents complete=true.
NativeBuilderPolyfaceOutput
assemble_native_builder_polyfaces(const std::vector<NativeBuilderPolyface> &,
                                  const NativeBuilderPolyfaceOptions &, TubeBudget &);
} // namespace p3d::swept_detail
