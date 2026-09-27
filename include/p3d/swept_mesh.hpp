#pragma once
#include <p3d/reader.hpp>
namespace p3d {
struct SweptBodyMeshOptions {
    double chord_tolerance = .01, angle_tolerance = .1;
    bool normals = true, parameters = true;
    std::uint32_t parameter_mode = 0;
    std::size_t max_control_points = 1000000, max_work = 10000000;
    std::size_t max_patches = 1000000, max_sample_nodes = 1000000;
};
struct SweptBodyMeshFaceData {
    std::array<Point2, 2> parameter_distance_range{}, parameter_range{};
    std::array<Point3, 2> point_range{}, normal_range{};
    std::uint64_t source_index = 0;
    // Native record contents; these are not material part IDs.
    std::array<std::int64_t, 3> face_indices{};
};
struct SweptBodyMeshResult {
    Json source;
    std::string status = "not_meshed"; // meshed / native_failure / incomplete / not_meshed
    std::vector<Point3> points, normals;
    std::vector<Point2> parameters;
    // Independent one-based indices, zero face terminators. A negative point
    // index hides its outgoing edge; do not replace it with a negative vertex ID.
    std::vector<std::int32_t> point_indices, normal_indices, parameter_indices, face_data_indices;
    std::vector<SweptBodyMeshFaceData> face_data;
    bool two_sided = false;
    Json layout = Json::object(), report = Json::object();
};
// Native mesh route from a decoded P3DSweptBody table: source conditions,
// whole-profile patches, sampling, side strips, caps, attributes and finalization.
// Keeps original source and reports; performs only native coordinate pooling.
// Tolerances drive native sampling, not a certified global surface-error bound.
// No material-part mapping, manifold proof or automatic scene replacement.
// Non-meshed results contain no partial mesh arrays. Allocation failure throws;
// unsupported layouts, unsafe data and resource failures return not_meshed.
SweptBodyMeshResult mesh_bgfb_swept_body(const Json &, const SweptBodyMeshOptions & = {});
} // namespace p3d
