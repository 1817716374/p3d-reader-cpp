#pragma once
#include "native_tube_mesh_cap_input.hpp"
#include "native_polyface_visitor.hpp"
#include "native_polygon_parameters.hpp"
namespace p3d::swept_detail {
struct NativeTubeMeshCapOptions {
    bool normals_required = true, parameters_required = true;
    std::uint32_t parameter_mode = 0;
};
struct NativeTubeMeshCap {
    NativeTubeMeshCapInput input;
    // Final addTriangulation point/face slots before native builder pooling.
    std::vector<Point3> polygon_points;
    std::vector<std::int32_t> polygon_indices;
    NativePolyfaceMesh mesh;
    bool native_succeeded = false, triangulation_succeeded = false, complete = false;
    Json report;
};
// Original fresh-builder polyline cap: fixed maximum face edges=3, no maximum
// edge length, no placement or builder winding override. The start-cap caller
// reverses each input ring. Native helper success means a mesh handle exists;
// it may be empty, and does not establish geometric validity or a closed solid.
NativeTubeMeshCap emit_native_tube_mesh_cap(const std::vector<std::vector<Point3>> &, bool reverse,
                                            const NativeTubeMeshCapOptions &, TubeBudget &);
struct NativeTubeMeshCapPair {
    std::optional<NativeTubeMeshCap> start, end;
    bool published = false, complete = false;
    Json report;
};
// Original outer gate: cap eligibility and equal ring counts, then start
// reverse=true followed by end reverse=false. Both handles must succeed before
// publication; detailed attempts remain available even if not published.
NativeTubeMeshCapPair emit_native_tube_mesh_cap_pair(const std::vector<std::vector<Point3>> &start,
                                                     const std::vector<std::vector<Point3>> &end,
                                                     bool eligible,
                                                     const NativeTubeMeshCapOptions &,
                                                     TubeBudget &);
} // namespace p3d::swept_detail
