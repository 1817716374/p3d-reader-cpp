#pragma once
#include "native_tube_mesh_trim_output.hpp"
namespace p3d::swept_detail {
struct TubeMeshGroupOptions {
    bool normals = true, parameters = true;
    // Resolved source-profile/path flags from the original caller. These are
    // not the closedU/closedV flags of an individual generated patch.
    bool profile_closed = false, path_closed = false;
    // The caller's resolved cap eligibility; collection alone creates no caps.
    bool collect_cap_boundaries = false;
};
struct TubeMeshGroupOutput {
    // Native patch-major, then strip-major order; each mesh retains its own
    // source pools and local signed indices. No invented geometry sharing.
    std::vector<TubeMeshRegularMesh> strips;
    std::vector<Point3> start_boundary, end_boundary;
    bool traversal_completed = false, strips_complete = false;
    Json report;
};
// Consume one successfully prepared getPatches group. Select original edge
// visibility from section discontinuities, connect both strip routes using
// one state, and collect native cap boundary arguments. No cap triangulation,
// final builder insertion, coordinate combination or material binding yet.
TubeMeshGroupOutput emit_tube_mesh_group(const TubeMeshGridPreparation &,
                                         const TubeMeshGroupOptions &, TubeBudget &);
} // namespace p3d::swept_detail
