#pragma once
#include "native_tube.hpp"
namespace p3d::swept_detail {
struct TubeMeshSourceConditions {
    bool accepted = false, profile_closed = false, path_closed = false, cap_eligible = false;
    Json report;
};
// Original source frame/range test and endpoint queries, before getPatches.
// A nonplanar path forces the mesh path-closure flag false; this does not change
// the source curve. A parity profile with any open child rejects the call.
TubeMeshSourceConditions tube_mesh_source_conditions(const Json &profile, const Json &path,
                                                     bool capped, TubeBudget &);
} // namespace p3d::swept_detail
