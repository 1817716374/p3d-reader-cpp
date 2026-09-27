#pragma once
#include "native_tube_mesh_group.hpp"
#include "native_tube_mesh_cap.hpp"
#include "native_polyface_prepare.hpp"
#include "native_polyface_finalize.hpp"
namespace p3d::swept_detail {
struct NativeTubeMeshAssemblyOptions {
    bool normals = true, parameters = true;
    bool profile_closed = false, path_closed = false, cap_eligible = false;
    std::uint32_t parameter_mode = 0;
};
struct NativeTubeMeshAssembly {
    std::vector<TubeMeshGroupOutput> groups;
    // Original group/patch/strip order, followed by both caps when published.
    std::vector<NativePolyfaceMesh> sources;
    NativeTubeMeshCapPair caps;
    NativePreparedPolyfaceAssembly assembly;
    std::optional<NativePolyfaceFinalization> finalized;
    bool complete = false;
    Json report;
};
// Consume already prepared getPatches groups. Caller must resolve the original
// profile/path closure and cap eligibility; this does not infer them from the
// generated surfaces. Incomplete groups/caps retain diagnostics but do not
// certify the assembled mesh. Source-file dispatch is a separate operation.
NativeTubeMeshAssembly
assemble_native_tube_mesh_groups(const std::vector<TubeMeshGridPreparation> &,
                                 const NativeTubeMeshAssemblyOptions &, TubeBudget &);
} // namespace p3d::swept_detail
