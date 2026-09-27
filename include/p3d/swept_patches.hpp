#pragma once
#include <p3d/reader.hpp>
namespace p3d {
struct SweptBodyPatchOptions {
    std::size_t max_control_points = 1000000;
    std::size_t max_work = 10000000;
    std::size_t max_patches = 1000000;
};
struct SweptBodyPatch {
    Json geometry; // BsplineSurface; null boundaries does NOT mean untrimmed.
    std::vector<std::vector<Point2>> boundary_points; // Active runtime surface fractions.
};
struct SweptBodyPatchGroup {
    // Index in whole-profile curve conversion, not a primitive/face/material ID.
    std::size_t source_curve = 0;
    std::vector<SweptBodyPatch> patches;
};
struct SweptBodyPatchResult {
    Json source;
    std::string status = "not_reconstructed"; // reconstructed / native_failure / not_reconstructed
    std::vector<SweptBodyPatchGroup> groups;
    Json report = Json::object();
};
// Reconstruct the native getPatches route used by body mesh preparation. Each
// complete section ring is converted before patch generation; output is NOT
// the three-level member grouping or face numbering of reconstruct_bgfb_swept_body.
// Keeps source ring/patch order, runtime trims and native working-path aliases.
// No cap generation, triangulation, material assignment or equality deduplication.
// capped is retained in source but has no effect at this stage.
// Native rejection may retain earlier groups. Unsupported/data/resource failure
// clears all derived output; memory allocation failure propagates.
SweptBodyPatchResult reconstruct_bgfb_swept_body_patches(const Json &,
                                                         const SweptBodyPatchOptions & = {});
} // namespace p3d
