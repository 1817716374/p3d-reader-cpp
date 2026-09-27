#pragma once
#include "native_tube_mesh_patch.hpp"
#include <p3d/swept_patches.hpp>
namespace p3d::swept_detail {
struct TubeMeshControlRange {
    Point3 low{}, high{};
    double squared_extent = 0;
    std::size_t accepted = 0, disconnected = 0, rejected_weights = 0;
    bool native_null = true, degenerate = true;
};
// Native weighted CONTROL-point range, not curve extrema or endpoint distance.
TubeMeshControlRange tube_mesh_control_range(const BsplineCurve &, TubeBudget &);
struct TubeMeshPathCurve {
    std::size_t strip = 0, selection_index = 0;
    double u = 0;
    BsplineCurve curve;
};
struct TubeMeshPathSamples {
    bool success = false;
    std::vector<TubeMeshPathCurve> curves; // Retained selection order; no equality deduplication.
    std::vector<double> parameters;
    Json report;
};
// 89c10: fixed iso-U selectors, native degenerate-range removal, shared V tree.
TubeMeshPathSamples sample_tube_mesh_path(const std::vector<BsplineSurface> &strips,
                                          bool source_profile_closed, double chord_tolerance,
                                          double angle_tolerance, std::size_t max_sample_nodes,
                                          TubeBudget &);
struct TubeMeshSampledPatch {
    TubeMeshPatchPreparation preparation;
    TubeMeshPathSamples path;
};
struct TubeMeshGridPreparation {
    bool success = false;
    TubeSectionMeshSamples section;
    std::vector<TubeMeshSampledPatch> patches;
    Json report;
};
// One getPatches group: first-patch U sampling, then each patch's boundary,
// strip-count correspondence and V sampling. No vertices, faces or cap welding.
TubeMeshGridPreparation prepare_tube_mesh_grid(const SweptBodyPatchGroup &,
                                               bool source_profile_closed, double chord_tolerance,
                                               double angle_tolerance, std::size_t max_sample_nodes,
                                               TubeBudget &);
} // namespace p3d::swept_detail
