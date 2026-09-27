#pragma once
#include "native_tube.hpp"
#include "native_curve_sampling.hpp"
namespace p3d::swept_detail {
struct TubeSectionMeshSamples {
    std::optional<BsplineCurve> reference;
    curve_detail::NativeKnotData knots;
    curve_detail::NativeCurveBreaks breaks;
    std::vector<double> break_set;
    // One normalized sample list per original compressed active knot interval.
    std::vector<std::vector<double>> interval_samples;
    std::vector<std::size_t> discontinuity_intervals;
    bool end_discontinuity = false, success = false;
    Json report;
};
// The first patch of one native getPatches group supplies isoV(0). The closed
// flag comes from the ORIGINAL PROFILE endpoint query, not surface.closedU.
// Prepares native U sampling only; no V rows, triangles, caps or material IDs.
TubeSectionMeshSamples sample_tube_mesh_section(const BsplineSurface &first_patch,
                                                bool source_profile_closed, double chord_tolerance,
                                                double angle_tolerance,
                                                std::size_t max_sample_nodes, TubeBudget &);
} // namespace p3d::swept_detail
