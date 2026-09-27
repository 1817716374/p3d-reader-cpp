#include "native_tube_mesh_path.hpp"
namespace p3d::swept_detail {
TubeMeshGridPreparation prepare_tube_mesh_grid(const SweptBodyPatchGroup &group,
                                               bool source_profile_closed, double chord_tolerance,
                                               double angle_tolerance, std::size_t max_sample_nodes,
                                               TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    TubeMeshGridPreparation out;
    out.report = {{"scope", "native_swept_mesh_group_grid"},
                  {"status", "native_failure"},
                  {"source_curve", group.source_curve},
                  {"mesh_generated", false}};
    require(group.patches.size() <= budget.max_control_points, "native grid patch count budget");
    auto fail = [&](const char *step) {
        out.report["failure_step"] = step;
        out.report["completed_patches"] = out.patches.size();
        out.report["work_used"] = budget.work;
        return std::move(out);
    };
    if (group.patches.empty())
        return fail("empty_patch_group");
    // Read one patch at a time. The first patch is also the source of the
    // group's original compressed U knots and section sampling intervals.
    auto s = BsplineSurface::from_bgfb(group.patches.front().geometry);
    out.section = sample_tube_mesh_section(s, source_profile_closed, chord_tolerance,
                                           angle_tolerance, max_sample_nodes, budget);
    if (!out.section.success)
        return fail("section_sampling");
    std::size_t controls = 0, samples = 0;
    for (std::size_t i = 0; i < group.patches.size(); ++i) {
        work.charge(1);
        if (i)
            s = BsplineSurface::from_bgfb(group.patches[i].geometry);
        auto prepared = prepare_tube_mesh_patch(s, group.patches[i].boundary_points, budget);
        out.report["current_patch"] = i;
        if (!prepared.success) {
            out.report["patch_report"] = prepared.report;
            return fail("patch_preparation");
        }
        // Native correspondence is positional and count-based. Do not rescale
        // later patch knots or search for equal geometry to repair a mismatch.
        if (prepared.strips.size() != out.section.interval_samples.size())
            return fail("strip_section_count");
        for (const auto &strip : prepared.strips) {
            require(strip.poles().size() <= budget.max_control_points - controls,
                    "native grid cumulative strip-control budget");
            controls += strip.poles().size();
        }
        auto path = sample_tube_mesh_path(prepared.strips, source_profile_closed, chord_tolerance,
                                          angle_tolerance, max_sample_nodes, budget);
        if (!path.success) {
            out.report["path_report"] = path.report;
            return fail("path_sampling");
        }
        require(path.parameters.size() <= budget.max_control_points - samples,
                "native grid cumulative path-sample budget");
        samples += path.parameters.size();
        out.patches.push_back({std::move(prepared), std::move(path)});
    }
    out.success = true;
    out.report["status"] = "prepared";
    out.report["completed_patches"] = out.patches.size();
    out.report["strip_controls"] = controls;
    out.report["path_samples"] = samples;
    out.report["work_used"] = budget.work;
    return out;
}
} // namespace p3d::swept_detail
