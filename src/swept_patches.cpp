#include <p3d/swept_patches.hpp>
#include "native_tube_patches.hpp"
namespace p3d {
SweptBodyPatchResult reconstruct_bgfb_swept_body_patches(const Json &table,
                                                         const SweptBodyPatchOptions &options) {
    SweptBodyPatchResult out;
    out.source = table;
    swept_detail::TubeBudget budget{options.max_control_points, options.max_work, 0};
    out.report = {{"scope", "whole_section_swept_patches"},
                  {"native_result", nullptr},
                  {"mesh_status", "not_evaluated"}};
    try {
        require(table.is_object() && table.value("_type", Json()) == "P3DSweptBody",
                "swept patch source type");
        require(options.max_control_points > 0 && options.max_control_points <= UINT32_MAX &&
                    options.max_work > 0 && options.max_patches > 0,
                "swept patch limits must be positive; control limit must fit uint32");
        auto native =
            swept_detail::generate_tube_patch_groups(table.at("profile"), table.at("path"), budget);
        out.report["generation"] = std::move(native.report);
        out.report["preparation"] = std::move(native.preparation.report);
        out.report["orientation"] = std::move(native.preparation.orientation);
        out.report["section_placement"] = std::move(native.preparation.sections.report);
        require(native.groups.size() <= options.max_patches, "swept patch group budget");
        std::size_t count = 0;
        for (auto &g : native.groups) {
            require(g.surfaces.size() <= options.max_patches - count, "swept patch count budget");
            count += g.surfaces.size();
            curve_detail::BezierWork{budget.work, budget.max_work}.charge(1);
            auto &group = out.groups.emplace_back();
            group.source_curve = g.source_curve;
            for (auto &s : g.surfaces) {
                curve_detail::BezierWork{budget.work, budget.max_work}.charge(1);
                require(s.pcurves.empty(), "native patch copy unexpectedly contains pcurves");
                group.patches.push_back({std::move(s.geometry), std::move(s.boundaries)});
            }
        }
        out.status = native.success ? "reconstructed" : "native_failure";
        out.report["native_result"] = native.success;
        out.report["patch_count"] = count;
    } catch (const std::bad_alloc &) {
        throw;
    } catch (const std::exception &e) {
        out.groups.clear();
        out.status = "not_reconstructed";
        out.report["native_result"] = nullptr;
        out.report["reason"] = e.what();
    }
    out.report["work_used"] = budget.work;
    return out;
}
} // namespace p3d
