#include "native_tube_facet_groups.hpp"
#include "native_tube_working_paths.hpp"
#include <algorithm>
namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
} // namespace
TubeFacetGroupGeneration generate_tube_facet_groups(TubeFacetPreparation preparation,
                                                    TubeBudget &b) {
    TubeFacetGroupGeneration out;
    out.preparation = std::move(preparation);
    out.report = {{"scope", "native_facet_member_groups"},
                  {"status", "native_failure"},
                  {"members", Json::array()},
                  {"reversed_groups", Json::array()},
                  {"final_entity_generated", false}};
    auto fail = [&](const char *step) {
        out.groups.clear();
        out.report["failure_step"] = step;
        out.report["work_used"] = b.work;
        return std::move(out);
    };
    auto &input = out.preparation;
    if (!input.success)
        return fail("input_preparation");
    require(input.placement && input.placement->success,
            "native facet group preparation lacks successful path placement");
    auto &paths = input.placement->branches;
    auto *prefix = facet_working_path(paths, paths.prefix),
         *suffix = facet_working_path(paths, paths.suffix);
    require(bool(prefix) == bool(input.placement->prefix_transform) &&
                bool(suffix) == bool(input.placement->suffix_transform),
            "native facet group path and placement presence disagree");
    require(input.profile.groups.size() <= b.max_control_points, "native facet group count budget");
    out.report["shared_path"] = prefix && prefix == suffix;
    std::size_t count = 0;
    for (std::size_t group = 0; group < input.profile.groups.size(); ++group) {
        charge(b, 1);
        std::vector<TubeFacetMemberSurfaces> members;
        for (std::size_t member = 0; member < input.profile.groups[group].size(); ++member) {
            require(count < b.max_control_points, "native facet cumulative member budget");
            ++count;
            auto sections = prepare_tube_facet_sections(input, group, member, b);
            Json visit{{"group", group},
                       {"source_member", member},
                       {"section_preparation", std::move(sections.report)}};
            if (!sections.success) {
                out.report["members"].push_back(std::move(visit));
                return fail("section_preparation");
            }
            TubeFacetMemberSurfaces generated{member, {}};
            auto result = append_tube_facet_surfaces(
                generated.surfaces, nullptr, prefix,
                sections.prefix.empty() ? nullptr : sections.prefix.front().get(), suffix,
                sections.suffix.empty() ? nullptr : sections.suffix.front().get(), b);
            const auto &working = result.generation.composition.working_sources;
            update_facet_working_path(prefix, working[0], b);
            // Shared references were updated in native prefix/suffix order by
            // generation. Both slots now hold the same final state; do not
            // overwrite it with an earlier prefix snapshot or initial input.
            if (suffix != prefix)
                update_facet_working_path(suffix, working[2], b);
            visit["output"] = std::move(result.report);
            visit["generation"] = std::move(result.generation.report);
            visit["surface_count"] = generated.surfaces.size();
            out.report["members"].push_back(std::move(visit));
            if (result.generation.status == TubeFacetSeamStatus::pending_general) {
                out.report["status"] = "pending_general";
                return fail("surface_generation");
            }
            // Unlike d00f0 itself, 835c0 requires a nonempty per-member result.
            if (generated.surfaces.empty())
                return fail("empty_member_surfaces");
            members.push_back(std::move(generated));
        }
        // Keep empty groups at this stage: native code appends them as well.
        out.groups.push_back(std::move(members));
    }
    const auto &flags = input.orientation.at("flags");
    require(flags.is_array(), "native facet group orientation flag array");
    for (std::size_t i = 0; i < flags.size(); ++i) {
        charge(b, 1);
        require(flags[i].is_boolean(), "native facet group orientation flag type");
        if (flags[i].get<bool>()) {
            require(i < out.groups.size(), "native facet orientation group index undefined");
            charge(b, out.groups[i].size());
            std::reverse(out.groups[i].begin(), out.groups[i].end());
            out.report["reversed_groups"].push_back(i);
        }
    }
    out.success = true;
    out.report["status"] = "groups_prepared";
    out.report["member_count"] = count;
    out.report["group_count"] = out.groups.size();
    out.report["first_member_available"] = !out.groups.empty() && !out.groups[0].empty();
    out.report["work_used"] = b.work;
    return out;
}
TubeFacetGroupGeneration generate_tube_facet_groups(const Json &profile, const Json &path,
                                                    TubeBudget &b) {
    return generate_tube_facet_groups(prepare_tube_facet_inputs(profile, path, b), b);
}
} // namespace p3d::swept_detail
