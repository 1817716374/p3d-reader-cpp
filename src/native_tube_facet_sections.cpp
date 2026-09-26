#include "native_tube_facet_sections.hpp"
#include "native_curve_area.hpp"
#include "native_curve_segment.hpp"
namespace p3d::swept_detail {
namespace {
void reverse_first(TubeFacetSectionBranches &out, TubeCurveViews &list, TubeBudget &budget,
                   Json &report) {
    require(!list.empty() && list.front(), "native facet section reversal has no first curve");
    require(budget.max_control_points <= UINT32_MAX, "native facet section reversal control limit");
    const auto old = list.front();
    auto working = std::make_shared<BsplineCurve>(*old);
    const bool ok = curve_detail::reverse_native_working_curve(
        *working, unsigned(budget.max_control_points), {budget.work, budget.max_work}, report);
    // The caller ignores ba7d0's status. Even a failed operation can have
    // changed the working geometry; preserve all actual pointer aliases.
    for (auto *views : {&out.original, &out.prefix, &out.suffix}) {
        curve_detail::BezierWork{budget.work, budget.max_work}.charge(views->size());
        for (auto &c : *views)
            if (c == old)
                c = working;
    }
    report["native_result"] = ok;
}
} // namespace
TubeFacetSectionBranches place_tube_facet_sections(const TubeCurveViews &source,
                                                   const Matrix4 *prefix, const Matrix4 *suffix,
                                                   bool ring_flag, TubeBudget &budget) {
    require(prefix || suffix, "native facet sections require a path branch");
    TubeFacetSectionBranches out;
    out.original = source;
    out.report = {{"scope", "native_facet_section_placement"},
                  {"ring_flag", ring_flag},
                  {"face_patches_generated", false}};
    auto fail = [&](const char *step) {
        out.report["status"] = "native_failure";
        out.report["failure_step"] = step;
        out.report["work_used"] = budget.work;
        return std::move(out);
    };
    if (prefix) {
        auto transformed =
            transform_tube_curve_list(out.original, prefix, suffix != nullptr, budget);
        out.prefix = std::move(transformed.curves);
        if (!suffix)
            out.original = out.prefix;
        const bool ok = transformed.report.at("native_result").get<bool>();
        out.report["prefix_transform"] = std::move(transformed.report);
        if (!ok)
            return fail("prefix_transform");
    }
    if (suffix) {
        auto transformed = transform_tube_curve_list(out.original, suffix, false, budget);
        out.original = std::move(transformed.curves);
        out.suffix = out.original;
        const bool ok = transformed.report.at("native_result").get<bool>();
        out.report["suffix_transform"] = std::move(transformed.report);
        if (!ok)
            return fail("suffix_transform");
    }
    // Even when no reversal is selected, d00f0 reads the first curve of every
    // present branch. Native empty-list access has no defined geometry result.
    require((!prefix || !out.prefix.empty()) && (!suffix || !out.suffix.empty()),
            "native facet section branch has no first curve");
    out.report["reversed_branch"] = nullptr;
    if (prefix && !ring_flag) {
        reverse_first(out, out.prefix, budget, out.report["reversal"]);
        out.report["reversed_branch"] = "prefix";
    }
    if (suffix && ring_flag) {
        reverse_first(out, out.suffix, budget, out.report["reversal"]);
        out.report["reversed_branch"] = "suffix";
    }
    out.success = true;
    out.report["status"] = "complete";
    out.report["work_used"] = budget.work;
    return out;
}
TubeFacetPreparation prepare_tube_facet_inputs(const Json &profile, const Json &path,
                                               TubeBudget &budget) {
    TubeFacetPreparation out;
    out.report = {{"scope", "native_facet_input_preparation"}, {"face_patches_generated", false}};
    auto fail = [&](const char *step) {
        out.report["status"] = "native_failure";
        out.report["failure_step"] = step;
        out.report["work_used"] = budget.work;
        return std::move(out);
    };
    out.report["source_validation"] = validate_tube_sources(profile, path, budget);
    if (!out.report.at("source_validation").at("accepted").get<bool>())
        return fail("source_validation");
    out.placement = prepare_tube_facet_path_placement(profile, path, budget);
    if (!out.placement->success)
        return fail("path_placement");
    out.orientation = curve_detail::native_facet_orientation_flags(
        profile, out.placement->branches.path.location.tangent, budget);
    if (!out.orientation.at("native_result").get<bool>())
        return fail("source_orientation");
    out.profile = partition_tube_facet_profile(std::make_shared<const Json>(profile), budget);
    if (!out.profile.report.at("native_result").get<bool>())
        return fail("profile_partition");
    out.success = true;
    out.report["status"] = "complete";
    out.report["work_used"] = budget.work;
    return out;
}
TubeFacetSectionBranches prepare_tube_facet_sections(const TubeFacetPreparation &input,
                                                     std::size_t group, std::size_t member,
                                                     TubeBudget &budget) {
    require(input.success && input.placement && input.placement->success,
            "native facet sections need successful input preparation");
    require(group < input.profile.groups.size() && member < input.profile.groups[group].size(),
            "native facet section member index");
    // 835c0 indexes the flag vector positionally, not by the source-ring map.
    // A compacted flag vector is not repaired by inserting inferred flags.
    const auto &flags = input.orientation.at("flags");
    require(flags.is_array() && group < flags.size() && flags[group].is_boolean(),
            "native facet source orientation flag index is undefined");
    const auto &p = *input.placement;
    auto prepared = prepare_tube_facet_member(input.profile.groups[group][member], budget);
    TubeCurveViews source{std::make_shared<const BsplineCurve>(std::move(prepared.curve))};
    auto out = place_tube_facet_sections(
        source, p.prefix_transform ? &*p.prefix_transform : nullptr,
        p.suffix_transform ? &*p.suffix_transform : nullptr, flags[group].get<bool>(), budget);
    out.report["source_preparation"] = std::move(prepared.report);
    out.report["source_group"] = group;
    out.report["partition_member"] = member;
    out.report["orientation_source_ring"] = input.orientation.at("source_rings").at(group);
    return out;
}
} // namespace p3d::swept_detail
