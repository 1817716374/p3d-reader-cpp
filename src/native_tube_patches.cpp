#include "native_tube_patches.hpp"
#include "native_curve_area.hpp"
#include "native_curve_segment.hpp"
#include "native_tube_working_paths.hpp"
namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
void reverse_at(TubeFacetSectionBranches &out, TubeCurveViews &list, std::size_t index,
                TubeBudget &b, Json &report) {
    require(index < list.size() && list[index], "native patch positional flag index undefined");
    require(b.max_control_points <= UINT32_MAX, "native patch reversal control limit");
    const auto old = list[index];
    charge(b, old->knots().size());
    for (unsigned i = 0; i < 4; ++i)
        charge(b, old->poles().size());
    auto working = std::make_shared<BsplineCurve>(*old);
    const bool ok = curve_detail::reverse_native_working_curve(
        *working, unsigned(b.max_control_points), {b.work, b.max_work}, report);
    for (auto *views : {&out.original, &out.prefix, &out.suffix}) {
        charge(b, views->size());
        for (auto &c : *views)
            if (c == old)
                c = working;
    }
    report["native_result"] = ok; // The native caller ignores this operation's status.
}
} // namespace
TubeFacetSectionBranches place_tube_patch_sections(const TubeCurveViews &source,
                                                   const Matrix4 *prefix, const Matrix4 *suffix,
                                                   const Json &flags, TubeBudget &b) {
    require(prefix || suffix, "native patch sections require a path branch");
    require(source.size() <= b.max_control_points && flags.is_array() &&
                flags.size() <= b.max_control_points,
            "native patch section/flag count budget or layout");
    charge(b, source.size());
    TubeFacetSectionBranches out;
    out.original = source;
    out.report = {{"scope", "native_patch_section_placement"},
                  {"status", "native_failure"},
                  {"reversals", Json::array()}};
    auto fail = [&](const char *step) {
        out.report["failure_step"] = step;
        out.report["work_used"] = b.work;
        return std::move(out);
    };
    // Native list assignment shares pointers. Only a present suffix causes
    // the prefix transform helper to clone every occurrence separately.
    if (prefix) {
        auto transformed = transform_tube_curve_list(out.original, prefix, suffix != nullptr, b);
        out.prefix = std::move(transformed.curves);
        if (!suffix)
            out.original = out.prefix;
        const bool ok = transformed.report.at("native_result").get<bool>();
        out.report["prefix_transform"] = std::move(transformed.report);
        if (!ok)
            return fail("prefix_transform");
    }
    if (suffix) {
        auto transformed = transform_tube_curve_list(out.original, suffix, false, b);
        out.original = std::move(transformed.curves);
        out.suffix = out.original;
        const bool ok = transformed.report.at("native_result").get<bool>();
        out.report["suffix_transform"] = std::move(transformed.report);
        if (!ok)
            return fail("suffix_transform");
    }
    // Flags are consumed positionally, not repaired using source_ring labels.
    // Unlike getFacets this walks the entire flag list after ALL transforms.
    for (std::size_t i = 0; i < flags.size(); ++i) {
        charge(b, 1);
        require(flags[i].is_boolean(), "native patch orientation flag type");
        const bool flag = flags[i].get<bool>();
        if ((prefix && !flag) || (suffix && flag)) {
            Json record;
            reverse_at(out, flag ? out.suffix : out.prefix, i, b, record);
            record["curve"] = i;
            record["branch"] = flag ? "suffix" : "prefix";
            out.report["reversals"].push_back(std::move(record));
        }
    }
    out.success = true;
    out.report["status"] = "complete";
    out.report["work_used"] = b.work;
    return out;
}
TubePatchPreparation prepare_tube_patch_inputs(const Json &profile, const Json &path,
                                               TubeBudget &b) {
    TubePatchPreparation out;
    out.report = {{"scope", "native_whole_section_patch_preparation"}};
    auto fail = [&](const char *step) {
        out.report["status"] = "native_failure";
        out.report["failure_step"] = step;
        out.report["work_used"] = b.work;
        return std::move(out);
    };
    out.report["source_validation"] = validate_tube_sources(profile, path, b);
    if (!out.report.at("source_validation").at("accepted").get<bool>())
        return fail("source_validation");
    out.placement = prepare_tube_facet_path_placement(profile, path, b);
    if (!out.placement->success)
        return fail("path_placement");
    out.orientation = curve_detail::native_facet_orientation_flags(
        profile, out.placement->branches.path.location.tangent, b);
    if (!out.orientation.at("native_result").get<bool>())
        return fail("source_orientation");
    auto curves = prepare_tube_facet_curves(profile, b);
    out.report["profile_conversion"] = std::move(curves.report);
    TubeCurveViews source;
    require(curves.curves.size() <= b.max_control_points, "native patch section count budget");
    charge(b, curves.curves.size());
    for (auto &c : curves.curves)
        source.push_back(std::make_shared<const BsplineCurve>(std::move(c)));
    const auto &p = *out.placement;
    out.sections = place_tube_patch_sections(
        source, p.prefix_transform ? &*p.prefix_transform : nullptr,
        p.suffix_transform ? &*p.suffix_transform : nullptr, out.orientation.at("flags"), b);
    if (!out.sections.success)
        return fail("section_placement");
    out.success = true;
    out.report["status"] = "complete";
    out.report["work_used"] = b.work;
    return out;
}
TubePatchGeneration generate_tube_patch_groups(TubePatchPreparation preparation, TubeBudget &b) {
    TubePatchGeneration out;
    out.preparation = std::move(preparation);
    out.report = {{"scope", "native_whole_section_patches"},
                  {"status", "native_failure"},
                  {"curves", Json::array()}};
    auto fail = [&](const char *step) {
        out.report["failure_step"] = step;
        out.report["retained_groups"] = out.groups.size();
        out.report["work_used"] = b.work;
        return std::move(out);
    };
    auto &input = out.preparation;
    if (!input.success)
        return fail("input_preparation");
    require(input.placement && input.placement->success && input.sections.success,
            "native patches lack successful path/section placement");
    auto &paths = input.placement->branches;
    auto *prefix = facet_working_path(paths, paths.prefix),
         *suffix = facet_working_path(paths, paths.suffix);
    require(bool(prefix) == bool(input.placement->prefix_transform) &&
                bool(suffix) == bool(input.placement->suffix_transform),
            "native patch path and placement presence disagree");
    auto &sections = input.sections;
    const auto count = sections.original.size();
    require(count <= b.max_control_points && (!prefix || sections.prefix.size() == count) &&
                (!suffix || sections.suffix.size() == count),
            "native patch section branch count mismatch/budget");
    out.report["shared_path"] = prefix && prefix == suffix;
    for (std::size_t i = 0; i < count; ++i) {
        charge(b, 1);
        TubePatchGroup group{i, {}};
        auto generated = append_tube_facet_surfaces(
            group.surfaces, nullptr, prefix, prefix ? sections.prefix[i].get() : nullptr, suffix,
            suffix ? sections.suffix[i].get() : nullptr, b);
        const auto &working = generated.generation.composition.working_sources;
        update_facet_working_path(prefix, working[0], b);
        if (suffix != prefix)
            update_facet_working_path(suffix, working[2], b);
        out.report["curves"].push_back({{"source_curve", i},
                                        {"generation", std::move(generated.generation.report)},
                                        {"output", std::move(generated.report)},
                                        {"surface_count", group.surfaces.size()}});
        require(generated.generation.status != TubeFacetSeamStatus::pending_general,
                "native whole-section patch generation unsupported");
        // d00f0 can return true with an empty inner chain. Here that stops
        // iteration without clearing previously appended whole-ring groups.
        if (group.surfaces.empty())
            return fail("empty_curve_surfaces");
        out.groups.push_back(std::move(group));
    }
    if (out.groups.empty())
        return fail("empty_output");
    out.success = true;
    out.report["status"] = "patches_prepared";
    out.report["group_count"] = out.groups.size();
    out.report["work_used"] = b.work;
    return out;
}
TubePatchGeneration generate_tube_patch_groups(const Json &profile, const Json &path,
                                               TubeBudget &b) {
    return generate_tube_patch_groups(prepare_tube_patch_inputs(profile, path, b), b);
}
} // namespace p3d::swept_detail
