#include "native_tube_path_branches.hpp"
#include "native_curve_planarity.hpp"
#include "native_curve_segment.hpp"
#include "native_curve_affine.hpp"
namespace p3d::swept_detail {
namespace {
bool near(double x, double y) {
    return std::abs(x - y) <= ((std::abs(x) + 1) + std::abs(y)) * 1e-10;
}
Json pieces(const std::vector<TubePathPiece> &v) {
    Json out = Json::array();
    for (const auto &p : v)
        out.push_back({{"working_index", p.index},
                       {"fraction0", p.first},
                       {"fraction1", p.last},
                       {"whole_member", p.whole}});
    return out;
}
TubePathBranch assemble(const std::vector<TubePathPiece> &pieces,
                        const std::vector<BsplineCurve> &curves, TubeBudget &budget,
                        const curve_detail::NativeCurveSegment *segment = nullptr) {
    TubePathBranch out;
    out.report = {{"status", "complete"}, {"present", !pieces.empty()}, {"joins", Json::array()}};
    if (pieces.empty())
        return out;
    for (const auto &piece : pieces)
        require(piece.index < curves.size() && (piece.whole || segment),
                "native branch member/segment assembly");
    if (pieces.front().whole) {
        out.reused_curve_index = pieces.front().index;
        out.report["initial_reused_working_index"] = *out.reused_curve_index;
    } else {
        out.constructed = segment->curve;
    }
    if (segment)
        out.report["subcurve"] = segment->report;
    const BsplineCurve *current = out.reused_curve_index ? &curves[*out.reused_curve_index]
                                  : out.constructed      ? &*out.constructed
                                                         : nullptr;
    for (std::size_t i = 1; i < pieces.size(); ++i) {
        const auto &piece = pieces[i];
        const BsplineCurve *incoming = piece.whole      ? &curves[piece.index]
                                       : segment->curve ? &*segment->curve
                                                        : nullptr;
        // The native wrapper delivers an allocated object even on segment
        // failure. Its null-poles case participates in explicit copy branches.
        if (!current || !incoming) {
            require(current || incoming, "native combination of two empty curve objects");
            curve_detail::BezierWork{budget.work, budget.max_work}.charge(
                (current ? current : incoming)->poles().size());
            out.report["joins"].push_back(
                {{"incoming_working_index", piece.index},
                 {"method", current ? "empty_right_copy_left" : "empty_left_copy_right"}});
            if (!current) {
                out.constructed = *incoming;
                out.reused_curve_index.reset();
                current = &*out.constructed;
            }
            continue;
        }
        auto joined = combine_tube_curves(*current, *incoming, false, true, budget);
        joined.report["incoming_working_index"] = pieces[i].index;
        out.report["joins"].push_back(std::move(joined.report));
        out.constructed = std::move(joined.curve);
        out.reused_curve_index.reset();
        current = &*out.constructed;
    }
    out.empty_curve_object = !current;
    out.report["empty_curve_object"] = out.empty_curve_object;
    out.report["unchanged_member_reused"] = out.reused_curve_index.has_value();
    return out;
}
} // namespace
TubePathBranchPlan plan_tube_path_branches(std::size_t count, std::size_t index, double fraction,
                                           bool whole_planar, bool member_planar,
                                           TubeBudget &budget) {
    require(count && index < count && std::isfinite(fraction), "native branch selection bounds");
    curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(count);
    TubePathBranchPlan out;
    std::string branch;
    auto append = [&](std::vector<TubePathPiece> &target, std::size_t first, std::size_t end) {
        for (std::size_t i = first; i < end; ++i)
            target.push_back({i});
    };
    if (whole_planar) {
        branch = "whole_path_planar";
        append(out.suffix, 0, count);
    } else if (member_planar) {
        branch = "selected_member_planar";
        // Native treats index zero first; a single member goes wholly to suffix.
        const auto boundary = index != 0 && index == count - 1 ? count : index;
        append(out.prefix, 0, boundary);
        append(out.suffix, boundary, count);
    } else {
        append(out.prefix, 0, index);
        if (near(fraction, 0)) {
            branch = "nonplanar_near_start";
            out.suffix.push_back({index});
        } else if (near(fraction, 1)) {
            branch = "nonplanar_near_end";
            out.prefix.push_back({index});
        } else {
            branch = "nonplanar_interior";
            out.prefix.push_back({index, 0, fraction, false});
            out.suffix.push_back({index, fraction, 1, false});
        }
        append(out.suffix, index + 1, count);
    }
    out.report = {{"scope", "native_facet_path_branch_order"},
                  {"branch", branch},
                  {"prefix", pieces(out.prefix)},
                  {"suffix", pieces(out.suffix)},
                  {"work_used", budget.work}};
    return out;
}
TubeFacetPathBranches prepare_tube_facet_path_branches(const Json &profile, const Json &path,
                                                       TubeBudget &budget) {
    TubeFacetPathBranches out;
    out.whole_path_planarity = curve_detail::native_curve_vector_planarity(
        path, budget.max_control_points, {budget.work, budget.max_work});
    out.path = prepare_tube_facet_path(profile, path, budget);
    auto &s = out.path.selection;
    out.plan = plan_tube_path_branches(
        s.curves.size(), s.index, s.fraction, out.whole_path_planarity.at("planar").get<bool>(),
        out.path.selected_member_planarity.at("planar").get<bool>(), budget);
    const auto partial = [](const auto &v) {
        return std::any_of(v.begin(), v.end(), [](const auto &p) { return !p.whole; });
    };
    std::optional<curve_detail::NativeCurveSegment> prefix, suffix;
    if (partial(out.plan.prefix) || partial(out.plan.suffix)) {
        require(budget.max_control_points <= UINT32_MAX, "native subcurve control limit");
        auto &selected = s.curves[s.index];
        // Both native segment calls precede ALL joins. The second tolerance
        // query observes the first call's source unweight/reweight round trip.
        prefix = curve_detail::native_curve_segment(selected, 0, s.fraction,
                                                    unsigned(budget.max_control_points),
                                                    {budget.work, budget.max_work});
        selected = curve_detail::with_poles(selected, prefix->working_poles);
        suffix = curve_detail::native_curve_segment(selected, s.fraction, 1,
                                                    unsigned(budget.max_control_points),
                                                    {budget.work, budget.max_work});
        selected = curve_detail::with_poles(selected, suffix->working_poles);
    }
    out.prefix = assemble(out.plan.prefix, s.curves, budget, prefix ? &*prefix : nullptr);
    out.suffix = assemble(out.plan.suffix, s.curves, budget, suffix ? &*suffix : nullptr);
    out.report = {{"status", "complete"},
                  {"scope", "branch_curve_assembly"},
                  {"subsequent_frames_evaluated", false},
                  {"work_used", budget.work}};
    return out;
}
} // namespace p3d::swept_detail
