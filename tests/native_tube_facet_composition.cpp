#include "native_tube_facet_composition.hpp"
#include "native_tube_orientation.hpp"
#include "native_curve_affine.hpp"
#include "bspline_frame.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineCurve curve(Json p, Json w = nullptr) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", 2},
                                    {"closed", false},
                                    {"poles", p},
                                    {"knots", nullptr},
                                    {"weights", w}});
}
Json surface(double offset) {
    return {{"_type", "BsplineSurface"},
            {"numPolesU", 3},
            {"numPolesV", 2},
            {"orderU", 2},
            {"orderV", 2},
            {"closedU", false},
            {"closedV", false},
            {"poles",
             {offset, 0., 0., offset + 1, 1., 0., offset + 2, 0., 0., offset, 0., 3., offset + 1,
              1., 3., offset + 2, 0., 3.}},
            {"knotsU", {2., 2., 3., 5., 5.}},
            {"knotsV", {7., 7., 9., 9.}},
            {"weights", {1., 2., 3., 4., 5., 6.}},
            {"boundaries", nullptr},
            {"numRulesU", 8},
            {"numRulesV", 9},
            {"holeOrigin", 1}};
}
bool same(const TubeFacetSeam &a, const TubeFacetSeam &b) {
    return a.classifier == b.classifier && a.incoming == b.incoming && a.outgoing == b.outgoing &&
           a.plane == b.plane;
}
Point3 neg(Point3 a) {
    for (auto &v : a)
        v = -v;
    return a;
}
} // namespace
unsigned native_tube_facet_composition_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto f) {
        bool caught = false;
        try {
            f();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "facet composition rejects invalid state or exceeded resources");
    };
    const auto x = curve({0., 0., 0., 2., 0., 0.});
    const auto y = curve({0., 0., 0., 0., 3., 0.});
    const auto section = curve({0., 0., 0., .2, 0., 0.});
    const auto s0 = surface(0), s1 = surface(10), s2 = surface(20);
    const TubeFacetSeam off{2, {}, {}, {}};
    TubeFacetSeam internal{17, Point3{1, 2, 3}, Point3{4, 5, 6},
                           std::array<Point3, 2>{Point3{7, 8, 9}, Point3{10, 11, 12}}};
    std::vector<TubeFacetNode> p{{s0, internal}, {s1, off}}, s{{s2, off}};
    auto compose = [&](const std::vector<TubeFacetNode> &a, const std::vector<TubeFacetNode> &b,
                       const BsplineCurve *pa, const BsplineCurve *pb) {
        TubeBudget budget;
        return compose_tube_facet_nodes(a, b, pa, pb, budget);
    };
    auto joined = compose(p, s, &x, &y);
    check(joined.prepared && joined.nodes.size() == 3 &&
              !joined.report["seams_applied"].get<bool>(),
          "two branches are prepared but never advertised as seam-processed geometry");
    check(joined.report["source_nodes"] == Json::array({{{"branch", "prefix"}, {"node", 1}},
                                                        {{"branch", "prefix"}, {"node", 0}},
                                                        {{"branch", "suffix"}, {"node", 0}}}),
          "prefix order reverses while suffix and exact source indices are retained");
    auto prefix_only = compose(p, {}, &x, nullptr);
    const auto &moved = prefix_only.nodes[0].end_seam;
    check(moved.classifier == 17 && moved.incoming == neg(*internal.outgoing) &&
              moved.outgoing == neg(*internal.incoming) &&
              (*moved.plane)[0] == (*internal.plane)[0] &&
              (*moved.plane)[1] == neg((*internal.plane)[1]),
          "seam migrates to preceding reversed node without recomputing its plane");
    check(joined.nodes[1].end_seam.incoming == Point3{-1, 0, 0} &&
              joined.nodes[1].end_seam.outgoing == Point3{0, 1, 0} &&
              (*joined.nodes[1].end_seam.plane)[1] == Point3{-.5, .5, 0},
          "new branch joint uses negated prefix tangent without an extra final sign reversal");
    check(joined.seams[0].incoming == joined.seams[1].outgoing &&
              joined.seams[0].outgoing == joined.seams[1].incoming &&
              joined.seams[0].plane == joined.seams[1].plane &&
              joined.nodes[0].end_seam.incoming == Point3{0, 1, 0} &&
              joined.nodes[0].end_seam.outgoing == Point3{-1, 0, 0},
          "new joint overwrites native shared allocations instead of detached descriptor copies");
    const auto shallow_path = curve({0., 0., 0., -2., 2e-6, 0.});
    auto invalidated = compose(p, s, &x, &shallow_path);
    check(invalidated.prepared && invalidated.report["evaluable_seam_references"] == false &&
              invalidated.seams[0].incoming && !invalidated.seams[0].incoming->alive &&
              !invalidated.seams[1].incoming && !invalidated.nodes[0].end_seam.incoming,
          "native freeing through the final node preserves a marked invalid preceding reference");
    check(joined.nodes[2].surface == s2 && joined.nodes[2].end_seam.classifier == 2,
          "suffix geometry is untouched and distinct ends do not request final seam");
    const auto shifted_y = curve({8., 9., 10., 8., 12., 10.});
    auto shifted = compose(p, s, &x, &shifted_y);
    check((*shifted.nodes[1].end_seam.plane)[0] == Point3{8, 9, 10},
          "joint origin is the second start query without inventing a start-coincidence check");
    for (std::size_t node = 0; node < 2; ++node) {
        const auto original = BsplineSurface::from_bgfb(p[1 - node].surface);
        const auto reversed = BsplineSurface::from_bgfb(joined.nodes[node].surface);
        for (std::size_t i = 0; i < original.poles().size(); ++i)
            check(reversed.poles()[i] == original.poles()[original.poles().size() - 1 - i] &&
                      reversed.weights()[i] == original.weights()[original.poles().size() - 1 - i],
                  "both directions reverse stored homogeneous controls and weights exactly");
        for (double u : {0., .17, .62, 1.})
            for (double v : {0., .31, 1.}) {
                const auto a = original.point_at(1 - u, 1 - v), b = reversed.point_at(u, v);
                bool near = true;
                for (unsigned k = 0; k < 3; ++k)
                    near &= std::abs(a[k] - b[k]) < 1e-12;
                check(near, "nonunit knot domains retain the two-axis parameter reversal identity");
            }
        check(reversed.num_rules_u() == 8 && reversed.num_rules_v() == 9 &&
                  reversed.hole_origin() == 1,
              "reversal preserves native rule metadata");
    }
    auto closed = p;
    closed.back().end_seam = internal;
    auto conflict = compose(closed, s, &x, &y);
    check(!conflict.prepared && conflict.nodes.empty(),
          "prefix closure plus nonparallel branch joint is native failure with no output chain");
    auto restored = compose(closed, {}, &x, nullptr);
    check(
        restored.prepared && restored.nodes.back().end_seam.incoming == neg(*internal.outgoing) &&
            restored.nodes.back().end_seam.outgoing == neg(*internal.incoming) &&
            restored.nodes.front().end_seam.incoming == neg(*internal.incoming) &&
            restored.nodes.front().end_seam.outgoing == neg(*internal.outgoing) &&
            restored.seams.front().plane == restored.seams.back().plane,
        "restoring closing values updates the shared storage also referenced by the previous node");
    auto parallel = compose(closed, s, &x, &x);
    check(parallel.prepared && parallel.report["joint_requested"] == false,
          "parallel branch starts allow prefix closing seam to survive");
    auto only_plane = closed;
    only_plane.back().end_seam.incoming.reset();
    only_plane.back().end_seam.outgoing.reset();
    auto plane_restored = compose(only_plane, {}, &x, nullptr);
    check(
        plane_restored.nodes.back().end_seam.classifier == 17 &&
            plane_restored.nodes.back().end_seam.incoming == internal.incoming &&
            plane_restored.nodes.back().end_seam.outgoing == internal.outgoing &&
            plane_restored.seams.front().plane == plane_restored.seams.back().plane,
        "plane-only closure retains and negates the last node's existing shared tangent pointers");
    auto partial = p;
    partial.back().end_seam.incoming = Point3{7, 8, 9};
    auto discarded = compose(partial, {}, &x, nullptr);
    check(discarded.nodes.back().end_seam.classifier == 2 &&
              !discarded.nodes.back().end_seam.incoming,
          "incomplete saved final tangent pair is discarded");
    auto stale_suffix = s;
    stale_suffix.back().end_seam = internal;
    auto stale = compose(p, stale_suffix, &x, &y);
    check(stale.nodes.back().end_seam.classifier == 2 &&
              stale.nodes.back().end_seam.plane == internal.plane &&
              stale.nodes.back().end_seam.incoming == internal.incoming,
          "nonclosing outer ends change classifier only and retain existing descriptor fields");
    const auto ending = curve({2., -3., 0., 2., 0., 0.});
    auto closure = compose(p, s, &x, &ending);
    check(closure.report["end_seam_requested"] == true &&
              closure.nodes.back().end_seam.incoming == Point3{0, 1, 0} &&
              closure.nodes.back().end_seam.outgoing == Point3{-1, 0, 0} &&
              (*closure.nodes.back().end_seam.plane)[0] == Point3{2, 0, 0},
          "coincident final endpoints use suffix incoming and reversed prefix outgoing tangent");
    auto suffix_only = compose({}, stale_suffix, nullptr, &y);
    check(suffix_only.prepared && suffix_only.nodes[0].surface == s2 &&
              same(suffix_only.nodes[0].end_seam, internal),
          "suffix-only chain preserves all fields");
    TubeFacetSeam zero = internal;
    zero.incoming = Point3{1, 0, 0};
    zero.outgoing = Point3{1, 0, 0};
    update_native_tube_facet_plane(zero, {0, 0, 0});
    check(zero.classifier == 2 && !zero.incoming && !zero.outgoing && zero.plane == internal.plane,
          "degenerate plane update retains the old native plane while clearing the pair");
    auto absent = internal;
    absent.outgoing.reset();
    const auto snapshot = absent;
    update_native_tube_facet_plane(absent, {99, 99, 99});
    check(same(absent, snapshot), "missing tangent pair makes the plane update a no-op");
    auto tiny = s0;
    tiny["knotsU"] = {0., 0., 2e-12, 1e-11, 1e-11};
    auto tiny_composed = compose({{tiny, off}}, {}, &x, nullptr);
    check(tiny_composed.prepared &&
              tiny_composed.nodes[0].surface["knotsU"] == Json({1e-11, 1e-11, 2e-12, 0., 0.}) &&
              tiny_composed.nodes[0].surface["knotsV"] == Json({0., 0., 1., 1.}) &&
              tiny_composed.report["prefix_reversals"][0]["evaluable_knot_order"] == false,
          "failed U knot normalization does not prevent V reversal or silently repair raw knots");
    auto build = [&](const BsplineCurve *a, const BsplineCurve *b) {
        TubeBudget budget;
        return prepare_tube_facet_composition(a, &section, b, &section, false, budget);
    };
    const auto actual = build(&x, &y);
    check(actual.prepared && actual.nodes.size() == 2 && actual.report["branches"].size() == 2,
          "complete branch generation connects to composition");
    check(build(nullptr, &y).nodes.size() == 1 && build(&x, nullptr).nodes.size() == 1,
          "single-sided builders keep native branch placement");
    const auto weighted = curve({.3, .7, .2, 1.1, 2.3, .4, 3.7, .8, .6}, {1.7, 2.9, 1.3});
    TubeBudget shared_budget;
    const auto shared = prepare_tube_facet_composition(&weighted, &section, &weighted, &section,
                                                       false, shared_budget);
    const auto f1 = native_bspline_frame_working(weighted, 0);
    const auto w1 = curve_detail::with_poles(weighted, f1.working_poles);
    const auto f2 = native_bspline_frame_working(w1, 0);
    check(shared.prepared && shared.report["shared_path"] == true &&
              shared.working_prefix_path->poles() == f2.working_poles &&
              shared.working_suffix_path->poles() == f2.working_poles,
          "shared source path sees sequential frame round trips on both branches");
    const auto equal_copy = weighted;
    TubeBudget copy_budget;
    auto separate = prepare_tube_facet_composition(&weighted, &section, &equal_copy, &section,
                                                   false, copy_budget);
    check(separate.report["shared_path"] == false &&
              separate.working_prefix_path->poles() == f1.working_poles &&
              separate.working_suffix_path->poles() == f1.working_poles,
          "equal curves at distinct source addresses are never deduplicated");
    check(weighted.poles().front() == Point3{.3, .7, .2}, "source objects remain read-only");
    TubeBudget alias_budget;
    auto alias = build_tube_facet_chain(weighted, weighted, false, alias_budget);
    TubeBudget reference_budget;
    auto reference = build_tube_facet_chain(w1, weighted, false, reference_budget);
    check(alias.nodes.size() == reference.nodes.size() &&
              alias.nodes.front().surface == reference.nodes.front().surface,
          "section sharing the trace sees the trace-frame update before patch construction");
    TubeBudget none_budget;
    check(!prepare_tube_facet_composition(nullptr, nullptr, nullptr, nullptr, false, none_budget)
                  .prepared &&
              !prepare_tube_facet_composition(&x, nullptr, nullptr, nullptr, false, none_budget)
                   .prepared,
          "absent paths and missing sections report native failure");
    rejects([&] { compose(p, s, nullptr, &y); });
    TubeBudget budget;
    compose_tube_facet_nodes(p, s, &x, &y, budget);
    TubeBudget exact;
    exact.max_work = budget.work;
    check(compose_tube_facet_nodes(p, s, &x, &y, exact).prepared,
          "exact work budget is sufficient");
    TubeBudget short_work;
    short_work.max_work = budget.work - 1;
    rejects([&] { compose_tube_facet_nodes(p, s, &x, &y, short_work); });
    TubeBudget short_storage;
    short_storage.max_control_points = 17;
    rejects([&] { compose_tube_facet_nodes(p, s, &x, &y, short_storage); });
    auto invalid = p;
    invalid[0].end_seam.incoming = Point3{NAN, 0, 0};
    rejects([&] { compose(invalid, s, &x, &y); });
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return build(&x, &y).nodes[0].surface; }));
    for (auto &job : jobs)
        check(job.get() == actual.nodes[0].surface,
              "composition working aliases are call-local and reentrant");
    return n;
}
