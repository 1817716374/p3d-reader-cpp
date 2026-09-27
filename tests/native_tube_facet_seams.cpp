#include "native_tube_facet_seams.hpp"
#include "native_control_lines.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json surface(bool second = false, Json weights = nullptr, double skew = 0) {
    Json poles = Json::array();
    for (unsigned row = 0; row < 2; ++row)
        for (unsigned i = 0; i < 3; ++i) {
            const double w = weights.is_null() ? 1. : weights[3 * row + i].get<double>();
            const Point3 p = second ? Point3{2., 2. * row - 1., double(i) + skew}
                                    : Point3{double(row), 0., double(i)};
            for (double v : p)
                poles.push_back(v * w);
        }
    return {{"_type", "BsplineSurface"},
            {"orderU", 2},
            {"orderV", 2},
            {"numPolesU", 3},
            {"numPolesV", 2},
            {"closedU", false},
            {"closedV", false},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"poles", poles},
            {"weights", weights},
            {"boundaries", nullptr},
            {"numRulesU", 9},
            {"numRulesV", 11},
            {"holeOrigin", 0}};
}
TubeFacetComposition chain(std::vector<Json> tables, std::vector<int> classifiers) {
    TubeFacetComposition out;
    out.prepared = true;
    out.report = Json::object();
    for (std::size_t i = 0; i < tables.size(); ++i) {
        TubeFacetSeam seam;
        seam.classifier = classifiers[i];
        out.nodes.push_back({tables[i], seam});
        TubeFacetSeamReferences refs;
        refs.classifier = classifiers[i];
        out.seams.push_back(refs);
    }
    return out;
}
} // namespace
unsigned native_tube_facet_seams_tests() {
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
        check(caught, "native facet seams reject invalid input or exceeded resources");
    };
    auto hit = native_control_line_pair({0, 0, 0}, {1, 0, 0}, {2, -1, 0}, {2, 1, 0}, .0001);
    check(hit.success && hit.first == Point3{2, 0, 0} && hit.second == hit.first &&
              hit.first_fraction == 2 && hit.second_fraction == .5 && hit.first_location == 0 &&
              hit.second_location == 2,
          "infinite control-line extension preserves an outside segment intersection");
    const double e = .125;
    for (double f : {-.25, -e, 0., e, .5, 1 - e, 1., 1 + e, 1.25}) {
        auto r = native_control_line_pair({0, 0, 0}, {1, 0, 0}, {f, -1, 0}, {f, 1, 0}, e);
        const int expected = f < -e || f > 1 + e ? 0 : f < e ? 1 : f < 1 - e ? 2 : 4;
        check(r.first_location == expected && r.first_fraction == f,
              "native location classification retains strict and inclusive endpoint tolerance "
              "boundaries");
    }
    auto skew =
        native_control_line_pair({0, 0, 0}, {1, 0, 0}, {.5, -1, .001}, {.5, 1, .001}, .0001);
    check(skew.success && skew.first[2] == 0 && skew.second[2] == .001,
          "line solver returns both skew closest points without inventing a common intersection");
    for (const auto &end : std::vector<Point3>{{0, 0, 0}, {2, 0, 0}})
        check(!native_control_line_pair({0, 0, 0}, {1, 0, 0}, {0, 0, 0}, end, .0001).success,
              "parallel and degenerate lines fail the absolute determinant guard");
    for (double h : {std::nextafter(1e-6, 0.), 1e-6, std::nextafter(1e-6, 1.)}) {
        auto r = native_control_line_pair({0, 0, 0}, {1, 0, 0}, {.5, 0, 0}, {.5, h, 0}, .0001);
        check(r.success == (h * h > 1e-12), "native determinant uses strict absolute threshold");
    }
    rejects([&] { native_control_line_pair({NAN, 0, 0}, {1, 0, 0}, {0, 0, 0}, {0, 1, 0}, .0001); });
    const auto a0 = surface(), b0 = surface(true);
    auto apply = [&](Json &a, Json &b, TubeFacetSeamReferences &s) {
        TubeBudget budget;
        return apply_tube_ruled_facet_seam(a, b, s, budget);
    };
    auto a = a0, b = b0;
    TubeFacetSeamReferences seam;
    seam.incoming = std::make_shared<TubeFacetSeamStorage<Point3>>(
        TubeFacetSeamStorage<Point3>{{1, 0, 0}, false});
    const auto allocation = seam.incoming;
    auto result = apply(a, b, seam);
    check(result.status == TubeFacetSeamStatus::complete && seam.classifier == 0 &&
              seam.incoming == allocation && !seam.incoming->alive,
          "direct control-line branch changes classifier without accessing or repairing unused "
          "tangent pointers");
    for (unsigned i = 0; i < 3; ++i) {
        check(BsplineSurface::from_bgfb(a).poles()[3 + i] == Point3{2, 0, double(i)} &&
                  BsplineSurface::from_bgfb(b).poles()[i] == Point3{2, 0, double(i)},
              "both control rows update to the native separate closest points");
        check(BsplineSurface::from_bgfb(a).poles()[i] == BsplineSurface::from_bgfb(a0).poles()[i] &&
                  BsplineSurface::from_bgfb(b).poles()[3 + i] ==
                      BsplineSurface::from_bgfb(b0).poles()[3 + i],
              "opposite control rows are unchanged");
    }
    check(a["numRulesU"] == 9 && a["numRulesV"] == 11 && a["knotsU"].is_null() &&
              a["weights"].is_null(),
          "direct seam replacement keeps source metadata and representation");
    auto wa = surface(false, {2., 3., 4., 5., 6., 7.});
    auto wb = surface(true, {7., 8., 9., 10., 11., 12.});
    const auto old_weights = wa["weights"];
    TubeFacetSeamReferences weighted_seam;
    auto weighted = apply(wa, wb, weighted_seam);
    check(weighted.status == TubeFacetSeamStatus::complete && wa["weights"] == old_weights,
          "weighted seam keeps the original weight array");
    for (unsigned i = 0; i < 3; ++i) {
        const auto p = BsplineSurface::from_bgfb(wa).poles()[3 + i];
        check(p == Point3{2 * (2. + i), 0, double(i) * (2. + i)},
              "first surface last row is reweighted using native first-row weights rather than its "
              "own weights");
        check(BsplineSurface::from_bgfb(wb).poles()[i] ==
                  Point3{2 * (7. + i), 0, double(i) * (7. + i)},
              "second surface first row uses its corresponding weights");
    }
    auto zero_a = a0;
    zero_a["weights"] = {0., 0., 0., 0., 0., 0.};
    auto zero_b = b0;
    zero_b["weights"] = {0., 0., 0., 0., 0., 0.};
    TubeFacetSeamReferences zero_seam;
    auto zero_result = apply(zero_a, zero_b, zero_seam);
    check(
        zero_result.status == TubeFacetSeamStatus::complete && zero_a["poles"][9] == 0. &&
            zero_b["poles"][0] == 0.,
        "protected zero-weight input retains XYZ for intersection and applies raw output weights");
    auto near_b = surface(true, nullptr, 1e-12), near_a = a0;
    TubeFacetSeamReferences near_seam;
    check(apply(near_a, near_b, near_seam).status == TubeFacetSeamStatus::complete &&
              BsplineSurface::from_bgfb(near_a).poles()[3][2] == 0 &&
              BsplineSurface::from_bgfb(near_b).poles()[0][2] == 1e-12,
          "near-equal skew closest points stay distinct without welding or averaging");
    auto far_b = b0;
    far_b["poles"][2] = .01;
    far_b["poles"][11] = .01;
    const auto far_original = far_b;
    auto far_a = a0;
    TubeFacetSeamReferences far_seam;
    auto pending = apply(far_a, far_b, far_seam);
    check(pending.status == TubeFacetSeamStatus::pending_general && far_a == a0 &&
              far_b == far_original && far_seam.classifier == -2,
          "skew control lines defer without committing partial row changes");
    auto late_fail = b0;
    // The final column is parallel; earlier valid candidates must not be written.
    late_fail["poles"][6] = 0.;
    late_fail["poles"][7] = 0.;
    late_fail["poles"][15] = 1.;
    late_fail["poles"][16] = 0.;
    auto initial_late = late_fail, late_a = a0;
    TubeFacetSeamReferences late_seam;
    check(apply(late_a, late_fail, late_seam).status == TubeFacetSeamStatus::native_failure &&
              late_a == a0 && late_fail == initial_late && late_seam.classifier == -2,
          "failure after valid columns preserves all surfaces until outer chain cleanup");
    auto curved = b0;
    curved["orderV"] = 3;
    curved["numPolesV"] = 3;
    for (unsigned i = 0; i < 9; ++i)
        curved["poles"].push_back(b0["poles"][9 + i]);
    auto curved_input = curved, ca = a0;
    TubeFacetSeamReferences curved_seam;
    check(apply(ca, curved, curved_seam).status == TubeFacetSeamStatus::pending_general &&
              curved == curved_input,
          "higher V order never uses the straight control-line branch as an approximation");
    auto successful_chain = chain({a0, b0}, {-2, 2});
    TubeBudget pass_budget;
    auto pass = process_tube_facet_seams(successful_chain, pass_budget);
    check(pass.status == TubeFacetSeamStatus::complete &&
              successful_chain.report["seams_applied"] == true &&
              pass.report["chain_finalized"] == false &&
              successful_chain.nodes[0].end_seam.classifier == 0,
          "chain seam pass commits supported seam but does not claim subsequent finalization");
    auto wrapped = chain({b0, a0}, {2, -2});
    TubeBudget wrap_budget;
    check(process_tube_facet_seams(wrapped, wrap_budget).status == TubeFacetSeamStatus::complete &&
              wrapped.nodes[1].surface == a && wrapped.nodes[0].surface == b,
          "last chain node processes its seam against the head surface");
    auto failed_chain = chain({a0, initial_late}, {-2, 2});
    TubeBudget fail_budget;
    check(process_tube_facet_seams(failed_chain, fail_budget).status ==
                  TubeFacetSeamStatus::native_failure &&
              failed_chain.nodes.empty() && failed_chain.seams.empty() && !failed_chain.prepared,
          "native seam failure clears the whole branch chain");
    auto partial_chain = chain({a0, b0, curved_input}, {-2, -2, 2});
    partial_chain.seams[1].incoming = std::make_shared<TubeFacetSeamStorage<Point3>>(
        TubeFacetSeamStorage<Point3>{{1, 0, 0}, true});
    partial_chain.seams[1].outgoing = std::make_shared<TubeFacetSeamStorage<Point3>>(
        TubeFacetSeamStorage<Point3>{{0, 1, 0}, true});
    partial_chain.seams[1].plane = std::make_shared<TubeFacetSeamStorage<std::array<Point3, 2>>>(
        TubeFacetSeamStorage<std::array<Point3, 2>>{{Point3{0, 0, 0}, Point3{1, 1, 0}}, true});
    TubeBudget partial_budget;
    auto partial_result = process_tube_facet_seams(partial_chain, partial_budget);
    check(partial_result.status == TubeFacetSeamStatus::pending_general &&
              partial_result.report["next_seam"] == 1 && partial_chain.nodes[0].surface == a &&
              partial_chain.nodes[1].surface == b && partial_chain.nodes[2].surface == curved_input,
          "unimplemented general branch retains preceding real seam edits with an explicit "
          "continuation cursor");
    rejects([&] {
        TubeBudget b;
        process_tube_facet_seams(partial_chain, b);
    });
    rejects([&] {
        auto fresh = chain({a0, b0}, {-2, 2});
        TubeBudget b;
        process_tube_facet_seams(fresh, b, 1);
    });
    const auto completed_first = partial_chain.nodes[0].surface;
    TubeBudget resumed_budget;
    check(process_tube_facet_seams(partial_chain, resumed_budget, 1).status ==
                  TubeFacetSeamStatus::pending_general &&
              partial_chain.nodes[0].surface == completed_first,
          "resuming a pending node does not reapply already completed control-row changes");
    const auto make_curve = [](Json points) {
        return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                        {"order", 2},
                                        {"closed", false},
                                        {"poles", points},
                                        {"knots", nullptr},
                                        {"weights", nullptr}});
    };
    const auto left_path = make_curve({0., 0., 0., -2., 0., 0.});
    const auto right_path = make_curve({0., 0., 0., 0., 3., 0.});
    const auto profile = make_curve({0., 0., 0., .2, 0., 0.});
    TubeBudget integrated_budget;
    auto integrated = prepare_tube_facet_composition(&left_path, &profile, &right_path, &profile,
                                                     false, integrated_budget);
    check(integrated.prepared && integrated.nodes.size() == 2 &&
              process_tube_facet_seams(integrated, integrated_budget).status ==
                  TubeFacetSeamStatus::complete,
          "generated independent branches connect through composition and actual straight seam "
          "application");
    auto self = chain({a0}, {-2});
    TubeBudget self_budget;
    check(process_tube_facet_seams(self, self_budget).status == TubeFacetSeamStatus::native_failure,
          "single-node final seam correctly uses the same surface twice");
    auto self_surface = a0;
    self_surface["numPolesV"] = 3;
    for (unsigned i = 0; i < 3; ++i) {
        self_surface["poles"].push_back(0.);
        self_surface["poles"].push_back(1.);
        self_surface["poles"].push_back(double(i));
    }
    TubeBudget self_hit_budget;
    TubeFacetSeamReferences self_hit_seam;
    check(apply_tube_ruled_facet_seam(self_surface, self_surface, self_hit_seam, self_hit_budget)
                      .status == TubeFacetSeamStatus::complete &&
              BsplineSurface::from_bgfb(self_surface).poles()[0] == Point3{1, 0, 0} &&
              BsplineSurface::from_bgfb(self_surface).poles()[6] == Point3{1, 0, 0},
          "self seam success retains both sequential row writes to the same source table");
    auto skipped = chain({Json(), Json()}, {2, 2});
    TubeBudget skip_budget;
    check(process_tube_facet_seams(skipped, skip_budget).status == TubeFacetSeamStatus::complete,
          "classifier two skips geometry access as the native pass does");
    auto empty = chain({}, {});
    TubeBudget empty_budget;
    check(process_tube_facet_seams(empty, empty_budget).status ==
              TubeFacetSeamStatus::native_failure,
          "empty seam chain has native failure status");
    auto measure_a = a0, measure_b = b0;
    TubeFacetSeamReferences measure_seam;
    TubeBudget measured;
    apply_tube_ruled_facet_seam(measure_a, measure_b, measure_seam, measured);
    TubeBudget short_work;
    short_work.max_work = measured.work - 1;
    rejects([&] {
        auto x = a0, y = b0;
        TubeFacetSeamReferences r;
        apply_tube_ruled_facet_seam(x, y, r, short_work);
    });
    TubeBudget short_storage;
    short_storage.max_control_points = 11;
    rejects([&] {
        auto x = a0, y = b0;
        TubeFacetSeamReferences r;
        apply_tube_ruled_facet_seam(x, y, r, short_storage);
    });
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async, [&] {
            auto x = a0, y = b0;
            TubeFacetSeamReferences r;
            apply(x, y, r);
            return x;
        }));
    for (auto &job : jobs)
        check(job.get() == a, "native facet seam arithmetic is reentrant");
    return n;
}
