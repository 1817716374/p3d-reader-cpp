#include "native_tube_curvature.hpp"
#include "native_curve_affine.hpp"
#include "bspline_frame.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineCurve curve(unsigned order, Json xyz, Json weights = nullptr, Json knots = nullptr) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"closed", false},
                                    {"poles", xyz},
                                    {"weights", weights},
                                    {"knots", knots}});
}
bool near(double a, double b) {
    return std::abs(a - b) < 1e-11 * (1 + std::abs(b));
}
} // namespace
unsigned native_tube_curvature_tests() {
    unsigned count = 0;
    auto check = [&](bool ok, const char *why) {
        ++count;
        require(ok, why);
    };
    auto rejects = [&](auto fn, const char *why) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    auto sample = [&](const BsplineCurve &c, double tolerance) {
        TubeBudget b;
        return sample_tube_facet_curvature(c, tolerance, b);
    };
    auto prepare = [&](const BsplineCurve &c) {
        TubeBudget b;
        return prepare_tube_facet_sampling(c, b);
    };
    const auto line = curve(2, {0., 0., 0., 2., 0., 0.});
    auto plain = sample(line, 0);
    check(plain.success && plain.tree.size() == 3 && plain.interior.empty(),
          "linear curvature retains three probes but no insertion samples");
    const auto &q = plain.report["queries"];
    check(q.size() == 5 && q[0]["fraction"] == .5 && q[1]["fraction"] == 0 &&
              q[2]["fraction"] == 1 && q[3]["fraction"] == .25 && q[4]["fraction"] == .75,
          "native query order is midpoint endpoints then left and right probes");
    check(!plain.tree[0].point && plain.tree[1].point == Point3{.5, 0, 0} &&
              plain.tree[2].point == Point3{1.5, 0, 0},
          "only child probes request stored positions");
    auto linear = prepare(line);
    check(linear.success && linear.ruled_fallback && linear.sampling.report["tolerance"] == .025 &&
              linear.report["endpoint_queries"].size() == 2,
          "callback uses reciprocal twenty times range diagonal and repeats endpoint queries");
    auto degenerate = prepare(curve(2, {3., 4., 5., 3., 4., 5.}));
    check(degenerate.success && degenerate.ruled_fallback &&
              degenerate.sampling.report["unbounded_tolerance"] == true &&
              degenerate.sampling.report["tolerance"].is_null(),
          "zero path range retains native infinite tolerance without nonfinite JSON");
    const auto parabola = curve(3, {0., 0., 0., .5, 0., 0., 1., 1., 0.});
    auto refined = sample(parabola, .01);
    check(refined.success && refined.interior.size() > 1 && refined.tree.size() > 3,
          "nonlinear curvature refines native midpoint tree");
    for (const auto &entry : refined.report["queries"]) {
        const double t = entry["fraction"];
        const double exact = 2 / std::pow(1 + 4 * t * t, 1.5);
        check(near(entry["curvature"], exact),
              "curvature agrees with independent analytic parabola");
    }
    double previous = -1;
    for (const auto &s : refined.interior) {
        check(s[0] > previous && s[0] > 0 && s[0] < 1,
              "only strictly ordered interior refinement nodes become insertion candidates");
        previous = s[0];
    }
    auto coarse = sample(parabola, 1);
    check(coarse.success && coarse.interior.empty() && coarse.tree.size() == 3,
          "large tolerance keeps quarter probes out of knot sequence");
    const auto &cq = coarse.report["queries"];
    const double a = cq[1]["curvature"], z = cq[2]["curvature"];
    const double threshold = std::max({std::abs((z * .25 + a * .75) - double(cq[3]["curvature"])),
                                       std::abs((z + a) * .5 - double(cq[0]["curvature"])),
                                       std::abs((z * .75 + a * .25) - double(cq[4]["curvature"]))});
    check(sample(parabola, threshold).interior.empty(),
          "refinement tolerance equality is accepted");
    check(!sample(parabola, std::nextafter(threshold, 0.)).interior.empty(),
          "refinement begins immediately below threshold");
    auto scaled = curve(3, {0., 0., 0., 50., 0., 0., 100., 100., 0.});
    const auto base_sampling = prepare(parabola), scaled_sampling = prepare(scaled);
    check(base_sampling.success && scaled_sampling.success && !base_sampling.ruled_fallback &&
              near(double(base_sampling.sampling.report["tolerance"]), 1 / (20 * std::sqrt(2.))) &&
              near(double(scaled_sampling.sampling.report["tolerance"]) * 100,
                   double(base_sampling.sampling.report["tolerance"])),
          "range and curvature scale consistently under uniform scaling");
    check(base_sampling.sampling.interior.size() == scaled_sampling.sampling.interior.size(),
          "uniform scaling preserves adaptive subdivision for non-boundary fixture");
    const auto rational = curve(3, {.3, .7, .2, 1.1, 2.3, .4, 3.7, .8, .6}, {1.7, 2.9, 1.3});
    auto weighted = sample(rational, .2);
    auto work = rational;
    for (const auto &entry : weighted.report["queries"]) {
        const auto f = native_bspline_frame_working(work, entry["fraction"]);
        work = curve_detail::with_poles(work, f.working_poles);
        check(f.report["curvature"] == entry["curvature"],
              "each native curvature query consumes preceding rational working state");
    }
    check(weighted.working_poles == work.poles() && rational.poles()[0] == Point3{.3, .7, .2},
          "ordered rational round trips are returned while source remains immutable");
    auto endpoint_work = prepare(rational);
    auto replay = rational;
    for (const auto &entry : endpoint_work.sampling.report["queries"])
        replay = curve_detail::with_poles(
            replay, native_bspline_frame_working(replay, entry["fraction"]).working_poles);
    for (double t : {0., 1.})
        replay =
            curve_detail::with_poles(replay, native_bspline_frame_working(replay, t).working_poles);
    check(replay.poles() == endpoint_work.sampling.working_poles,
          "caller endpoint queries follow tree collection on the same working curve");
    const auto corner = curve(3, {0., 0., 0., .5, 0., 0., 1., 0., 0., 1., 1., 0., 2., 1., 0.},
                              nullptr, {0., 0., 0., .5, .5, 1., 1., 1.});
    auto failed = sample(corner, 0);
    check(!failed.success && failed.tree.empty() && failed.interior.empty() &&
              failed.report["failure"] == "native_depth_limit" &&
              failed.report["deepest_probe"] == 102,
          "native depth failure occurs after depth-102 probes and frees whole tree");
    TubeBudget measured;
    sample_tube_facet_curvature(parabola, .1, measured);
    TubeBudget exact;
    exact.max_work = measured.work;
    check(sample_tube_facet_curvature(parabola, .1, exact).success && exact.work == measured.work,
          "adaptive tree uses deterministic cumulative work budget");
    TubeBudget short_budget;
    short_budget.max_work = measured.work - 1;
    rejects([&] { sample_tube_facet_curvature(parabola, .1, short_budget); },
            "adaptive tree rejects exhausted work budget");
    TubeBudget small;
    small.max_control_points = 3;
    rejects([&] { sample_tube_facet_curvature(parabola, .01, small); },
            "tree node allocation is bounded");
    rejects([&] { sample(line, -1); }, "negative tolerance rejected");
    rejects([&] { sample(line, NAN); }, "NaN tolerance rejected");
    rejects([&] { sample(curve(2, {0., 0., 0., 1., 1., 1.}, {0., 0.}), 1); },
            "undefined rational curvature does not masquerade as an accepted flat interval");
    std::vector<std::future<bool>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(std::async(std::launch::async, [&] {
            return sample(parabola, .01).interior == refined.interior;
        }));
    for (auto &job : jobs)
        check(job.get(), "curvature sampling has no shared mutable tree");
    return count;
}
