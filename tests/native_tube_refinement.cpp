#include "native_tube_refinement.hpp"
#include "native_curve_affine.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineCurve curve(unsigned order, Json poles, Json weights = nullptr, Json knots = nullptr) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"closed", false},
                                    {"poles", poles},
                                    {"weights", weights},
                                    {"knots", knots}});
}
std::array<long double, 4> de_boor(const BsplineCurve &c, double fraction) {
    const auto p = c.order() - 1;
    const auto domain = c.knot_domain();
    const long double u = (1 - static_cast<long double>(fraction)) * domain[0] +
                          static_cast<long double>(fraction) * domain[1];
    const auto &k = c.knots();
    std::size_t span = p;
    while (span + 1 < c.poles().size() && u >= k[span + 1])
        ++span;
    std::vector<std::array<long double, 4>> d;
    for (unsigned j = 0; j <= p; ++j) {
        const auto i = span - p + j;
        const auto &v = c.poles()[i];
        d.push_back({v[0], v[1], v[2], c.rational() ? c.weights()[i] : 1});
    }
    for (unsigned r = 1; r <= p; ++r)
        for (unsigned j = p + 1; j-- > r;) {
            const auto i = span - p + j;
            const long double a = (u - k[i]) / (k[i + p - r + 1] - k[i]);
            for (unsigned xyz = 0; xyz < 4; ++xyz)
                d[j][xyz] = (1 - a) * d[j - 1][xyz] + a * d[j][xyz];
        }
    return d[p];
}
bool same_geometry(const BsplineCurve &a, const BsplineCurve &b) {
    for (unsigned i = 0; i <= 32; ++i) {
        const auto x = de_boor(a, double(i) / 32), y = de_boor(b, double(i) / 32);
        for (unsigned j = 0; j < 4; ++j)
            if (std::abs(x[j] - y[j]) > 2e-10L * (1 + std::abs(x[j])))
                return false;
    }
    return true;
}
} // namespace
unsigned native_tube_refinement_tests() {
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
    auto refine = [&](const BsplineCurve &c, const std::vector<double> &knots) {
        TubeBudget b;
        return refine_tube_facet_curve(c, knots, b);
    };
    for (unsigned order = 2; order <= 26; ++order) {
        Json xyz = Json::array(), w = Json::array();
        for (unsigned i = 0; i < order; ++i) {
            xyz.push_back(double(i) * .7);
            xyz.push_back(std::sin(double(i)));
            xyz.push_back(double(i % 3));
            w.push_back(1 + double(i % 5) * .2);
        }
        for (bool rational : {false, true}) {
            const auto source = curve(order, xyz, rational ? w : Json());
            const auto result = refine(source, {.125, .25, .5, .625, .75, .875});
            check(result.curve.poles().size() == order + 6 && result.curve.order() == order &&
                      result.curve.rational() == rational,
                  "bulk refinement retains order and weight representation");
            check(same_geometry(source, result.curve),
                  "orders 2 through 26 preserve independent homogeneous de Boor values");
            check(source.poles().size() == order && source.knots().size() == 2 * order,
                  "refinement leaves source controls and knots immutable");
        }
    }
    const auto cubic = curve(4, {0., 0., 0., .25, 1., 0., .75, -1., 2., 1., 0., 3.});
    auto repeated = refine(cubic, {.2, .2, .2, .2, .8, .8});
    check(std::count(repeated.curve.knots().begin(), repeated.curve.knots().end(), .2) == 4 &&
              same_geometry(cubic, repeated.curve),
          "duplicate insertions retain requested full multiplicity");
    const auto domain = curve(3, {0., 0., 0., 1., 2., 0., 2., 1., 1., 4., 0., 0.}, nullptr,
                              {2., 2., 2., 3., 5., 5., 5.});
    const auto raw = refine(domain, {2.25, 3., 3.5, 4.75});
    check(raw.curve.knot_domain() == domain.knot_domain() && same_geometry(domain, raw.curve),
          "raw nonunit domain and existing knots are not normalized or removed");
    const auto unclamped = curve(3, {0., 0., 0., 1., 2., 0., 3., 1., 1., 4., 0., 0.}, nullptr,
                                 {-2., -1., 0., 1., 2., 3., 4.});
    auto uc = refine(unclamped, {.25, .75, 1.25, 1.75});
    check(same_geometry(unclamped, uc.curve),
          "unclamped open storage is refined without endpoint replacement");
    const auto signed_curve = curve(3, {1., 2., 3., 4., 5., 6., 7., 8., 9.}, {-2., 0., 3.});
    auto signed_result = refine(signed_curve, {.2, .4, .7});
    check(signed_result.curve.rational() && same_geometry(signed_curve, signed_result.curve),
          "homogeneous refinement keeps negative and zero weights without deweighting");
    const auto simple = curve(2, {0., 0., 0., 1., 1., 0.});
    auto near_end = refine(simple, {1 - 5e-11});
    check(near_end.report["near_knot_copies"] == 1 &&
              near_end.curve.poles()[1] == simple.poles()[1],
          "native absolute near-knot branch copies control instead of interpolating");
    auto outside_near = refine(simple, {1 - 2e-10});
    check(outside_near.report["near_knot_copies"] == 0 && outside_near.curve.poles()[1][0] < 1,
          "outside absolute near-knot tolerance computes interpolation");
    // Cancellation-sensitive fixture distinguishes the native .5 anchor switch.
    const auto rounding = curve(2, {1e16, 1., 0., 1., 3., 0.});
    const auto anchored = refine(rounding, {.25});
    const double expected = 1e16 + (1 - 1e16) * .25;
    check(anchored.curve.poles()[1][0] == expected,
          "native four-dimensional interpolation anchors at nearer endpoint");
    TubeBudget measured;
    refine_tube_facet_curve(cubic, {.25, .5, .75}, measured);
    TubeBudget exact;
    exact.max_work = measured.work;
    check(refine_tube_facet_curve(cubic, {.25, .5, .75}, exact).curve.poles().size() == 7 &&
              exact.work == measured.work,
          "bulk refinement cumulative budget is deterministic");
    TubeBudget short_budget;
    short_budget.max_work = measured.work - 1;
    rejects([&] { refine_tube_facet_curve(cubic, {.25, .5, .75}, short_budget); },
            "bulk refinement enforces shared work budget");
    TubeBudget controls;
    controls.max_control_points = 6;
    rejects([&] { refine_tube_facet_curve(cubic, {.25, .5, .75}, controls); },
            "refined control allocation is bounded");
    rejects([&] { refine(cubic, {}); },
            "empty native insertion input has no initialized endpoints");
    rejects([&] { refine(cubic, {.5, .25}); }, "unordered insertion input rejected");
    rejects([&] { refine(cubic, {-1.}); }, "out-of-domain span search cannot hang");
    rejects([&] { refine(cubic, {NAN}); }, "nonfinite insertion rejected");
    auto closed = BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                           {"order", 2},
                                           {"closed", true},
                                           {"poles", {0., 0., 0., 1., 0., 0., 1., 1., 0.}},
                                           {"knots", nullptr},
                                           {"weights", nullptr}});
    rejects([&] { refine(closed, {.5}); },
            "native open-storage helper does not fabricate closed tail knots");
    auto prepare = [&](const BsplineCurve &c) {
        TubeBudget b;
        return prepare_tube_facet_trace(c, b);
    };
    const auto parabola = curve(3, {0., 0., 0., .5, 0., 0., 1., 1., 0.});
    auto trace = prepare(parabola);
    check(trace.success && !trace.sampling.ruled_fallback && trace.trace &&
              trace.trace->poles().size() > 3 &&
              trace.greville.size() == trace.trace->poles().size(),
          "native curvature samples drive bulk refinement and one mean per refined control");
    check(same_geometry(parabola, *trace.trace),
          "connected adaptive refinement retains actual path geometry");
    check(trace.curvature_knots.size() == trace.trace->knots().size(),
          "curvature samples align with actual refined knot storage");
    for (std::size_t i = 0; i < trace.curvature_knots.size(); ++i)
        check(trace.curvature_knots[i][0] == trace.trace->knots()[i],
              "curvature knot parameter retains insertion correspondence");
    for (std::size_t i = 0; i < trace.greville.size(); ++i) {
        check(trace.greville[i][0] ==
                      (trace.curvature_knots[i + 1][0] + trace.curvature_knots[i + 2][0]) / 2 &&
                  trace.greville[i][1] ==
                      (trace.curvature_knots[i + 1][1] + trace.curvature_knots[i + 2][1]) / 2,
              "parameter and curvature use the same native Greville window");
    }
    auto line_trace = prepare(simple);
    check(line_trace.success && line_trace.sampling.ruled_fallback && line_trace.greville.empty() &&
              line_trace.curvature_knots.empty() && line_trace.trace->poles() == simple.poles(),
          "ruled fallback bypasses tensor-row refinement");
    const auto rational = curve(3, {.3, .7, .2, 1.1, 2.3, .4, 3.7, .8, .6}, {1.7, 2.9, 1.3});
    auto weighted = prepare(rational);
    const auto working =
        curve_detail::with_poles(rational, weighted.sampling.sampling.working_poles);
    check(weighted.success && same_geometry(working, *weighted.trace),
          "adaptive bulk refinement starts from sampling's rational working poles");
    rejects([&] { prepare(domain); },
            "facet callback preparation rejects a multi-span source instead of guessing a split");
    auto nonunit =
        curve(3, {0., 0., 0., .5, 0., 0., 1., 1., 0.}, nullptr, {2., 2., 2., 4., 4., 4.});
    rejects([&] { prepare(nonunit); },
            "callback-specific prepared Bezier must have normalized knots");
    std::vector<std::future<std::vector<Point3>>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return prepare(parabola).trace->poles(); }));
    for (auto &job : jobs)
        check(job.get() == trace.trace->poles(), "adaptive trace preparation is reentrant");
    return count;
}
