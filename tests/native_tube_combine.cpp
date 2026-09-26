#include "native_tube.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineCurve make(unsigned order, Json poles, Json knots, Json weights = nullptr,
                  bool closed = false) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"closed", closed},
                                    {"poles", poles},
                                    {"weights", weights},
                                    {"knots", knots}});
}
BsplineCurve line(double x, double y, double lo = 0, double hi = 1) {
    return make(2, {x, 0, 0, y, 0, 0}, {lo, lo, hi, hi});
}
bool near(Point3 p, Point3 q) {
    return std::hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]) < 1e-11;
}
} // namespace
unsigned native_tube_combine_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn, const char *why) {
        bool failed = false;
        try {
            fn();
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, why);
    };
    auto combine = [](const BsplineCurve &a, const BsplineCurve &b, bool reparam = true,
                      bool force = false) {
        TubeBudget budget;
        return combine_open_tube_curves(a, b, force, reparam, budget);
    };
    const auto a = line(0, 2), b = line(2, 8);
    const auto joined = combine(a, b);
    check(joined.curve.knots() == std::vector<double>({0, 0, .25, 1, 1}) &&
              joined.curve.poles() == std::vector<Point3>({{0, 0, 0}, {2, 0, 0}, {8, 0, 0}}),
          "native join allocates parameter intervals by control-polygon lengths");
    for (double t : {0., .1, .25, .4, .8, 1.})
        check(near(joined.curve.point_at(t), {8 * t, 0, 0}), "line join evaluation");
    const auto unscaled = combine(a, b, false);
    check(unscaled.curve.knots() == std::vector<double>({0, 0, .5, 1, 1}),
          "without reparameterization source knot intervals determine the join");
    const auto raw = combine(line(0, 2, 2, 4), line(2, 8, 10, 11));
    check(raw.curve.knots() == std::vector<double>({0, 0, .4, 1, 1}),
          "raw source spans participate before normalization, not normalized input spans");
    auto gap = combine(a, line(5, 11));
    check(!gap.report["contiguous"].get<bool>() &&
              gap.curve.knots() == std::vector<double>({0, 0, .25, .25, 1, 1}) &&
              gap.curve.poles().size() == 4,
          "disconnected sources retain both endpoint controls and a full-multiplicity break");
    check(near(gap.curve.point_at(.25), {5, 0, 0}) &&
              near(gap.curve.point_at(std::nextafter(.25, 0.)), {2, 0, 0}),
          "gap has distinct one-sided limits and no generated bridge");
    auto forced = combine(a, line(5, 11), true, true);
    check(forced.curve.poles().size() == 3 && forced.report["contiguous"] == true &&
              near(forced.curve.point_at(.25), {2, 0, 0}),
          "explicit native force drops the incoming control even across a gap");
    const auto p = make(2, {-1, 0, 1, 0, 0, 1}, {0, 0, 1, 1});
    for (double d : {1e-8, std::nextafter(2e-8, 0.), 2e-8, 3e-8}) {
        const auto q = make(2, {d, 0, 1, 1, 0, 1}, {0, 0, 1, 1});
        check(combine(p, q, false).report["contiguous"] == (d < 2e-8),
              "Cartesian endpoint relative-plus-absolute tolerance is strict");
    }
    const auto hleft = make(2, {0, 0, 0, 4, 0, 0}, {0, 0, 1, 1}, {2, 2});
    const auto mixed = combine(hleft, b);
    check(mixed.curve.weights() == std::vector<double>({2, 2, 1}) &&
              mixed.curve.poles()[1][0] == 4 && mixed.report["contiguous"] == true,
          "endpoint comparison unweights, output does not rescale source homogeneous weights");
    const auto hright = make(2, {-6, 0, 0, -24, 0, 0}, {0, 0, 1, 1}, {-3, -3});
    const auto signed_mix = combine(a, hright);
    check(signed_mix.curve.weights() == std::vector<double>({1, 1, -3}) &&
              signed_mix.curve.poles().back()[0] == -24,
          "negative right weights retained, unit weights filled for nonrational left");
    const auto zero = line(2, 2, 7, 9);
    const auto right_copy = combine(zero, b);
    check(right_copy.report["retained_source"] == "right" && right_copy.curve.knots() == b.knots(),
          "zero left polygon keeps the right curve as a copy");
    const auto left_copy = combine(line(0, 2, 7, 9), zero);
    check(left_copy.report["retained_source"] == "left" &&
              left_copy.curve.knots() == std::vector<double>({7, 7, 9, 9}) &&
              left_copy.report["normalized_knots"] == false,
          "degenerate copy does not normalize the retained knot domain");
    rejects([&] { combine(zero, zero); },
            "both negligible polygons fail rather than synthesize a line");
    check(combine(zero, zero, false).curve.poles().size() == 3,
          "length degeneracy checks do not run when reparameterization is disabled");
    check(combine(line(0, 1), line(0, 1e10)).report["retained_source"] == "both" &&
              combine(line(0, 1), line(0, 2e10)).report["retained_source"] == "right" &&
              combine(line(0, 2e10), line(0, 1)).report["retained_source"] == "left",
          "relative length cutoff is strict and handles both sides");
    check(combine(line(0, 1e-12), line(0, 1e-12)).report["retained_source"] == "both",
          "absolute length cutoff retains exactly one trillionth");
    const auto tiny = combine(line(0, 2, 0, 2e-11), line(2, 8, 0, 2e-11), false);
    check(tiny.curve.knots() == std::vector<double>({0, 0, 2e-11, 4e-11, 4e-11}) &&
              tiny.report["normalized_knots"] == false,
          "ignored native normalization failure preserves tiny nonzero domain");
    const auto threshold = combine(line(0, 2, 0, 5e-11), line(2, 8, 0, 5e-11), false);
    check(threshold.report["normalized_knots"] == true,
          "normalization accepts a domain exactly at threshold");
    const auto unclamped = make(2, {2, 0, 0, 8, 0, 0}, {-1, 0, 1, 3});
    const auto clamped_tail = combine(a, unclamped, false);
    check(clamped_tail.curve.knots() == std::vector<double>({0, 0, .5, 1, 1}),
          "native open normalization overwrites trailing outer knots with one");
    const auto leading = combine(make(2, {0, 0, 0, 2, 0, 0}, {-1, 0, 1, 1}), b, false);
    check(leading.curve.knots() == std::vector<double>({-.5, 0, .5, 1, 1}),
          "native normalization does not clamp leading outer knots");
    const auto rounding = make(2, {1, 0, 0, 2, 0, 0}, {7, 7, 9, 9}, {49, 49});
    const auto rounded = combine(zero, rounding);
    const double rounded_one = (1.0 / 49.0) * 49.0;
    check(rounded_one != 1.0 && rounded.curve.poles()[0][0] == rounded_one &&
              rounding.poles()[0][0] == 1.0 && rounded.curve.knots() == rounding.knots(),
          "degenerate copy keeps reciprocal-multiply round trip while source stays immutable");
    const auto no_roundtrip = combine(rounding, b, false, true);
    check(no_roundtrip.curve.poles()[0][0] == 1,
          "no reparameterization means no length-driven coordinate round trip");
    const auto interior_zero = make(3, {0, 0, 0, 1, 1, 0, 2, 0, 0}, {0, 0, 0, 1, 1, 1}, {1, 0, 1});
    const auto quadratic = make(3, {2, 0, 0, 3, 1, 0, 4, 0, 0}, {0, 0, 0, 1, 1, 1});
    check(combine(interior_zero, quadratic, false).curve.weights()[1] == 0,
          "zero interior weight retained when no polygon unweighting is requested");
    rejects([&] { combine(interior_zero, quadratic); },
            "zero weight cannot silently bypass length computation");
    const auto endpoint_zero = make(2, {0, 0, 0, 2, 0, 0}, {0, 0, 1, 1}, {1, 0});
    rejects([&] { combine(endpoint_zero, b, false); },
            "automatic join requires finite unweighted endpoints");
    check(combine(endpoint_zero, b, false, true).curve.weights()[1] == 0,
          "forced join bypasses endpoint unweighting as native wrapper does");
    rejects([&] { combine(a, quadratic); }, "core does not implicitly elevate degrees");
    rejects([&] { combine(make(2, {0, 0, 0, 1, 0, 0}, {-1, 0, 1, 2, 3}, nullptr, true), a); },
            "core does not implicitly open periodic sources");
    rejects([&] { combine(line(-1e308, 1e308), b); }, "length overflow is explicit failure");
    rejects(
        [&] {
            TubeBudget budget;
            budget.max_work = 1;
            combine_open_tube_curves(a, b, false, true, budget);
        },
        "work budget guards operations");
    rejects(
        [&] {
            TubeBudget budget;
            budget.max_control_points = 2;
            combine_open_tube_curves(zero, b, false, true, budget);
        },
        "allocation limit checked before degenerate copy branches");
    // Higher degree: compare interior evaluations on each source span. Identical
    // seam weights avoid changing the incoming rational representation at join.
    const auto q1 = make(3, {0, 0, 0, 1, 2, 0, 2, 0, 0}, {0, 0, 0, 1, 1, 1});
    const auto q2 = make(3, {2, 0, 0, 3, -2, 0, 4, 0, 0}, {0, 0, 0, 1, 1, 1});
    const auto qq = combine(q1, q2);
    for (unsigned i = 0; i <= 40; ++i) {
        const double t = i / 40.;
        check(near(qq.curve.point_at(t * .5), q1.point_at(t)), "quadratic left span preserved");
        check(near(qq.curve.point_at(.5 + t * .5), q2.point_at(t)),
              "quadratic right span preserved");
    }
    std::vector<std::future<TubeCurve>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(std::async(std::launch::async, [&] { return combine(q1, q2); }));
    for (auto &job : jobs) {
        auto r = job.get();
        check(r.curve.poles() == qq.curve.poles() && r.report == qq.report,
              "parallel callers share immutable sources without state races");
    }
    return n;
}
