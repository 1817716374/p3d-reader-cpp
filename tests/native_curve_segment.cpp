#include "native_curve_segment.hpp"
#include "native_curve_affine.hpp"
#include <future>
using namespace p3d;
using namespace p3d::curve_detail;
namespace {
BsplineCurve curve(unsigned order, const std::vector<Point3> &poles, Json knots = nullptr,
                   Json weights = nullptr, bool closed = false) {
    Json flat = Json::array();
    for (const auto &p : poles)
        for (double x : p)
            flat.push_back(x);
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"poles", flat},
                                    {"knots", knots},
                                    {"weights", weights},
                                    {"closed", closed}});
}
} // namespace
unsigned native_curve_segment_tests() {
    unsigned checks = 0;
    auto check = [&](bool b, const char *why) {
        ++checks;
        require(b, why);
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
    auto near = [](Point3 a, Point3 b, double tolerance = 2e-9) {
        for (unsigned k = 0; k < 3; ++k)
            if (std::abs(a[k] - b[k]) > tolerance * (1 + std::abs(b[k])))
                return false;
        return true;
    };
    auto segment = [&](const BsplineCurve &c, double a, double b) {
        std::size_t used = 0;
        return native_curve_segment(c, a, b, 10000, {used, 10000000});
    };
    const std::vector<Point3> points{{0, 0, 0}, {1, 3, 1}, {3, -1, 2}, {4, 2, 0}};
    const auto cubic = curve(4, points);
    const auto original = cubic.poles();
    for (const auto interval : std::vector<std::array<double, 2>>{
             {0, .5}, {.5, 1}, {.2, .8}, {.8, .2}, {-.1, .7}, {1.1, .3}}) {
        auto r = segment(cubic, interval[0], interval[1]);
        check(r.success && r.curve && !r.curve->closed(), "cubic subcurve result");
        for (unsigned i = 0; i <= 20; ++i) {
            const double t = double(i) / 20;
            const double a = std::clamp(interval[0], 0., 1.);
            const double b = std::clamp(interval[1], 0., 1.);
            const double u = a + (b - a) * t, v = 1 - u;
            Point3 expected{};
            for (unsigned k = 0; k < 3; ++k)
                expected[k] = v * v * v * points[0][k] + 3 * v * v * u * points[1][k] +
                              3 * v * u * u * points[2][k] + u * u * u * points[3][k];
            check(near(r.curve->point_at(t), expected), "independent Bernstein subcurve geometry");
        }
    }
    check(cubic.poles() == original, "subcurve original source immutable");
    for (unsigned order = 2; order <= 26; ++order) {
        std::vector<Point3> poles;
        std::vector<double> ws, knots(order, -2.);
        knots.insert(knots.end(), {-.8, 1., 1., 2.8});
        knots.insert(knots.end(), order, 4.);
        for (unsigned i = 0; i < order + 4; ++i) {
            const double w = .5 + .2 * (i % 5);
            ws.push_back(w);
            poles.push_back({w * i, w * std::sin(double(i)), w * std::cos(double(i))});
        }
        const auto source = curve(order, poles, knots, ws);
        for (auto interval : std::vector<std::array<double, 2>>{{.1, .9}, {.9, .1}}) {
            const auto r = segment(source, interval[0], interval[1]);
            check(r.success && r.curve->order() == order, "native subcurve supported order range");
            for (double t : {.0, .13, .27, .69, .87, 1.})
                check(near(r.curve->point_at(t),
                           source.point_at(interval[0] + (interval[1] - interval[0]) * t)),
                      "nonuniform repeated-knot rational interval geometry across orders");
        }
    }
    auto split = segment(cubic, 0, .5);
    check(split.curve->poles() ==
              std::vector<Point3>{{0, 0, 0}, {.5, 1.5, .5}, {1.25, 1.25, 1}, {2, 1, 1.125}},
          "half cubic exact de Casteljau controls");
    auto nonunit = curve(4, points, {-3., -3., -3., -3., 5., 5., 5., 5.});
    auto whole = segment(nonunit, -.1, 1.1);
    check(whole.success && whole.curve->knots() == nonunit.knots() &&
              whole.curve->poles() == nonunit.poles(),
          "whole-copy branch retains raw domain");
    auto backwards = segment(nonunit, 1, 0);
    check(backwards.success && backwards.curve->knot_domain() == std::array<double, 2>{0, 1},
          "whole reverse normalizes domain");
    for (unsigned i = 0; i <= 10; ++i)
        check(near(backwards.curve->point_at(i / 10.), nonunit.point_at(1 - i / 10.)),
              "whole reverse direction");
    auto portion = segment(nonunit, .125, .875);
    for (unsigned i = 0; i <= 10; ++i)
        check(near(portion.curve->point_at(i / 10.), cubic.point_at(.125 + .75 * i / 10.)),
              "nonunit domain fractions preserved");
    std::vector<Point3> weighted = points;
    const std::vector<double> weights{.7, 1.9, 2.3, .8};
    for (unsigned i = 0; i < weighted.size(); ++i)
        for (auto &x : weighted[i])
            x *= weights[i];
    auto rational = curve(4, weighted, nullptr, weights);
    auto rational_part = segment(rational, .2, .9);
    for (unsigned i = 0; i <= 20; ++i)
        check(near(rational_part.curve->point_at(i / 20.), rational.point_at(.2 + .7 * i / 20.)),
              "rational subcurve preserves homogeneous controls");
    auto working = rational.poles();
    native_bspline_knot_tolerance(rational, working);
    check(rational_part.working_poles == working && rational.poles() == weighted,
          "source tolerance round trip returned separately");
    auto again = segment(with_poles(rational, working), .9, 1);
    native_bspline_knot_tolerance(rational, working);
    check(again.working_poles == working, "sequential source tolerance state");
    const auto tiny = segment(cubic, .5, .5 + 1e-15);
    check(!tiny.success && !tiny.curve &&
              tiny.report.at("reason") == "interval_below_knot_tolerance",
          "too-small request native empty object");
    const auto beyond = segment(cubic, 1.1, 1.2);
    check(!beyond.success && !beyond.curve &&
              beyond.report.at("reason") == "too_few_segment_controls",
          "clamped collapsed interval is native failure");
    const auto short_domain = curve(2, {{0, 0, 0}, {1, 0, 0}}, {0., 0., 1e-9, 1e-9});
    const auto normalize_fail = segment(short_domain, .1, .15);
    check(!normalize_fail.success && !normalize_fail.curve &&
              normalize_fail.report.at("reason") == "segment_domain_too_short_to_normalize",
          "normalization threshold frees segment geometry");
    auto broken = curve(2, {{0, 0, 0}, {1, 0, 0}, {8, 0, 0}, {9, 0, 0}}, {0., 0., .5, .5, 1., 1.});
    auto left = segment(broken, 0, .5), right = segment(broken, .5, 1);
    check(left.curve->point_at(1) == Point3{1, 0, 0} && right.curve->point_at(0) == Point3{8, 0, 0},
          "full-order knot preserves disconnected one-sided endpoints");
    auto unclamped =
        curve(3, {{0, 0, 0}, {1, 3, 0}, {3, -1, 0}, {4, 0, 0}}, {-2., -1., 0., 1., 2., 3., 4.});
    auto edge = segment(unclamped, .2, 1);
    check(edge.success && edge.report.at("last_insertion").at("control_copy") == "right_wrap",
          "native right wrap applied to nonclamped OPEN curve");
    for (unsigned i = 0; i <= 10; ++i)
        check(near(edge.curve->point_at(i / 10.), unclamped.point_at(.2 + .8 * i / 10.)),
              "nonclamped endpoint slice geometry");
    loft_detail::Curve cluster;
    cluster.degree = 2;
    cluster.knots = {0, 0, 0, .4, .4 + 6e-11, .4 + 12e-11, 1, 1, 1};
    cluster.poles.resize(6, {1, 2, 3, 1});
    Json insertion;
    std::size_t cost = 0;
    check(insert_open_native_knot(cluster, .4, 8e-11, 3, 100, {cost, 100000}, insertion) &&
              insertion.at("current_multiplicity") == 3 && insertion.at("added") == 0 &&
              insertion.at("snapped_knot") == .4 + 12e-11,
          "transitive near-knot cluster updates snapped comparison value");
    check(!insert_open_native_knot(cluster, -.1, 8e-11, 3, 100, {cost, 100000}, insertion),
          "raw insertion rejects out of domain before snapping");
    const auto periodic =
        curve(2, {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}}, nullptr, nullptr, true);
    for (auto interval : std::vector<std::array<double, 2>>{{.2, .7}, {.8, 1.2}, {-1.2, -.7}}) {
        auto r = segment(periodic, interval[0], interval[1]);
        check(r.success && r.curve && !r.curve->closed(), "periodic partial opening");
        for (unsigned i = 0; i <= 10; ++i) {
            double f = interval[0] + (interval[1] - interval[0]) * i / 10.;
            while (f < 0)
                f += 1;
            while (f > 1)
                f -= 1;
            check(near(r.curve->point_at(i / 10.), periodic.point_at(f)),
                  "periodic forward interval follows wrapped source");
        }
    }
    const auto full = segment(periodic, 0, 1);
    const auto periodic_reverse = segment(periodic, .7, .2);
    check(periodic_reverse.success && periodic_reverse.report.contains("working_reversal") &&
              periodic_reverse.report.at("result_reversal_success") == true,
          "periodic reverse retains both native reversal operations");
    for (unsigned i = 0; i <= 10; ++i)
        check(near(periodic_reverse.curve->point_at(i / 10.), periodic.point_at(.2 + .5 * i / 10.)),
              "periodic reverse final native orientation");
    check(full.success && full.curve->closed() && full.curve->knots() == periodic.knots(),
          "full periodic copy remains closed");
    const auto shifted = segment(periodic, .2, 1.2);
    check(shifted.success && !shifted.curve->closed() &&
              near(shifted.curve->point_at(0), periodic.point_at(.2)),
          "whole periodic shifted origin");
    auto periodic_raw = periodic.knots();
    for (auto &k : periodic_raw)
        k = 2 + 8 * k;
    auto scaled_periodic = curve(2, periodic.poles(), periodic_raw, nullptr, true);
    auto scaled_partial = segment(scaled_periodic, .2, .7);
    check(!scaled_partial.success && !scaled_partial.curve &&
              scaled_partial.report.at("reason") == "last_insertion_failed",
          "native raw distance after periodic normalization is not silently rescaled");
    auto failed_open = segment(scaled_periodic, -1, 1);
    check(!failed_open.success && failed_open.curve && failed_open.curve->closed(),
          "whole copy can retain geometry while opening returns failure");
    cost = 0;
    auto expected = native_curve_segment(rational, .2, .9, 100, {cost, 1000000});
    const auto exact = cost;
    cost = 0;
    check(native_curve_segment(rational, .2, .9, 100, {cost, exact}).report == expected.report,
          "exact subcurve work budget");
    rejects(
        [&] {
            std::size_t used = 0;
            native_curve_segment(rational, .2, .9, 100, {used, exact - 1});
        },
        "subcurve work budget exhaustion");
    rejects(
        [&] {
            std::size_t used = 0;
            native_curve_segment(cubic, .2, .9, 4, {used, 100000});
        },
        "subcurve intermediate insertion control budget");
    rejects([&] { segment(cubic, NAN, 1); }, "subcurve nonfinite parameter");
    rejects([&] { segment(curve(2, {{1, 0, 0}, {2, 0, 0}}, nullptr, {0, 1}), 0, .5); },
            "zero source weight tolerance query unsupported arithmetic");
    rejects(
        [&] {
            std::size_t used = 0;
            native_curve_segment(periodic, 1e6, 1e6 + .5, 100, {used, 1000});
        },
        "periodic repeated wrapping obeys shared work budget");
    std::vector<std::future<NativeCurveSegment>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(std::async(std::launch::async, [&] { return segment(rational, .2, .9); }));
    for (auto &job : jobs) {
        auto r = job.get();
        check(r.report == rational_part.report && r.curve->poles() == rational_part.curve->poles(),
              "concurrent subcurve reads independent working state");
    }
    return checks;
}
