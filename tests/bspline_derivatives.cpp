#include "internal.hpp"
#include <future>

using namespace p3d;
namespace {
Json table(unsigned order, bool closed, Json poles, Json weights = nullptr, Json knots = nullptr) {
    return {{"_type", "BsplineCurve"}, {"order", order},     {"closed", closed},
            {"poles", poles},          {"weights", weights}, {"knots", knots}};
}
bool near(const Point3 &a, const Point3 &b, double tolerance = 2e-11) {
    for (unsigned i = 0; i < 3; ++i)
        if (std::abs(a[i] - b[i]) > tolerance * std::max({1., std::abs(a[i]), std::abs(b[i])}))
            return false;
    return true;
}
} // namespace

unsigned bspline_derivative_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto rejects = [&](auto fn, const char *message) {
        bool threw = false;
        try {
            fn();
        } catch (const std::exception &) {
            threw = true;
        }
        check(threw, message);
    };
    const auto cubic =
        BsplineCurve::from_bgfb(table(4, false, {0, 0, 0, 1. / 3, 0, 0, 2. / 3, 1. / 3, 0, 1, 1, 1},
                                      nullptr, {2, 2, 2, 2, 6, 6, 6, 6}));
    for (double f : {0., .1, .49, .5, .75, 1.}) {
        const auto d = cubic.native_derivatives_at(f, 5);
        check(d.size() == 6 && near(d[0], {f, f * f, f * f * f}), "native cubic derivative point");
        check(near(d[1], {.25, .5 * f, .75 * f * f}) && near(d[2], {0, .125, .375 * f}) &&
                  near(d[3], {0, 0, 6. / 64}),
              "derivatives use source knots, not fraction scale");
        check(d[4] == Point3{} && d[5] == Point3{}, "polynomial derivatives above degree are zero");
    }
    check(cubic.native_derivatives_at(-3) == cubic.native_derivatives_at(0) &&
              cubic.native_derivatives_at(8) == cubic.native_derivatives_at(1),
          "open native derivative fractions clamp to endpoints");
    check(cubic.native_derivatives_at(.5, 0).size() == 1,
          "zero derivative order returns one point");

    // x(f) = 2f/(1+f), whose derivatives remain nonzero above degree one.
    const auto rational =
        BsplineCurve::from_bgfb(table(2, false, {0, 0, 0, 2, 0, 0}, {1, 2}, {2, 2, 12, 12}));
    for (double f : {0., .25, .5, .8, 1.}) {
        const auto d = rational.native_derivatives_at(f, 24);
        check(near(d[0], {2 * f / (1 + f), 0, 0}), "rational derivative position");
        double factorial = 1;
        for (unsigned n = 1; n <= 24; ++n) {
            factorial *= n;
            const double value =
                (n % 2 ? 2 : -2) * factorial / (std::pow(1 + f, n + 1) * std::pow(10., n));
            check(near(d[n], {value, 0, 0}),
                  "rational quotient derivatives above polynomial degree");
        }
    }
    const auto jump = BsplineCurve::from_bgfb(
        table(2, false, {0, 0, 0, 1, 0, 0, 10, 0, 0, 13, 0, 0}, nullptr, {0, 0, .5, .5, 1, 1}));
    const auto jd = jump.native_derivatives_at(.5);
    check(near(jd[0], {10, 0, 0}) && near(jd[1], {6, 0, 0}),
          "full multiplicity knot selects the right-hand polynomial and derivative");

    const auto periodic =
        BsplineCurve::from_bgfb(table(3, true, {0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0}));
    const auto pd = periodic.native_derivatives_at(0);
    check(near(pd[0], {1, 0, 0}) && near(pd[1], {8, 0, 0}) && near(pd[2], {-32, 32, 0}),
          "closed quadratic uses cyclic poles and source knot derivatives");
    for (double f : {.125, .25, .625})
        check(periodic.native_derivatives_at(f) == periodic.native_derivatives_at(f + 4) &&
                  periodic.native_derivatives_at(f) == periodic.native_derivatives_at(f - 4),
              "closed fractions wrap in both directions");
    check(periodic.native_derivatives_at(2) == periodic.native_derivatives_at(1) &&
              periodic.native_derivatives_at(-1) == periodic.native_derivatives_at(0),
          "closed exact positive and negative periods preserve endpoint side");
    check(periodic.native_derivatives_at(1)[2] != pd[2],
          "periodic end and start retain their one-sided second derivatives");

    const double s = std::sqrt(3.) / 2;
    auto circle_table =
        table(3, true, {1, 0, 0, .5, s, 0, -.5, s, 0, -1, 0, 0, -.5, -s, 0, .5, -s, 0, 1, 0, 0},
              {1, .5, 1, .5, 1, .5, 1},
              {-1. / 3, 0, 0, 0, 1. / 3, 1. / 3, 2. / 3, 2. / 3, 1, 1, 1, 4. / 3});
    auto circle = BsplineCurve::from_bgfb(circle_table);
    for (double f : {0., .1, .25, .5, .8, 1.}) {
        const auto d = circle.native_derivatives_at(f);
        const double dot = d[0][0] * d[1][0] + d[0][1] * d[1][1];
        check(near(d[0], circle.point_at(f)) && std::abs(dot) < 1e-12,
              "accepted special seam retains rational circle position and tangent");
    }
    circle_table["poles"][18] = 1.1;
    auto failed_seam = BsplineCurve::from_bgfb(circle_table);
    check(near(failed_seam.native_derivatives_at(0)[0], {1, 2 * s, 0}),
          "failed native endpoint check disables knot-only special pole shift");
    circle_table["poles"][18] = 1.;
    circle_table["weights"][6] = 1. + 2e-10;
    failed_seam = BsplineCurve::from_bgfb(circle_table);
    check(near(failed_seam.native_derivatives_at(0)[0], {1, 2 * s, 0}),
          "failed endpoint weight check independently disables special pole shift");
    circle_table["weights"][6] = 1.;
    for (auto &x : circle_table["weights"])
        x = -x.get<double>();
    for (auto &x : circle_table["poles"])
        x = -x.get<double>();
    const auto negative_circle = BsplineCurve::from_bgfb(circle_table);
    for (double f : {0., .2, .7, 1.}) {
        const auto a = circle.native_derivatives_at(f),
                   b = negative_circle.native_derivatives_at(f);
        check(near(a[0], b[0]) && near(a[1], b[1]) && near(a[2], b[2]) && near(a[3], b[3]),
              "negative homogeneous representation participates in native weighted range");
    }

    auto narrow =
        BsplineCurve::from_bgfb(table(2, false, {0, 0, 0, 1, 0, 0}, nullptr, {0, 0, 1e-14, 1e-14}));
    check(near(narrow.native_derivatives_at(0)[1], {1e14, 0, 0}),
          "native knot tolerance equality is accepted");
    rejects([&] { narrow.native_derivatives_at(.5); },
            "derivative denominator can fail native tolerance after successful interpolation");
    check(near(narrow.native_derivatives_at(.5, 0)[0], {.5, 0, 0}),
          "point-only request does not evaluate derivative denominator");
    const auto adjusted_tolerance = BsplineCurve::from_bgfb(
        table(2, false, {0, 0, 0, .1, 0, 0, .2, 0, 0}, nullptr, {0, 0, 1e-4, 1e9, 1e9}));
    check(near(adjusted_tolerance.native_derivatives_at(5e-14)[1], {1000, 0, 0}),
          "source knot gap greater than 1e-5 reduces domain-scaled native tolerance");
    const auto unadjusted_tolerance = BsplineCurve::from_bgfb(
        table(2, false, {0, 0, 0, .1, 0, 0, .2, 0, 0}, nullptr, {0, 0, 1e-6, 1e9, 1e9}));
    rejects([&] { unadjusted_tolerance.native_derivatives_at(0, 0); },
            "smaller source knot gap does not trigger native tolerance reduction");
    const auto too_narrow =
        BsplineCurve::from_bgfb(table(2, false, {0, 0, 0, 1, 0, 0}, nullptr, {0, 0, 5e-15, 5e-15}));
    rejects([&] { too_narrow.native_derivatives_at(0, 0); },
            "interpolation denominator below native tolerance is rejected");
    rejects([&] { rational.native_derivatives_at(.5, 25); }, "native derivative order limit");
    rejects([&] { rational.native_derivatives_at(std::numeric_limits<double>::infinity()); },
            "non-finite derivative parameter rejected");
    rejects([&] { rational.native_derivatives_at(std::numeric_limits<double>::max()); },
            "parameter mapping overflow rejected");
    const auto zero = BsplineCurve::from_bgfb(table(2, false, {0, 0, 0, 1, 0, 0}, {1, 0}));
    rejects([&] { zero.native_derivatives_at(0); }, "native tolerance cannot unweight zero pole");
    const auto singular = BsplineCurve::from_bgfb(table(2, false, {0, 0, 0, 1, 0, 0}, {1, -1}));
    rejects([&] { singular.native_derivatives_at(.5); }, "zero evaluated weight rejected");

    Json poles = Json::array();
    for (unsigned i = 0; i < 26; ++i)
        for (double x : {double(i), 0., 0.})
            poles.push_back(x);
    const auto maximum = BsplineCurve::from_bgfb(table(26, false, poles));
    check(near(maximum.native_derivatives_at(.25, 1)[1], {25, 0, 0}),
          "native order 26 is supported");
    poles.insert(poles.end(), {26., 0., 0.});
    const auto excessive = BsplineCurve::from_bgfb(table(27, false, poles));
    rejects([&] { excessive.native_derivatives_at(.5); }, "native fixed order bound is explicit");

    const auto source_poles = circle.poles();
    const auto source_weights = circle.weights();
    const auto source_knots = circle.knots();
    const auto expected = circle.native_derivatives_at(.2);
    std::vector<std::future<std::vector<Point3>>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return circle.native_derivatives_at(.2); }));
    for (auto &job : jobs)
        check(job.get() == expected,
              "shared native derivative object is deterministic across threads");
    check(circle.poles() == source_poles && circle.weights() == source_weights &&
              circle.knots() == source_knots,
          "native tolerance roundtrip never mutates source arrays");
    return checks;
}
