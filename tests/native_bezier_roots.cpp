#include "native_bezier_roots.hpp"
#include "native_curve_plane.hpp"
#include <future>
using namespace p3d;
using namespace p3d::curve_detail;
namespace {
std::vector<double> polynomial(std::initializer_list<long double> roots) {
    std::vector<long double> power{1};
    for (auto root : roots) {
        std::vector<long double> next(power.size() + 1);
        for (std::size_t i = 0; i < power.size(); ++i) {
            next[i] -= root * power[i];
            next[i + 1] += power[i];
        }
        power = std::move(next);
    }
    const auto d = power.size() - 1;
    auto choose = [](std::size_t n, std::size_t k) {
        long double v = 1;
        for (std::size_t i = 1; i <= k; ++i)
            v *= static_cast<long double>(n + 1 - i) / i;
        return v;
    };
    std::vector<double> bernstein;
    for (std::size_t i = 0; i <= d; ++i) {
        long double b = 0;
        for (std::size_t j = 0; j <= i; ++j)
            b += power[j] * choose(i, j) / choose(d, j);
        bernstein.push_back(double(b));
    }
    return bernstein;
}
std::vector<BezierPole> curve(const std::vector<double> &a) {
    std::vector<BezierPole> p;
    for (std::size_t i = 0; i < a.size(); ++i)
        p.push_back({a[i], double(i) / double(a.size() - 1), 0, 1});
    return p;
}
} // namespace
unsigned native_bezier_roots_tests() {
    unsigned count = 0;
    auto check = [&](bool v, const char *why) {
        ++count;
        require(v, why);
    };
    auto near = [](double a, double b, double t = 2e-10) { return std::abs(a - b) <= t; };
    auto rejects = [&](auto fn, const char *why) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    auto solve = [](const std::vector<double> &a, bool endpoints = false) {
        std::size_t work = 0;
        return native_bezier_roots(a, {work, 10000000}, endpoints);
    };
    for (const auto &a : {std::vector<double>{-1, 3}, {3, -1}, {0, 1}, {1, 0}, {-1, -2}, {1, 2}}) {
        const auto r = solve(a);
        check(r.success && r.working_coefficients == a, "linear root source unchanged");
        if (a[0] * a[1] > 0)
            check(r.parameters.empty(), "linear same-sign root excluded");
        else
            check(r.parameters.size() == 1 && near(r.parameters[0], -a[0] / (a[1] - a[0])),
                  "linear root and endpoint included");
    }
    for (unsigned n : {2u, 3u, 4u, 6u, 7u, 26u, 78u}) {
        const auto r = solve(std::vector<double>(n, 0));
        check(r.success && r.parameters.size() == n && r.parameters.front() == 0 &&
                  r.parameters.back() == 1,
              "zero coefficients return native all-parameter sentinel");
        for (unsigned i = 1; i + 1 < n; ++i)
            check(r.parameters[i] == double(i) / double(n - 1),
                  "all-parameter sentinel equispaced");
        check(solve(std::vector<double>(n, 1)).parameters.empty(), "nonzero constant has no roots");
    }
    check(solve(std::vector<double>(4, 1e-200)).parameters.size() == 4,
          "native range-product underflow preserves all-parameter sentinel");
    for (auto test : {std::pair{polynomial({.25L, .75L}), std::vector<double>{.25, .75}},
                      std::pair{polynomial({.5L, .5L}), std::vector<double>{.5}},
                      std::pair{polynomial({-1.L, .5L}), std::vector<double>{.5}},
                      std::pair{polynomial({.25L, .5L, .75L}), std::vector<double>{.25, .5, .75}},
                      std::pair{polynomial({0.L, .5L, 1.L}), std::vector<double>{0, .5, 1}},
                      std::pair{polynomial({.25L, .25L, .75L}), std::vector<double>{.25, .75}},
                      std::pair{polynomial({.5L, .5L, .5L}), std::vector<double>{.5}},
                      std::pair{polynomial({.125L, .375L, .625L, .875L}),
                                std::vector<double>{.125, .375, .625, .875}},
                      std::pair{polynomial({.125L, .25L, .5L, .75L, .875L}),
                                std::vector<double>{.125, .25, .5, .75, .875}},
                      std::pair{polynomial({.125L, .25L, .375L, .625L, .75L, .875L}),
                                std::vector<double>{.125, .25, .375, .625, .75, .875}}}) {
        const auto before = test.first;
        const auto r = solve(test.first);
        check(r.success && test.first == before && r.parameters.size() == test.second.size(),
              "known distinct root count including repeated and exterior roots");
        for (std::size_t i = 0; i < test.second.size(); ++i) {
            const double u = r.parameters[i];
            check(near(u, test.second[i]), "independent factored polynomial root");
            // Independent long-double de Casteljau, unlike production scalar evaluation.
            std::vector<long double> p(test.first.begin(), test.first.end());
            for (std::size_t n = p.size(); n > 1; --n)
                for (std::size_t k = 0; k + 1 < n; ++k)
                    p[k] = (1 - static_cast<long double>(u)) * p[k] +
                           static_cast<long double>(u) * p[k + 1];
            check(std::abs(p[0]) < 2e-10L, "independent polynomial residual");
        }
    }
    {
        const auto r = solve({1, -1 - 1e-12, 1});
        check(r.parameters.size() == 2 && r.parameters[0] < .5 && r.parameters[1] > .5,
              "native quadratic tolerance retains roots newer upstream merges");
        check(solve({1, -1 - 1e-15, 1}).parameters.size() == 1,
              "native discriminant tolerance merges closer roots");
        check(solve({1, 0, -1}).parameters == std::vector<double>{.5}, "quadratic linear fallback");
        check(solve({1, -0.5, 1}).parameters.empty(), "quadratic negative discriminant");
    }
    for (unsigned n = 4; n <= 78; ++n) {
        std::vector<double> a;
        for (unsigned i = 0; i < n; ++i)
            a.push_back(double(i) / double(n - 1) - .37);
        const auto r = solve(a);
        check(r.success && r.parameters.size() == 1 && near(r.parameters[0], .37),
              "all supported orders recover degree-elevated linear root");
        for (auto &x : a)
            x = -x;
        const auto reverse = solve(a);
        check(reverse.success && reverse.parameters.size() == 1 && near(reverse.parameters[0], .37),
              "decreasing monotonic bracket works at every order");
    }
    {
        const auto raw = solve({1e-12, 1}), extra = solve({1e-12, 1}, true);
        check(raw.parameters.empty() && extra.parameters.size() == 1 && extra.parameters[0] < 0 &&
                  near(extra.parameters[0], -1e-12, 1e-20),
              "optional endpoint root may be outside zero");
        const auto end = solve({-1, -1e-12}, true);
        check(end.parameters.size() == 1 && end.parameters[0] > 1 &&
                  near(end.parameters[0], 1 + 1e-12, 1e-15),
              "signed Newton step retains root just past one");
        check(solve({1e-7, 1}, true).parameters.empty(), "distant exterior root not added");
        const auto appended = solve(polynomial({-1e-12L, .5L}), true);
        check(appended.parameters.size() == 2 && near(appended.parameters[0], .5) &&
                  appended.parameters[1] < 0,
              "endpoint root appended without sorting");
    }
    const BezierPole plane{1, 0, 0, 0};
    for (const auto &a :
         {polynomial({.25L, .75L}), polynomial({.25L, .5L, .75L}), std::vector<double>{1e-12, 1}}) {
        std::size_t used = 0;
        const auto p = curve(a), before = p;
        const auto r = native_bezier_plane_intersections(p, plane, 26, {used, 1000000});
        check(r.success && !r.all_parameters && p == before &&
                  r.points.size() == r.parameters.size(),
              "plane query preserves source and paired homogeneous results");
        for (std::size_t i = 0; i < r.points.size(); ++i)
            check(near(r.points[i][0], 0) && near(r.points[i][1], r.parameters[i]) &&
                      near(r.points[i][3], 1),
                  "independent plane intersection coordinates");
    }
    {
        std::size_t used = 0;
        const auto p = curve(polynomial({.25L, .75L}));
        const auto partial = native_bezier_plane_intersections(p, plane, 1, {used, 1000000});
        check(!partial.success && partial.parameters.size() == 1 &&
                  near(partial.parameters[0], .25),
              "output overflow retains intersection prefix");
        const auto none = native_bezier_plane_intersections(p, plane, 0, {used, 1000000});
        check(!none.success && none.points.empty(), "zero capacity rejects discrete intersection");
        const auto all =
            native_bezier_plane_intersections(curve({0, 0, 0}), plane, 0, {used, 1000000});
        check(all.success && all.all_parameters && all.points.empty(),
              "coplanar sentinel emits no arbitrary points");
        const auto w0 = native_bezier_plane_intersections({{-1, 1, 2, 0}, {1, 3, 4, 0}}, plane, 2,
                                                          {used, 1000000});
        check(w0.success && w0.points == std::vector<BezierPole>{{0, 2, 3, 0}},
              "zero-weight homogeneous intersection preserved");
        const auto arithmetic = native_bezier_plane_intersections(
            {{1e16, -1e16, 1, -1}, {1e16, -1e16, 1, -1}}, {1, 1, 1, 1}, 2, {used, 1000000});
        check(arithmetic.all_parameters && arithmetic.points.empty(),
              "native plane dot XYZ then W order");
    }
    for (const auto &a : {std::vector<double>{},
                          {1},
                          std::vector<double>(79, 1),
                          {std::numeric_limits<double>::infinity(), 1}})
        rejects([&] { solve(a); }, "invalid scalar root inputs rejected");
    rejects([&] { solve({-1e308, 1e308}); }, "root overflow not silently rescaled");
    {
        std::size_t used = 0;
        const auto a = polynomial({.25L, .5L, .75L});
        const auto r = native_bezier_roots(a, {used, 1000000}, true);
        const auto needed = used;
        used = 0;
        rejects([&] { native_bezier_roots(a, {used, needed - 1}, true); },
                "cumulative root work budget");
        used = 0;
        check(native_bezier_roots(a, {used, needed}, true).parameters == r.parameters,
              "exact root budget");
        used = 0;
        rejects(
            [&] {
                native_bezier_plane_intersections(std::vector<BezierPole>(27), plane, 27,
                                                  {used, 1000000});
            },
            "plane curve separate order limit");
    }
    std::vector<std::future<BezierRoots>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(std::async(std::launch::async, [&] {
            return solve(polynomial({.125L, .375L, .625L, .875L}), true);
        }));
    const auto expected = solve(polynomial({.125L, .375L, .625L, .875L}), true);
    for (auto &job : jobs) {
        const auto r = job.get();
        check(r.success == expected.success && r.parameters == expected.parameters &&
                  r.working_coefficients == expected.working_coefficients,
              "parallel root calls use immutable Pascal table and independent state");
    }
    auto spline = [](unsigned order, Json poles, Json weights = nullptr, bool closed = false,
                     Json knots = nullptr) {
        return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                        {"order", order},
                                        {"poles", poles},
                                        {"weights", weights},
                                        {"closed", closed},
                                        {"knots", knots}});
    };
    auto intersect = [&](const BsplineCurve &c, std::size_t limit = 100) {
        std::size_t work = 0;
        return native_curve_plane_intersections(c, plane, limit, {work, 1000000});
    };
    {
        const auto c = spline(2, {-1, 0, 0, 0, 1, 0, 1, 2, 0}, nullptr, false, {2, 2, 5, 8, 8});
        const auto r = intersect(c);
        check(r.segments == 2 && r.intersections.size() == 2,
              "shared span endpoint reported twice");
        for (unsigned i = 0; i < 2; ++i) {
            const auto &hit = r.intersections[i];
            check(hit.fraction == .5 && hit.point == Point3{0, 1, 0} && hit.source_span == i &&
                      hit.projection_succeeded,
                  "original non-unit domain converted to whole-curve fraction");
        }
        rejects([&] { intersect(c, 1); },
                "aggregate intersection output budget does not truncate silently");
    }
    {
        const auto c = spline(2, {-1, 0, 0, 1, 0, 0, -2, 10, 0, 2, 10, 0}, nullptr, false,
                              {0, 0, .5, .5, 1, 1});
        const auto r = intersect(c);
        check(r.intersections.size() == 2 && r.skipped_intervals == 1,
              "full internal knot break skips null span and preserves both supports");
        check(r.intersections[0].fraction == .25 && r.intersections[1].fraction == .75 &&
                  r.intersections[1].point == Point3{0, 10, 0} &&
                  r.intersections[1].source_span == 2,
              "incoming span endpoint not overwritten by prior span");
    }
    {
        const double w = std::sqrt(.5);
        const auto c = spline(
            3, {1, 0, 0, w, w, 0, 0, 1, 0, -w, w, 0, -1, 0, 0, -w, -w, 0, 0, -1, 0, w, -w, 0},
            {1, w, 1, w, 1, w, 1, w}, true,
            {-.25, 0, 0, 0, .25, .25, .5, .5, .75, .75, 1, 1, 1.25});
        const auto r = intersect(c);
        check(r.segments == 4 && r.skipped_intervals == 4 && r.intersections.size() == 4,
              "periodic rational circle preserves duplicate quadrant endpoints");
        for (unsigned i = 0; i < 4; ++i)
            check(near(r.intersections[i].fraction, i < 2 ? .25 : .75) &&
                      near(r.intersections[i].point[1], i < 2 ? 1. : -1.),
                  "periodic support shift and projected circle intersections");
    }
    {
        const auto c = spline(3, {-1, 0, 0, 0, 2, 0, 1, 0, 0, 2, 2, 0}, nullptr, false,
                              {-1, 0, 1, 2, 3, 4, 5});
        const auto r = intersect(c);
        check(r.intersections.size() == 1 && near(r.intersections[0].fraction, .25),
              "unclamped B-spline support saturation retains parameter");
        const auto p = c.point_at(r.intersections[0].fraction);
        for (unsigned i = 0; i < 3; ++i)
            check(near(p[i], r.intersections[0].point[i]),
                  "intersection agrees with independent B-spline point query");
    }
    for (double w : {0., 1e-20, -2.}) {
        const auto r = intersect(spline(2, {-1, 0, 0, 1, 2, 0}, {w, w}));
        check(r.intersections.size() == 1 && r.intersections[0].weight == w &&
                  r.intersections[0].projection_succeeded == (w != 0),
              "curve plane projection tests exact weight zero");
        check(r.intersections[0].point == (w == 0 ? Point3{} : Point3{0, 1. / w, 0}),
              "zero weight becomes zero point while tiny nonzero weight remains projected");
    }
    {
        std::size_t used = 0;
        const auto c = spline(2, {0, 0, 0, 0, 1, 0});
        const auto r = native_curve_plane_intersections(c, plane, 0, {used, 1000000});
        check(r.intersections.empty() && r.all_parameter_segments == 1,
              "coplanar B-spline span has no isolated intersections");
        used = 0;
        rejects([&] { native_curve_plane_intersections(c, plane, 10, {used, 1}); },
                "curve plane span work is bounded");
    }
    return count;
}
