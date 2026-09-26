#include "native_curve_closest.hpp"
#include <future>
using namespace p3d;
using namespace p3d::curve_detail;
namespace {
long double value(std::vector<long double> a, long double s) {
    for (std::size_t n = a.size(); n > 1; --n)
        for (std::size_t i = 0; i + 1 < n; ++i)
            a[i] = (1 - s) * a[i] + s * a[i + 1];
    return a[0];
}
std::array<long double, 4> value(const std::vector<BezierPole> &p, long double s) {
    std::array<long double, 4> out{};
    for (unsigned axis = 0; axis < 4; ++axis) {
        std::vector<long double> a;
        for (const auto &h : p)
            a.push_back(h[axis]);
        out[axis] = value(a, s);
    }
    return out;
}
} // namespace
unsigned native_curve_closest_tests() {
    unsigned count = 0;
    auto check = [&](bool b, const char *why) {
        ++count;
        require(b, why);
    };
    auto near = [](double a, double b, double e = 1e-9) {
        return std::abs(a - b) <= e * (1 + std::abs(b));
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
    auto closest = [](const std::vector<BezierPole> &p, const Point3 &q, double s0 = 0.,
                      double s1 = 1.) {
        std::size_t used = 0;
        return native_bezier_closest_point(p, q, s0, s1, {used, 100000000});
    };
    auto perpendiculars = [](const std::vector<BezierPole> &p, const BezierPole &q) {
        std::size_t used = 0;
        return native_bezier_perpendiculars(p, q, {used, 100000000});
    };
    // Verify product polynomials by independent long-double evaluation, across
    // every fast-path pair and high-degree Pascal fallback including order 78.
    for (unsigned na : {1u, 2u, 3u, 4u, 5u, 9u, 26u, 50u, 78u})
        for (unsigned nb : {1u, 2u, 3u, 4u, 5u, 17u, 26u}) {
            if (na + nb > 79)
                continue;
            std::vector<double> a(na), b(nb);
            for (unsigned i = 0; i < na; ++i)
                a[i] = std::sin(i + .25);
            for (unsigned i = 0; i < nb; ++i)
                b[i] = std::cos(i + .75);
            std::size_t used = 0;
            const auto c = native_bezier_product(a, b, {used, 1000000});
            check(c.size() == na + nb - 1, "native product degree");
            for (long double s : {0.L, .125L, .375L, .75L, 1.L}) {
                const auto expected = value(std::vector<long double>(a.begin(), a.end()), s) *
                                      value(std::vector<long double>(b.begin(), b.end()), s);
                check(std::abs(value(std::vector<long double>(c.begin(), c.end()), s) - expected) <
                          2e-14L,
                      "independent Bernstein product evaluation");
            }
        }
    {
        std::size_t used = 0;
        const auto product = native_bezier_product({1, -5e15, 1e16}, {1, .5, 1}, {used, 1000});
        check(product[2] == 1. / 6., "native quadratic product descending cancellation order");
    }
    const std::vector<BezierPole> line{{0, 0, 0, 1}, {4, 0, 0, 1}};
    auto r = closest(line, {1, 2, 3});
    check(r.found && r.parameter == .25 && r.squared_distance == 13 &&
              r.homogeneous == BezierPole{1, 0, 0, 1},
          "line perpendicular minimum");
    check(closest(line, {-1, 2, 0}).parameter == 0 && closest(line, {5, 2, 0}).parameter == 1,
          "outside projection clamps by endpoints");
    check(closest(line, {1, 2, 0}, 1, 0).parameter == .25, "reversed endpoint query");
    check(closest(line, {1, 2, 0}, .5, .75).parameter == .5, "restricted interval endpoint");
    check(closest(line, {-4, 0, 0}, -1, -2).parameter == -1, "explicit extrapolated endpoints");
    const std::vector<BezierPole> constant{{2, 3, 4, 1}, {2, 3, 4, 1}};
    check(perpendiculars(constant, {0, 0, 0, 1}).all_parameters &&
              closest(constant, {0, 0, 0}, 1, 0).parameter == 1,
          "constant curve sentinel and first equal endpoint");
    check(perpendiculars(line, {0, 1, 0, 0}).parameters.empty(),
          "constant directional polynomial has no isolated roots");
    const std::vector<BezierPole> rational{{0, 0, 0, 1}, {8, 0, 0, 2}};
    r = closest(rational, {1, 2, 0});
    check(r.found && near(r.parameter, 1. / 7) && near(r.squared_distance, 4),
          "rational line parameter differs from Cartesian line");
    auto negative = rational;
    for (auto &h : negative)
        for (double &x : h)
            x = -x;
    auto neg = closest(negative, {1, 2, 0});
    check(near(neg.parameter, r.parameter) && near(neg.squared_distance, r.squared_distance),
          "negative homogeneous scale");
    const std::vector<BezierPole> zero{{0, 0, 0, 0}, {0, 0, 0, 0}};
    check(!closest(zero, {1, 0, 0}).found, "all zero weights produce no candidate");
    check(closest({{2, 0, 0, 0}, {2, 0, 0, 1}}, {2, 0, 0}).parameter == 1,
          "zero weight endpoint skipped");
    const std::vector<BezierPole> tiny{{0, 0, 0, 1e-20}, {4e-20, 0, 0, 1e-20}};
    r = closest(tiny, {1, 2, 0});
    check(r.found && near(r.parameter, .25) && near(r.squared_distance, 4),
          "tiny nonzero weights do not use projection epsilon");
    for (double delta : {5e-9, 2e-8}) {
        auto p = line;
        p[1][3] += delta;
        auto q = perpendiculars(p, {1, 2, 0, 1});
        check(q.unit_weight_branch == (delta < 1e-8) &&
                  q.coefficients.size() == (delta < 1e-8 ? 2 : 3),
              "native near-one weight branch and polynomial degree");
    }
    // y=x^2 on [-1,1], nearest to (0,1) has equal minima at +/-sqrt(.5).
    const std::vector<BezierPole> parabola{{-1, 1, 0, 1}, {0, -1, 0, 1}, {1, 1, 0, 1}};
    r = closest(parabola, {0, 1, 0});
    check(r.found && near(r.squared_distance, .75) &&
              near(std::abs(r.homogeneous[0]), std::sqrt(.5)),
          "all stationary candidates compared instead of first root");
    // Independent dense lower bound and stationarity residual on polynomial and
    // rational curves spanning both product branches, up to native max order.
    for (unsigned n : {3u, 4u, 5u, 8u, 26u})
        for (bool weighted : {false, true}) {
            std::vector<BezierPole> p;
            for (unsigned i = 0; i < n; ++i) {
                double w = weighted ? .7 + .02 * i : 1.;
                p.push_back({w * (double(i) / (n - 1)), w * std::sin(double(i) * .5),
                             w * std::cos(double(i) * .3), w});
            }
            const auto original = p;
            const Point3 target{.37, .1, .2};
            r = closest(p, target);
            check(r.found && p == original && r.parameter >= 0 && r.parameter <= 1,
                  "bounded immutable closest query");
            long double sampled = 1e30L;
            for (unsigned i = 0; i <= 1000; ++i) {
                const auto h = value(p, static_cast<long double>(i) / 1000);
                long double d2 = 0;
                for (unsigned axis = 0; axis < 3; ++axis) {
                    auto dx = h[axis] / h[3] - target[axis];
                    d2 += dx * dx;
                }
                sampled = std::min(sampled, d2);
            }
            check(r.squared_distance <= double(sampled) + 1e-9,
                  "closest is at least as close as independent dense samples");
            if (r.parameter > 0 && r.parameter < 1) {
                const auto h = value(p, r.parameter);
                std::vector<BezierPole> derivative;
                for (unsigned i = 1; i < n; ++i) {
                    BezierPole d{};
                    for (unsigned axis = 0; axis < 4; ++axis)
                        d[axis] = (p[i][axis] - p[i - 1][axis]) * (n - 1);
                    derivative.push_back(d);
                }
                const auto d = value(derivative, r.parameter);
                long double dot = 0;
                for (unsigned axis = 0; axis < 3; ++axis)
                    dot += (h[axis] / h[3] - target[axis]) * (d[axis] * h[3] - h[axis] * d[3]);
                check(std::abs(dot) < 1e-8L, "independent rational perpendicular residual");
            }
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
    auto query = [&](const BsplineCurve &c, const Point3 &p) {
        std::size_t used = 0;
        return native_curve_closest_point(c, p, {used, 100000000});
    };
    auto c = spline(2, {0, 0, 0, 2, 0, 0, 4, 0, 0}, nullptr, false, {2, 2, 5, 8, 8});
    auto q = query(c, {2, 1, 0});
    check(q.found && q.fraction == .5 && q.point == Point3{2, 0, 0} && q.source_span == 0 &&
              q.segments == 2,
          "shared endpoint keeps first span and original domain");
    c = spline(2, {0, 0, 0, 1, 0, 0, 9, 0, 0, 10, 0, 0}, nullptr, false, {2, 2, 5, 5, 8, 8});
    q = query(c, {9, 1, 0});
    check(q.found && q.source_span == 2 && q.point == Point3{9, 0, 0} && q.skipped_intervals == 1 &&
              q.fraction == .5,
          "internal break keeps original incoming control point");
    c = spline(3, {0, 0, 0, 1, 2, 0, 3, -1, 0, 4, 0, 0}, nullptr, false, {-2, -1, 0, 1, 2, 3, 4});
    q = query(c, {2, 1, 0});
    check(q.found && near(q.point[0], c.point_at(q.fraction)[0]) &&
              near(q.point[1], c.point_at(q.fraction)[1]),
          "nonclamped closest agrees with independent spline evaluation");
    c = spline(2, {0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0}, nullptr, true);
    q = query(c, {1, -1, 0});
    check(q.found && near(q.point[0], 1) && near(q.point[1], 0) && near(q.squared_distance, 1),
          "periodic curve includes native wrapped spans");
    c = spline(2, {0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0}, {1e-320, 1e-320, 1, 1}, false,
               {0, 0, .5, .5, 1, 1});
    q = query(c, {1, 0, 0});
    check(q.found && q.point == Point3{1, 0, 0} && q.source_span == 2,
          "project only final winner after tiny earlier candidate");
    c = spline(2, {0, 0, 0, 0, 0, 0}, {0, 0});
    check(!query(c, {1, 0, 0}).found, "whole curve without valid weight reports absent result");
    std::size_t used = 0;
    native_bezier_closest_point(parabola, {0, 1, 0}, 0, 1, {used, 1000000});
    const auto cost = used;
    used = 0;
    native_bezier_closest_point(parabola, {0, 1, 0}, 0, 1, {used, cost});
    check(used == cost, "exact cumulative budget");
    used = 0;
    rejects([&] { native_bezier_closest_point(parabola, {0, 1, 0}, 0, 1, {used, cost - 1}); },
            "budget exhaustion");
    rejects([&] { closest(std::vector<BezierPole>(27), {0, 0, 0}); }, "oversized curve");
    rejects([&] { closest(line, {NAN, 0, 0}); }, "nonfinite query");
    rejects([&] { closest(line, {1e308, 0, 0}); }, "distance overflow");
    rejects(
        [&] {
            std::size_t w = 0;
            native_bezier_product({}, {1}, {w, 1000});
        },
        "empty product");
    const auto expected = closest(parabola, {0, 1, 0});
    std::vector<std::future<BezierClosestPoint>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return closest(parabola, {0, 1, 0}); }));
    for (auto &job : jobs) {
        auto hit = job.get();
        check(hit.parameter == expected.parameter && hit.homogeneous == expected.homogeneous,
              "parallel closest uses independent state");
    }
    return count;
}
