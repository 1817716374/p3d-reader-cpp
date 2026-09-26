#include "native_curve_range.hpp"
#include <future>
using namespace p3d;
using namespace p3d::curve_detail;
namespace {
std::array<long double, 4> evaluate(const std::vector<BezierPole> &p, long double u) {
    std::vector<std::array<long double, 4>> h;
    for (const auto &v : p)
        h.push_back({v[0], v[1], v[2], v[3]});
    for (std::size_t n = h.size(); n > 1; --n)
        for (std::size_t i = 0; i + 1 < n; ++i)
            for (unsigned axis = 0; axis < 4; ++axis)
                h[i][axis] = (1 - u) * h[i][axis] + u * h[i + 1][axis];
    return h[0];
}
} // namespace
unsigned native_curve_range_tests() {
    unsigned count = 0;
    auto check = [&](bool b, const char *why) {
        ++count;
        require(b, why);
    };
    auto near = [](double a, double b) { return std::abs(a - b) < 1e-9 * (1 + std::abs(b)); };
    auto query = [](const std::vector<BezierPole> &p) {
        std::size_t work = 0;
        return native_bezier_range(p, {work, 100000000});
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
    const std::vector<BezierPole> quad{{0, 0, 0, 1}, {2, 3, 0, 1}, {4, 0, 0, 1}};
    auto r = query(quad);
    check(r.present && r.low == Point3{0, 0, 0} && r.high == Point3{4, 1.5, 0},
          "quadratic analytic box excludes off-curve control");
    check(r.extrema_evaluations == 3 && r.rejected_weights == 0,
          "constant-axis root grid evaluated without endpoint addition");
    const std::vector<BezierPole> rational{{0, 0, 0, 1}, {2, 4, 0, 2}, {2, 0, 0, 1}};
    r = query(rational);
    check(r.present && near(r.low[0], 0) && near(r.high[0], 2) && near(r.high[1], 4. / 3),
          "rational quadratic analytic extremum");
    auto neg = rational;
    for (auto &p : neg)
        for (double &v : p)
            v = -v;
    const auto rn = query(neg);
    check(rn.low == r.low && rn.high == r.high, "negative homogeneous scaling range");
    r = query({{1, 2, 3, 1}, {1, 2, 3, 1}, {1, 2, 3, 1}, {1, 2, 3, 1}});
    check(r.present && r.low == Point3{1, 2, 3} && r.high == r.low && r.extrema_evaluations == 9,
          "zero derivative grids are not discarded as no discrete extrema");
    for (double w : {0., 1e-12, -1e-12}) {
        r = query({{w, 0, 0, w}, {2 * w, 0, 0, w}});
        check(!r.present && r.rejected_weights == 2,
              "endpoint range strict absolute weight threshold");
    }
    const double above = std::nextafter(1e-12, 1.);
    r = query({{above, 0, 0, above}, {2 * above, 0, 0, above}});
    check(r.present && near(r.low[0], 1) && near(r.high[0], 2),
          "just above range weight threshold accepted");
    const double w = std::ldexp(1., -37);
    r = query({{0, 0, 0, 0}, {2 * w, 0, 0, w}, {0, 0, 0, 0}});
    check(r.present && r.low == Point3{2, 0, 0} && r.high == r.low && r.extrema_evaluations == 12,
          "all-parameter grids recover interior accepted weights");
    check(r.rejected_weights == 8, "rejected endpoint and repeated grid weights counted");
    // Independent dense evaluation checks extrema enclosure, not a control hull.
    for (unsigned n : {3u, 4u, 5u, 8u, 26u})
        for (bool weighted : {false, true}) {
            std::vector<BezierPole> p;
            for (unsigned i = 0; i < n; ++i) {
                double a = double(i) / (n - 1), w0 = weighted ? .5 + .1 * i : 1.;
                p.push_back({a * w0, std::sin(7 * a) * w0, std::cos(5 * a) * w0, w0});
            }
            const auto original = p;
            r = query(p);
            check(r.present && p == original, "immutable range source");
            Point3 sampled_low{1e20, 1e20, 1e20}, sampled_high{-1e20, -1e20, -1e20};
            for (unsigned i = 0; i <= 2000; ++i) {
                auto h = evaluate(p, static_cast<long double>(i) / 2000);
                for (unsigned a = 0; a < 3; ++a) {
                    double x = double(h[a] / h[3]);
                    sampled_low[a] = std::min(sampled_low[a], x);
                    sampled_high[a] = std::max(sampled_high[a], x);
                }
            }
            for (unsigned a = 0; a < 3; ++a) {
                check(r.low[a] <= sampled_low[a] + 1e-9 && r.high[a] >= sampled_high[a] - 1e-9,
                      "native roots contain independent dense samples");
                check(sampled_low[a] - r.low[a] < 1e-5 && r.high[a] - sampled_high[a] < 1e-5,
                      "range extrema are tight against dense sampling");
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
    auto range = [&](const BsplineCurve &c, std::size_t limit = 1000) {
        std::size_t work = 0;
        return native_curve_range(c, limit, {work, 100000000});
    };
    auto c = spline(2, {-2, 0, 0, -1, 0, 0, 7, 1, 0, 8, 1, 0}, nullptr, false, {2, 2, 5, 5, 8, 8});
    r = range(c);
    check(r.low == Point3{-2, 0, 0} && r.high == Point3{8, 1, 0} && r.segments == 2 &&
              r.skipped_intervals == 1,
          "full knot break retains both original independent sides");
    c = spline(3, {0, 0, 0, 1, 3, 0, 2, -1, 0, 4, 0, 0}, nullptr, false, {-2, -1, 0, 1, 2, 3, 4});
    r = range(c);
    check(r.present && r.segments == 2, "nonclamped curve saturated in original domain");
    for (unsigned i = 0; i <= 100; ++i) {
        auto p = c.point_at(double(i) / 100);
        check(p[0] >= r.low[0] - 1e-10 && p[0] <= r.high[0] + 1e-10 && p[1] >= r.low[1] - 1e-10 &&
                  p[1] <= r.high[1] + 1e-10,
              "range contains independent spline evaluator");
    }
    c = spline(2, {0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0}, nullptr, true);
    r = range(c);
    check(r.present && r.low == Point3{0, 0, 0} && r.high == Point3{2, 2, 0} && r.segments == 4,
          "periodic wrapped range spans");
    rejects([&] { range(c, 7); }, "cumulative segment controls bounded");
    std::size_t used = 0;
    native_bezier_range(rational, {used, 1000000});
    const auto cost = used;
    used = 0;
    native_bezier_range(rational, {used, cost});
    check(used == cost, "exact range arithmetic budget");
    used = 0;
    rejects([&] { native_bezier_range(rational, {used, cost - 1}); }, "range arithmetic exhausted");
    rejects([&] { query(std::vector<BezierPole>(27)); }, "unsupported range order");
    rejects([&] { query({{NAN, 0, 0, 1}, {0, 0, 0, 1}}); }, "nonfinite range source");
    std::vector<std::future<NativeCurveRange>> jobs;
    const auto expected = query(rational);
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(std::async(std::launch::async, [&] { return query(rational); }));
    for (auto &job : jobs) {
        auto hit = job.get();
        check(hit.low == expected.low && hit.high == expected.high &&
                  hit.extrema_evaluations == expected.extrema_evaluations,
              "parallel native range independent state");
    }
    return count;
}
