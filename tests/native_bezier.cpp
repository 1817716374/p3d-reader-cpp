#include "native_bezier.hpp"
#include <future>
using namespace p3d;
using namespace p3d::curve_detail;
namespace {
using Poles = std::vector<BezierPole>;
bool near(double a, double b, double tolerance = 3e-11) {
    return std::abs(a - b) <= tolerance * std::max({1., std::abs(a), std::abs(b)});
}
std::pair<Point3, Point3> reference(const Poles &p, double t) {
    // Independent Bernstein sum, with a separately evaluated derivative curve.
    auto basis = [&](unsigned degree, unsigned i) {
        double binomial = 1;
        for (unsigned j = 1; j <= i; ++j)
            binomial *= double(degree - i + j) / j;
        return binomial * std::pow(t, i) * std::pow(1 - t, degree - i);
    };
    BezierPole h{}, dh{};
    const unsigned degree = unsigned(p.size() - 1);
    for (unsigned i = 0; i <= degree; ++i)
        for (unsigned k = 0; k < 4; ++k)
            h[k] += basis(degree, i) * p[i][k];
    for (unsigned i = 0; i < degree; ++i)
        for (unsigned k = 0; k < 4; ++k)
            dh[k] += basis(degree - 1, i) * degree * (p[i + 1][k] - p[i][k]);
    Point3 point{}, tangent{};
    for (unsigned k = 0; k < 3; ++k) {
        point[k] = h[k] / h[3];
        tangent[k] = (dh[k] * h[3] - h[k] * dh[3]) / (h[3] * h[3]);
    }
    return {point, tangent};
}
} // namespace
unsigned native_bezier_tests() {
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
    std::size_t used = 0;
    BezierWork work{used, 100000000};
    for (unsigned order : {2u, 3u, 4u, 8u, 16u, 26u})
        for (double sign : {1., -1.}) {
            Poles p;
            for (unsigned i = 0; i < order; ++i) {
                const double w = sign * (.5 + .125 * i);
                p.push_back({w * std::sin(double(i)), w * .1 * i * i, w * .3 * i, w});
            }
            const auto saved = p;
            for (double t : {0., .01, .23, .5, .77, .99, 1.}) {
                const auto value = native_bezier_point_tangent(p, t, work);
                const auto expected = reference(p, t);
                for (unsigned k = 0; k < 3; ++k) {
                    check(near(value.point[k], expected.first[k]),
                          "native point agrees with independent rational Bernstein evaluation");
                    check(near(value.tangent[k], expected.second[k]),
                          "native tangent agrees with independent derivative control curve");
                }
                check(!value.weight_fallback, "ordinary positive or negative W stays above cutoff");
            }
            check(p == saved, "native evaluator preserves homogeneous source controls");
        }
    for (double w : {0., 1e-12, -1e-12, std::nextafter(1e-12, 0.), std::nextafter(1e-12, 1.),
                     std::nextafter(-1e-12, -1.)}) {
        const auto p = native_bezier_point_tangent(
            {{2 * w, 3 * w, 4 * w, w}, {4 * w, 3 * w, 4 * w, w}}, 0, work);
        const bool fallback = std::abs(w) <= 1e-12;
        check(p.weight_fallback == fallback &&
                  (fallback ? p.point == Point3{} && p.tangent == Point3{}
                            : near(p.point[0], 2) && near(p.point[1], 3) && near(p.point[2], 4) &&
                                  near(p.tangent[0], 2) && p.tangent[1] == 0 && p.tangent[2] == 0),
              "native safe reciprocal uses absolute strict 1e-12 cutoff including negative W");
    }
    const auto cancelled = native_bezier_point_tangent({{1, 2, 3, 1}, {2, 4, 6, -1}}, .5, work);
    check(cancelled.weight_fallback && cancelled.point == Point3{} && cancelled.tangent == Point3{},
          "zero evaluated W uses native zero reciprocal instead of a thrown pole singularity");
    const auto extrapolated = native_bezier_point_tangent({{1, 2, 3, 1}, {2, 4, 6, 1}}, -2, work);
    check(extrapolated.point == Point3{-1, -2, -3} && extrapolated.tangent == Point3{1, 2, 3},
          "Bezier fractions extrapolate without a hidden clamp");
    const Poles right_angle{{0, 0, 0, 1}, {1, 0, 0, 1}, {1, 1, 0, 1}};
    check(native_bezier_edge_count(right_angle, true, 0, .3, 0, work) == 6,
          "native ninety-degree control turn uses degree-minus-one times angular ceiling");
    check(native_bezier_edge_count(right_angle, true, .01, 0, 0, work) == 11,
          "native chord estimate retains its 1.25 multiplier");
    check(native_bezier_edge_count(right_angle, true, 0, 0, .1, work) == 20,
          "native length estimate multiplies ceiling by degree");
    check(native_bezier_edge_count(right_angle, true, -1, -1, -1, work) == 2,
          "nonpositive tolerances disable refinements");
    check(native_bezier_edge_count({{0, 0, 0, 1}, {0, 0, 0, 1}, {0, 2, 0, 1}}, true, 0, .3, 0,
                                   work) == 6,
          "zero control edge normalizes to X and participates in angle estimation");
    Poles straight;
    for (unsigned i = 0; i < 26; ++i)
        straight.push_back({double(i), 0, 0, 1});
    check(
        native_bezier_edge_count(straight, true, 0, .3, 0, work) == 25,
        "native higher-order straight curve is not collapsed by newer upstream collinearity rule");
    check(native_bezier_edge_count({{7, 8, 9, 0}, {2, 0, 0, 2}, {3, 3, 0, 3}}, false, 0, .3, 0,
                                   work) == 6,
          "zero homogeneous W projects to zero only in edge estimation");
    for (double sign : {1., -1.}) {
        Poles scaled = right_angle;
        for (auto &p : scaled)
            for (auto &x : p)
                x *= 7 * sign;
        check(native_bezier_edge_count(scaled, false, 0, .3, 0, work) == 6,
              "signed rational control projection preserves angular count");
    }
    check(native_bezier_edge_count({}, false, 0, .3, 0, work) == 0,
          "edge estimator returns zero for fewer than two poles");
    const Poles segment{{1, 0, 0, 1}, {0, 1, 0, 1}};
    const auto triangle = native_bezier_moments(segment, 0, 1, {}, work);
    check(near(triangle.normal[2], .5) && near(triangle.centroid_tensor[0][2], 1. / 6) &&
              near(triangle.centroid_tensor[1][2], 1. / 6) && triangle.evaluations == 17 &&
              triangle.weight_fallbacks == 0,
          "straight chord adds independent origin triangle area and first moment");
    const auto reverse = native_bezier_moments(segment, 1, 0, {}, work);
    check(near(reverse.normal[2], -.5) && near(reverse.centroid_tensor[0][2], -1. / 6),
          "reversed integration interval retains oriented moments");
    const auto empty = native_bezier_moments(segment, .4, .4, {}, work);
    check(empty.normal == Point3{} && empty.evaluations == 17,
          "zero-width interval keeps native query sequence and zero area");
    const Poles loop{{0, 0, 0, 1}, {1, 0, 0, 1}, {1, 1, 0, 1}, {0, 0, 0, 1}};
    const auto loop_moment = native_bezier_area(loop, true, {}, work);
    check(near(loop_moment.normal[2], 3. / 20) &&
              near(loop_moment.centroid_tensor[0][2], 9. / 140) &&
              near(loop_moment.centroid_tensor[1][2], 9. / 280),
          "closed cubic loop matches exact polynomial area and first moments");
    const double w = std::sqrt(.5), pi = std::acos(-1.);
    const Poles quarter{{1, 0, 0, 1}, {w, w, 0, w}, {0, 1, 0, 1}};
    const auto circle = native_bezier_area(quarter, false, {}, work);
    check(circle.edges == 6 && circle.evaluations == 102 && near(circle.normal[2], pi / 4) &&
              near(circle.centroid_tensor[0][2], 1. / 3) &&
              near(circle.centroid_tensor[1][2], 1. / 3),
          "rational quarter-circle sector matches analytic area and centroid moments");
    Poles ellipse = quarter;
    for (auto &p : ellipse) {
        p[0] = 2 * p[0] + 10 * p[3];
        p[1] = 3 * p[1] + 20 * p[3];
        p[2] = 30 * p[3];
    }
    const auto ellipse_moment = native_bezier_area(ellipse, false, {10, 20, 30}, work);
    check(near(ellipse_moment.normal[2], 1.5 * pi) &&
              near(ellipse_moment.centroid_tensor[0][2], 4) &&
              near(ellipse_moment.centroid_tensor[1][2], 6),
          "translated anisotropic ellipse retains reference-relative first moments");
    const Poles rational_loop{{0, 0, 0, 1}, {1.4, 0, 0, .7}, {3, 2.5, 0, 1.5}, {0, 0, 0, 2}};
    const auto rational = native_bezier_area(rational_loop, false, {}, work);
    std::array<double, 3> integral{};
    constexpr unsigned steps = 4096;
    for (unsigned i = 0; i <= steps; ++i) {
        const auto p = reference(rational_loop, double(i) / steps);
        const double x = p.first[0], y = p.first[1], dx = p.second[0], dy = p.second[1];
        const double factor = i == 0 || i == steps ? 1. : i % 2 ? 4. : 2.;
        integral[0] += factor * .5 * (x * dy - y * dx);
        integral[1] += factor * .5 * x * x * dy;
        integral[2] -= factor * .5 * y * y * dx;
    }
    for (double &x : integral)
        x /= 3 * steps;
    check(near(rational.normal[2], integral[0], 1e-9) &&
              near(rational.centroid_tensor[0][2], integral[1], 1e-9) &&
              near(rational.centroid_tensor[1][2], integral[2], 1e-9),
          "rational loop moments agree with independent Green-theorem Simpson integrals");
    const auto zero = native_bezier_area({{1, 2, 3, 0}, {4, 5, 6, 0}}, false, {}, work);
    check(zero.normal == Point3{} && zero.weight_fallbacks == 17,
          "native weight fallback is reported and is not a denominator regularity certificate");
    auto concurrent = std::async(std::launch::async, [&] {
        std::size_t count = 0;
        return native_bezier_area(quarter, false, {}, {count, 10000000});
    });
    const auto parallel = concurrent.get();
    check(parallel.normal == circle.normal && parallel.centroid_tensor == circle.centroid_tensor,
          "Bezier quadrature owns local state and is deterministic under parallel calls");
    rejects([&] { native_bezier_point_tangent({}, 0, work); }, "empty evaluator input rejects");
    rejects([&] { native_bezier_point_tangent(Poles(27, {1, 2, 3, 1}), 0, work); },
            "Bezier order storage is bounded before evaluation");
    rejects(
        [&] {
            native_bezier_point_tangent(segment, std::numeric_limits<double>::infinity(), work);
        },
        "nonfinite Bezier parameter rejects");
    rejects([&] { native_bezier_edge_count(right_angle, true, 1e-300, 0, 0, work); },
            "edge estimates beyond signed integer range reject before conversion");
    rejects(
        [&] {
            std::size_t count = 0;
            native_bezier_area(quarter, false, {}, {count, 100});
        },
        "estimation and all quadrature queries share a bounded work counter");
    return checks;
}
