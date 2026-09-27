#include "native_xy_newton.hpp"
#include <future>
using namespace p3d;
using namespace p3d::curve_detail;
namespace {
bool near(double a, double b, double e = 2e-12) {
    return std::abs(a - b) <= e;
}
std::vector<BezierPole> parabola(unsigned degree) {
    std::vector<BezierPole> p;
    for (unsigned i = 0; i <= degree; ++i)
        p.push_back({double(i) / degree,
                     double(i) * (double(i) - 1) / (double(degree) * (degree - 1)), 0, 1});
    return p;
}
} // namespace
unsigned native_xy_newton_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto f) {
        bool failed = false;
        try {
            f();
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, "XY Newton rejects unsupported or unsafe arithmetic and resource exhaustion");
    };
    auto solve = [](const XYNewtonEvaluator &e, Point2 seed) {
        std::size_t work = 0;
        return native_xy_newton(e, seed, {work, 10000000});
    };
    const XYNewtonEvaluator linear = [](double u, double v, XYNewtonValue &q) {
        q = {{u - 2, v + 3}, {{{1, 0}, {0, 1}}}};
        return true;
    };
    const auto line = solve(linear, {10, 5});
    check(line.success && line.converged && line.parameters_applied &&
              line.parameters == Point2{2, -3} && line.iterations == 3 && !line.diagonal_fallback,
          "exact linear root needs two successive small steps after its first update");
    const auto at_root = solve(linear, {2, -3});
    check(at_root.success && at_root.iterations == 2,
          "already exact root still takes two native evaluations");
    const auto exponential = solve(
        [](double u, double v, XYNewtonValue &q) {
            const double a = std::exp(u), b = std::exp(v);
            q = {{a, b}, {{{a, 0}, {0, b}}}};
            return true;
        },
        {0, 0});
    check(exponential.success && !exponential.converged && exponential.parameters_applied &&
              exponential.parameters == Point2{-20, -20} && exponential.iterations == 20 &&
              exponential.reason == "iteration_limit",
          "native iteration limit keeps improved final update without requiring a zero residual");
    const XYNewtonEvaluator unbalanced = [](double u, double v, XYNewtonValue &q) {
        q = {{u - 1, 1e-12 * (v - 2)}, {{{1, 0}, {0, 1e-12}}}};
        return true;
    };
    const auto diagonal = solve(unbalanced, {0, 0});
    check(diagonal.success && diagonal.diagonal_fallback && !diagonal.converged &&
              diagonal.full_iterations == 1 && diagonal.diagonal_iterations == 20 &&
              diagonal.parameters[0] == 1 &&
              near(diagonal.parameters[1], -2 * std::expm1(20 * std::log1p(-1e-12)), 1e-24),
          "row imbalance restarts diagonal route with magnitude-scaled steps and raw-step "
          "convergence");
    const auto diagonal_root = solve(unbalanced, {1, 2});
    check(diagonal_root.success && diagonal_root.diagonal_iterations == 2 &&
              diagonal_root.parameters == Point2{1, 2} && diagonal_root.converged,
          "diagonal route still applies its two successive convergence checks");
    const auto stalled = solve(
        [](double, double, XYNewtonValue &q) {
            q = {{7, 9}, {{{0, 0}, {0, 0}}}};
            return true;
        },
        {.2, .8});
    check(stalled.success && stalled.diagonal_fallback && !stalled.converged &&
              stalled.reason == "diagonal_stalled" && stalled.parameters == Point2{.2, .8},
          "both protected diagonal divisions failing preserves native successful no-change return");
    // The evaluator is deliberately scripted here: its call coordinates expose
    // whether the native restart uses the original or intermediate seed.
    std::vector<Point2> trace;
    const auto restart = solve(
        [&](double u, double v, XYNewtonValue &q) {
            trace.push_back({u, v});
            if (trace.size() == 1)
                q = {{u - 1, v - 2}, {{{1, 0}, {0, 1}}}};
            else
                q = {{1, 1}, {{{0, 0}, {0, 0}}}};
            return true;
        },
        {0, 0});
    check(
        restart.success && trace == std::vector<Point2>{{0, 0}, {1, 2}, {0, 0}} &&
            restart.parameters == Point2{0, 0} && restart.full_iterations == 2,
        "singular full-step restart discards earlier working iterates and starts at original seed");
    unsigned calls = 0;
    const auto failed = solve(
        [&](double u, double v, XYNewtonValue &q) {
            if (++calls == 2)
                return false;
            return linear(u, v, q);
        },
        {10, 5});
    check(!failed.success && !failed.parameters_applied && failed.parameters == Point2{10, 5} &&
              failed.reason == "evaluation_failed",
          "evaluation failure never publishes a preceding successful working step");
    // Force tiny proposed steps while residuals get worse: return success must
    // not overwrite original coordinates merely because step tolerance passes.
    calls = 0;
    const auto rolled_back = solve(
        [&](double, double, XYNewtonValue &q) {
            ++calls;
            q = {{calls == 1 ? 1. : 4., 0}, {{{1e14, 0}, {0, 1e14}}}};
            return true;
        },
        {1, 1});
    check(rolled_back.success && rolled_back.converged && !rolled_back.parameters_applied &&
              rolled_back.parameters == Point2{1, 1},
          "successful small-step convergence may retain original seed when residual worsens");
    auto bezier = [](const std::vector<BezierPole> &a, const std::vector<BezierPole> &b,
                     Point2 seed) {
        std::size_t used = 0;
        return native_bezier_xy_newton(a, b, seed, {used, 10000000});
    };
    const std::vector<BezierPole> horizontal{{0, .25, 40, 1}, {1, .25, -90, 1}};
    for (unsigned degree = 2; degree <= 25; ++degree) {
        const auto p = parabola(degree), original = p;
        const auto r = bezier(p, horizontal, {.8, .2});
        check(r.success && r.converged && r.parameters_applied && near(r.parameters[0], .5) &&
                  near(r.parameters[1], .5),
              "all supported polynomial Bezier degrees converge to the analytic parabola crossing");
        check(p == original, "Newton evaluations never mutate input Bezier poles");
    }
    const std::vector<BezierPole> extended{{-1, .25, 0, 1}, {1, .25, 0, 1}};
    const auto outside = bezier(parabola(2), extended, {-.8, .1});
    check(outside.success && near(outside.parameters[0], -.5) && near(outside.parameters[1], .25),
          "Newton extrapolates; unit-interval acceptance is a separate chordal caller operation");
    const std::vector<BezierPole> rational_a{{0, 0, 0, 2}, {3, 0, 0, 3}};
    const std::vector<BezierPole> rational_b{{2, -4, 100, 4}, {2, 4, -100, 4}};
    const auto rational = bezier(rational_a, rational_b, {.7, .2});
    check(rational.success && near(rational.parameters[0], .4) && near(rational.parameters[1], .5),
          "rational XY Newton uses the quotient derivative and ignores different Z coordinates");
    auto negative_a = rational_a, negative_b = rational_b;
    for (auto &p : negative_a)
        for (auto &x : p)
            x = -x;
    for (auto &p : negative_b)
        for (auto &x : p)
            x = -x;
    const auto negative = bezier(negative_a, negative_b, {.7, .2});
    check(negative.success && negative.parameters == rational.parameters,
          "uniformly negative homogeneous representation preserves projected Newton steps");
    const auto singular = bezier({{0, 0, 0, 1}, {1, 0, 0, -1}}, rational_b, {.5, .5});
    check(!singular.success && singular.reason == "evaluation_failed" &&
              singular.parameters == Point2{.5, .5},
          "exact zero evaluated W fails instead of returning the area-query zero projection");
    const auto threshold = bezier({{0, 0, 0, 1e-15}, {1, 0, 0, 1e-15}}, rational_b, {.2, .5});
    check(!threshold.success, "evaluated W at the native protected-division threshold fails");
    const double unity = std::nextafter(1., 2.);
    const auto near_unit =
        bezier({{0, 0, 0, unity}, {1, 0, 0, unity}}, {{.5, -1, 0, 1}, {.5, 1, 0, 1}}, {.3, .8});
    check(near_unit.success && near_unit.parameters == Point2{.5, .5},
          "near-unit input weights keep raw XY without rational division");
    const auto parallel =
        bezier({{0, 0, 0, 1}, {1, 0, 0, 1}}, {{0, 1, 0, 1}, {1, 1, 0, 1}}, {.3, .3});
    check(parallel.success && parallel.converged && parallel.diagonal_fallback &&
              parallel.parameters == Point2{.3, .3},
          "native successful step test is not a certificate that separated parallel curves "
          "intersect");
    std::size_t used = 0;
    native_bezier_xy_newton(rational_a, rational_b, {.7, .2}, {used, 10000000});
    const auto required = used;
    used = 0;
    const auto exact = native_bezier_xy_newton(rational_a, rational_b, {.7, .2}, {used, required});
    check(exact.parameters == rational.parameters && used == required,
          "exact shared Newton work budget");
    rejects([&] {
        std::size_t w = 0;
        native_bezier_xy_newton(rational_a, rational_b, {.7, .2}, {w, required - 1});
    });
    rejects([&] { solve({}, {0, 0}); });
    rejects([&] { solve(linear, {std::numeric_limits<double>::infinity(), 0}); });
    rejects([&] { bezier(std::vector<BezierPole>(27, {1, 2, 3, 1}), rational_b, {0, 0}); });
    rejects([&] {
        solve(
            [](double, double, XYNewtonValue &q) {
                q = {{1, 1}, {{{1e300, 0}, {0, 1e300}}}};
                return true;
            },
            {0, 0});
    });
    auto future =
        std::async(std::launch::async, [&] { return bezier(rational_a, rational_b, {.7, .2}); });
    const auto concurrent = bezier(rational_a, rational_b, {.7, .2});
    check(future.get().parameters == concurrent.parameters,
          "Newton has no shared mutable iteration state");
    return n;
}
