#include "native_tube.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json make(unsigned order, const std::vector<std::vector<Point3>> &columns,
          const std::vector<double> &knots, const std::vector<std::vector<double>> &weights = {}) {
    Json xyz = Json::array(), w = Json::array();
    for (std::size_t v = 0; v < columns[0].size(); ++v)
        for (std::size_t u = 0; u < columns.size(); ++u) {
            for (double x : columns[u][v])
                xyz.push_back(x);
            if (!weights.empty())
                w.push_back(weights[u][v]);
        }
    return {{"_type", "BsplineSurface"},
            {"numPolesU", columns.size()},
            {"numPolesV", columns[0].size()},
            {"orderU", 2},
            {"orderV", order},
            {"closedU", false},
            {"closedV", false},
            {"knotsU", nullptr},
            {"knotsV", knots},
            {"poles", xyz},
            {"weights", weights.empty() ? Json() : w},
            {"boundaries", nullptr},
            {"numRulesU", 7},
            {"numRulesV", 13},
            {"holeOrigin", 1}};
}
bool near(double a, double b, double tol = 5e-10) {
    return std::abs(a - b) <= tol * std::max({1., std::abs(a), std::abs(b)});
}
bool near(const Point3 &a, const Point3 &b, double tol = 5e-10) {
    for (unsigned k = 0; k < 3; ++k)
        if (!near(a[k], b[k], tol))
            return false;
    return true;
}
} // namespace
unsigned native_tube_surface_elevate_tests() {
    unsigned count = 0;
    auto check = [&](bool ok, const char *why) {
        ++count;
        require(ok, why);
    };
    auto rejects = [&](auto f, const char *why) {
        bool caught = false;
        try {
            f();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    auto elevate = [](const Json &j, unsigned d) {
        TubeBudget b;
        b.max_work = 100000000;
        return elevate_tube_surface_v(BsplineSurface::from_bgfb(j), d, b);
    };
    const std::vector<std::vector<Point3>> lines{
        {{0, 0, 0}, {6, 3, 9}}, {{1, -2, 7}, {-5, 1, 10}}, {{2, 5, 3}, {5, -1, 0}}};
    const auto line = make(2, lines, {0, 0, 1, 1});
    for (unsigned degree : {2u, 3u, 5u, 12u, 25u}) {
        const auto r = elevate(line, degree);
        const auto s = BsplineSurface::from_bgfb(r.surface);
        check(s.v().order() == degree + 1 && s.v().pole_count() == degree + 1,
              "surface elevation target order and row count");
        for (unsigned row = 0; row <= degree; ++row)
            for (unsigned u = 0; u < 3; ++u) {
                Point3 expected{};
                for (unsigned axis = 0; axis < 3; ++axis)
                    expected[axis] =
                        lines[u][0][axis] + (lines[u][1][axis] - lines[u][0][axis]) * row / degree;
                check(near(s.poles()[row * 3 + u], expected),
                      "independent Bernstein line control net");
            }
        check(s.num_rules_u() == 7 && s.num_rules_v() == 13 && s.hole_origin() == 1 &&
                  s.u().knots() == BsplineSurface::from_bgfb(line).u().knots(),
              "surface elevation preserves transverse knots and rule counts");
    }
    // Independent homogeneous Bernstein elevation: Q_i = i/3 P_(i-1) +
    // (1-i/3) P_i, including W. Signed weights must not be normalized.
    const std::vector<std::vector<Point3>> controls{{{0, 1, 2}, {3, 6, 0}, {6, 0, 4}},
                                                    {{2, 3, 5}, {7, -3, 9}, {-1, 4, 6}}};
    for (const auto &weights : {std::vector<std::vector<double>>{},
                                std::vector<std::vector<double>>{{1., .6, 2.}, {-2., -.7, -1.}}}) {
        const auto j = make(3, controls, {0, 0, 0, 1, 1, 1}, weights);
        const auto source = BsplineSurface::from_bgfb(j);
        const auto s = BsplineSurface::from_bgfb(elevate(j, 3).surface);
        for (unsigned u = 0; u < 2; ++u)
            for (unsigned i = 1; i < 3; ++i) {
                Point3 expected{};
                for (unsigned k = 0; k < 3; ++k)
                    expected[k] = i / 3. * controls[u][i - 1][k] + (1 - i / 3.) * controls[u][i][k];
                check(near(s.poles()[i * 2 + u], expected), "quadratic surface Bernstein controls");
                if (!weights.empty())
                    check(near(s.weights()[i * 2 + u],
                               i / 3. * weights[u][i - 1] + (1 - i / 3.) * weights[u][i]),
                          "quadratic surface signed Bernstein weights");
            }
        for (unsigned u = 0; u <= 8; ++u)
            for (unsigned v = 0; v <= 12; ++v) {
                const auto a = source.homogeneous_at(u / 8., v / 12.);
                const auto b = s.homogeneous_at(u / 8., v / 12.);
                for (unsigned k = 0; k < 4; ++k)
                    check(near(a[k], b[k]),
                          "elevation preserves full homogeneous surface evaluation");
            }
    }
    const auto broken = make(2,
                             {{{0, 0, 0}, {2, 0, 0}, {5, 0, 0}, {11, 0, 0}},
                              {{0, 1, 0}, {4, 1, 0}, {9, 1, 0}, {19, 1, 0}}},
                             {0, 0, .5, .5, 1, 1});
    const auto discontinuous = BsplineSurface::from_bgfb(elevate(broken, 2).surface);
    check(discontinuous.v().knots() == std::vector<double>({0, 0, 0, .5, .5, .5, 1, 1, 1}),
          "internal full multiplicity retained across all columns");
    check(discontinuous.poles() == std::vector<Point3>{{0, 0, 0},
                                                       {0, 1, 0},
                                                       {1, 0, 0},
                                                       {2, 1, 0},
                                                       {2, 0, 0},
                                                       {4, 1, 0},
                                                       {5, 0, 0},
                                                       {9, 1, 0},
                                                       {8, 0, 0},
                                                       {14, 1, 0},
                                                       {11, 0, 0},
                                                       {19, 1, 0}},
          "discontinuous columns retain independent left and right controls");
    // V matrices use raw source knots as native derivative fractions. For a
    // nonunit domain the native result need not preserve the source shape.
    const auto shifted = make(2, lines, {2, 2, 7, 7});
    const auto raw = BsplineSurface::from_bgfb(elevate(shifted, 2).surface);
    check(raw.v().knots() == std::vector<double>({2, 2, 2, 7, 7, 7}),
          "surface elevation does not normalize a nonunit knot domain");
    for (unsigned u = 0; u < 3; ++u) {
        Point3 expected{};
        for (unsigned k = 0; k < 3; ++k)
            expected[k] = lines[u][1][k] - .5 * (lines[u][1][k] - lines[u][0][k]);
        check(near(raw.poles()[3 + u], expected),
              "raw knot derivative callback uses native knot derivatives");
    }
    const auto multispan =
        make(2, {{{0, 0, 0}, {2, 0, 0}, {10, 0, 0}}, {{0, 1, 0}, {2, 1, 0}, {10, 1, 0}}},
             {2, 2, 4, 7, 7});
    const auto multi = BsplineSurface::from_bgfb(elevate(multispan, 2).surface);
    check(near(multi.poles()[2][0], 22. / 3.) && multi.poles()[4][0] == 2 &&
              multi.poles()[6][0] == 6,
          "nonunit multispan native fraction callback is not replaced by normalized shape "
          "preservation");
    const auto rounding = make(2, {{{1, 0, 0}, {2, 0, 0}}, {{3, 1, 0}, {4, 1, 0}}}, {0, 0, 1, 1},
                               {{49, 49}, {49, 49}});
    const auto rounded = BsplineSurface::from_bgfb(elevate(rounding, 3).surface);
    for (unsigned u = 0; u < 2; ++u) {
        double last = u == 0 ? 2. : 4.;
        for (unsigned i = 0; i < 4; ++i)
            last = (last * (1. / 49.)) * 49.;
        check(rounded.poles()[u] == (u == 0 ? Point3{1, 0, 0} : Point3{3, 1, 0}),
              "first column control saved before derivative weight round trips");
        check(rounded.poles()[6 + u][0] == last,
              "plan tolerance round trip does not leak into column derivative controls");
    }
    const std::vector<Point3> long_column{{0, 0, 0}, {1e6, 0, 0}, {2e6, 0, 0}, {3e6, 0, 0}};
    const std::vector<Point3> short_column{{0, 1, 0}, {.1, 1, 0}, {.2, 1, 0}, {.3, 1, 0}};
    const std::vector<double> dense{0, 0, .5, .5 + 1e-11, 1, 1};
    const auto common = elevate(make(2, {long_column, long_column}, dense), 2);
    check(common.report["knot_tolerance"] == 1e-14 && common.surface["numPolesV"] == 7,
          "first long column retains both nearby internal knots");
    rejects([&] { elevate(make(2, {long_column, short_column}, dense), 2); },
            "later column derivative tolerance can reject the shared first-column plan");
    const auto swapped = elevate(make(2, {short_column, long_column}, dense), 2);
    check(swapped.surface["numPolesV"] == 6 &&
              swapped.surface["knotsV"] == Json({0, 0, 0, .5, .5, .5, 1, 1, 1}),
          "swapping first column changes shared native knot plan without per-column re-planning");
    auto periodic_u = line;
    periodic_u["closedU"] = true;
    const auto su = BsplineSurface::from_bgfb(periodic_u);
    const auto eu = BsplineSurface::from_bgfb(elevate(periodic_u, 3).surface);
    check(eu.u().closed() && eu.u().knots() == su.u().knots(),
          "periodic transverse direction is retained");
    auto periodic_v = line;
    periodic_v["closedV"] = true;
    periodic_v["knotsV"] = nullptr;
    check(elevate(periodic_v, 1).surface["closedV"] == true, "same-degree periodic V copy");
    rejects([&] { elevate(periodic_v, 2); }, "periodic V elevation is explicitly unimplemented");
    auto zero = line;
    zero["weights"] = {0, 1, 1, 1, 1, 1};
    check(elevate(zero, 1).surface["weights"] == zero["weights"], "same-degree zero weight copy");
    rejects([&] { elevate(zero, 2); }, "actual elevation rejects zero control weight");
    auto exterior = line;
    exterior["knotsV"] = {-1, 0, 1, 2};
    check(elevate(exterior, 1).surface["knotsV"] == exterior["knotsV"],
          "same-degree exterior knot copy");
    rejects([&] { elevate(exterior, 2); }, "unwritten native knot storage is not invented");
    rejects([&] { elevate(line, 0); }, "surface degree reduction rejected");
    rejects([&] { elevate(line, 26); }, "native maximum surface degree enforced");
    for (bool work_limit : {false, true})
        rejects(
            [&] {
                TubeBudget b;
                if (work_limit)
                    b.max_work = 10;
                else
                    b.max_control_points = 8; // Input has 6, output would need 9.
                elevate_tube_surface_v(BsplineSurface::from_bgfb(line), 2, b);
            },
            "surface budget limits cumulative work and total output controls");
    const auto before = line.dump();
    auto f = std::async(std::launch::async, [&] { return elevate(line, 5).surface; });
    auto g = std::async(std::launch::async, [&] { return elevate(line, 5).surface; });
    check(f.get() == g.get() && line.dump() == before,
          "surface elevation is independent and source immutable");
    return count;
}
