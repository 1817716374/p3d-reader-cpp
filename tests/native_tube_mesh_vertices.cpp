#include "native_tube_mesh_vertices.hpp"
#include "native_surface_sample.hpp"
#include "native_pcurve_points.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
bool near(double a, double b, double tolerance = 3e-11) {
    return std::abs(a - b) <= tolerance * std::max({1., std::abs(a), std::abs(b)});
}
bool near(Point3 a, Point3 b, double tolerance = 3e-11) {
    for (unsigned i = 0; i < 3; ++i)
        if (!near(a[i], b[i], tolerance))
            return false;
    return true;
}
Json surface(bool rational = false, unsigned nu = 2, unsigned nv = 2, unsigned ou = 2,
             unsigned ov = 2, bool cu = false, bool cv = false) {
    Json p = Json::array(), w = Json::array();
    for (unsigned j = 0; j < nv; ++j)
        for (unsigned i = 0; i < nu; ++i) {
            const double weight = rational ? 1. + i : 1.;
            for (double x : {double(i), double(j), double(i * j)})
                p.push_back(x * weight);
            w.push_back(weight);
        }
    return {{"_type", "BsplineSurface"},
            {"numPolesU", nu},
            {"numPolesV", nv},
            {"orderU", ou},
            {"orderV", ov},
            {"closedU", cu},
            {"closedV", cv},
            {"poles", p},
            {"weights", rational ? w : Json()},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"numRulesU", 0},
            {"numRulesV", 0},
            {"holeOrigin", 0},
            {"boundaries", nullptr}};
}
TubeMeshVertex vertex(const BsplineSurface &s, double u, double v,
                      std::array<double, 2> interval = {0, 1}, std::size_t pi = 0,
                      std::size_t pc = 1) {
    TubeBudget budget;
    return evaluate_tube_mesh_vertex(s, u, v, interval, pi, pc, budget);
}
} // namespace
unsigned native_tube_mesh_vertices_tests() {
    unsigned count = 0;
    auto check = [&](bool ok, const char *why) {
        ++count;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool thrown = false;
        try {
            fn();
        } catch (const std::exception &) {
            thrown = true;
        }
        check(thrown, "native surface/vertex invalid inputs and budgets fail explicitly");
    };
    for (bool rational : {false, true}) {
        const auto s = BsplineSurface::from_bgfb(surface(rational));
        for (double u : {0., .17, .5, .91, 1.})
            for (double v : {0., .23, .7, 1.}) {
                const auto a = detail::native_surface_sample(s, u, v);
                const double x = rational ? 2 * u / (1 + u) : u,
                             dx = rational ? 2 / ((1 + u) * (1 + u)) : 1;
                check(near(a.point, {x, v, x * v}) && near(a.du, {dx, 0, dx * v}) &&
                          near(a.dv, {0, 1, x}),
                      "analytic polynomial and rational tensor-product point/raw derivatives");
                check(near(a.weight, rational ? 1 + u : 1) && a.weight_du == (rational ? 1 : 0) &&
                          a.weight_dv == 0,
                      "homogeneous weight and raw weight derivatives are retained");
                const auto b = vertex(s, u, v, {2, 5}, 2, 7);
                const double norm = std::sqrt(v * v + x * x + 1);
                check(near(b.normal, {-v / norm, -x / norm, 1 / norm}) && !b.normal_fallback,
                      "native orientation is dU cross dV and reciprocal-normalized");
                check(
                    near(b.parameter[0], 2 + 3 * u) && b.parameter[1] == 2. / 7 + v / 7 &&
                        near(b.point, a.point),
                    "source U interval and two separately divided V terms form vertex parameters");
            }
        auto domain = surface(rational);
        domain["knotsU"] = {-2, -2, 3, 3};
        domain["knotsV"] = {.2, .2, 2.2, 2.2};
        const auto d = BsplineSurface::from_bgfb(domain);
        const auto a = detail::native_surface_sample(d, .4, .7);
        check(near(a.point, s.point_at(.24, .28)),
              "surface mapping retains native upper one instead of full domain");
        const double dx = rational ? 2 / std::pow(1.24, 2) : 1;
        const double x = rational ? .48 / 1.24 : .24;
        check(near(a.du, {dx / 5, 0, dx * .28 / 5}) && near(a.dv, {0, .5, x * .5}),
              "native derivatives omit the fraction mapping scale factors");
        check(near(a.point, detail::pcurve_surface_point(d, .4, .7)),
              "existing point-only caller agrees");
        const auto clamped = vertex(d, 2, 2);
        check(near(clamped.point, s.point_at(.6, .4)) && clamped.parameter == Point2{2, 2},
              "source UV attributes do not inherit evaluation clamping");
    }
    // Compare unrelated basis implementation and central differences across
    // higher orders, rational weights and independent periodic directions.
    for (bool rational : {false, true})
        for (bool cu : {false, true})
            for (bool cv : {false, true}) {
                const auto s = BsplineSurface::from_bgfb(surface(rational, 5, 6, 3, 4, cu, cv));
                for (double u : {.137, .413, .823})
                    for (double v : {.197, .467, .797}) {
                        const auto a = detail::native_surface_sample(s, u, v);
                        constexpr double h = 1e-6;
                        auto plus = s.point_at(u + h, v), minus = s.point_at(u - h, v);
                        Point3 du{}, dv{};
                        for (unsigned k = 0; k < 3; ++k)
                            du[k] = (plus[k] - minus[k]) / (2 * h);
                        plus = s.point_at(u, v + h);
                        minus = s.point_at(u, v - h);
                        for (unsigned k = 0; k < 3; ++k)
                            dv[k] = (plus[k] - minus[k]) / (2 * h);
                        check(near(a.point, s.point_at(u, v)) && near(a.du, du, 2e-7) &&
                                  near(a.dv, dv, 2e-7),
                              "independent tensor evaluator and finite differences agree on "
                              "regular/periodic domains");
                    }
                if (cu)
                    check(near(detail::native_surface_sample(s, -1, .4).point,
                               detail::native_surface_sample(s, 0, .4).point),
                          "closed negative fractions clamp rather than wrap");
                if (cv)
                    check(near(detail::native_surface_sample(s, .4, 2).point,
                               detail::native_surface_sample(s, .4, 1).point),
                          "closed large fractions clamp rather than wrap");
            }
    auto scaled = surface(true);
    for (auto &p : scaled["poles"])
        p = p.get<double>() * 1e-200;
    for (auto &w : scaled["weights"])
        w = w.get<double>() * 1e-200;
    auto tiny = BsplineSurface::from_bgfb(scaled);
    auto expected = detail::native_surface_sample(BsplineSurface::from_bgfb(surface(true)), .3, .4);
    auto got = detail::native_surface_sample(tiny, .3, .4);
    check(near(got.point, expected.point) && near(got.du, expected.du) && near(got.dv, expected.dv),
          "tiny evaluated weights are not discarded or replaced");
    for (auto &p : scaled["poles"])
        p = -p.get<double>();
    for (auto &w : scaled["weights"])
        w = -w.get<double>();
    check(near(detail::native_surface_sample(BsplineSurface::from_bgfb(scaled), .3, .4).du,
               expected.du),
          "negative homogeneous scale preserves point and derivative");
    auto zero = surface(true);
    zero["weights"] = {1, -1, 1, -1};
    rejects([&] { detail::native_surface_sample(BsplineSurface::from_bgfb(zero), .5, .5); });
    auto flat = surface();
    for (auto &p : flat["poles"])
        p = 0;
    auto fallback = vertex(BsplineSurface::from_bgfb(flat), .3, .7);
    check(fallback.normal_fallback && fallback.normal == Point3{1, 0, 0},
          "zero cross product uses native positive X fallback");
    auto small = surface();
    for (auto &p : small["poles"])
        p = p.get<double>() * 1e-100;
    check(
        vertex(BsplineSurface::from_bgfb(small), .3, .7).normal_fallback,
        "native squared magnitude underflow retains X fallback rather than robust renormalization");
    auto reversed = surface();
    reversed["poles"] = {1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0};
    check(vertex(BsplineSurface::from_bgfb(reversed), .5, .5).normal[2] < 0,
          "reversed U preserves normal sign");
    SweptBodyPatchGroup group;
    group.patches = {{surface(), {}}, {surface(), {}}};
    TubeBudget budget;
    auto grid = prepare_tube_mesh_grid(group, false, .04, .15, 100000, budget);
    check(grid.success && grid.patches.size() == 2,
          "actual preparation feeds regular vertex evaluator");
    TubeBudget vb;
    auto nodes = evaluate_tube_mesh_regular_vertices(grid.patches[1], grid.section, 0, 1, 2, vb);
    check(nodes.u_count == grid.section.interval_samples[0].size() &&
              nodes.v_count == grid.patches[1].path.parameters.size() &&
              nodes.vertices.size() == nodes.u_count * nodes.v_count,
          "regular lattice dimensions follow original U/V samples");
    for (std::size_t j = 0; j < nodes.v_count; ++j)
        for (std::size_t i = 0; i < nodes.u_count; ++i) {
            const double u = grid.section.interval_samples[0][i],
                         v = grid.patches[1].path.parameters[j];
            const auto &n = nodes.vertices[j * nodes.u_count + i];
            check(near(n.point, {u, v, u * v}) && n.parameter == Point2{u, .5 + v / 2},
                  "V-major lattice evaluation preserves original sample order");
        }
    auto duplicate = grid;
    duplicate.section.interval_samples[0].insert(duplicate.section.interval_samples[0].begin(), 0);
    TubeBudget db;
    auto repeated =
        evaluate_tube_mesh_regular_vertices(duplicate.patches[0], duplicate.section, 0, 0, 2, db);
    check(repeated.u_count == nodes.u_count + 1 &&
              repeated.vertices[0].point == repeated.vertices[1].point,
          "equal source parameters remain independent original records");
    auto trimmed = grid.patches[0];
    trimmed.preparation.boundaries.lower = grid.section.reference;
    rejects([&] {
        TubeBudget b;
        evaluate_tube_mesh_regular_vertices(trimmed, grid.section, 0, 0, 2, b);
    });
    rejects([&] {
        TubeBudget b;
        b.max_control_points = 3;
        evaluate_tube_mesh_regular_vertices(grid.patches[0], grid.section, 0, 0, 2, b);
    });
    const auto s = BsplineSurface::from_bgfb(surface());
    rejects([&] {
        TubeBudget b;
        b.max_work = 0;
        evaluate_tube_mesh_vertex(s, .2, .3, {0, 1}, 0, 1, b);
    });
    rejects([&] { vertex(s, .2, .3, {0, 1}, 1, 1); });
    rejects([&] { vertex(s, .2, .3, {0, 1}, 0, 0); });
    rejects([&] { vertex(s, std::numeric_limits<double>::infinity(), .3); });
    auto f = std::async(std::launch::async, [&] { return vertex(s, .2, .3).point; });
    check(f.get() == vertex(s, .2, .3).point &&
              s.poles() == BsplineSurface::from_bgfb(surface()).poles(),
          "independent parallel evaluations preserve immutable source arrays");
    return count;
}
