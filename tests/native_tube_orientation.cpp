#include "native_tube_orientation.hpp"
#include "native_surface_iso.hpp"
#include "native_bspline_area.hpp"
#include "native_knot_normalize.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json surface(unsigned order = 2, bool line = false) {
    const std::vector<Point3> base =
        line ? std::vector<Point3>{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}}
             : std::vector<Point3>{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 0}};
    Json p = Json::array();
    const auto n = line && order == 2 ? 2 : base.size();
    for (unsigned j = 0; j < 2; ++j)
        for (std::size_t i = 0; i < n; ++i) {
            p.push_back(base[i][0]);
            p.push_back(base[i][1]);
            p.push_back(double(j));
        }
    return {{"_type", "BsplineSurface"},
            {"numPolesU", n},
            {"numPolesV", 2},
            {"orderU", order},
            {"orderV", 2},
            {"closedU", false},
            {"closedV", false},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"poles", p},
            {"weights", nullptr},
            {"boundaries", nullptr},
            {"numRulesU", 7},
            {"numRulesV", 9},
            {"holeOrigin", 1}};
}
bool near(double a, double b) {
    return std::abs(a - b) < 2e-10 * std::max({1., std::abs(a), std::abs(b)});
}
Point3 normal(const Json &j) {
    std::size_t used = 0;
    auto c = detail::native_iso_v_curve(BsplineSurface::from_bgfb(j), 0, {used, 10000000});
    return curve_detail::native_bspline_area(c.curve, {used, 10000000}).normal;
}
} // namespace
unsigned native_tube_orientation_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "native orientation rejects unsupported input or budget");
    };
    const auto square = surface();
    for (bool reverse_u : {false, true}) {
        TubeBudget b;
        const auto a = BsplineSurface::from_bgfb(square);
        const auto reversed = reverse_tube_surface(a, reverse_u, b);
        const auto r = BsplineSurface::from_bgfb(reversed.surface);
        check(reversed.report["knots_normalized"] == true && r.num_rules_u() == 7 &&
                  r.num_rules_v() == 9 && r.hole_origin() == 1,
              "reversal preserves source rule counts and hole origin");
        for (double u : {0., .03, .27, .61, .93, 1.})
            for (double v : {0., .18, .6, 1.}) {
                const auto p = a.point_at(reverse_u ? 1 - u : u, reverse_u ? v : 1 - v),
                           q = r.point_at(u, v);
                for (unsigned k = 0; k < 3; ++k)
                    check(near(p[k], q[k]), "open clamped reversal parameter identity");
            }
        auto twice = reverse_tube_surface(r, reverse_u, b);
        check(BsplineSurface::from_bgfb(twice.surface).poles() == a.poles(),
              "double reversal restores controls");
    }
    auto weighted = square;
    weighted["weights"] = Json::array();
    for (unsigned i = 0; i < 10; ++i) {
        const double w = i % 3 == 0 ? 0 : (i % 2 ? -2 : 3);
        weighted["weights"].push_back(w);
    }
    TubeBudget wb;
    const auto wr = reverse_tube_surface(BsplineSurface::from_bgfb(weighted), true, wb);
    for (unsigned j = 0; j < 2; ++j)
        for (unsigned i = 0; i < 5; ++i) {
            check(wr.surface["weights"][j * 5 + i] == weighted["weights"][j * 5 + 4 - i],
                  "reversal copies signed and zero weights without deweighting");
            for (unsigned k = 0; k < 3; ++k)
                check(wr.surface["poles"][3 * (j * 5 + i) + k] ==
                          weighted["poles"][3 * (j * 5 + 4 - i) + k],
                      "reversal copies homogeneous coordinates exactly");
        }
    auto periodic = square;
    periodic["closedU"] = true;
    TubeBudget pb;
    const auto ps = BsplineSurface::from_bgfb(periodic);
    const auto pr = reverse_tube_surface(ps, true, pb);
    const auto pk = BsplineSurface::from_bgfb(pr.surface).u().knots();
    check(pr.surface["closedU"] == true && pr.surface["knotsV"] == ps.v().knots(),
          "periodic reversal retains closed flag and untouched direction knots");
    for (unsigned i = 0; i < ps.u().order(); ++i) {
        check(near(pk[i], pk[ps.u().pole_count() + i] - 1),
              "closed lower extension is reconstructed");
        check(
            near(pk[ps.u().pole_count() + ps.u().order() - 1 + i], pk[ps.u().order() - 1 + i] + 1),
            "closed upper extension is reconstructed");
    }
    periodic["orderU"] = 3;
    TubeBudget closed_orientation_budget;
    const auto co =
        orient_tube_surfaces({periodic, periodic}, {0, 0, 1}, closed_orientation_budget);
    check(co.report["completed_all_rings"] == true && normal(co.surfaces[0])[2] == 1 &&
              normal(co.surfaces[1])[2] == -1,
          "closed quadratic rings retain opposite physical normals after periodic reversal");
    std::vector<double> k{-2, -1, 0, .1, .4, .8, 1, 1.1, 1.4};
    check(curve_detail::normalize_native_knots(k, 4, 3, true) && near(k[0], -.6) &&
              near(k[1], -.2) && k[2] == 0 && k[6] == 1,
          "closed normalization replaces arbitrary outer support extensions");
    for (double range : {std::nextafter(1e-10, 0.), 1e-10, std::nextafter(1e-10, 1.)}) {
        std::vector<double> knots{range, range, 0, 0};
        const auto old = knots;
        const bool ok = curve_detail::normalize_native_knots(knots, 2, 2, false);
        check(ok == (range >= 1e-10), "native knot threshold uses strict comparison");
        check(ok ? knots == std::vector<double>{0, 0, 1, 1} : knots == old,
              "failed normalization leaves reversed vector untouched");
    }
    auto tiny = surface(2, true);
    tiny["knotsU"] = {0, 0, 1e-11, 1e-11};
    TubeBudget tinyb;
    const auto tr = reverse_tube_surface(BsplineSurface::from_bgfb(tiny), true, tinyb);
    check(tr.report["native_return"] == 0 && tr.report["knots_normalized"] == false &&
              tr.surface["knotsU"] == Json({1e-11, 1e-11, 0, 0}),
          "native reversal ignores tiny-domain normalization failure and reports it explicitly");
    rejects([&] { BsplineSurface::from_bgfb(tr.surface); });
    auto trim = square;
    trim["boundaries"] = {{"_type", "CurveVector"}, {"type", 4}, {"curves", Json::array()}};
    rejects([&] {
        TubeBudget b;
        reverse_tube_surface(BsplineSurface::from_bgfb(trim), true, b);
    });

    for (Point3 tangent : {Point3{0, 0, 7}, Point3{0, 0, -7}, Point3{0, 0, 0}}) {
        TubeBudget b;
        const auto oriented = orient_tube_surfaces({square, square}, tangent, b);
        const bool positive = tangent[2] > 0;
        check(oriented.report["native_result"] == true &&
                  oriented.report["completed_all_rings"] == true &&
                  oriented.report["visited_rings"] == 2,
              "all nondegenerate rings visited in source order");
        check(oriented.report["rings"][0]["reversed_u"] == !positive &&
                  oriented.report["rings"][1]["reversed_u"] == positive,
              "outer and inner use opposite original-index orientation rules");
        check(normal(oriented.surfaces[0])[2] == (positive ? 1 : -1) &&
                  normal(oriented.surfaces[1])[2] == (positive ? -1 : 1),
              "actual resulting surface normals agree with decisions");
        if (tangent == Point3{})
            check(oriented.report["unit_start_tangent"] == Json({1, 0, 0}),
                  "zero tangent uses native X-axis normalization fallback");
    }
    for (double z : {std::nextafter(1e-14, 0.), 1e-14, std::nextafter(1e-14, 1.)}) {
        TubeBudget b;
        const auto o = orient_tube_surfaces({square, square}, {1, 0, z}, b);
        check(o.report["rings"][0]["reversed_u"] == (z <= 1e-14) &&
                  o.report["rings"][1]["reversed_u"] == (z > 1e-14),
              "strict native dot threshold");
    }
    for (unsigned order : {2u, 4u}) {
        TubeBudget b;
        const auto line = surface(order, true);
        const auto o = orient_tube_surfaces({square, line, square}, {0, 0, -1}, b);
        check(o.report["native_result"] == (order == 2) &&
                  o.report["completed_all_rings"] == false && o.report["visited_rings"] == 2 &&
                  o.report["rings"].size() == 2,
              "area failure exits entire loop with order-dependent native result");
        check(o.surfaces[0] != square && o.surfaces[1] == line && o.surfaces[2] == square,
              "early return keeps prior reversal and leaves remaining surfaces untouched");
    }
    TubeBudget nb;
    const auto null = orient_tube_surfaces({square, nullptr, square}, {0, 0, -1}, nb);
    check(null.report["native_result"] == false && null.report["visited_rings"] == 2 &&
              null.surfaces[2] == square,
          "null native surface returns false after prior mutations");
    TubeBudget emptyb;
    const auto empty = orient_tube_surfaces({}, Point3{}, emptyb);
    check(empty.report["native_result"] == true && empty.report["completed_all_rings"] == true,
          "empty native surface list succeeds");
    TubeBudget budget;
    orient_tube_surfaces({square, square}, {0, 0, 1}, budget);
    const auto needed = budget.work;
    TubeBudget exact;
    exact.max_work = needed;
    orient_tube_surfaces({square, square}, {0, 0, 1}, exact);
    check(exact.work == needed, "exact shared orientation budget");
    rejects([&] {
        TubeBudget b;
        b.max_work = needed - 1;
        orient_tube_surfaces({square, square}, {0, 0, 1}, b);
    });
    rejects([&] {
        TubeBudget b;
        b.max_control_points = 19;
        orient_tube_surfaces({square, square}, {0, 0, 1}, b);
    });
    auto f = std::async(std::launch::async, [&] {
        TubeBudget b;
        return orient_tube_surfaces({square, square}, {0, 0, 1}, b);
    });
    TubeBudget b;
    const auto parallel = orient_tube_surfaces({square, square}, {0, 0, 1}, b);
    check(parallel.surfaces == f.get().surfaces && square == surface(),
          "orientation is deterministic across threads and leaves source JSON unchanged");
    return n;
}
