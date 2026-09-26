#include "native_surface_iso.hpp"
#include "native_pcurve_points.hpp"
#include "native_bspline_area.hpp"
#include <future>
using namespace p3d;
namespace {
Json grid(unsigned nu, unsigned nv, unsigned ou, unsigned ov, bool rational = true, bool cu = false,
          bool cv = false, Json ku = nullptr, Json kv = nullptr) {
    Json poles = Json::array(), weights = Json::array();
    for (unsigned j = 0; j < nv; ++j)
        for (unsigned i = 0; i < nu; ++i) {
            const double w = rational ? 1 + .2 * i + .3 * j + .1 * i * j : 1;
            poles.push_back(w * (i + .15 * j * j));
            poles.push_back(w * (j + .21 * i * i));
            poles.push_back(w * (.3 * i * j + std::sin(double(i + j))));
            if (rational)
                weights.push_back(w);
        }
    return {{"_type", "BsplineSurface"},
            {"numPolesU", nu},
            {"numPolesV", nv},
            {"orderU", ou},
            {"orderV", ov},
            {"closedU", cu},
            {"closedV", cv},
            {"knotsU", ku},
            {"knotsV", kv},
            {"poles", poles},
            {"weights", rational ? weights : Json()},
            {"boundaries", nullptr},
            {"numRulesU", 0},
            {"numRulesV", 0},
            {"holeOrigin", 0}};
}
detail::NativeIsoCurve iso(const BsplineSurface &s, double v) {
    std::size_t used = 0;
    return detail::native_iso_v_curve(s, v, {used, 100000000});
}
bool near(double a, double b) {
    return std::abs(a - b) <= 3e-11 * std::max({1., std::abs(a), std::abs(b)});
}
} // namespace
unsigned native_surface_iso_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool failed = false;
        try {
            fn();
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, "isocurve rejects invalid arithmetic or resource bounds");
    };
    auto verify = [&](const Json &j) {
        const auto s = BsplineSurface::from_bgfb(j);
        const auto original = s.poles();
        for (double v : {-1., 0., .13, .5, .87, 1., 2.}) {
            const auto c = iso(s, v);
            check(c.curve.order() == s.u().order() && c.curve.closed() == s.u().closed() &&
                      c.curve.knots() == s.u().knots() &&
                      c.curve.poles().size() == s.u().pole_count() &&
                      c.curve.rational() == s.rational(),
                  "isocurve preserves varying-axis representation");
            for (double u : {0., .04, .31, .62, .97, 1.}) {
                const auto a = c.curve.point_at(u), b = s.point_at(u, std::clamp(v, 0., 1.));
                for (unsigned k = 0; k < 3; ++k)
                    check(near(a[k], b[k]),
                          "isocurve agrees with independent tensor-product surface basis");
            }
        }
        check(s.poles() == original, "isocurve leaves source controls immutable");
    };
    verify(grid(4, 5, 3, 3, false));
    verify(grid(4, 5, 3, 3, true));
    verify(
        grid(4, 5, 3, 3, true, false, false, {-2, -1, 0, .4, 1, 2, 3}, {2, 2, 2, 3, 5, 9, 9, 9}));
    verify(grid(4, 5, 3, 3, true, true, true));
    verify(grid(4, 5, 3, 3, true, false, false, nullptr, {2, 3, 4, 4.3, 4.6, 5, 6, 7}));
    verify(grid(4, 8, 3, 3, true, false, true, nullptr,
                {-.25, 0, 0, 0, .25, .25, .5, .5, .75, .75, 1, 1, 1.25}));
    verify(grid(4, 5, 3, 3, true, false, true, {-2, -1, 0, .4, 1, 2, 3}));
    auto negative = grid(4, 5, 3, 3, true, true, false);
    for (auto &p : negative["poles"])
        p = -p.get<double>();
    for (auto &w : negative["weights"])
        w = -w.get<double>();
    verify(negative);
    for (unsigned order : {2u, 4u, 8u, 16u, 26u})
        verify(grid(3, order + 1, 2, order));

    auto roundtrip = grid(2, 2, 2, 2);
    roundtrip["poles"] = {.1, .2, 0, 22, 0, 0, 0, 0, 11, 22, 0, 11};
    roundtrip["weights"] = {11, 11, 11, 11};
    const auto rs = BsplineSurface::from_bgfb(roundtrip);
    const auto rc = iso(rs, 0);
    check(rc.curve.poles()[0][0] == .10000000000000002 &&
              rc.curve.poles()[0][1] == .20000000000000004 && rs.poles()[0][0] == .1,
          "endpoint performs native divide/multiply rather than copying first row");
    auto cancel = grid(2, 2, 2, 2);
    cancel["poles"] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    cancel["weights"] = {1, 2, -1, -2};
    const auto zs = BsplineSurface::from_bgfb(cancel);
    const auto z = iso(zs, .5);
    check(z.zero_weight_fallbacks == 2 && z.curve.weights() == std::vector<double>{0, 0} &&
              z.curve.poles() == std::vector<Point3>{{0, 0, 0}, {0, 0, 0}},
          "raw zero weight survives point fallback and zeros reweighted XYZ");
    const auto cp = BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                             {"order", 2},
                                             {"poles", {1, 2, 3, 7, 8, 9}},
                                             {"weights", {1, -1}},
                                             {"knots", nullptr},
                                             {"closed", false}});
    const auto evaluated = detail::pcurve_point(cp, .5);
    check(evaluated.weight == 0 && evaluated.zero_weight_fallback &&
              evaluated.point == Point3{4, 5, 6},
          "curve point exposes raw W separately from Cartesian fallback");

    // Closed U circle extruded along V: the V0 curve is the input required by
    // the swept-surface area orientation rule, with periodic shift preserved.
    const double w = std::sqrt(.5), pi = std::acos(-1.);
    std::vector<Point3> xy = {{1, 0, 0},  {w, w, 0},   {0, 1, 0},  {-w, w, 0},
                              {-1, 0, 0}, {-w, -w, 0}, {0, -1, 0}, {w, -w, 0}};
    std::vector<double> weights = {1, w, 1, w, 1, w, 1, w};
    auto cylinder = grid(8, 2, 3, 2, true, true, false,
                         {-.25, 0, 0, 0, .25, .25, .5, .5, .75, .75, 1, 1, 1.25});
    cylinder["poles"] = Json::array();
    cylinder["weights"] = Json::array();
    for (unsigned j = 0; j < 2; ++j)
        for (unsigned i = 0; i < 8; ++i) {
            cylinder["poles"].push_back(xy[i][0]);
            cylinder["poles"].push_back(xy[i][1]);
            cylinder["poles"].push_back(3 * j * weights[i]);
            cylinder["weights"].push_back(weights[i]);
        }
    verify(cylinder);
    for (double v : {0., .37, 1.}) {
        const auto c = iso(BsplineSurface::from_bgfb(cylinder), v);
        std::size_t used = 0;
        const auto a = curve_detail::native_bspline_area(c.curve, {used, 10000000});
        check(near(a.area, pi) && near(a.centroid[0], 0) && near(a.centroid[1], 0) &&
                  near(a.centroid[2], 3 * v) && near(a.normal[2], 1),
              "periodic isocurve feeds complete native area and normal pipeline");
    }
    auto trimmed = cylinder;
    trimmed["boundaries"] = {{"_type", "CurveVector"}, {"type", 4}, {"curves", Json::array()}};
    check(iso(BsplineSurface::from_bgfb(trimmed), 0).curve.poles() ==
              iso(BsplineSurface::from_bgfb(cylinder), 0).curve.poles(),
          "complete isocurve is not clipped by trim boundaries");
    std::size_t used = 0;
    detail::native_iso_v_curve(rs, .4, {used, 10000000});
    const auto required = used;
    used = 0;
    detail::native_iso_v_curve(rs, .4, {used, required});
    check(used == required, "exact isocurve shared work budget");
    check(iso(BsplineSurface::from_bgfb(grid(27, 2, 27, 2)), 0).curve.order() == 27,
          "fixed-axis order limit does not unnecessarily restrict varying-axis order");
    rejects([&] {
        std::size_t u = 0;
        detail::native_iso_v_curve(rs, .4, {u, required - 1});
    });
    rejects([&] {
        std::size_t u = 0;
        detail::native_iso_v_curve(rs, .4, {u, required}, 1);
    });
    rejects([&] {
        std::size_t u = 2;
        detail::native_iso_v_curve(rs, .4, {u, 1});
    });
    rejects([&] { iso(rs, std::numeric_limits<double>::infinity()); });
    rejects([&] { iso(BsplineSurface::from_bgfb(grid(2, 27, 2, 27)), 0); });
    auto f = std::async(std::launch::async, [&] { return iso(rs, .43); });
    const auto parallel = iso(rs, .43), other = f.get();
    check(parallel.curve.poles() == other.curve.poles() &&
              parallel.curve.weights() == other.curve.weights(),
          "independent isocurve calls share no mutable state");
    return n;
}
