#include "native_tube_facet_extension.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json surface(bool second, unsigned order, bool weighted = false) {
    Json xyz = Json::array(), weights = Json::array();
    for (unsigned row = 0; row < order; ++row)
        for (unsigned u = 0; u < 2; ++u) {
            const double v = double(row) / (order - 1), w = weighted ? 2. * (row + 1) : 1.;
            const Point3 p =
                second ? Point3{2, 2 * v - 1, double(u) + .01} : Point3{v, 0, double(u)};
            for (double x : p)
                xyz.push_back(x * w);
            if (weighted)
                weights.push_back(w);
        }
    return {{"_type", "BsplineSurface"},
            {"orderU", 2},
            {"orderV", order},
            {"numPolesU", 2},
            {"numPolesV", order},
            {"closedU", false},
            {"closedV", false},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"poles", xyz},
            {"weights", weighted ? weights : Json()},
            {"numRulesU", 5},
            {"numRulesV", 7},
            {"holeOrigin", 1},
            {"boundaries", nullptr}};
}
TubeFacetSeamReferences seam() {
    TubeFacetSeamReferences s;
    s.incoming = std::make_shared<TubeFacetSeamStorage<Point3>>(
        TubeFacetSeamStorage<Point3>{{1, 0, 0}, true});
    s.outgoing = std::make_shared<TubeFacetSeamStorage<Point3>>(
        TubeFacetSeamStorage<Point3>{{0, 1, 0}, true});
    s.plane = std::make_shared<TubeFacetSeamStorage<std::array<Point3, 2>>>(
        TubeFacetSeamStorage<std::array<Point3, 2>>{{Point3{100, 0, 0}, Point3{1, 1, 0}}, true});
    return s;
}
bool near(double a, double b) {
    return std::abs(a - b) <= 2e-10 * std::max({1., std::abs(a), std::abs(b)});
}
} // namespace
unsigned native_tube_facet_extension_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
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
    auto apply = [&](Json &a, Json &b) {
        auto refs = seam();
        TubeBudget budget;
        budget.max_work = 100000000;
        auto result = apply_tube_facet_seam(a, b, refs, budget);
        check(result.status == TubeFacetSeamStatus::complete && refs.classifier == 1,
              "high-order seam reaches native classifier one success");
        return result;
    };
    for (bool weighted : {false, true})
        for (unsigned order : {3u, 4u, 8u, 26u}) {
            auto a = surface(false, order, weighted), b = surface(true, order, weighted);
            const auto a0 = BsplineSurface::from_bgfb(a), b0 = BsplineSurface::from_bgfb(b);
            auto result = apply(a, b);
            const auto x = BsplineSurface::from_bgfb(a), y = BsplineSurface::from_bgfb(b);
            check(x.v().pole_count() == 2 * order - 1 && y.v().pole_count() == 2 * order - 1,
                  "extension adds one degree of rows on each high-order side");
            check(x.num_rules_u() == 6 && y.num_rules_u() == 6 && x.num_rules_v() == 2 &&
                      y.num_rules_v() == 2 && x.hole_origin() == 1 && y.hole_origin() == 0,
                  "append preserves first hole origin while prepend takes extension metadata");
            for (unsigned v = 0; v < order; ++v)
                for (unsigned u = 0; u < 2; ++u)
                    for (unsigned k = 0; k < 3; ++k)
                        check(near(x.poles()[2 * v + u][k], a0.poles()[2 * v + u][k]),
                              "first source grid retained through forced append");
            for (unsigned v = 1; v < order; ++v)
                for (unsigned u = 0; u < 2; ++u)
                    for (unsigned k = 0; k < 3; ++k)
                        check(near(y.poles()[2 * (order - 1 + v) + u][k], b0.poles()[2 * v + u][k]),
                              "second source rows after the dropped first are retained");
            for (unsigned row = 0; row < order; ++row) {
                const double f = double(row) / (order - 1);
                const auto i = 2 * (order - 1 + row), j = 2 * row;
                const double wa = weighted ? 2. * order : 1., wb = weighted ? 2. : 1.;
                check(near(x.poles()[i][0], (1 + 1.5 * f) * wa),
                      "first extension Bernstein rows use native one-point-five tangent "
                      "displacement");
                check(near(y.poles()[j][1], (-1.15 + .15 * f) * wb),
                      "second extension reverses before elevation and clamps its zero extent to "
                      "point one");
                if (weighted)
                    check(near(x.weights()[i], wa) && near(y.weights()[j], wb),
                          "extension duplicates true endpoint weights without rescaling source "
                          "columns");
            }
            check(result.report["plane"]["first_extension"] == 1 &&
                      result.report["plane"]["second_extension"] == 0 &&
                      result.report["plane"]["extension"]["second"]["extension_parameter"] == .1,
                  "line intersection extent and clamped high-order extension remain distinct");
        }
    auto low = surface(false, 2, true), high = surface(true, 3, true);
    apply(low, high);
    const auto mixed = BsplineSurface::from_bgfb(low);
    check(near(mixed.poles()[2][0], 7) && mixed.weights()[2] == 4 && low["numRulesU"] == 5,
          "low-order last row uses base weights for division and multiplication and keeps rule "
          "count");
    auto first_high = surface(false, 3), last_low = surface(true, 2);
    for (unsigned i = 0; i < 4; ++i)
        last_low["poles"][3 * i + 1] = last_low["poles"][3 * i + 1].get<double>() + 2;
    apply(first_high, last_low);
    check(near(BsplineSurface::from_bgfb(last_low).poles()[0][1], -.5),
          "low-order second row subtracts positive raw extent without a minimum");
    auto self = surface(false, 3, true);
    auto refs = seam();
    TubeBudget self_budget;
    auto self_result = apply_tube_facet_seam(self, self, refs, self_budget);
    check(self_result.status == TubeFacetSeamStatus::complete && self["numPolesV"] == 7 &&
              self["numRulesU"] == 7 && self["holeOrigin"] == 0,
          "self-seam sequentially appends and prepends to one shared current surface");
    const auto ss = BsplineSurface::from_bgfb(self);
    check(near(ss.poles().front()[1], -.3) && near(ss.poles().back()[0], 6.9),
          "self extension uses saved original endpoint profiles on both ends");
    auto mismatched_a = surface(false, 3), mismatched_b = surface(true, 3);
    mismatched_a["knotsU"] = {2, 2, 7, 7};
    mismatched_b["knotsU"] = {10, 10, 20, 20};
    apply(mismatched_a, mismatched_b);
    check(mismatched_b["knotsU"] == mismatched_a["knotsU"],
          "second extension profile uses the first surface U parameters as native code does");
    auto combine = [&](Json a, Json b) {
        TubeBudget budget;
        return combine_tube_surfaces_v(BsplineSurface::from_bgfb(a), BsplineSurface::from_bgfb(b),
                                       budget);
    };
    auto ca = surface(false, 2), cb = surface(true, 2, true);
    const auto cb0 = BsplineSurface::from_bgfb(cb);
    auto unweighted = BsplineSurface::from_bgfb(combine(ca, cb).surface);
    check(!unweighted.rational() && unweighted.poles().back() == cb0.poles().back(),
          "native combine takes left rational flag even if right controls have weights");
    auto rational = BsplineSurface::from_bgfb(combine(cb, ca).surface);
    check(rational.rational() && rational.weights().back() == 1 && rational.weights().front() == 2,
          "rational left combine supplies unit weights for polynomial right controls");
    ca["closedU"] = true;
    ca["knotsU"] = nullptr;
    check(combine(ca, cb).surface["closedU"] == true,
          "forced combine retains left transverse closure without compatibility conversion");
    rejects([&] { combine(ca, surface(true, 3)); },
            "equal V order remains a native combination precondition");
    const auto before_a = surface(false, 3, true), before_b = surface(true, 3, true);
    auto aa = before_a, bb = before_b;
    auto bad_refs = seam();
    TubeBudget limited;
    limited.max_control_points = 12;
    // The plane inputs fit, but a degree-3 extension requires a larger combined grid.
    aa = surface(false, 4);
    bb = surface(true, 4);
    const auto keep_a = aa, keep_b = bb;
    limited.max_control_points = 16;
    limited.max_work = 2000;
    rejects([&] { apply_tube_facet_seam(aa, bb, bad_refs, limited); },
            "limited extension work fails explicitly");
    check(aa == keep_a && bb == keep_b && bad_refs.classifier == -2,
          "failed extension does not publish either current seam side");
    auto prepared_refs = seam();
    TubeBudget preparation_budget;
    auto prepared =
        prepare_tube_facet_plane_seam(before_a, before_b, prepared_refs, preparation_budget);
    TubeBudget short_storage;
    short_storage.max_control_points = 8;
    rejects([&] { extend_tube_facet_plane_seam(prepared, prepared_refs, short_storage); },
            "combined high-order control grid observes output budget");
    check(prepared.status == TubeFacetSeamStatus::pending_general,
          "failed extension preserves its resumable plane preparation");
    auto run = [&] {
        auto a = before_a, b = before_b;
        auto s = seam();
        TubeBudget budget;
        apply_tube_facet_seam(a, b, s, budget);
        return Json::array({a, b});
    };
    auto f = std::async(std::launch::async, run), g = std::async(std::launch::async, run);
    check(f.get() == g.get(), "concurrent seam extensions have independent working storage");
    return checks;
}
