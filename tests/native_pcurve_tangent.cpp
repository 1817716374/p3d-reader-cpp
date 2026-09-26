#include "native_pcurve_points.hpp"
#include "native_tube_orientation.hpp"
#include <future>
using namespace p3d;
namespace {
BsplineCurve curve(unsigned order, Json poles, Json weights = nullptr, bool closed = false,
                   Json knots = nullptr) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"poles", poles},
                                    {"weights", weights},
                                    {"closed", closed},
                                    {"knots", knots}});
}
bool near(double a, double b, double tol = 2e-9) {
    return std::abs(a - b) <= tol * std::max({1., std::abs(a), std::abs(b)});
}
Json surface(unsigned order = 2, bool degenerate = false) {
    Json points = degenerate ? Json{0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0}
                             : Json{0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0};
    const auto n = points.size() / 3;
    Json poles = points;
    for (std::size_t i = 0; i < points.size(); ++i)
        poles.push_back(i % 3 == 2 ? 1. : points[i].get<double>());
    return {{"_type", "BsplineSurface"},
            {"numPolesU", n},
            {"numPolesV", 2},
            {"orderU", order},
            {"orderV", 2},
            {"closedU", false},
            {"closedV", false},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"poles", poles},
            {"weights", nullptr},
            {"boundaries", nullptr},
            {"numRulesU", 0},
            {"numRulesV", 0},
            {"holeOrigin", 0}};
}
} // namespace
unsigned native_pcurve_tangent_tests() {
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
        check(caught, "native point tangent rejects invalid input or resource bounds");
    };
    auto verify = [&](const BsplineCurve &c) {
        const auto poles = c.poles();
        const auto weights = c.weights();
        const auto domain = c.knot_domain();
        const double span = domain[1] - domain[0];
        for (double f : {.03, .17, .37, .63, .87, .98}) {
            const auto a = detail::pcurve_point_tangent(c, f);
            const auto p = detail::pcurve_point(c, f);
            check(a.value.point == p.point && a.value.weight == p.weight &&
                      a.value.zero_weight_fallback == p.zero_weight_fallback,
                  "derivative branch leaves point-query evaluation unchanged");
            // Independent public tensor basis queried at neighboring fractions;
            // five-point central difference is unrelated to native derivative recurrence.
            const double h = 1e-5;
            const auto m2 = c.point_at(f - 2 * h), m1 = c.point_at(f - h), p1 = c.point_at(f + h),
                       p2 = c.point_at(f + 2 * h);
            for (unsigned k = 0; k < 3; ++k) {
                const double d = (m2[k] - 8 * m1[k] + 8 * p1[k] - p2[k]) / (12 * h * span);
                check(near(a.tangent[k], d, 3e-7),
                      "native knot derivative matches independent point difference");
            }
        }
        check(c.poles() == poles && c.weights() == weights,
              "native derivative does not reweight source controls");
        const auto a = detail::pcurve_point_tangent(c, -.2), b = detail::pcurve_point_tangent(c, 0);
        const auto x = detail::pcurve_point_tangent(c, 1.2), y = detail::pcurve_point_tangent(c, 1);
        check(a.tangent == b.tangent && a.value.point == b.value.point && x.tangent == y.tangent &&
                  x.value.point == y.value.point,
              "open and closed fractions clamp instead of wrap");
    };
    const auto line = curve(2, {2, 3, 4, 12, -2, 24}, nullptr, false, {2, 2, 7, 7});
    verify(line);
    for (double f : {0., .3, 1.}) {
        const auto tangent = detail::pcurve_point_tangent(line, f).tangent;
        check(near(tangent[0], 2) && near(tangent[1], -1) && near(tangent[2], 4),
              "raw-knot line derivative does not multiply domain span");
    }
    for (unsigned order : {2u, 3u, 4u, 8u, 16u, 26u})
        for (bool rational : {false, true}) {
            Json p = Json::array(), w = Json::array();
            for (unsigned i = 0; i < order + 2; ++i) {
                const double weight = rational ? 1 + .1 * i : 1;
                p.push_back(weight * i);
                p.push_back(weight * std::sin(double(i)));
                p.push_back(weight * .2 * i * i);
                if (rational)
                    w.push_back(weight);
            }
            verify(curve(order, p, rational ? w : Json()));
        }
    verify(curve(3, {0, 0, 0, 1, 2, 0, 3, -1, 0, 5, 3, 0, 7, 1, 0}, nullptr, false,
                 {-2, -1, 0, .3, .6, 1, 2, 3}));
    verify(curve(3, {2, 0, 1, 0, 3, 2, -2, 0, 3, 0, -3, 0}, nullptr, true));
    verify(curve(3, {0, 0, 0, -3, -6, 0, -12, 4, -8}, {-2, -3, -4}));
    const double w = std::sqrt(.5);
    verify(curve(3, {1, 0, 0, w, w, 0, 0, 1, 0, -w, w, 0, -1, 0, 0, -w, -w, 0, 0, -1, 0, w, -w, 0},
                 {1, w, 1, w, 1, w, 1, w}, true,
                 {-.25, 0, 0, 0, .25, .25, .5, .5, .75, .75, 1, 1, 1.25}));
    const auto jump =
        curve(2, {0, 0, 0, 1, 0, 0, 2, 1, 0, 2, 4, 0}, nullptr, false, {0, 0, .5, .5, 1, 1});
    const auto jd = detail::pcurve_point_tangent(jump, .5);
    check(jd.value.point == Point3{2, 1, 0} && jd.tangent == Point3{0, 6, 0},
          "full knot break selects native right-hand point and tangent");
    const auto zero = detail::pcurve_point_tangent(curve(2, {1, 2, 3, 7, 8, 9}, {1, -1}), .5);
    check(zero.value.zero_weight_fallback && zero.value.weight == 0 &&
              zero.weight_derivative == -2 && zero.value.point == Point3{4, 5, 6} &&
              zero.tangent == Point3{14, 16, 18},
          "zero-weight tangent uses Cartesian fallback point and divisor one");
    const auto raw = curve(2, {.1, 0, 0, .2, 0, 11}, {11, 11});
    const auto original = raw.poles();
    detail::pcurve_point_tangent(raw, 0);
    check(raw.poles() == original,
          "point-tangent query does not perform frame-query weight round trips");
    rejects([&] { detail::pcurve_point_tangent(line, std::numeric_limits<double>::quiet_NaN()); });

    const auto square = surface();
    const auto path = curve(2, {0, 0, 0, 0, 0, 10}, nullptr, false, {2, 2, 7, 7});
    swept_detail::TubeBudget budget;
    const auto oriented =
        swept_detail::orient_tube_surfaces_from_trace({square, square}, path, budget);
    const auto &q = oriented.report["start_tangent_query"];
    check(q["knot_tangent"] == Json({0, 0, 2}) && q["fraction_tangent"] == Json({0, 0, 10}) &&
              q["first_unit_tangent"] == Json({0, 0, 1}) &&
              oriented.report["native_result"] == true,
          "orientation caller scales knot derivative by domain span and normalizes before ring "
          "processing");
    check(oriented.report["rings"][0]["reversed_u"] == false &&
              oriented.report["rings"][1]["reversed_u"] == true,
          "automatic tangent feeds opposite outer and inner directions");
    swept_detail::TubeBudget twice_budget;
    const auto twice = swept_detail::orient_tube_surfaces_from_trace(
        {square}, curve(2, {0, 0, 0, 7, 5, 13}), twice_budget);
    check(twice.report["start_tangent_query"]["first_unit_tangent"] !=
                  twice.report["unit_start_tangent"] &&
              near(twice.report["unit_start_tangent"][0], .44905020936970896),
          "caller and ring visitor perform two separate normalizations with retained rounding");
    swept_detail::TubeBudget db;
    const auto stopped = swept_detail::orient_tube_surfaces_from_trace(
        {square, surface(4, true), square}, curve(2, {0, 0, 0, 0, 0, -10}), db);
    check(stopped.report["native_result"] == false && stopped.report["visited_rings"] == 2 &&
              stopped.surfaces[0] != square && stopped.surfaces[2] == square,
          "failed caller preserves earlier reversed and later untouched generated surfaces");
    swept_detail::TubeBudget eb;
    const auto empty = swept_detail::orient_tube_surfaces_from_trace({}, path, eb);
    check(empty.report["native_result"] == false && empty.report["start_tangent_query"].is_null(),
          "generation caller rejects empty output without querying tangent");
    rejects([&] {
        swept_detail::TubeBudget b;
        b.max_work = budget.work - 1;
        swept_detail::orient_tube_surfaces_from_trace({square, square}, path, b);
    });
    auto f = std::async(std::launch::async, [&] { return detail::pcurve_point_tangent(raw, .37); });
    const auto concurrent = detail::pcurve_point_tangent(raw, .37), other = f.get();
    check(concurrent.tangent == other.tangent && concurrent.value.point == other.value.point,
          "native point tangent has no shared mutation across threads");
    return n;
}
