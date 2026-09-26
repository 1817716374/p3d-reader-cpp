#include "native_tube.hpp"
#include "loft_curve.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
using p3d::loft_detail::Curve;
namespace {
BsplineCurve curve(unsigned order, Json poles, Json weights = nullptr, bool closed = false,
                   Json knots = nullptr) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"closed", closed},
                                    {"poles", poles},
                                    {"weights", weights},
                                    {"knots", knots}});
}
Json surface(const std::vector<Curve> &columns) {
    Json xyz = Json::array(), weights = Json::array();
    const auto nv = columns.front().poles.size();
    for (std::size_t v = 0; v < nv; ++v)
        for (const auto &c : columns) {
            require(c.poles.size() == nv && c.knots == columns.front().knots,
                    "test columns must have matching source V layout");
            for (unsigned axis = 0; axis < 3; ++axis)
                xyz.push_back(c.poles[v][axis]);
            weights.push_back(c.poles[v][3]);
        }
    return {{"_type", "BsplineSurface"},
            {"orderU", 2},
            {"orderV", columns.front().degree + 1},
            {"closedU", false},
            {"closedV", false},
            {"numPolesU", columns.size()},
            {"numPolesV", nv},
            {"knotsU", {2, 2, 5, 5}},
            {"knotsV", columns.front().knots},
            {"poles", xyz},
            {"weights", columns.front().rational ? weights : Json()},
            {"numRulesU", -7},
            {"numRulesV", 123},
            {"holeOrigin", 0},
            {"boundaries", nullptr}};
}
bool near(const Point3 &a, const Point3 &b) {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < 1e-9;
}
} // namespace
unsigned native_tube_close_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn, const char *why) {
        bool threw = false;
        try {
            fn();
        } catch (const std::exception &) {
            threw = true;
        }
        check(threw, why);
    };
    auto verify = [&](const Json &input) {
        const auto source = BsplineSurface::from_bgfb(input);
        TubeBudget b;
        auto result = close_tube_surface_v(source, b);
        check(result.report["applied"] == true && result.report["native_return_code"] == 0,
              "compatible columns close successfully");
        const auto closed = BsplineSurface::from_bgfb(result.surface);
        for (double u : {0., .27, 1.})
            for (double v : {0., .07, .25, .5, .81, 1.})
                check(near(source.point_at(u, v), closed.point_at(u, v)),
                      "periodic representation agrees with independent open surface evaluation");
        check(result.surface["knotsU"] == input["knotsU"] && result.surface["numRulesU"] == -7 &&
                  result.surface["numRulesV"] == 123 && !closed.u().closed() &&
                  closed.boundaries().is_null(),
              "U parameterization and source rule counts are retained");
        check(source.poles() == BsplineSurface::from_bgfb(input).poles(),
              "source surface storage is unchanged");
        return result;
    };
    auto polygon = Curve::from_bspline(
        curve(2, {0, 0, 0, 2, 0, 0, 2, 3, 0, 0, 0, 0}, nullptr, false, {0, 0, .2, .7, 1, 1}), 100);
    auto other = polygon;
    for (auto &p : other.poles)
        p[2] = 4;
    const auto linear_input = surface({polygon, other});
    const auto linear = verify(linear_input);
    check(linear.surface["numPolesV"] == 3 && linear.surface["closedV"] == true &&
              std::abs(linear.surface["knotsV"][0].get<double>() + .3) < 1e-15 &&
              Json::array({linear.surface["knotsV"][1], linear.surface["knotsV"][2],
                           linear.surface["knotsV"][3], linear.surface["knotsV"][4],
                           linear.surface["knotsV"][5]}) == Json({0, .2, .7, 1, 1.2}),
          "linear closure drops duplicate row and extends the native cyclic knot vector");
    // A quadratic loop with only three poles takes the special, unreduced form.
    auto special = Curve::from_bspline(curve(3, {0, 0, 0, 2, 3, 0, 0, 0, 0}), 100);
    other = special;
    for (auto &p : other.poles)
        p[2] = 4;
    const auto special_result = verify(surface({special, other}));
    check(special_result.surface["numPolesV"] == 3 &&
              special_result.surface["knotsV"] == Json({-1, 0, 0, 0, 1, 1, 1, 2}) &&
              special_result.report["columns"][0]["method"] == "special_periodic",
          "special periodic form retains source poles instead of duplicating or averaging them");
    // Closed native curves opened by the separate opening algorithm provide
    // smooth reference surfaces; the closure must recover their geometry.
    Json regular_input;
    for (unsigned order : {3u, 4u, 5u, 8u}) {
        Json xyz = Json::array(), weights = Json::array();
        for (unsigned i = 0; i < order + 4; ++i) {
            const double w = 1. + .125 * (i % 3);
            xyz.push_back(std::cos(i) * w);
            xyz.push_back(std::sin(i) * w);
            xyz.push_back(.1 * i * w);
            weights.push_back(w);
        }
        const auto periodic = curve(order, xyz, weights, true);
        const auto opened = Curve::from_bspline(periodic, 100);
        other = opened;
        for (auto &p : other.poles)
            p[2] += 3 * p[3];
        auto input = surface({opened, other});
        auto r = verify(input);
        check(r.report["columns"][0]["method"] == "regular_periodic" &&
                  r.surface["numPolesV"] == periodic.poles().size(),
              "smooth periodic columns use native unclamping and overlap reduction");
        if (order == 3)
            regular_input = input;
    }
    auto line = Curve::from_bspline(curve(2, {1, 2, 3, 4, 5, 6}), 100);
    other = line;
    for (auto &p : other.poles)
        p[2] += 2;
    const auto line_input = surface({line, other});
    const auto unchanged = verify(line_input);
    check(unchanged.surface == line_input && unchanged.report["output_closed_v"] == false,
          "two-pole linear columns copy successfully without forcing closure");
    // A later-column failure cannot leak already closed earlier columns.
    auto bad = linear_input;
    bad["poles"][bad["poles"].size() - 1] = 8;
    TubeBudget fail_budget;
    auto failure = close_tube_surface_v(BsplineSurface::from_bgfb(bad), fail_budget);
    check(failure.surface == bad && failure.report["applied"] == false &&
              failure.report["native_return_code"] == 1 && failure.report["columns"].size() == 2,
          "endpoint failure in the second column retains the whole original surface");
    // Break the smooth overlap in only the second column while retaining ends.
    bad = regular_input;
    bad["poles"][9] = bad["poles"][9].get<double>() + .5;
    TubeBudget mismatch_budget;
    auto mismatch = close_tube_surface_v(BsplineSurface::from_bgfb(bad), mismatch_budget);
    check(mismatch.surface == bad && mismatch.report["applied"] == false &&
              mismatch.report["reason"] == "column_pole_count_mismatch" &&
              mismatch.report["native_return_code"] == 0,
          "native zero status with incompatible column counts does not claim applied closure");
    // Stored homogeneous endpoint checks precede deweighting. Zero and negative
    // weights are preserved rather than excluded by the loft Cartesian contract.
    for (double sign : {0., -1.}) {
        auto c = polygon;
        c.rational = true;
        for (auto &p : c.poles)
            p[3] = sign;
        other = c;
        for (auto &p : other.poles)
            p[2] = 2;
        const auto input = surface({c, other});
        TubeBudget b;
        const auto r = close_tube_surface_v(BsplineSurface::from_bgfb(input), b);
        check(r.report["applied"] == true &&
                  r.surface["weights"] == Json::array({sign, sign, sign, sign, sign, sign}),
              "closure retains zero or negative stored weights without deweighting");
        c.poles.back()[3] = sign + 1e-6;
        TubeBudget w;
        const auto rejected =
            close_tube_surface_v(BsplineSurface::from_bgfb(surface({c, other})), w);
        check(rejected.report["applied"] == false &&
                  rejected.report["columns"][0]["reason"] == "endpoint_weight_mismatch",
              "unequal homogeneous endpoint weights reject closure");
    }
    TubeBudget copy_budget;
    auto copied = close_tube_surface_v(BsplineSurface::from_bgfb(linear.surface), copy_budget);
    check(copied.surface == linear.surface && copied.report["method"] == "already_closed_copy" &&
              copied.report["columns"].empty(),
          "already periodic surface is copied unchanged");
    // Apply the actual tube pipeline, not only manually assembled columns.
    const auto section = curve(2, {1, 0, 0, 2, 0, 0});
    const auto trace = curve(2, {0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0}, nullptr, true);
    TubeBudget tube_budget;
    const auto generated = tube_surface(section, trace, false, tube_budget);
    auto closed = close_tube_surface_v(BsplineSurface::from_bgfb(generated.surface), tube_budget);
    check(closed.report["applied"] == true && closed.surface["closedV"] == true &&
              generated.surface["closedV"] == false,
          "closed trace tube construction and separate swept-body closure are connected");
    const auto before = BsplineSurface::from_bgfb(generated.surface);
    const auto after = BsplineSurface::from_bgfb(closed.surface);
    for (double t : {0., .1, .25, .5, .8, 1.})
        check(near(before.point_at(.4, t), after.point_at(.4, t)),
              "closed tube conversion preserves assembled sweep geometry");
    rejects(
        [&] {
            TubeBudget b{7, 10000, 0};
            close_tube_surface_v(BsplineSurface::from_bgfb(linear_input), b);
        },
        "closure respects input grid budget");
    rejects(
        [&] {
            TubeBudget b{100, 10, 0};
            close_tube_surface_v(BsplineSurface::from_bgfb(linear_input), b);
        },
        "closure respects work budget before allocating column results");
    rejects(
        [&] {
            TubeBudget b{100, 10, 11};
            close_tube_surface_v(BsplineSurface::from_bgfb(linear_input), b);
        },
        "invalid prior shared work is rejected");
    bad = linear_input;
    bad["knotsV"] = {2, 2, 2.2, 2.7, 3, 3};
    rejects(
        [&] {
            TubeBudget b;
            close_tube_surface_v(BsplineSurface::from_bgfb(bad), b);
        },
        "normalized tube-only closure does not silently rewrite arbitrary source domains");
    auto shared = BsplineSurface::from_bgfb(regular_input);
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async, [&] {
            TubeBudget b;
            return close_tube_surface_v(shared, b).surface;
        }));
    const auto expected = jobs.front().get();
    for (std::size_t i = 1; i < jobs.size(); ++i)
        check(jobs[i].get() == expected,
              "concurrent closure of one immutable surface is deterministic");
    return n;
}
