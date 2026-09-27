#include "native_curve_sampling.hpp"
#include <future>
#include <limits>
using namespace p3d;
using namespace p3d::curve_detail;
namespace {
BsplineCurve curve(unsigned order, Json poles, Json knots = nullptr, Json weights = nullptr,
                   bool closed = false) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"poles", poles},
                                    {"knots", knots},
                                    {"weights", weights},
                                    {"closed", closed}});
}
NativeCurveBreaks breaks(const BsplineCurve &c) {
    std::size_t used = 0;
    return native_curve_c1_breaks(c, 10000, 10000, {used, 10000000});
}
NativeCurveSampleTree sample(const std::vector<const BsplineCurve *> &c, double chord,
                             double angle = 0, std::array<double, 2> interval = {0, 1}) {
    std::size_t used = 0;
    return native_curve_sample_tree(c, interval, chord, angle, 10000, 100000, {used, 100000000});
}
bool near(double a, double b) {
    return std::abs(a - b) < 1e-12;
}
} // namespace
unsigned native_curve_sampling_tests() {
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
        check(caught, "curve sampling rejects exhausted budget or non-finite input");
    };
    const auto line = curve(2, {0, 0, 0, 1, 0, 0});
    const auto speed = curve(2, {0, 0, 0, 1, 0, 0, 3, 0, 0});
    check(breaks(speed).parameters == std::vector<double>{0, 1},
          "native C1 query accepts collinear derivatives with different speed");
    const auto corner = curve(2, {0, 0, 0, 1, 0, 0, 1, 1, 0});
    auto corners = breaks(corner);
    check(corners.parameters == std::vector<double>{0, .5, 1} &&
              corners.report["segments"][1]["reason"] == "angle",
          "corner keeps one-sided tangent break");
    const auto gap = curve(2, {0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0}, {0, 0, .5, .5, 1, 1});
    auto gaps = breaks(gap);
    check(gaps.parameters == std::vector<double>{0, .5, 1} &&
              gaps.report["segments"][1]["reason"] == "position",
          "full multiplicity retains position gap");
    std::size_t used = 0;
    auto pieces = native_curve_make_beziers(gap, 1000, {used, 1000000});
    check(pieces.size() == 2 && pieces[0].poles().back()[0] == 1 &&
              pieces[1].poles().front()[0] == 2 &&
              pieces[1].knot_domain() == std::array<double, 2>{0, 1},
          "independent Bezier spans normalize knots without stitching endpoints");
    const auto cusp =
        curve(3, {0, 0, 0, .5, 0, 0, 1, 0, 0, 1, 0, 0, 2, 0, 0}, {0, 0, 0, .5, .5, 1, 1, 1});
    auto cusps = breaks(cusp);
    check(cusps.parameters == std::vector<double>{0, .5, 1} &&
              cusps.report["segments"][1]["reason"] == "near_zero_tangent",
          "vanishing one-sided tangent uses polygon-scaled cusp threshold");
    const auto shifted = curve(2, {0, 0, 0, 1, 0, 0, 1, 1, 0}, {2, 2, 4, 6, 6});
    check(breaks(shifted).parameters == std::vector<double>{2, 2.5, 3},
          "native break normalization retains source knotA offset");
    const auto weighted = curve(2, {0, 0, 0, 4, 0, 0}, nullptr, {2, 2});
    check(near(breaks(weighted).report["segments"][0]["polygon_length"].get<double>(), 2),
          "homogeneous polygon length uses projected endpoint distance");
    const auto zero = curve(2, {1, 2, 3, 4, 5, 6}, nullptr, {0, 0});
    auto zeros = breaks(zero);
    check(zeros.report["segments"][0]["polygon_length"] == 0 &&
              zeros.report["segments"][0]["end_weight_fallback"] == true,
          "zero homogeneous weight product uses native zero distance and endpoint fallback");
    const auto near_unit = curve(2, {2, 3, 4, 5, 6, 7}, nullptr, {1 + 5e-9, 1 - 5e-9});
    auto unit = native_curve_make_beziers(near_unit, 1000, {used, 1000000});
    check(unit.size() == 1 && unit[0].weights().empty() && unit[0].poles() == near_unit.poles(),
          "native near-unit weight removal does not divide homogeneous XYZ");
    auto rational = native_curve_make_beziers(weighted, 1000, {used, 1000000});
    check(rational[0].weights() == weighted.weights() && rational[0].poles() == weighted.poles(),
          "non-unit weights and homogeneous controls remain paired");
    const double w = std::sqrt(.5);
    const auto circle = curve(
        3, {1, 0, 0, w, w, 0, 0, 1, 0, -w, w, 0, -1, 0, 0, -w, -w, 0, 0, -1, 0, w, -w, 0},
        {-.25, 0, 0, 0, .25, .25, .5, .5, .75, .75, 1, 1, 1.25}, {1, w, 1, w, 1, w, 1, w}, true);
    check(breaks(circle).parameters == std::vector<double>{0, 1},
          "periodic rational circle retains smooth internal joins");
    auto circle_pieces = native_curve_make_beziers(circle, 1000, {used, 1000000});
    check(circle_pieces.size() == 4, "special periodic support skips zero knot spans");
    for (std::size_t i = 0; i < circle_pieces.size(); ++i)
        for (double f : {0., .2, .7, 1.}) {
            auto a = circle_pieces[i].point_at(f), b = circle.point_at((i + f) / 4.);
            check(near(a[0], b[0]) && near(a[1], b[1]),
                  "periodic independent span agrees with source evaluation");
        }
    auto kd = native_curve_knot_data(circle, 1000, {used, 1000000});
    check(kd.well_ordered && kd.closed && kd.all == circle.knots() && kd.left == 1 &&
              kd.right == 5 &&
              kd.compressed == std::vector<double>{-.25, 0, .25, .5, .75, 1, 1.25} &&
              kd.multiplicities == std::vector<std::size_t>{1, 3, 2, 2, 2, 2, 1},
          "periodic raw knot-data active indices and multiplicities");
    const auto tiny_span =
        curve(2, {0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0}, {0, 0, .5, .5 + 1e-15, 1, 1});
    auto tk = native_curve_knot_data(tiny_span, 1000, {used, 1000000});
    auto tb = breaks(tiny_span);
    check(tk.compressed == std::vector<double>{0, .5, 1} && tk.multiplicities[1] == 2 &&
              tb.report["skipped_intervals"] == 1,
          "near-null knots use native representative compression");
    auto straight = sample({&line}, .01);
    check(straight.success && straight.parameters == std::vector<double>{0, 1} &&
              straight.nodes.size() == 3,
          "accepted straight interval retains provisional quarter nodes but emits only endpoints");
    check(straight.nodes[0].sample.parameter == .5 && straight.nodes[1].sample.parameter == .25 &&
              straight.nodes[2].sample.parameter == .75 && straight.nodes[1].parent == 0,
          "native midpoint tree preserves parent and quarter positions");
    const auto parabola = curve(3, {0, 0, 0, .5, 0, 0, 1, 1, 0}); // (t,t*t,0)
    auto quadratic = sample({&parabola}, .07);
    check(quadratic.success && quadratic.parameters == std::vector<double>{0, .5, 1},
          "analytic parabola chord error quarters on each bisection");
    auto multi = sample({&line, &parabola, &parabola}, .07);
    check(multi.parameters == quadratic.parameters && multi.nodes[0].sample.points.size() == 3 &&
              multi.report["point_tangent_evaluations"].get<std::size_t>() ==
                  3 * quadratic.report["point_tangent_evaluations"].get<std::size_t>(),
          "shared fraction tree responds to every curve and retains duplicate references");
    auto angular = sample({&parabola}, 0, .3);
    check(angular.success && angular.parameters.size() > 2,
          "angle-only sampling refines curved tangent field");
    // z=t(t-1/4)(t-1/2)(t-1): zero at the midpoint and left quarter,
    // nonzero at the right quarter. Native right-quarter test ignores Z.
    const auto zcurve =
        curve(5, {0, 0, 0, .25, 0, -.03125, .5, 0, 1. / 12, .75, 0, -.09375, 1, 0, 0});
    const auto ycurve =
        curve(5, {0, 0, 0, .25, -.03125, 0, .5, 1. / 12, 0, .75, -.09375, 0, 1, 0, 0});
    check(sample({&zcurve}, 1e-8).parameters == std::vector<double>{0, 1},
          "right-quarter-only spatial deviation is ignored by native XY predicate");
    check(sample({&ycurve}, 1e-4).parameters.size() > 2,
          "same deviation in XY triggers native refinement");
    auto off = sample({&line}, 1e-12, 1e-12);
    check(!off.success && off.nodes.empty() && off.parameters.empty() &&
              off.report["failure"] == "both_tolerances_near_zero",
          "near-zero tolerance failure clears tree");
    auto depth = sample({&line}, 0, -.1);
    check(!depth.success && depth.nodes.empty() &&
              depth.report["failure"] == "native_depth_limit" &&
              depth.report["deepest_allocated_level"] == 32,
          "negative native angle reaches depth guard after provisional child allocation");
    check(sample({}, 0, 0).success, "empty curve vector bypasses per-curve tolerance failure");
    check(sample({&line}, .01, 0, {1, 0}).parameters == std::vector<double>{1, 0},
          "reverse sample interval retains requested direction");
    auto domain = sample({&shifted}, 10);
    check(domain.nodes[0].sample.tangents[0] == Point3{0, 2, 0},
          "tree tangent is derivative with respect to normalized fraction");
    rejects([&] {
        std::size_t u = 0;
        native_curve_c1_breaks(gap, 100, 1, {u, 1000000});
    });
    rejects([&] {
        std::size_t u = 0;
        native_curve_make_beziers(gap, 3, {u, 1000000});
    });
    rejects([&] {
        std::size_t u = 0;
        native_curve_sample_tree({&line}, {0, 1}, .1, 0, 100, 2, {u, 1000000});
    });
    rejects([&] {
        std::size_t u = 0;
        native_curve_sample_tree({&line}, {0, 1}, .1, 0, 100, 100, {u, 0});
    });
    rejects([&] { sample({&line}, std::numeric_limits<double>::infinity()); });
    auto future =
        std::async(std::launch::async, [&] { return sample({&parabola}, .07).parameters; });
    check(future.get() == sample({&parabola}, .07).parameters && gap.poles()[2] == Point3{2, 0, 0},
          "parallel independent queries leave sources unchanged");
    return n;
}
