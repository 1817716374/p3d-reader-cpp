#include "internal.hpp"
#include <future>
using namespace p3d;
namespace {
BsplineCurve curve(unsigned order, Json poles, Json weights = nullptr, Json knots = nullptr,
                   bool closed = false) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"poles", poles},
                                    {"weights", weights},
                                    {"knots", knots},
                                    {"closed", closed}});
}
bool near(const Point3 &a, const Point3 &b, double tolerance = 2e-11) {
    for (unsigned i = 0; i < 3; ++i)
        if (std::abs(a[i] - b[i]) > tolerance * std::max({1., std::abs(a[i]), std::abs(b[i])}))
            return false;
    return true;
}
Point3 column(const Json &frame, unsigned c) {
    const auto &m = frame.at("frame");
    return {m[0][c], m[1][c], m[2][c]};
}
} // namespace
unsigned bspline_frame_tests() {
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
    const auto line = curve(2, {2, 3, 4, 8, 3, 4});
    auto f = line.native_frame_at(.5);
    check(f["status"] == "computed" && f["method"] == "axis_fallback_frame" &&
              f["basis_status"] == "nondegenerate" && f["curvature"] == 0,
          "straight B-spline selects native axis fallback");
    check(near(column(f, 0), {1, 0, 0}) && near(column(f, 1), {0, 1, 0}) &&
              near(column(f, 2), {0, 0, 1}) && near(column(f, 3), {5, 3, 4}) &&
              f["frame"][3] == Json({0, 0, 0, 1}),
          "native frame uses T/N/B columns and source position");
    const auto vertical = curve(2, {0, 0, 0, 0, 0, 5});
    f = vertical.native_frame_at(0);
    check(near(column(f, 0), {0, 0, 1}) && near(column(f, 1), {1, 0, 0}) &&
              near(column(f, 2), {0, 1, 0}),
          "near vertical tangent uses Y as reference axis");
    for (double x : {.009, .011}) {
        const auto tilted = curve(2, {0, 0, 0, x, 0, std::sqrt(1 - x * x)}).native_frame_at(0);
        check(x < .01 ? column(tilted, 1)[0] > .9 : column(tilted, 1)[1] > .9,
              "axis choice uses the native absolute tangent component threshold");
    }
    const auto cubic = curve(4, {0, 0, 0, 1. / 3, 0, 0, 2. / 3, 1. / 3, 0, 1, 1, 1});
    f = cubic.native_frame_at(0);
    check(f["method"] == "derivative_frame" && near(column(f, 0), {1, 0, 0}) &&
              near(column(f, 1), {0, 1, 0}) && near(column(f, 2), {0, 0, 1}) && f["curvature"] == 2,
          "regular curve takes Frenet frame directly from first and second derivatives");
    for (double t : {.1, .5, .9, 1.}) {
        f = cubic.native_frame_at(t);
        const auto a = column(f, 0), b = column(f, 1), c = column(f, 2);
        const double speed = std::sqrt(1 + 4 * t * t + 9 * t * t * t * t);
        check(near(a, {1 / speed, 2 * t / speed, 3 * t * t / speed}) &&
                  near(column(f, 3), {t, t * t, t * t * t}),
              "nonplanar polynomial Frenet tangent and position follow analytic derivatives");
        check(std::abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) < 1e-12 &&
                  near({a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                        a[0] * b[1] - a[1] * b[0]},
                       c),
              "regular native frame retains its right-handed orientation");
    }
    const auto inflection = curve(4, {0, 0, 0, 1. / 3, 0, 0, 2. / 3, 0, 0, 1, -1, 0});
    f = inflection.native_frame_at(0);
    check(f["method"] == "control_polygon_frame" && near(column(f, 1), {0, -1, 0}) &&
              near(column(f, 2), {0, 0, -1}) && f["curvature"] == 0,
          "zero second derivative uses ordered control polygon normal");
    f = curve(4, {0, 0, 0, 1e-4 / 3, 0, 0, 2e-4 / 3, 0, 0, 1e-4, -1e-4, 0}).native_frame_at(0);
    check(f["method"] == "axis_fallback_frame" && near(column(f, 1), {0, 1, 0}),
          "small polygon area triggers the native fixed-axis fallback instead of polygon normal");
    f = curve(3, {0, 0, 0, .5, 0, 0, 1, -5e-6, 0}).native_frame_at(0);
    check(f["method"] == "derivative_frame" && column(f, 1)[1] < 0,
          "second derivative and cross-product threshold equality remain nondegenerate");
    const auto large_domain =
        curve(3, {0, 0, 0, .5, 0, 0, 1, -1, 0}, nullptr, {0, 0, 0, 1e6, 1e6, 1e6});
    f = large_domain.native_frame_at(0);
    check(f["method"] == "control_polygon_frame" && f["curvature"] == 0,
          "native absolute derivative threshold depends on source knot parameter scale");
    const auto low_curvature = curve(3, {0, 0, 0, 5e6, 0, 0, 1e7, -1, 0});
    f = low_curvature.native_frame_at(0);
    check(f["method"] == "control_polygon_frame" && f["curvature"] == 0,
          "curvature below 1e-12 enters fallback even with nondegenerate derivatives");

    const double w = std::sqrt(.5);
    const auto circle = curve(3, {1, 0, 0, w, w, 0, 0, 1, 0}, {1, w, 1});
    for (double t : {0., .2, .5, .8, 1.}) {
        f = circle.native_frame_at(t);
        const auto p = column(f, 3);
        check(
            f["method"] == "derivative_frame" && near(p, circle.point_at(t)) &&
                near(column(f, 0), {-p[1], p[0], 0}) && near(column(f, 1), {-p[0], -p[1], 0}) &&
                near(column(f, 2), {0, 0, 1}) && std::abs(f["curvature"].get<double>() - 1) < 1e-12,
            "rational frame chain rule recovers exact conic tangent, inward normal and curvature");
    }
    const auto constant = curve(2, {3, 4, 5, 3, 4, 5});
    f = constant.native_frame_at(.4);
    check(f["status"] == "computed" && f["basis_status"] == "degenerate" &&
              near(column(f, 0), {}) && near(column(f, 1), {}) && near(column(f, 2), {}) &&
              near(column(f, 3), {3, 4, 5}),
          "native zero tangent keeps a degenerate basis without repair");
    check(line.native_frame_at(-1) == line.native_frame_at(0) &&
              line.native_frame_at(5) == line.native_frame_at(1),
          "open frame parameter clamp");
    const auto periodic = curve(3, {0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0}, nullptr, nullptr, true);
    check(periodic.native_frame_at(.125) == periodic.native_frame_at(2.125) &&
              periodic.native_frame_at(.125) == periodic.native_frame_at(-1.875),
          "closed frame parameter wraps with native endpoint-side convention");
    rejects([&] { curve(2, {0, 0, 0, 1, 0, 0}, {1, 0}).native_frame_at(0); },
            "zero control weight rejected");
    rejects([&] { curve(2, {0, 0, 0, 1, 0, 0}, {1, -1}).native_frame_at(.5); },
            "zero evaluated weight rejected");
    rejects(
        [&] { curve(2, {0, 0, 0, 1, 0, 0}, nullptr, {0, 0, 1e-14, 1e-14}).native_frame_at(.5); },
        "native frame propagates derivative knot-tolerance failure");
    rejects([&] { line.native_frame_at(std::numeric_limits<double>::quiet_NaN()); },
            "non-finite frame parameter rejected");
    const auto saved = circle.poles();
    const auto expected = circle.native_frame_at(.3);
    auto task = std::async(std::launch::async, [&] { return circle.native_frame_at(.3); });
    check(circle.native_frame_at(.3) == expected && task.get() == expected &&
              circle.poles() == saved,
          "native frame reads source concurrently without cumulative weight-roundtrip mutations");
    return checks;
}
