#include "native_tube.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json arc(double sweep, double start = 0, double x = 1, double y = 1) {
    return {{"_type", "EllipticArc"},
            {"arc",
             {{"centerX", 3},
              {"centerY", 4},
              {"centerZ", 5},
              {"vector0X", x},
              {"vector0Y", 0},
              {"vector0Z", 0},
              {"vector90X", 0},
              {"vector90Y", y},
              {"vector90Z", 0},
              {"startRadians", start},
              {"sweepRadians", sweep}}}};
}
Json bspline(unsigned order, Json poles, Json weights = nullptr, bool closed = false,
             Json knots = nullptr) {
    return {{"_type", "BsplineCurve"}, {"order", order},     {"closed", closed},
            {"poles", poles},          {"weights", weights}, {"knots", knots}};
}
bool near(const Point3 &a, const Point3 &b, double tol = 1e-11) {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < tol;
}
} // namespace
unsigned native_tube_curve_tests() {
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
    auto convert = [&](const Json &value) {
        TubeBudget b;
        return convert_tube_primitive(value, b);
    };
    const Json line{{"_type", "LineSegment"},
                    {"segment",
                     {{"point0X", 1},
                      {"point0Y", 2},
                      {"point0Z", 3},
                      {"point1X", 5},
                      {"point1Y", 6},
                      {"point1Z", 7}}}};
    auto segment = convert(line);
    check(segment.curve.order() == 2 &&
              segment.curve.poles() == std::vector<Point3>({{1, 2, 3}, {5, 6, 7}}) &&
              segment.report["native_primitive_type"] == 1 && !segment.curve.rational(),
          "native line conversion retains exact endpoints and order two");
    auto polyline =
        convert({{"_type", "LineString"}, {"points", {0, 0, 0, 1, 0, 0, 3, 0, 0, 4, 0, 0}}});
    check(polyline.curve.poles().size() == 2 && polyline.report["native_primitive_type"] == 2 &&
              polyline.report["conversion"]["control_source_indices"] == Json({0, 3}),
          "source LineString uses fitted conversion rather than boundary uniform-knot conversion");
    constexpr double pi = 3.141592653589793;
    for (double sweep : {pi / 2, pi, 1.8 * pi, 2 * pi, -pi / 2, -pi, -1.8 * pi, -2 * pi}) {
        const auto source = arc(sweep);
        const auto a = convert(source);
        check(a.report["native_primitive_type"] == 3 && a.curve.rational() && a.curve.order() == 3,
              "ellipse primitive uses native type three and rational quadratic controls");
        check(a.curve.closed() == (std::abs(sweep) > 6.283185307178586),
              "native ellipse closure threshold and signed sweep are retained");
        const auto spans = std::abs(sweep) <= 2.0943951023931953   ? 1u
                           : std::abs(sweep) <= 4.1887902047863905 ? 2u
                                                                   : 3u;
        check(a.curve.poles().size() == 2 * spans + 1,
              "native ellipse uses up to three quadratic spans without dropping repeated end "
              "controls");
        for (double t : {0., .13, .5, .77, 1.}) {
            const auto p = a.curve.point_at(t);
            check(std::abs(std::hypot(p[0] - 3, p[1] - 4) - 1) < 1e-10 &&
                      std::abs(p[2] - 5) < 1e-12,
                  "independent rational basis evaluation preserves circle radius and plane");
        }
        if (!a.curve.closed()) {
            check(near(a.curve.point_at(0), {4, 4, 5}) &&
                      near(a.curve.point_at(1), {3 + std::cos(sweep), 4 + std::sin(sweep), 5}),
                  "open circular arc conversion preserves analytic start and end");
        }
    }
    const auto elliptic = convert(arc(pi / 2, pi / 4, 2, 1));
    check(near(elliptic.curve.point_at(0), {3 + 2 / std::sqrt(5.), 4 + 2 / std::sqrt(5.), 5}),
          "unequal axes preserve native angle adjustment instead of ordinary parametric ellipse "
          "conversion");
    for (double t : {0., .25, .5, .75, 1.}) {
        const auto p = elliptic.curve.point_at(t);
        check(std::abs((p[0] - 3) * (p[0] - 3) / 4 + (p[1] - 4) * (p[1] - 4) - 1) < 1e-10,
              "adjusted native angle still evaluates on the analytic ellipse");
    }
    for (bool closed : {false, true}) {
        const Json source =
            bspline(3, {0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9}, {0, -2, 3, -4}, closed,
                    closed ? Json({-2, -1, 2, 3, 4, 5, 8, 9, 10}) : Json({1, 2, 3, 4, 5, 6, 7}));
        const auto original = BsplineCurve::from_bgfb(source);
        const auto copied = convert(source);
        check(copied.report["native_primitive_type"] == 4 &&
                  copied.curve.poles() == original.poles() &&
                  copied.curve.weights() == original.weights() &&
                  copied.curve.knots() == original.knots() && copied.curve.closed() == closed,
              "B-spline native type four copies non-unit unclamped/periodic data including zero "
              "and signed weights");
        check(source["knots"][0] == (closed ? -2 : 1),
              "source knot domain is not normalized in place");
    }
    const auto discontinuous = convert(
        bspline(2, {0, 0, 0, 1, 0, 0, 9, 0, 0, 10, 0, 0}, nullptr, false, {0, 0, .5, .5, 1, 1}));
    check(discontinuous.curve.poles().size() == 4 &&
              near(discontinuous.curve.point_at(.25), {.5, 0, 0}) &&
              near(discontinuous.curve.point_at(.75), {9.5, 0, 0}),
          "primitive copy does not force continuity across full-multiplicity source jumps");
    Json large = Json::array();
    for (unsigned i = 0; i < 27; ++i) {
        large.push_back(i);
        large.push_back(0);
        large.push_back(0);
    }
    const auto high = convert(bspline(27, large));
    check(high.curve.order() == 27,
          "copying a source spline does not impose the later sweep evaluation order limit");
    Json original = arc(pi / 2);
    const auto before = original;
    const auto a = convert(original);
    check(original == before,
          "native primitive conversion leaves the decoded source table unchanged");
    TubeBudget three{3, 1000, 0};
    check(convert_tube_primitive(arc(pi / 2), three).curve.poles().size() == 3,
          "short arc fits a three-control budget despite the seven-control upper bound");
    rejects(
        [&] {
            TubeBudget b{3, 1000, 0};
            convert_tube_primitive(arc(2 * pi), b);
        },
        "long arc converted control count is checked");
    rejects(
        [&] {
            TubeBudget b{100, 1, 0};
            convert_tube_primitive(original, b);
        },
        "arc conversion uses shared work budget");
    rejects([&] { convert({{"_type", "LineString"}, {"points", {1, 2, 3, 4}}}); },
            "malformed XYZ layout is rejected");
    rejects([&] { convert({{"_type", "LineString"}, {"points", {1, 2, 3}}}); },
            "one-point source LineString fails native conversion");
    rejects(
        [&] {
            TubeBudget b{2, 1000, 0};
            convert_tube_primitive(bspline(3, {0, 0, 0, 1, 2, 3, 4, 5, 6}), b);
        },
        "B-spline input control budget is checked before parsing arrays");
    auto excessive = bspline(2, {0, 0, 0, 1, 0, 0});
    excessive["knots"] = Json::array({0, 0, 0, 0, 0, 0, 1, 1});
    rejects([&] { convert(excessive); },
            "malformed auxiliary knot arrays are bounded before conversion");
    for (const char *type : {"AkimaCurve", "InterpolationCurve", "TransitionSpiral", "CurveVector",
                             "PointString", "Unknown"})
        rejects([&] { convert({{"_type", type}}); },
                "swept primitive converter does not inherit broader boundary acceptance");
    rejects([&] { convert(nullptr); }, "null primitive cannot be converted");
    std::vector<std::future<std::vector<Point3>>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return convert(original).curve.poles(); }));
    for (auto &job : jobs)
        check(job.get() == a.curve.poles(),
              "shared source primitive supports deterministic concurrent conversion");
    TubeBudget b;
    const auto trace = convert_tube_primitive(
        {{"_type", "LineString"}, {"points", {0, 0, 0, 0, 0, 1, 0, 0, 3}}}, b);
    const auto section = convert_tube_primitive(arc(2 * pi, 0, .5, .5), b);
    const auto tube = tube_surface(section.curve, trace.curve, false, b);
    check(tube.surface["closedU"] == true && tube.surface["numPolesV"] == 2,
          "converted closed ellipse and fitted LineString reach native tube construction");
    return n;
}
