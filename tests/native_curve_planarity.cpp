#include "native_curve_planarity.hpp"
#include "native_tube_path_selection.hpp"
#include <future>
using namespace p3d;
using namespace p3d::curve_detail;
namespace {
Json line(Point3 a, Point3 b) {
    Json s;
    for (unsigned i = 0; i < 3; ++i) {
        s[std::string("point0") + "XYZ"[i]] = a[i];
        s[std::string("point1") + "XYZ"[i]] = b[i];
    }
    return {{"_type", "LineSegment"}, {"segment", s}};
}
Json arc(Point3 c, Point3 u, Point3 v, double start, double sweep) {
    Json a{{"startRadians", start}, {"sweepRadians", sweep}};
    for (unsigned i = 0; i < 3; ++i) {
        a[std::string("center") + "XYZ"[i]] = c[i];
        a[std::string("vector0") + "XYZ"[i]] = u[i];
        a[std::string("vector90") + "XYZ"[i]] = v[i];
    }
    return {{"_type", "EllipticArc"}, {"arc", a}};
}
Json group(Json values) {
    Json a = Json::array();
    for (const auto &v : values)
        a.push_back({{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", 1}, {"curves", a}};
}
} // namespace
unsigned native_curve_planarity_tests() {
    unsigned count = 0;
    auto check = [&](bool ok, const char *why) {
        ++count;
        require(ok, why);
    };
    auto near = [](double a, double b) { return std::abs(a - b) < 1e-9 * (1 + std::abs(b)); };
    auto rejects = [&](auto fn, const char *why) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    Matrix4 identity{};
    for (unsigned i = 0; i < 4; ++i)
        identity[i][i] = 1;
    auto range = [&](const Json &j, const Matrix4 &m) {
        std::size_t work = 0;
        return native_primitive_range(j, m, 10000, {work, 10000000});
    };
    auto planar = [&](const Json &j) {
        std::size_t work = 0;
        return native_primitive_planarity(j, 10000, {work, 10000000});
    };
    const auto segment = line({1, 1, 1}, {2, 3, 4});
    auto r = range(segment, identity);
    check(r.present && r.low == Point3{1, 1, 1} && r.high == Point3{2, 3, 4}, "line exact range");
    auto m = identity;
    m[0] = {1e16, -1e16, 1, 1};
    r = range(line({1, 1, 1}, {1, 1, 1}), m);
    check(r.low[0] == 2 && r.high[0] == 2, "range point translation added last");
    const double huge = std::numeric_limits<double>::max();
    r = range(line({huge, 1, 1}, {1, 2, 3}), identity);
    check(r.low == Point3{1, 2, 3} && r.high == r.low,
          "disconnect endpoint skipped before transform");
    r = range(line({huge, 1, 1}, {1, huge, 3}), identity);
    check(!r.present, "all disconnect endpoints make null range");
    constexpr double pi = 3.141592653589793, tau = 2 * pi;
    const auto ellipse = arc({3, 5, 7}, {2, 0, 0}, {0, 1, 0}, 0, tau);
    r = range(ellipse, identity);
    check(r.low == Point3{1, 4, 7} && r.high == Point3{5, 6, 7}, "full ellipse analytic range");
    r = range(arc({0, 0, 0}, {2, 0, 0}, {0, 1, 0}, 0, pi / 2), identity);
    check(near(r.low[0], 0) && r.low[1] == 0 && r.high[0] == 2 && r.high[1] == 1,
          "quarter ellipse range includes endpoints");
    for (double start : {-7., -.3, 0., 4.})
        for (double sweep : {-9., -2., 0., .7, 2., 9.}) {
            const Point3 c{2, -3, 4}, u{2, 1, -1}, v{1, -3, 2};
            r = range(arc(c, u, v, start, sweep), identity);
            bool contained = true;
            for (unsigned k = 0; k <= 1000; ++k) {
                long double theta =
                    static_cast<long double>(start) + static_cast<long double>(sweep) * k / 1000;
                for (unsigned i = 0; i < 3; ++i) {
                    const double p = double(c[i] + u[i] * std::cos(theta) + v[i] * std::sin(theta));
                    contained &= p >= r.low[i] - 1e-10 && p <= r.high[i] + 1e-10;
                }
            }
            check(contained, "ellipse native range contains independent angular samples");
        }
    check(native_range_angle_in_sweep(-5e-13, 0, .1) && !native_range_angle_in_sweep(-2e-12, 0, .1),
          "angle tolerance before start");
    check(native_range_angle_in_sweep(.1 + 5e-13, 0, .1) &&
              !native_range_angle_in_sweep(.1 + 2e-12, 0, .1),
          "angle tolerance after end");
    for (double turns : {-10., -1., 0., 1., 10.}) {
        check(native_range_angle_in_sweep(turns * tau, 0, 0),
              "zero sweep periodic equivalent point");
        check(native_range_angle_in_sweep(turns * tau - .5, 0, -1),
              "reversed sweep periodic point");
    }
    NativeCurveRange box;
    box.present = true;
    box.low = {0, 0, 0};
    box.high = {1, 1, 2e-10};
    check(native_range_z(box).planar, "native scale dependent Z accepts thin box");
    box.high[2] = 4e-10;
    check(!native_range_z(box).planar, "native Z rejects thickness above tolerance");
    box.high = {1e16, 1, 1};
    const auto z = native_range_z(box);
    check(z.rounded_span == 0 && z.planar, "native Z preserves rounded addition subtraction");
    box.high = {1, 1, 1};
    check(!native_range_z(box).planar, "unit cube not planar");
    check(planar(segment).at("planar") == true, "original line local frame is planar");
    check(planar(ellipse).at("planar") == true, "original ellipse local frame is planar");
    Json spline{
        {"_type", "BsplineCurve"}, {"order", 4},
        {"closed", false},         {"poles", {0., 0., 0., 1., 0., 0., 1., 1., 1., 2., 2., 0.}},
        {"weights", nullptr},      {"knots", nullptr}};
    check(planar(spline).at("planar") == false, "spatial cubic has nonzero local Z range");
    spline["poles"] = {0., 0., 7., 1., 0., 9., 1., 1., 12., 2., 2., 17.};
    check(planar(spline).at("planar") == true, "tilted planar cubic");
    spline["weights"] = {1., 2., .5, 1.};
    spline["poles"] = {0., 0., 7., 2., 0., 18., .5, .5, 6., 2., 2., 17.};
    const auto original = spline;
    const auto expected = planar(spline);
    check(expected.at("planar") == true && spline == original, "rational planar curve immutable");
    m = identity;
    m[2][3] = 10;
    r = range(spline, m);
    check(near(r.low[2], 17) && near(r.high[2], 27), "homogeneous weighted translation range");
    const auto source =
        group({line({0, 0, 0}, {1, 0, 0}), line({1, 0, 0}, {1, 1, 0}), line({1, 1, 0}, {1, 1, 1})});
    swept_detail::TubeBudget budget;
    const auto prepared =
        swept_detail::prepare_tube_facet_path(group({line({0, 0, 0}, {0, 0, 0})}), source, budget);
    check(prepared.selected_member_planarity.at("planar") == true,
          "selected original member planar even when whole path nonplanar");
    check(prepared.selected_member_planarity.at("working_index") == prepared.selection.index,
          "planarity uses remapped selected member");
    std::size_t used = 0;
    native_primitive_planarity(segment, 100, {used, 1000000});
    const auto cost = used;
    used = 0;
    native_primitive_planarity(segment, 100, {used, cost});
    check(used == cost, "exact shared planarity budget");
    used = 0;
    rejects([&] { native_primitive_planarity(segment, 100, {used, cost - 1}); },
            "planarity work limit");
    rejects(
        [&] {
            std::size_t w = 0;
            native_primitive_planarity(spline, 3, {w, 1000000});
        },
        "source control limit");
    rejects([&] { planar(Json{{"_type", "Unknown"}}); }, "unsupported is not false planarity");
    rejects([&] { range(line({NAN, 0, 0}, {1, 0, 0}), identity); }, "nonfinite primitive");
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(std::async(std::launch::async, [&] { return planar(spline); }));
    for (auto &job : jobs)
        check(job.get() == expected, "planarity parallel state independence");
    return count;
}
