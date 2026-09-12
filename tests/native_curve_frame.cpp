#include "internal.hpp"
#include <future>
#include <random>
using namespace p3d;
namespace {
Json point(Point3 p, const std::string &prefix) {
    return {{prefix + "X", p[0]}, {prefix + "Y", p[1]}, {prefix + "Z", p[2]}};
}
Json line(Point3 a, Point3 b) {
    auto s = point(a, "point0");
    s.update(point(b, "point1"));
    return {{"_type", "LineSegment"}, {"segment", s}};
}
Json poly(Json p, const char *type = "LineString") {
    return {{"_type", type}, {"points", p}};
}
Json arc(double sweep = 1.5707963267948966) {
    auto a = point({2, 3, 4}, "center");
    a.update(point({1, 0, 0}, "vector0"));
    a.update(point({0, 1, 0}, "vector90"));
    a.update({{"startRadians", 0}, {"sweepRadians", sweep}});
    return {{"_type", "EllipticArc"}, {"arc", a}};
}
Json array(std::initializer_list<Json> children) {
    Json j{{"_type", "CurveVector"}, {"type", 1}, {"curves", Json::array()}};
    for (const auto &c : children)
        j["curves"].push_back({{"geometry", c}});
    return j;
}
Json bsp(unsigned order, Json poles, Json weights = nullptr, Json knots = nullptr,
         bool closed = false) {
    return {{"_type", "BsplineCurve"}, {"order", order}, {"poles", poles},
            {"weights", weights},      {"knots", knots}, {"closed", closed}};
}
Point3 column(const Json &f, unsigned c) {
    const auto &m = f.at("frame");
    return {m[0][c], m[1][c], m[2][c]};
}
bool near(Point3 a, Point3 b, double tol = 2e-11) {
    for (unsigned i = 0; i < 3; ++i)
        if (std::abs(a[i] - b[i]) > tol * std::max({1., std::abs(a[i]), std::abs(b[i])}))
            return false;
    return true;
}
Point3 cross(Point3 a, Point3 b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
Point3 unit(Point3 a) {
    auto n = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (n == 0)
        return {1, 0, 0};
    for (auto &x : a)
        x /= n;
    return a;
}
Json decoded_primitive(std::uint8_t tag, std::initializer_list<double> data) {
    Bytes b(56 + data.size() * 8);
    std::memcpy(b.data(), "bg0001fb", 8);
    auto write = [&](std::size_t at, auto v) { std::memcpy(b.data() + at, &v, sizeof(v)); };
    write(8, std::uint32_t(12));
    write(12, std::uint16_t(8));
    write(14, std::uint16_t(12));
    write(16, std::uint16_t(4));
    write(18, std::uint16_t(8));
    write(20, std::int32_t(8));
    b[24] = tag;
    write(28, std::uint32_t(20));
    write(32, std::uint16_t(6));
    write(34, std::uint16_t(8 + data.size() * 8));
    write(36, std::uint16_t(8));
    write(48, std::int32_t(16));
    std::size_t at = 56;
    for (auto x : data) {
        write(at, x);
        at += 8;
    }
    return decode_bgfb(b).at("geometry");
}
} // namespace
unsigned native_curve_frame_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    const auto x = line({2, 3, 4}, {5, 3, 4});
    const auto decoded_line = decoded_primitive(1, {2, 3, 4, 5, 3, 4});
    const auto decoded_arc =
        decoded_primitive(2, {2, 3, 4, 1, 0, 0, 0, 1, 0, 0, 1.5707963267948966});
    check(native_curve_frame(array({decoded_line}), 2) == native_curve_frame(array({x}), 2),
          "line frame accepts the actual decoded BGFB struct fields");
    check(native_curve_frame(array({decoded_arc}), 2) == native_curve_frame(array({arc()}), 2),
          "arc frame accepts the actual decoded BGFB struct fields");
    for (int preference : {0, 1, 2, -1, 3}) {
        const auto f = native_curve_frame(array({x}), preference);
        check(f["status"] == "computed" && near(column(f, 0), {1, 0, 0}) &&
                  near(column(f, 1), {0, 1, 0}) && near(column(f, 2), {0, 0, 1}) &&
                  near(column(f, 3), {2, 3, 4}),
              "line reference frame and native preference fallback");
        check(f["method"] == (preference == 1 ? "endpoint_axis_frame" : "primitive_segment_frame"),
              "parallel endpoints fall through except in preference one");
    }
    auto f = native_curve_frame(array({arc()}), 2);
    check(f["method"] == "endpoint_tangents_frame" && near(column(f, 0), {0, 1, 0}) &&
              near(column(f, 1), {-1, 0, 0}) && near(column(f, 2), {0, 0, 1}) &&
              near(column(f, 3), {3, 3, 4}),
          "arc endpoint frame columns and origin");
    f = native_curve_frame(array({arc()}));
    check(f["method"] == "local_derivatives_frame" && f["square_normalization_success"] == true &&
              near(column(f, 1), {-1, 0, 0}),
          "local arc derivatives take priority over primitive frame");
    f = native_curve_frame(array({arc(-1.5707963267948966)}));
    check(near(column(f, 0), {0, -1, 0}) && near(column(f, 2), {0, 0, -1}),
          "negative sweep preserves native frame handedness");
    f = native_curve_frame(array({x, line({2, 5, 4}, {5, 5, 4})}), 2);
    check(f["method"] == "later_origin_frame" && near(column(f, 1), {0, 1, 0}) &&
              f["source_paths"] == Json({"/curves/0/geometry", "/curves/1/geometry"}),
          "later child displacement precedes later tangent and records both sources");
    f = native_curve_frame(array({x, line({2, 3, 4}, {2, 3, 9})}));
    check(f["method"] == "later_derivatives_frame" && near(column(f, 1), {0, 0, 1}) &&
              near(column(f, 2), {0, -1, 0}),
          "coincident child origins can use distinct tangents");
    const auto nested = array({array({line({9, 8, 7}, {9, 8, 10})}), x});
    f = native_curve_frame(nested, 1);
    check(f["method"] == "endpoint_axis_frame" && near(column(f, 3), {2, 3, 4}),
          "nested array supplies no endpoint callback to its parent");
    f = native_curve_frame(nested, 2);
    check(f["method"] == "primitive_segment_frame" && near(column(f, 3), {9, 8, 7}) &&
              f["source_paths"] == Json({"/curves/0/geometry/curves/0/geometry"}),
          "nested arrays run local preference zero in original order");
    f = native_curve_frame(array({x, array({line({2, 5, 4}, {5, 5, 4})})}));
    check(f["method"] == "primitive_segment_frame" && near(column(f, 3), {2, 5, 4}),
          "cross-child derivative search does not flatten nested arrays");
    f = native_curve_frame(array({array({}), x}));
    check(f["status"] == "computed" && near(column(f, 3), {2, 3, 4}),
          "empty nested array falls through");
    for (const char *kind : {"LineString", "PointString"}) {
        f = native_curve_frame(array({poly({0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 3, 2}, kind)}));
        check(f["method"] == "primitive_polyline_frame" && near(column(f, 0), {0, 0, 1}) &&
                  near(column(f, 1), {0, 1, 0}) && near(column(f, 2), {-1, 0, 0}),
              "polyline final frame skips coincident initial points and searches its own plane");
        f = native_curve_frame(array({poly({0, 0, 0, 1e-12, 0, 0, 0, 0, 2}, kind)}));
        check(near(column(f, 0), {0, 0, 1}),
              "almost equal point uses native scale-dependent tolerance");
        f = native_curve_frame(array({poly({0, 0, 0, 1e-8, 0, 0, 0, 0, 2}, kind)}));
        check(near(column(f, 0), {1, 0, 0}) && near(column(f, 1), {0, 0, 1}),
              "distinct initial point is retained for reference plane");
        f = native_curve_frame(array({poly({2, 3, 4}, kind)}), 1);
        check(f["status"] == "computed" && near(column(f, 0), {1, 0, 0}),
              "single point endpoint normalization uses X");
        f = native_curve_frame(array({poly({2, 3, 4}, kind)}), 2);
        check(f["status"] == "native_failure" && !f.contains("frame"),
              "single point has no final polyline frame");
        f = native_curve_frame(array({poly({1, 1, 1, 1, 1, 1}, kind)}));
        check(f["status"] == "native_failure", "fully coincident polyline final frame fails");
    }
    f = native_curve_frame(array({line({8, 9, 10}, {8, 9, 10})}));
    check(f["status"] == "computed" && f["basis_status"] == "nondegenerate" &&
              near(column(f, 3), {8, 9, 10}),
          "zero-length segment explicitly uses translated identity");
    f = native_curve_frame(array({arc(0)}));
    check(f["status"] == "computed" && f["basis_status"] == "degenerate" &&
              f["square_normalization_success"] == false && near(column(f, 0), {0, 0, 0}),
          "generic zero-sweep arc preserves failed normalization matrix and native success");
    auto cubic = bsp(4, {0, 0, 0, 1. / 3, 0, 0, 2. / 3, 1. / 3, 0, 1, 1, 1});
    f = native_curve_frame(array({cubic}));
    check(f["method"] == "local_derivatives_frame" && near(column(f, 0), {1, 0, 0}) &&
              near(column(f, 1), {0, 1, 0}),
          "B-spline local derivatives are evaluated independently of endpoints");
    f = native_curve_frame(array({bsp(2, {0, 0, 0, 1, 0, 0})}));
    check(f["method"] == "primitive_axis_fallback_frame",
          "B-spline final fallback uses its own frame rules");
    const auto zero_weight = bsp(2, {3, 4, 5, 5, 7, 9}, {0, 0});
    f = native_curve_frame(array({zero_weight}), 1);
    check(f["status"] == "computed" && near(column(f, 3), {3, 4, 5}) &&
              near(column(f, 0), unit({2, 3, 4})),
          "endpoint blending replaces evaluated zero weight with one");
    f = native_curve_frame(array({zero_weight}), 0);
    check(f["status"] == "not_evaluated" && !f.contains("frame"),
          "undefined derivative weight arithmetic is not a verified native callback failure");
    const auto narrow = bsp(2, {0, 0, 0, 1, 0, 0}, nullptr, {0, 0, 1e-15, 1e-15});
    check(native_curve_frame(array({narrow}), 1)["status"] == "computed",
          "endpoint kernel has no derivative knot tolerance");
    check(native_curve_frame(array({narrow}), 0)["status"] == "native_failure",
          "verified native knot failure permits final failure");
    const double s = std::sqrt(3.) / 2;
    const auto special =
        bsp(3, {1, 0, 0, .5, s, 0, -.5, s, 0, -1, 0, 0, -.5, -s, 0, .5, -s, 0, 1.1, 0, 0},
            {1, .5, 1, .5, 1, .5, 1},
            {-1. / 3, 0, 0, 0, 1. / 3, 1. / 3, 2. / 3, 2. / 3, 1, 1, 1, 4. / 3}, true);
    f = native_curve_frame(array({special}), 1);
    check(f["status"] == "computed" && near(column(f, 3), {1, 0, 0}) &&
              near(column(f, 0), {0, 1, 0}),
          "closed endpoint query retains knot-only shift despite failed pole seam check");
    f = native_curve_frame(array({special}), 0);
    check(f["status"] == "computed" && near(column(f, 3), {1, 2 * s, 0}),
          "same closed source uses distinct native pole check in local derivative search");
    f = native_curve_frame(
        array({bsp(3, {0, 0, 0, 0, 0, 0, 1, 0, 0}), line({0, 0, 2}, {0, 1, 2})}));
    check(f["method"] == "later_origin_frame" && near(column(f, 0), {1, 0, 0}) &&
              near(column(f, 1), {0, 0, 1}),
          "stationary initial point can choose second derivative as reference direction");
    f = native_curve_frame(
        array({bsp(4, {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0}), line({0, 2, 0}, {0, 2, 1})}));
    check(f["method"] == "later_origin_frame" && near(column(f, 0), {1, 0, 0}) &&
              near(column(f, 1), {0, 1, 0}),
          "first significant third derivative participates in later-child search");
    f = native_curve_frame(array({poly({1e8, 0, 0, 1e8 + 1e-4, 0, 0, 1e8, 0, 2})}));
    check(near(column(f, 0), {0, 0, 1}),
          "native almost-equal point threshold depends on coordinates as well as separation");
    const Json akima{{"_type", "AkimaCurve"},
                     {"points", {-1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 2, 0, 1, 3, 0, 1, 3, 1}}};
    f = native_curve_frame(array({akima}), 2);
    check(f["status"] == "computed" && near(column(f, 3), {1, 0, 0}),
          "Akima uses its native converted curve endpoints");
    const Json interpolation{{"_type", "InterpolationCurve"},
                             {"order", 4},
                             {"closed", false},
                             {"isChordLenKnots", 0},
                             {"isColinearTangents", 0},
                             {"isChordLenTangents", 0},
                             {"isNaturalTangents", 0},
                             {"startTangent", nullptr},
                             {"endTangent", nullptr},
                             {"knots", nullptr},
                             {"fitPoints", {1, 2, 3, 3, 4, 4, 7, 4, 5}}};
    f = native_curve_frame(array({interpolation}), 2);
    check(f["status"] == "computed" && near(column(f, 3), {1, 2, 3}),
          "interpolation curve conversion feeds the frame selector");
    const Json transform{{"axx", 1}, {"axy", 0}, {"axz", 0}, {"axw", 5}, {"ayx", 0}, {"ayy", 1},
                         {"ayz", 0}, {"ayw", 6}, {"azx", 0}, {"azy", 0}, {"azz", 1}, {"azw", 7}};
    const Json spiral{{"_type", "TransitionSpiral"},
                      {"detail",
                       {{"transform", transform},
                        {"fractionA", 0.},
                        {"fractionB", 1.},
                        {"bearing0Radians", .3},
                        {"bearing1Radians", 1.32},
                        {"curvature0", .2},
                        {"curvature1", 1.},
                        {"spiralType", 10},
                        {"constructionHint", 7}}},
                      {"extraData", {3, 9}},
                      {"directDetail", nullptr}};
    f = native_curve_frame(array({spiral}), 2);
    check(f["status"] == "computed" && near(column(f, 3), {5, 6, 7}),
          "spiral frame uses the transformed native B-spline fit");
    const auto unknown = Json{{"_type", "UnknownCurve"}};
    f = native_curve_frame(array({arc(), unknown}), 0);
    check(f["status"] == "computed",
          "unvisited later primitives do not invalidate an earlier local selection");
    f = native_curve_frame(array({arc(), unknown}), 2);
    check(f["status"] == "not_evaluated" && f["source_path"] == "/curves/1/geometry",
          "endpoint pass cannot silently skip an unknown final tangent");
    check(native_curve_frame(array({nullptr, arc()}))["status"] == "computed",
          "null outer child is skipped");
    check(native_curve_frame(array({x, nullptr}))["status"] == "not_evaluated",
          "native undefined later-null callback is explicit");
    check(native_curve_frame(array({}))["status"] == "native_failure",
          "empty array reports native failure");
    check(native_curve_frame(x)["status"] == "not_evaluated", "root input must be a CurveVector");
    auto deep = array({x});
    for (unsigned i = 0; i < 82; ++i)
        deep = array({deep});
    check(native_curve_frame(deep)["status"] == "not_evaluated",
          "nested source cannot exhaust the stack");
    auto large = line({0, 0, 0}, {1e300, 1e300, 1e300});
    check(native_curve_frame(array({large}), 1)["status"] == "not_evaluated",
          "overflow never becomes a fabricated frame");
    // Independent rational Bezier endpoint identities across every native
    // order: no recursive basis or derivative implementation in the oracle.
    std::mt19937_64 random(73019);
    std::uniform_real_distribution<double> coordinate(-10, 10), weight(.2, 3);
    for (unsigned order = 2; order <= 26; ++order) {
        for (unsigned sample = 0; sample < 12; ++sample) {
            Json p = Json::array(), w = Json::array();
            std::vector<Point3> cart(order);
            std::vector<double> weights(order);
            for (unsigned i = 0; i < order; ++i) {
                weights[i] = weight(random);
                w.push_back(weights[i]);
                for (unsigned a = 0; a < 3; ++a) {
                    cart[i][a] = coordinate(random);
                    p.push_back(cart[i][a] * weights[i]);
                }
            }
            Point3 a{}, b{};
            for (unsigned i = 0; i < 3; ++i) {
                a[i] = (cart[1][i] - cart[0][i]) * weights[1] / weights[0];
                b[i] = (cart[order - 1][i] - cart[order - 2][i]) * weights[order - 2] /
                       weights[order - 1];
            }
            a = unit(a);
            b = unit(b);
            f = native_curve_frame(array({bsp(order, p, w)}), 2);
            if (order == 2) {
                check(f["status"] == "computed",
                      "rational degree-one endpoint frame falls through safely");
                continue;
            }
            const auto normal = unit(cross(a, b));
            check(f["method"] == "endpoint_tangents_frame" && near(column(f, 0), a) &&
                      near(column(f, 2), normal) && near(column(f, 1), cross(normal, a)) &&
                      near(column(f, 3), cart[0]),
                  "rational Bezier endpoint frame matches analytic identities");
        }
    }
    const auto before = nested.dump();
    const auto expected = native_curve_frame(nested, 2);
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 6; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return native_curve_frame(nested, 2); }));
    for (auto &job : jobs)
        check(job.get() == expected, "reference frame queries have no shared mutable state");
    check(nested.dump() == before, "native curve frame preserves the source tree");
    return checks;
}
