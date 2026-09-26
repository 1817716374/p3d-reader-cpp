#include "native_curve_area.hpp"
#include <future>
using namespace p3d;
using namespace p3d::curve_detail;
using p3d::swept_detail::TubeBudget;
namespace {
Json group(int type, Json input) {
    Json members = Json::array();
    for (const auto &v : input)
        members.push_back({{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", members}};
}
Json poly(std::vector<Point3> p) {
    Json a = Json::array();
    for (auto v : p)
        for (double x : v)
            a.push_back(x);
    return {{"_type", "LineString"}, {"points", a}};
}
Json rectangle(double x, double y, double w, double h, bool reverse = false) {
    std::vector<Point3> p{{x, y, 0}, {x + w, y, 0}, {x + w, y + h, 0}, {x, y + h, 0}, {x, y, 0}};
    if (reverse)
        std::reverse(p.begin(), p.end());
    return group(2, {poly(p)});
}
Json line(Point3 a, Point3 b, bool spline = false) {
    if (spline)
        return {{"_type", "BsplineCurve"}, {"order", 2},
                {"closed", false},         {"poles", {a[0], a[1], a[2], b[0], b[1], b[2]}},
                {"weights", nullptr},      {"knots", nullptr}};
    Json s;
    for (unsigned i = 0; i < 3; ++i) {
        s[std::string("point0") + "XYZ"[i]] = a[i];
        s[std::string("point1") + "XYZ"[i]] = b[i];
    }
    return {{"_type", "LineSegment"}, {"segment", s}};
}
Json ellipse(double sweep) {
    return {{"_type", "EllipticArc"},
            {"arc",
             {{"centerX", 4.},
              {"centerY", -3.},
              {"centerZ", 1.},
              {"vector0X", 2.},
              {"vector0Y", 0.},
              {"vector0Z", 0.},
              {"vector90X", 0.},
              {"vector90Y", 3.},
              {"vector90Z", 0.},
              {"startRadians", 0.},
              {"sweepRadians", sweep}}}};
}
} // namespace
unsigned native_curve_area_tests() {
    unsigned count = 0;
    auto check = [&](bool ok, const char *why) {
        ++count;
        require(ok, why);
    };
    auto rejects = [&](auto fn, const char *why) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    auto near = [](double a, double b) {
        return std::abs(a - b) <= 2e-10 * std::max({1., std::abs(a), std::abs(b)});
    };
    auto evaluate = [](const Json &v) {
        TubeBudget b;
        return native_curve_vector_area(v, b);
    };
    auto flags = [](const Json &v, Point3 t = Point3{0, 0, 1}) {
        TubeBudget b;
        return native_facet_orientation_flags(v, t, b);
    };
    const auto square = rectangle(1, 2, 4, 3);
    auto a = evaluate(square);
    check(a.value.valid && a.value.area == 12 && a.value.centroid == Point3{3, 3.5, 0} &&
              a.value.normal == Point3{0, 0, 1},
          "source polyline rectangle exact area, centroid and normal");
    for (int type : {0, 1, 2, 3, -1, 6}) {
        auto v = square;
        v["type"] = type;
        a = evaluate(v);
        check(a.value.valid == (type == 2 || type == 3),
              "area dispatch uses source boundary type rather than geometric closure");
        if (type != 2 && type != 3)
            check(a.value.centroid == Point3{} && a.value.area == 0,
                  "native unsupported boundary failure leaves zero outputs");
    }
    std::vector<Point3> p{{0, 0, 0}, {2, 0, 1}, {2, 3, 3}, {0, 3, 2}, {0, 0, 0}};
    const double norm = std::sqrt(61.);
    a = evaluate(group(3, {poly(p)}));
    check(near(a.value.area, norm) && near(a.value.centroid[0], 1) &&
              near(a.value.centroid[1], 1.5) && near(a.value.centroid[2], 1.5),
          "inclined source polygon area and centroid use three-dimensional moments");
    for (unsigned i = 0; i < 3; ++i)
        check(near(a.value.normal[i], Point3{-3, -4, 6}[i] / norm),
              "inclined source normal follows edge cross product");
    for (unsigned mask = 0; mask < 16; ++mask) {
        Json edges = Json::array();
        for (unsigned i = 0; i < 4; ++i)
            edges.push_back(line(p[i], p[i + 1], (mask >> i) & 1));
        a = evaluate(group(2, edges));
        check(
            a.value.valid && near(a.value.area, norm),
            "mixed lines and B-splines share one source-group reference and interval accumulation");
        for (unsigned i = 0; i < 3; ++i)
            check(near(a.value.centroid[i], Point3{1, 1.5, 1.5}[i]),
                  "mixed primitive centroid retains common-reference moments");
    }
    const Json parabola{{"_type", "BsplineCurve"}, {"order", 3},
                        {"closed", false},         {"poles", {0., 0., 0., 1., 2., 0., 2., 0., 0.}},
                        {"weights", nullptr},      {"knots", nullptr}};
    a = evaluate(group(2, {line({-1, 0, 0}, {0, 0, 0}), parabola, line({2, 0, 0}, {-1, 0, 0})}));
    check(a.value.valid && near(a.value.area, 4. / 3) && near(a.value.centroid[0], 1) &&
              near(a.value.centroid[1], .4) && near(a.value.normal[2], -1),
          "nonlinear spline moments use the preceding source primitive's reference point");
    auto zero_reference = line({0, 0, 0}, {1, 0, 0}, true);
    zero_reference["weights"] = {0., 1.};
    a = evaluate(group(2, {zero_reference}));
    check(a.value.reference_weight_fallback && a.value.reference == Point3{},
          "source group preserves zero-weight reference query status without substituting Bezier "
          "rules");
    constexpr double pi = 3.141592653589793;
    for (double sweep : {pi, -pi, 2 * pi, -2 * pi}) {
        a = evaluate(group(2, {ellipse(sweep)}));
        const bool full = std::abs(sweep) > pi;
        check(a.value.valid && near(a.value.area, full ? 6 * pi : 3 * pi),
              "analytic ellipse visitor retains signed sector area");
        check(near(a.value.centroid[0], 4) &&
                  near(a.value.centroid[1], full ? -3 : -3 + std::copysign(4 / pi, sweep)) &&
                  near(a.value.centroid[2], 1),
              "ellipse and semicircle centroids match analytic sector formulas");
        check(near(a.value.normal[2], std::copysign(1., sweep)),
              "analytic ellipse winding determines source normal");
    }
    const auto outer = rectangle(0, 0, 10, 8), hole = rectangle(1, 2, 2, 3);
    for (bool reverse : {false, true})
        for (int type : {4, 5})
            for (bool swap : {false, true}) {
                const auto h = rectangle(1, 2, 2, 3, reverse);
                const auto region =
                    group(type, swap ? Json::array({h, outer}) : Json::array({outer, h}));
                a = evaluate(region);
                const double sign = type == 4 ? -1. : 1.;
                const double expected = 80 + sign * 6;
                check(a.value.valid && near(a.value.area, expected),
                      "parity subtracts and union adds children after native largest-normal "
                      "alignment");
                check(near(a.value.centroid[0], (80 * 5 + sign * 6 * 2) / expected) &&
                          near(a.value.centroid[1], (80 * 4 + sign * 6 * 3.5) / expected),
                      "region centroid uses signed child moments without inferring containment");
                check(a.report["largest_member"] == (swap ? 1 : 0),
                      "largest-area child is selected independently of source order");
            }
    a = evaluate(group(4, {outer, outer}));
    check(!a.value.valid && a.value.area == 0 && a.report["largest_member"] == 0,
          "equal-area parity duplicates cancel and first largest wins without deduplication");
    a = evaluate(group(5, {outer, outer}));
    check(a.value.valid && a.value.area == 160,
          "union retains duplicate native source contributions");
    auto disjoint_hole = rectangle(100, 100, 2, 3);
    a = evaluate(group(4, {outer, disjoint_hole}));
    check(a.value.valid && near(a.value.area, 74),
          "native parity arithmetic does not reinterpret geometric nesting");
    a = evaluate(group(2, {group(4, {outer, hole})}));
    check(a.value.valid && near(a.value.area, 86),
          "basic region nested groups visit their primitives directly instead of reclassifying "
          "boundaries");
    a = evaluate(group(4, {outer, group(1, {line({0, 0, 0}, {1, 0, 0})})}));
    check(!a.value.valid && a.report["failure_member"] == 1 && a.value.normal == Point3{},
          "failed child area rejects region and leaves native outputs zero");
    a = evaluate(group(2, {}));
    check(!a.value.valid, "empty region has no reference point");
    a = evaluate(group(2, {poly({{1, 0, 0}, {2, 0, 0}, {3, 0, 0}})}));
    check(!a.value.valid && a.value.centroid == Point3{1, 0, 0},
          "zero-area primitive visitor returns its source reference point");
    const auto open = group(1, {line({0, 0, 0}, {1, 0, 0})});
    auto f = flags(open);
    check(f["native_result"] == true && f["flags"] == Json::array({false}),
          "single open non-region gets a false direction flag");
    auto closed_open = square;
    closed_open["type"] = 1;
    f = flags(closed_open);
    check(f["native_result"] == false && f["flags"].empty(),
          "geometrically closed open-type source fails area-based facet orientation");
    f = flags(group(4, {outer, open, hole}));
    check(f["native_result"] == true && f["flags"] == Json::array({false, true}) &&
              f["source_rings"] == Json::array({0, 2}),
          "failed-area open parity member emits no flag and preserves original ring indices");
    f = flags(group(4, {outer, closed_open, hole}));
    check(f["native_result"] == false && f["flags"] == Json::array({false}) &&
              f["rings"].size() == 2,
          "closed area failure stops after retaining earlier flags");
    f = flags(group(4, {open, hole}));
    check(f["flags"] == Json::array({true}) && f["source_rings"] == Json::array({1}),
          "orientation uses source index rather than compacted flag index");
    for (double z : {0., 1e-14, std::nextafter(1e-14, 1.), -1., 1.}) {
        f = flags(group(4, {outer, hole}), {0, 0, z});
        check(f["flags"] == Json::array({!(z > 1e-14), z > 1e-14}),
              "source tangent is not normalized and native dot threshold is strict");
    }
    f = flags(group(4, {}));
    check(f["native_result"] == true && f["flags"].empty(),
          "empty parity source flag helper succeeds without creating flags");
    TubeBudget measured;
    const auto input = group(5, {outer, group(4, {outer, hole})});
    const auto before = input;
    const auto expected = native_curve_vector_area(input, measured);
    TubeBudget bounded;
    bounded.max_work = measured.work - 1;
    rejects([&] { native_curve_vector_area(input, bounded); },
            "recursive source area has one cumulative work budget");
    TubeBudget small;
    small.max_control_points = 2;
    rejects([&] { native_curve_vector_area(square, small); },
            "source polyline controls are bounded");
    Json deep = square;
    for (unsigned i = 0; i < 258; ++i)
        deep = group(2, {deep});
    rejects([&] { evaluate(deep); }, "source area recursion has a bounded depth");
    auto future = std::async(std::launch::async, [&] { return evaluate(input).value.centroid; });
    check(future.get() == expected.value.centroid && input == before,
          "source region area is concurrent and keeps native input immutable");
    return count;
}
