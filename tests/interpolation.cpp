#include "internal.hpp"
using namespace p3d;
namespace {
Json table(const std::vector<Point3> &points, bool closed = false) {
    Json flat = Json::array();
    for (auto p : points)
        for (double x : p)
            flat.push_back(x);
    return {{"_type", "InterpolationCurve"},
            {"order", 4},
            {"closed", closed},
            {"isChordLenKnots", 0},
            {"isColinearTangents", 0},
            {"isChordLenTangents", 0},
            {"isNaturalTangents", 0},
            {"startTangent", nullptr},
            {"endTangent", nullptr},
            {"knots", nullptr},
            {"fitPoints", flat}};
}
bool near(Point3 a, Point3 b, double e = 1e-11) {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) <= e;
}
Point3 derivative(const BsplineCurve &c, double t, unsigned degree) {
    // Independent central finite difference; use just inside the periodic seam.
    constexpr double h = 1e-4;
    const auto a = c.point_at(t - h), b = c.point_at(t), d = c.point_at(t + h);
    Point3 r{};
    for (unsigned k = 0; k < 3; ++k)
        r[k] = degree == 1 ? (d[k] - a[k]) / (2 * h) : (d[k] - 2 * b[k] + a[k]) / (h * h);
    return r;
}
} // namespace
unsigned interpolation_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto rejects = [&](const Json &v, const char *message) {
        bool failed = false;
        try {
            InterpolationCurve::from_bgfb(v);
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, message);
    };
    const auto two = InterpolationCurve::from_bgfb(table({{1, 2, 3}, {4, 8, 12}}, true));
    check(!two.bspline().closed() && two.bspline().poles().size() == 4 &&
              two.report()["endpoint_condition"] == "two_point_line",
          "two fit points force open cubic line");
    for (unsigned i = 0; i <= 100; ++i) {
        const double t = i / 100.;
        check(near(two.bspline().point_at(t), {1 + 3 * t, 2 + 6 * t, 3 + 9 * t}),
              "two point cubic evaluates to exact line");
    }
    const auto input = table({{-1, 1, 0}, {0, 0, 0}, {1, 1, 0}});
    const auto quadratic = InterpolationCurve::from_bgfb(input);
    check(quadratic.source() == input &&
              quadratic.parameters() == std::vector<double>({0, .5, 1}) &&
              quadratic.bspline().poles().size() == 5,
          "source preserved and native cubic pole count");
    auto natural_input = input;
    natural_input["isNaturalTangents"] = -2;
    const auto natural = InterpolationCurve::from_bgfb(natural_input);
    for (unsigned i = 0; i <= 100; ++i) {
        const double t = i / 100., x = 2 * t - 1, u = std::min(t, 1 - t);
        check(near(quadratic.bspline().point_at(t), {x, x * x, 0}),
              "Bessel conditions reproduce an analytic parabola");
        check(near(natural.bspline().point_at(t), {x, 1 - 3 * u + 4 * u * u * u, 0}),
              "natural spline matches independent piecewise cubic");
    }
    auto inactive = input;
    inactive["order"] = 17;
    inactive["knots"] = {91, -4, 100};
    inactive["startTangent"] = {{"x", 5}, {"y", 8}, {"z", 13}};
    inactive["endTangent"] = {{"x", -1}, {"y", -2}, {"z", -3}};
    inactive["isChordLenTangents"] = 9;
    inactive["isChordLenKnots"] = -1;
    const auto untouched = InterpolationCurve::from_bgfb(inactive);
    check(untouched.source() == inactive &&
              untouched.bspline().poles() == quadratic.bspline().poles() &&
              untouched.bspline().knots() == quadratic.bspline().knots(),
          "BGFB inactive fields preserved without changing derived cubic");
    const auto nonuniform = InterpolationCurve::from_bgfb(table({{0, 0, 0}, {1, 2, 1}, {4, 2, 0}}));
    const double middle = std::sqrt(6.) / (std::sqrt(6.) + std::sqrt(10.));
    for (unsigned i = 0; i <= 100; ++i) {
        const double t = i / 100., l1 = t * (t - 1) / (middle * (middle - 1)),
                     l2 = t * (t - middle) / (1 - middle);
        check(near(nonuniform.bspline().point_at(t), {l1 + 4 * l2, 2 * l1 + 2 * l2, l1}),
              "nonuniform Bessel curve matches independent Lagrange quadratic in three dimensions");
    }
    const std::vector<Point3> irregular{{0, 0, 0}, {1, 0, 0}, {1, 2, 0}, {-2, 2, 1}, {-3, -1, 0}};
    for (bool closed : {false, true})
        for (int chord : {0, 1})
            for (int nat : {0, 1}) {
                auto v = table(irregular, closed);
                v["isChordLenKnots"] = chord;
                v["isNaturalTangents"] = nat;
                const auto c = InterpolationCurve::from_bgfb(v);
                check(c.bspline().closed() == closed &&
                          c.report()["closure_point_appended"] == closed,
                      "closed input appends source first point with explicit provenance");
                for (std::size_t i = 0; i < irregular.size(); ++i)
                    check(near(c.bspline().point_at(c.parameters()[i]), irregular[i]),
                          "all original fit points lie on derived spline");
                if (closed) {
                    check(near(c.bspline().point_at(0), c.bspline().point_at(1)),
                          "periodic interpolation closes position");
                    // Compare derivatives extrapolated to either seam from inside the first/last
                    // cubic.
                    const double h = 1e-4;
                    auto d1 = derivative(c.bspline(), h, 1), d2 = derivative(c.bspline(), 1 - h, 1);
                    auto a1 = derivative(c.bspline(), h, 2), a2 = derivative(c.bspline(), 1 - h, 2);
                    for (unsigned k = 0; k < 3; ++k) {
                        d1[k] -= h * a1[k];
                        d2[k] += h * a2[k];
                    }
                    check(near(d1, d2, 1e-4),
                          "periodic seam preserves first derivative for nonuniform data");
                    auto b1 = derivative(c.bspline(), 2 * h, 2),
                         b2 = derivative(c.bspline(), 1 - 2 * h, 2);
                    for (unsigned k = 0; k < 3; ++k) {
                        a1[k] = 2 * a1[k] - b1[k];
                        a2[k] = 2 * a2[k] - b2[k];
                    }
                    check(near(a1, a2, 1e-5),
                          "periodic seam preserves second derivative for nonuniform data");
                }
            }
    auto square = table({{1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}}, true);
    const auto periodic = InterpolationCurve::from_bgfb(square);
    const std::vector<Point3> expected{{0, -1.5, 0}, {1.5, 0, 0}, {0, 1.5, 0}, {-1.5, 0, 0}};
    for (unsigned i = 0; i < 4; ++i)
        check(near(periodic.bspline().poles()[i], expected[i]),
              "native periodic pole rotation agrees with analytic uniform system");
    check(periodic.prepared_point_indices() == std::vector<std::size_t>({0, 1, 2, 3, 0}),
          "appended seam point maps back to source first point");
    auto marked = table({{1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}, {1, 1e-6, 0}}, true);
    marked["isChordLenKnots"] = 1;
    const auto marker = InterpolationCurve::from_bgfb(marked);
    check(marker.prepared_points().back() == Point3({1, 1e-6, 0}) &&
              !marker.report()["closure_point_appended"].get<bool>() &&
              near(marker.bspline().point_at(1), {1, 0, 0}),
          "near closure marker remains unsnapped but periodic RHS uses first point");
    auto folded = table({{0, 0, 0}, {1, 2, 0}, {0, 1e-6, 0}});
    const auto fold = InterpolationCurve::from_bgfb(folded);
    check(fold.prepared_point_indices() == std::vector<std::size_t>({0, 1}) &&
              fold.report()["ignored_points"][0]["reason"] == "native_three_point_closure",
          "three point physical closure folds even without closed flag");
    const auto triangle =
        InterpolationCurve::from_bgfb(table({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}, true));
    check(!triangle.bspline().closed() && triangle.prepared_points().size() == 4 &&
              triangle.bspline().poles().size() == 6,
          "triangle closure uses native open C2 spline");
    auto triangle_aligned = table({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}, true);
    triangle_aligned["isColinearTangents"] = 1;
    check(InterpolationCurve::from_bgfb(triangle_aligned).report()["endpoint_handles_aligned"] ==
              true,
          "end handle alignment tests prepared triangle closure after native append");
    const auto filtered = InterpolationCurve::from_bgfb(
        table({{0, 0, 0}, {1e-6, 0, 0}, {1, 0, 0}, {0, 0, 0}, {0, 1, 0}}));
    check(filtered.prepared_point_indices() == std::vector<std::size_t>({0, 2, 3, 4}),
          "only adjacent nearby points filtered without global deduplication");
    const auto threshold =
        InterpolationCurve::from_bgfb(table({{0, 0, 0}, {1e-5, 0, 0}, {1, 0, 0}}));
    check(threshold.prepared_points().size() == 3,
          "adjacent distance at native threshold retained");
    rejects(table({{0, 0, 0}, {1e-5, 0, 0}}),
            "two point endpoint check includes tolerance equality");
    rejects(table({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}),
            "coincident fit points cannot invent a curve");
    rejects(table({{0, 0, 0}}), "too few source points");
    auto bad = input;
    bad["fitPoints"][1] = "x";
    rejects(bad, "nonnumeric point");
    bad = input;
    bad["fitPoints"][1] = std::numeric_limits<double>::infinity();
    rejects(bad, "nonfinite point");
    bad = input;
    bad["fitPoints"][1] = std::numeric_limits<double>::max();
    rejects(bad, "distance overflow is not a disconnect marker");
    bad = input;
    bad["fitPoints"].push_back(1);
    rejects(bad, "malformed XYZ vector");
    std::vector<Point3> large;
    for (unsigned i = 0; i < 4998; ++i)
        large.push_back({double(i), 0, 0});
    check(InterpolationCurve::from_bgfb(table(large)).bspline().poles().size() == 5000,
          "maximum native prepared open input");
    large.push_back(large.back());
    check(InterpolationCurve::from_bgfb(table(large)).prepared_points().size() == 4998,
          "native prepared count limit applied after filtering");
    large.back()[0] += 1;
    rejects(table(large), "native prepared count limit enforced");
    auto loop = table({{.2, .2, 0}, {.8, .2, 0}, {.8, .8, 0}, {.2, .8, 0}, {.2, .2, 0}});
    const auto unaligned = InterpolationCurve::from_bgfb(loop);
    loop["isColinearTangents"] = 1;
    const auto aligned = InterpolationCurve::from_bgfb(loop);
    const auto &a = aligned.bspline().poles();
    const auto &b = unaligned.bspline().poles();
    Point3 ds{}, de{};
    for (unsigned k = 0; k < 3; ++k) {
        ds[k] = a[1][k] - a.front()[k];
        de[k] = a.back()[k] - a[a.size() - 2][k];
    }
    const double ls = std::hypot(ds[0], ds[1], ds[2]), le = std::hypot(de[0], de[1], de[2]);
    for (unsigned k = 0; k < 3; ++k) {
        ds[k] /= ls;
        de[k] /= le;
    }
    check(aligned.report()["endpoint_handles_aligned"] == true && near(ds, de),
          "native optional end handle alignment preserves direction continuity");
    check(std::abs(ls - std::hypot(b[1][0] - b[0][0], b[1][1] - b[0][1], b[1][2] - b[0][2])) <
              1e-12,
          "colinear option retains Bessel handle length");
    loop["isNaturalTangents"] = 1;
    check(!InterpolationCurve::from_bgfb(loop).report()["endpoint_handles_aligned"].get<bool>(),
          "natural conditions precede colinear option");
    return checks;
}
