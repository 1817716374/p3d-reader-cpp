#include "native_curve_xy.hpp"
#include <future>
using namespace p3d;
using namespace p3d::curve_detail;
namespace {
bool near(double a, double b) {
    return std::abs(a - b) < 3e-11;
}
BsplineCurve curve(Json poles, unsigned order = 2, Json knots = nullptr, bool closed = false) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"poles", poles},
                                    {"order", order},
                                    {"knots", knots},
                                    {"closed", closed},
                                    {"weights", nullptr}});
}
} // namespace
unsigned native_curve_xy_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool failed = false;
        try {
            fn();
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, "native curve XY rejects invalid input or budget exhaustion");
    };
    auto bezier = [](const std::vector<BezierPole> &a, const std::vector<BezierPole> &b) {
        std::size_t used = 0;
        return native_bezier_xy_intersections(a, b, 10000, {used, 100000000});
    };
    auto query = [](const BsplineCurve &a, const BsplineCurve &b) {
        std::size_t used = 0;
        return native_curve_xy_intersections(a, b, 10000, {used, 100000000});
    };
    const std::vector<BezierPole> a{{0, 0, 9, 1}, {1, 0, 9, 1}};
    const std::vector<BezierPole> b{{.5, -1, -9, 1}, {.5, 1, -9, 1}};
    const auto lines = bezier(a, b);
    check(lines.samples_a == 2 && lines.samples_b == 2 && lines.candidates == 1 &&
              lines.parameters == std::vector<Point2>{{.5, .5}},
          "two line Beziers intersect in XY even with distinct constant Z coordinates");
    check(bezier(b, a).parameters == std::vector<Point2>{{.5, .5}},
          "negative determinant orientation preserves line parameter pairs");
    check(bezier(a, a).parameters.empty() &&
              bezier(a, {{0, 1, 0, 1}, {1, 1, 0, 1}}).parameters.empty(),
          "native chordal route adds no intervals for coincident or parallel segments");
    const auto near_end = bezier(a, {{-5e-13, -1, 0, 1}, {-5e-13, 1, 0, 1}});
    check(near_end.parameters.size() == 1 && near_end.parameters[0][0] == -5e-13,
          "native line and final fraction tolerance retain a slightly extrapolated endpoint");
    check(bezier(a, {{-2e-12, -1, 0, 1}, {-2e-12, 1, 0, 1}}).parameters.empty(),
          "line candidate outside the relative determinant tolerance is excluded");
    const std::vector<BezierPole> p{{0, 0, 0, 1}, {.5, 0, 0, 1}, {1, 1, 0, 1}};
    const std::vector<BezierPole> horizontal{{0, .25, 0, 1}, {1, .25, 0, 1}};
    const auto polynomial = bezier(p, horizontal);
    check(polynomial.samples_a == 25 && polynomial.samples_b == 2 && !polynomial.parameters.empty(),
          "higher-order source gets the native fixed expanded sample grid");
    for (const auto &q : polynomial.parameters)
        check(near(q[0], .5) && near(q[1], .5),
              "chordal candidates and Newton reproduce the analytic parabola-line crossing");
    const auto rational = bezier({{0, 0, 0, 2}, {3, 0, 0, 3}}, b);
    check(rational.parameters.size() == 1 && near(rational.parameters[0][0], .4) &&
              near(rational.parameters[0][1], .5),
          "rational chordal endpoint seed is refined in the original rational parameter");
    const auto zero = bezier({{1, 2, 3, 0}, {4, 5, 6, 0}}, b);
    check(zero.parameters.empty() && zero.candidates == 0,
          "sampling uses the native zero projection before line-candidate detection");
    // T14(2t-1) in the Bernstein basis. The two transposed curves' fixed
    // sample polylines have 103 crossings (also checked with exact rationals).
    auto choose = [](unsigned count, unsigned k) {
        double c = 1;
        for (unsigned i = 1; i <= k; ++i)
            c = c * (count - i + 1) / i;
        return c;
    };
    std::vector<BezierPole> oscillating_a, oscillating_b;
    for (unsigned i = 0; i <= 14; ++i) {
        double c = choose(28, 2 * i) / choose(14, i);
        if (i % 2)
            c = -c;
        const double t = -1. + 2. * i / 14;
        oscillating_a.push_back({c, t, 0, 1});
        oscillating_b.push_back({t, c, 0, 1});
    }
    const auto capped = bezier(oscillating_a, oscillating_b);
    check(capped.candidates == 103 && capped.retained_candidates == 100 &&
              capped.parameters.size() == 100,
          "native candidate cap retains the first hundred of 103 ordered polyline intersections");
    const auto zigzag = curve({0, -1, 0, 1, 1, 0, 2, -1, 0, 3, 1, 0});
    const auto axis = curve({0, 0, 10, 3, 0, 10});
    const auto hits = query(zigzag, axis);
    check(hits.span_pairs == 3 && hits.intersections.size() == 3,
          "spline query traverses each nonempty A span with each B span");
    for (std::size_t i = 0; i < hits.intersections.size(); ++i) {
        const auto &h = hits.intersections[i];
        check(h.first_span == i && h.second_span == 0 && near(h.fractions[0], (i + .5) / 3) &&
                  near(h.fractions[1], (i + .5) / 3),
              "native span traversal returns original global fractions in source order");
    }
    const auto reverse = query(axis, zigzag);
    check(reverse.intersections.size() == 3 && reverse.intersections[2].second_span == 2,
          "reverse query swaps source span roles without sorting by geometric location");
    const auto nonunit = curve({0, -1, 0, 1, 1, 0, 2, -1, 0, 3, 1, 0}, 2, {2, 2, 4, 7, 9, 9});
    const auto domain_hits = query(nonunit, curve({0, 0, 0, 3, 0, 0}, 2, {-4, -4, 5, 5}));
    check(domain_hits.intersections.size() == 3 &&
              near(domain_hits.intersections[0].fractions[0], 1. / 7) &&
              near(domain_hits.intersections[1].fractions[0], .5) &&
              near(domain_hits.intersections[2].fractions[0], 6. / 7),
          "nonunit span intervals map to each curve's own native global fraction domain");
    const auto shared_end =
        query(curve({0, -1, 0, 1, 0, 0, 2, 1, 0, 3, 0, 0, 4, -1, 0}), curve({0, 0, 0, 4, 0, 0}));
    check(shared_end.intersections.size() == 4 &&
              shared_end.intersections[0].fractions == Point2{.25, .25} &&
              shared_end.intersections[1].fractions == Point2{.25, .25} &&
              shared_end.intersections[2].fractions == Point2{.75, .75} &&
              shared_end.intersections[3].fractions == Point2{.75, .75},
          "shared span endpoint intersections are repeated, not deduplicated");
    const auto square = curve({0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0}, 2, nullptr, true);
    const auto cut = curve({-.5, .5, 0, 1.5, .5, 0});
    const auto closed = query(square, cut);
    check(closed.intersections.size() == 2,
          "periodic support traversal includes the wrapping edge");
    for (const auto &h : closed.intersections) {
        const auto s = square.point_at(h.fractions[0]), t = cut.point_at(h.fractions[1]);
        check(near(s[0], t[0]) && near(s[1], t[1]) && near(t[1], .5),
              "periodic intersection fractions agree with independent curve evaluation");
    }
    const auto input = zigzag.poles();
    auto future = std::async(std::launch::async, [&] { return query(zigzag, axis); });
    const auto local = query(zigzag, axis), other = future.get();
    check(local.intersections.size() == other.intersections.size() && zigzag.poles() == input &&
              local.intersections.back().fractions == other.intersections.back().fractions,
          "parallel queries share source curves without mutable caches");
    std::size_t used = 0;
    native_curve_xy_intersections(zigzag, axis, 3, {used, 100000000});
    const auto required = used;
    used = 0;
    const auto bounded = native_curve_xy_intersections(zigzag, axis, 3, {used, required});
    check(bounded.intersections.size() == 3 && used == required,
          "exact intersection work/output bounds");
    rejects([&] {
        std::size_t w = 0;
        native_curve_xy_intersections(zigzag, axis, 2, {w, required});
    });
    rejects([&] {
        std::size_t w = 0;
        native_curve_xy_intersections(zigzag, axis, 3, {w, required - 1});
    });
    rejects([&] { bezier({}, b); });
    return n;
}
