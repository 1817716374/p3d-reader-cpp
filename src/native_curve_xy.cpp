// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Adapted from bezierDPoint4d.cpp, polyline3d.cpp and MSBsplineCurve_ByBezier.cpp;
// native candidate ordering/caps, knot arithmetic and bounded immutable input.
// See THIRD_PARTY.md.
#include "native_curve_xy.hpp"
#include "native_bezier_support.hpp"
namespace p3d::curve_detail {
namespace {
using bezier_support::finite;
struct Samples {
    std::vector<double> fractions;
    std::vector<Point3> points;
};
Samples sample(const std::vector<BezierPole> &p, BezierWork work) {
    require(p.size() >= 2 && p.size() <= 26, "native XY Bezier order must be 2..26");
    Samples out;
    const unsigned count = p.size() == 2 ? 2 : 25;
    for (unsigned i = 0; i < count; ++i) {
        const double u = count == 2 ? double(i) : -.1 + double(i) * .05;
        out.fractions.push_back(u);
        // Base11930 has the same |W|<=1e-12 zero projection as this point
        // evaluator, distinct from Newton's |W|<=1e-15 failure rule.
        out.points.push_back(native_bezier_point_tangent(p, u, work).point);
    }
    return out;
}
std::optional<Point2> segment_hit(const Point3 &a0, const Point3 &a1, const Point3 &b0,
                                  const Point3 &b1) {
    const double ax = finite(a1[0] - a0[0]), ay = finite(a1[1] - a0[1]);
    const double bx = finite(b1[0] - b0[0]), by = finite(b1[1] - b0[1]);
    const double rx = finite(b0[0] - a0[0]), ry = finite(b0[1] - a0[1]);
    double d = finite(finite(by * ax) - finite(bx * ay));
    double na = finite(finite(rx * by) - finite(ry * bx));
    double nb = finite(finite(rx * ay) - finite(ry * ax));
    if (d < 0) {
        d = -d;
        na = -na;
        nb = -nb;
    }
    if (d == 0)
        return std::nullopt;
    const double epsilon = finite(d * 1e-12), upper = finite(epsilon + d);
    if (na < -epsilon || na > upper || nb < -epsilon || nb > upper)
        return std::nullopt;
    return Point2{finite(na / d), finite(nb / d)};
}
double fraction(const BsplineCurve &curve, std::size_t span, double local) {
    const auto order = curve.order();
    const auto &k = curve.knots();
    const auto domain = curve.knot_domain();
    const double knot =
        finite(finite(finite(k[span + order] - k[span + order - 1]) * local) + k[span + order - 1]);
    return finite(finite(knot - domain[0]) / finite(domain[1] - domain[0]));
}
} // namespace
BezierXYIntersections native_bezier_xy_intersections(const std::vector<BezierPole> &a,
                                                     const std::vector<BezierPole> &b,
                                                     std::size_t max_output, BezierWork work) {
    const auto sa = sample(a, work), sb = sample(b, work);
    BezierXYIntersections out;
    out.samples_a = sa.points.size();
    out.samples_b = sb.points.size();
    std::vector<Point2> seeds;
    seeds.reserve(100);
    for (std::size_t i = 0; i + 1 < sa.points.size(); ++i)
        for (std::size_t j = 0; j + 1 < sb.points.size(); ++j) {
            work.charge(48);
            const auto hit =
                segment_hit(sa.points[i], sa.points[i + 1], sb.points[j], sb.points[j + 1]);
            if (!hit)
                continue;
            ++out.candidates;
            if (seeds.size() == 100)
                continue;
            seeds.push_back(
                {finite(sa.fractions[i] +
                        finite((*hit)[0] * finite(sa.fractions[i + 1] - sa.fractions[i]))),
                 finite(sb.fractions[j] +
                        finite((*hit)[1] * finite(sb.fractions[j + 1] - sb.fractions[j])))});
        }
    out.retained_candidates = seeds.size();
    for (const auto &seed : seeds) {
        const auto r = native_bezier_xy_newton(a, b, seed, work);
        if (!r.success) {
            ++out.failed_newton;
            continue;
        }
        if (std::abs(finite(r.parameters[0] - .5)) > .500000000001 ||
            std::abs(finite(r.parameters[1] - .5)) > .500000000001) {
            ++out.outside_parameters;
            continue;
        }
        require(out.parameters.size() < max_output,
                "native XY intersection output budget exceeded");
        out.parameters.push_back(r.parameters);
    }
    return out;
}
CurveXYIntersections native_curve_xy_intersections(const BsplineCurve &a, const BsplineCurve &b,
                                                   std::size_t max_output, BezierWork work) {
    auto spans = [&](const BsplineCurve &c) {
        require(c.order() >= 2 && c.order() <= 26 && c.poles().size() <= INT32_MAX,
                "native XY curve order or control count exceeded");
        const auto count = c.closed() ? c.poles().size() : c.poles().size() - c.order() + 1;
        work.charge(c.knots().size());
        return count;
    };
    const auto na = spans(a), nb = spans(b);
    CurveXYIntersections out;
    for (std::size_t i = 0; i < na; ++i) {
        work.charge(1);
        if (bezier_support::null_interval(a.knots()[i + a.order() - 1], a.knots()[i + a.order()]))
            continue;
        work.charge(std::size_t(8) * a.order() * a.order() + 24 * a.order());
        const auto pa = bezier_support::extract(a, i);
        for (std::size_t j = 0; j < nb; ++j) {
            work.charge(1);
            if (bezier_support::null_interval(b.knots()[j + b.order() - 1],
                                              b.knots()[j + b.order()]))
                continue;
            work.charge(std::size_t(8) * b.order() * b.order() + 24 * b.order());
            const auto pb = bezier_support::extract(b, j);
            const auto hits =
                native_bezier_xy_intersections(pa, pb, max_output - out.intersections.size(), work);
            ++out.span_pairs;
            out.candidates += hits.candidates;
            out.discarded_candidates += hits.candidates - hits.retained_candidates;
            out.failed_newton += hits.failed_newton;
            out.outside_parameters += hits.outside_parameters;
            for (const auto &p : hits.parameters) {
                work.charge(16);
                out.intersections.push_back({{fraction(a, i, p[0]), fraction(b, j, p[1])}, i, j});
            }
        }
    }
    return out;
}
} // namespace p3d::curve_detail
