#include "native_bspline_area.hpp"
#include "native_bezier_support.hpp"
#include "native_tube.hpp"
#include <future>
using namespace p3d;
using namespace p3d::curve_detail;
namespace {
BsplineCurve curve(unsigned order, Json poles, Json weights = nullptr, bool closed = false,
                   Json knots = nullptr) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"poles", poles},
                                    {"weights", weights},
                                    {"closed", closed},
                                    {"knots", knots}});
}
bool near(double a, double b, double tolerance = 2e-10) {
    return std::abs(a - b) <= tolerance * std::max({1., std::abs(a), std::abs(b)});
}
BsplineArea area(const BsplineCurve &c) {
    std::size_t used = 0;
    return native_bspline_area(c, {used, 100000000});
}
} // namespace
unsigned native_bspline_area_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool threw = false;
        try {
            fn();
        } catch (const std::exception &) {
            threw = true;
        }
        check(threw, "B-spline area rejects exhausted budget or unsupported order");
    };
    const auto triangle = curve(2, {0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 0});
    const auto a = area(triangle);
    check(a.valid && near(a.area, 3) && a.normal == Point3{0, 0, 1},
          "closed polygon area and normal");
    check(near(a.centroid[0], 2. / 3) && near(a.centroid[1], 1) && a.centroid[2] == 0,
          "closed polygon analytic centroid");
    check(a.segments == 3 && a.edges == 3 && a.evaluations == 51 && !a.weight_fallbacks,
          "streamed polygon native integral counts");
    const auto reversed = area(curve(2, {0, 0, 0, 0, 3, 0, 2, 0, 0, 0, 0, 0}));
    check(near(reversed.area, a.area) && reversed.normal == Point3{0, 0, -1} &&
              near(reversed.centroid[0], a.centroid[0]) &&
              near(reversed.centroid[1], a.centroid[1]),
          "reverse changes normal and leaves centroid and unsigned area");
    const auto cubic = area(curve(4, {0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0}));
    check(near(cubic.area, 3. / 20) && near(cubic.centroid[0], 3. / 7) &&
              near(cubic.centroid[1], 3. / 14),
          "closed cubic exact area and centroid");
    const double w = std::sqrt(.5), pi = std::acos(-1.);
    const auto circle = curve(
        3, {1, 0, 0, w, w, 0, 0, 1, 0, -w, w, 0, -1, 0, 0, -w, -w, 0, 0, -1, 0, w, -w, 0},
        {1, w, 1, w, 1, w, 1, w}, true, {-.25, 0, 0, 0, .25, .25, .5, .5, .75, .75, 1, 1, 1.25});
    const auto ca = area(circle);
    check(near(ca.area, pi) && near(ca.centroid[0], 0) && near(ca.centroid[1], 0),
          "rational periodic circle exact area and centroid");
    check(circle.periodic_pole_shift() == -1 && ca.segments == 4 && ca.skipped_intervals == 4 &&
              ca.edges == 24 && ca.evaluations == 408,
          "periodic support shift and null spans");
    // Independent affine ellipse oracle on a plane whose normal is +X.
    Json ep = Json::array();
    for (std::size_t i = 0; i < circle.poles().size(); ++i) {
        const auto &p = circle.poles()[i];
        const auto weight = circle.weights()[i];
        ep.push_back(5 * weight);
        ep.push_back(2 * p[0] + 7 * weight);
        ep.push_back(3 * p[1] - 4 * weight);
    }
    const auto ellipse = area(curve(3, ep, circle.weights(), true, circle.knots()));
    check(near(ellipse.area, 6 * pi) && near(ellipse.normal[0], 1) &&
              near(ellipse.centroid[0], 5) && near(ellipse.centroid[1], 7) &&
              near(ellipse.centroid[2], -4),
          "translated spatial ellipse analytic area and centroid");
    const auto discontinuous =
        curve(2, {0, 0, 0, 1, 0, 0, 2, 1, 0, 2, 3, 0}, nullptr, false, {0, 0, .5, .5, 1, 1});
    const auto da = area(discontinuous);
    check(near(da.area, 2) && near(da.centroid[0], 4. / 3) && near(da.centroid[1], 4. / 3) &&
              da.segments == 2 && da.skipped_intervals == 1,
          "discontinuous support retains original incoming endpoint");
    swept_detail::TubeBudget tb;
    const auto prepared = swept_detail::prepare_tube_trace(discontinuous, tb);
    check(prepared.segments[1].poles()[0] == Point3{1, 0, 0} &&
              discontinuous.poles()[2] == Point3{2, 1, 0},
          "shared extraction preserves distinct tube endpoint replacement semantics");
    const auto open = area(curve(2, {0, 0, 0, 2, 0, 0, 2, 2, 0}));
    check(open.valid && near(open.area, 2) && near(open.centroid[0], 4. / 3) &&
              near(open.centroid[1], 2. / 3),
          "native area does not reject open boundary");
    const auto zero = area(curve(2, {3, 4, 5, 6, 4, 5}));
    check(!zero.valid && zero.area == 0 && zero.normal == Point3{} &&
              zero.centroid == Point3{3, 4, 5},
          "zero area falls back to reference point and zero normal");
    const auto zw = area(curve(2, {3, 4, 5, 6, 8, 10}, {0, 0}));
    check(!zw.valid && zw.reference_weight_fallback && zw.centroid == Point3{3, 4, 5} &&
              zw.weight_fallbacks == 17,
          "reference and Bezier zero-weight rules remain distinct");
    const auto tiny = area(curve(2, {0, 0, 0, 1e-10, 0, 0, 0, 1e-10, 0, 0, 0, 0}));
    check(tiny.valid && tiny.area > 0 && tiny.area < 1e-19,
          "B-spline visitor does not impose a polygon area cutoff");
    const auto all_null =
        area(curve(2, {3, 4, 5, 6, 4, 5}, nullptr, false, {1e9, 1e9, 1e9 + 1e-6, 1e9 + 1e-6}));
    check(!all_null.valid && all_null.segments == 0 && all_null.skipped_intervals == 1 &&
              all_null.evaluations == 0 && all_null.centroid == Point3{3, 4, 5},
          "all near-null spans retain source reference without fabricating an area");
    // Independent basis evaluation checks saturation, periodic wrap and raw
    // non-clamped support extraction; no tube endpoint replacement is involved.
    auto supports = [&](const BsplineCurve &c) {
        const auto count = c.closed() ? c.poles().size() : c.poles().size() - c.order() + 1;
        const auto domain = c.knot_domain();
        for (std::size_t i = 0; i < count; ++i) {
            const auto low = c.knots()[i + c.order() - 1], high = c.knots()[i + c.order()];
            if (bezier_support::null_interval(low, high))
                continue;
            const auto p = bezier_support::extract(c, i);
            for (double t : {.03, .19, .47, .83, .98}) {
                std::size_t used = 0;
                const auto x = native_bezier_point_tangent(p, t, {used, 1000000}).point;
                const auto y =
                    c.point_at((low + (high - low) * t - domain[0]) / (domain[1] - domain[0]));
                for (unsigned k = 0; k < 3; ++k)
                    check(near(x[k], y[k]),
                          "independent B-spline basis agrees with extracted support");
            }
        }
    };
    supports(circle);
    supports(discontinuous);
    const auto nonclamped = curve(3, {0, 0, 0, 1, 2, 0, 3, -1, 0, 5, 3, 0, 7, 1, 0}, nullptr, false,
                                  {-2, -1, 0, .3, .6, 1, 2, 3});
    supports(nonclamped);
    for (unsigned order : {2u, 4u, 8u, 16u, 26u}) {
        Json p = Json::array();
        for (unsigned i = 0; i < order + 2; ++i) {
            p.push_back(double(i));
            p.push_back(std::sin(double(i)));
            p.push_back(0.);
        }
        supports(curve(order, p));
    }
    // Polygon integration is independent of the moment kernel and its
    // derivatives. Include the reference closure of the open non-clamped curve.
    auto previous = nonclamped.point_at(0), first = previous;
    double twice = 0, mx = 0, my = 0;
    auto edge = [&](const Point3 &q) {
        const double z = previous[0] * q[1] - q[0] * previous[1];
        twice += z;
        mx += (previous[0] + q[0]) * z;
        my += (previous[1] + q[1]) * z;
        previous = q;
    };
    for (unsigned i = 1; i <= 12000; ++i)
        edge(nonclamped.point_at(double(i) / 12000));
    edge(first);
    const auto nc = area(nonclamped);
    check(near(nc.area, std::abs(twice) * .5, 2e-6) &&
              near(nc.centroid[0], mx / (3 * twice), 2e-6) &&
              near(nc.centroid[1], my / (3 * twice), 2e-6),
          "independent polygon limit for nonclamped area");
    const double boundary = 1e-14 / (1 - 1e-14);
    for (double h : {std::nextafter(boundary, 0.), boundary, std::nextafter(boundary, 1.)}) {
        const auto s = area(curve(2, {0, 0, 0, 1, 0, 0, 1, 1, 0}, nullptr, false, {0, 0, h, 1, 1}));
        check(s.segments == (((1 + h) * 1e-14 > h) ? 1 : 2), "native strict null-span threshold");
    }
    Json negative = Json::array(), nw = Json::array();
    for (std::size_t i = 0; i < circle.poles().size(); ++i) {
        for (double v : circle.poles()[i])
            negative.push_back(-v);
        nw.push_back(-circle.weights()[i]);
    }
    const auto negative_area = area(curve(3, negative, nw, true, circle.knots()));
    check(near(negative_area.area, ca.area) && near(negative_area.centroid[0], 0),
          "negative homogeneous scale retains circle region");
    std::size_t used = 0;
    native_bspline_area(circle, {used, 100000000});
    const auto required = used;
    used = 0;
    native_bspline_area(circle, {used, required});
    check(used == required, "exact shared work budget");
    rejects([&] {
        std::size_t u = 0;
        native_bspline_area(circle, {u, required - 1});
    });
    rejects([&] {
        std::size_t u = 3;
        native_bspline_area(circle, {u, 2});
    });
    rejects([&] {
        Json p = Json::array();
        for (unsigned i = 0; i < 27; ++i) {
            p.push_back(double(i));
            p.push_back(0.);
            p.push_back(0.);
        }
        area(curve(27, p));
    });
    auto f = std::async(std::launch::async, [&] { return area(circle); });
    const auto parallel = area(circle);
    const auto other = f.get();
    check(parallel.normal_sum == other.normal_sum &&
              parallel.centroid_tensor == other.centroid_tensor,
          "area calls on shared immutable curve are deterministic and independent");
    return n;
}
