#include "native_tube.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
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
bool near(const Point3 &a, const Point3 &b, double tolerance = 1e-10) {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) <= tolerance;
}
} // namespace
unsigned native_tube_path_tests() {
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
    auto verify_trace = [&](const BsplineCurve &trace) {
        TubeBudget b;
        const auto p = prepare_tube_trace(trace, b);
        const auto domain = trace.knot_domain();
        for (std::size_t i = 0; i < p.segments.size(); ++i) {
            const auto interval = p.report["segments"][i]["source_knot_interval"];
            const double low = interval[0], high = interval[1];
            for (double t : {0., .03, .23, .5, .82, 1.}) {
                const double fraction =
                    (low + (high - low) * t - domain[0]) / (domain[1] - domain[0]);
                check(near(p.segments[i].point_at(t), trace.point_at(fraction)),
                      "native Bezier preparation matches independent source B-spline basis "
                      "evaluation");
            }
            check(p.segments[i].knots().front() == 0 && p.segments[i].knots().back() == 1 &&
                      p.segments[i].poles().size() == trace.order() && !p.segments[i].closed(),
                  "trace callbacks receive normalized single Beziers with source order");
        }
        return p;
    };
    const auto quadratic = curve(3, {0, 0, 0, 1, 2, 1, 3, -1, 2, 5, 3, 4, 7, 1, 5}, nullptr, false,
                                 {2, 2, 2, 3, 6, 9, 9, 9});
    const auto q = verify_trace(quadratic);
    check(q.segments.size() == 3 && q.report["skipped_intervals"] == 0,
          "nonuniform non-unit source domain preserves three distinct intervals");
    for (unsigned order : {2u, 5u, 10u, 26u}) {
        Json poles = Json::array();
        for (unsigned i = 0; i < order + 2; ++i) {
            poles.push_back(double(i));
            poles.push_back(std::sin(double(i)));
            poles.push_back(.1 * i * i);
        }
        verify_trace(curve(order, poles));
    }
    verify_trace(curve(4, {0, 0, 0, 1, 2, 1, 2, -1, 3, 3, 1, 4, 4, 3, 2, 5, 2, 6, 7, 1, 5}, nullptr,
                       false, {0, 0, 0, 0, .2, .7, .7, 1, 1, 1, 1}));
    verify_trace(curve(3, {0, 0, 0, 1, 2, 1, 3, -1, 2, 5, 3, 4, 7, 1, 5}, nullptr, false,
                       {-2, -1, 0, .3, .6, 1, 2, 3}));
    verify_trace(curve(3, {2, 0, 1, 0, 3, 2, -2, 0, 3, 0, -3, 0}, nullptr, true));
    const auto rational = curve(3, {0, 0, 0, 2, 4, 2, 9, -3, 6, 10, 6, 8, 7, 1, 5}, {1, 2, 3, 2, 1},
                                false, {2, 2, 2, 3, 6, 9, 9, 9});
    verify_trace(rational);
    verify_trace(curve(3, {0, 0, 0, -2, -4, -2, -9, 3, -6, -10, -6, -8, -7, -1, -5},
                       {-1, -2, -3, -2, -1}, false, {2, 2, 2, 3, 6, 9, 9, 9}));
    const double w = std::sqrt(.5);
    const auto circle = curve(
        3, {1, 0, 0, w, w, 0, 0, 1, 0, -w, w, 0, -1, 0, 0, -w, -w, 0, 0, -1, 0, w, -w, 0},
        {1, w, 1, w, 1, w, 1, w}, true, {-.25, 0, 0, 0, .25, .25, .5, .5, .75, .75, 1, 1, 1.25});
    const auto cp = verify_trace(circle);
    check(circle.periodic_pole_shift() == -1 && cp.segments.size() == 4 &&
              cp.report["skipped_intervals"] == 4 &&
              cp.segments.front().poles().front() == cp.segments.back().poles().back(),
          "clamped-like periodic convention shifts poles and wraps the closing circle segment");
    const double boundary = 1e-14 / (1 - 1e-14);
    for (double h : {std::nextafter(boundary, 0.), boundary, std::nextafter(boundary, 1.)}) {
        TubeBudget b;
        const auto p = prepare_tube_trace(
            curve(2, {0, 0, 0, 0, 0, 1, 0, 0, 3}, nullptr, false, {0, 0, h, 1, 1}), b);
        const bool null = ((1. + h) * 1e-14 > h);
        check(p.segments.size() == (null ? 1 : 2),
              "strict native null-interval threshold is retained");
        if (null)
            check(p.segments.front().poles().front() == Point3{0, 0, 1},
                  "skipped leading interval does not inject original first control");
    }
    TubeBudget shifted_budget;
    auto skipped = prepare_tube_trace(curve(2, {0, 0, 0, 0, 0, 1, 0, 0, 3}, nullptr, false,
                                            {1e9, 1e9, 1e9 + 1e-6, 1e9 + 1, 1e9 + 1}),
                                      shifted_budget);
    check(skipped.segments.size() == 1,
          "large raw knot origin changes native near-null classification before normalization");
    const auto jump =
        curve(2, {0, 0, 0, 1, 0, 0, 9, 0, 0, 10, 0, 0}, nullptr, false, {0, 0, .5, .5, 1, 1});
    TubeBudget jump_budget;
    const auto jp = prepare_tube_trace(jump, jump_budget);
    check(jp.segments[1].poles().front() == Point3{1, 0, 0} &&
              jp.report["replaced_leading_controls"].size() == 1 &&
              jp.report["replaced_leading_controls"][0]["incoming_homogeneous"] ==
                  Json({9, 0, 0, 1}) &&
              jump.point_at(.5) == Point3{9, 0, 0},
          "prepareCurve shares previous end across full-multiplicity discontinuity without editing "
          "source");
    TubeBudget signed_budget;
    const auto signed_jump = prepare_tube_trace(curve(2, {0, 0, 0, 2, 0, 0, -3, 0, 0, -6, 0, 0},
                                                      {2, 2, -3, -3}, false, {0, 0, .5, .5, 1, 1}),
                                                signed_budget);
    check(signed_jump.segments[1].weights()[0] == 2 && signed_jump.segments[1].weights()[1] == -3,
          "shared leading homogeneous control retains previous weight even across a sign change");
    TubeBudget zero_budget;
    const auto zp = prepare_tube_trace(curve(2, {0, 0, 0, 1, 0, 0}, {0, 1}), zero_budget);
    check(zp.segments[0].weights()[0] == 0,
          "trace preparation preserves zero homogeneous weight before downstream geometry checks");
    const auto section = curve(2, {.1, 0, 0, .2, .3, 0});
    TubeBudget cb;
    const auto tube = tube_surface(section, circle, false, cb);
    const auto s = BsplineSurface::from_bgfb(tube.surface);
    check(s.v().pole_count() == 9 && !s.v().closed() && tube.report["patches"].size() == 4 &&
              tube.report["joins"].size() == 3 && !tube.report["joins"][2]["closure"].is_null(),
          "periodic trace produces four patches and one final closure through the real pipeline");
    for (double u : {0., .27, .6, 1.})
        for (double v : {0., .04, .25, .42, .5, .67, .75, .92, 1.}) {
            const auto c = circle.point_at(v);
            const double radius = .9 - .1 * u;
            check(
                near(s.point_at(u, v), {radius * c[0], radius * c[1], .3 * u}),
                "closed circular sweep equals independent radial-offset circle and axial profile");
        }
    TubeBudget line_budget;
    const auto straight = tube_surface(
        section, curve(2, {0, 0, 0, 0, 0, 1, 0, 0, 5}, nullptr, false, {2, 2, 3, 8, 8}), false,
        line_budget);
    const auto ls = BsplineSurface::from_bgfb(straight.surface);
    for (double u : {0., .3, 1.})
        for (double v : {0., .2, .5, .8, 1.}) {
            const double z = v <= .5 ? 2 * v : 1 + 8 * (v - .5);
            check(near(ls.point_at(u, v), {.1 + .1 * u, .3 * u, z}),
                  "source unequal path spans become native equal segment intervals, not arclength "
                  "intervals");
        }
    TubeBudget single_budget;
    const auto singleton = tube_surface(section, curve(2, {0, 0, 0, 0, 0, 1}), true, single_budget);
    check(singleton.report["patches"].size() == 1 && singleton.report["joins"].empty(),
          "single Bezier route does not fabricate an assembly callback");
    TubeBudget single_closed_budget;
    const auto single_closed =
        tube_surface(section, curve(2, {0, 0, 0, 0, 0, 1}, nullptr, true, {-1, 0, 0, 1, 1}), false,
                     single_closed_budget);
    const auto one_closed = BsplineSurface::from_bgfb(single_closed.surface);
    check(single_closed.report["trace_preparation"]["source_closed"] == true &&
              single_closed.report["joins"].empty() &&
              !near(one_closed.point_at(0, 0), one_closed.point_at(0, 1)),
          "one-span closed source does not bypass native first-copy branch to weld its endpoints");
    auto concurrent = std::async(std::launch::async, [&] {
        TubeBudget b;
        return tube_surface(section, circle, false, b).surface;
    });
    check(concurrent.get() == tube.surface,
          "complete B-spline tube pipeline is independently thread safe");
    rejects(
        [&] {
            TubeBudget b;
            prepare_tube_trace(curve(2, {0, 0, 0, 1, 0, 0}, nullptr, false, {0, 0, 1e-16, 1e-16}),
                               b);
        },
        "all null intervals reject instead of exposing uninitialized native output");
    rejects(
        [&] {
            TubeBudget b;
            b.max_control_points = 8;
            prepare_tube_trace(quadratic, b);
        },
        "expanded Bezier controls respect allocation budget");
    rejects(
        [&] {
            TubeBudget b;
            b.max_work = 10;
            prepare_tube_trace(quadratic, b);
        },
        "extraction workload is charged before control allocation");
    rejects(
        [&] {
            TubeBudget b;
            b.max_control_points = 17;
            tube_surface(section, circle, false, b);
        },
        "full tensor grid is bounded before patch generation");
    rejects(
        [&] {
            TubeBudget b;
            b.max_work = cb.work - 1;
            tube_surface(section, circle, false, b);
        },
        "trace preparation, frame evaluation, patches and joins share one budget");
    return n;
}
