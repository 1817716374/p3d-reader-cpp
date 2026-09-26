#include "native_tube_facet_patch.hpp"
#include "native_curve_affine.hpp"
#include "bspline_frame.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineCurve curve(unsigned order, Json xyz, Json weights = nullptr, bool closed = false) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"closed", closed},
                                    {"poles", xyz},
                                    {"weights", weights},
                                    {"knots", nullptr}});
}
bool near(double a, double b) {
    return std::abs(a - b) < 2e-11 * (1 + std::abs(b));
}
bool near(const Point3 &a, const Point3 &b) {
    return near(a[0], b[0]) && near(a[1], b[1]) && near(a[2], b[2]);
}
const Matrix3 xy_frame{{{0, 1, 0}, {0, 0, 1}, {1, 0, 0}}};
} // namespace
unsigned native_tube_facet_patch_tests() {
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
    auto generate = [&](const BsplineCurve &section, const BsplineCurve &path, bool rigid = false) {
        TubeBudget b;
        return generate_tube_facet_patch(section, path, xy_frame, rigid, b);
    };
    const auto section = curve(2, {1., 2., 3., -2., 4., -1.});
    const auto line = curve(2, {2., 3., 4., 7., 3., 4.});
    auto straight = generate(section, line);
    check(straight.success && straight.surface && straight.report["ruled_fallback"] == true,
          "zero-curvature facet builds ruled fallback");
    const auto ss = BsplineSurface::from_bgfb(*straight.surface);
    check(ss.poles() == std::vector<Point3>{{5, 4, 6}, {1, 1, 8}, {10, 4, 6}, {6, 1, 8}},
          "ruled profile keeps all local coordinates and source start translation");
    check(straight.final_frame == xy_frame && (*straight.surface)["numRulesU"] == 2 &&
              (*straight.surface)["numRulesV"] == 2,
          "ruled branch retains incoming frame and sets rule counts");
    check(near(ss.point_at(.25, .4), Point3{6, 3.25, 6.5}),
          "ruled surface independently evaluates expected affine interpolation");
    const auto rational_section = curve(2, {2., 4., 6., -6., 12., -3.}, {2., 3.});
    auto rational_straight = generate(rational_section, line);
    const auto rss = BsplineSurface::from_bgfb(*rational_straight.surface);
    check(rss.weights() == std::vector<double>{2, 3, 2, 3} &&
              rss.poles() == std::vector<Point3>{{10, 8, 12}, {3, 3, 24}, {20, 8, 12}, {18, 3, 24}},
          "ruled output translates homogeneous profile and duplicates its weights");
    const auto rational_line = curve(2, {4., 6., 8., 21., 9., 12.}, {2., 3.});
    const auto rl = generate(section, rational_line);
    check(rl.success && (*rl.surface)["weights"].is_null() &&
              BsplineSurface::from_bgfb(*rl.surface).poles() == ss.poles(),
          "ruled path endpoint weights affect displacement but not surface rationality");
    const auto closed_section = curve(2, {0., 0., 0., 1., 0., 0., 1., 1., 0.}, nullptr, true);
    const auto closed = generate(closed_section, line);
    check((*closed.surface)["closedU"] == true &&
              (*closed.surface)["knotsU"] == Json(closed_section.knots()),
          "ruled fallback retains native periodic section knot storage");
    const auto zero_weight_profile = curve(2, {1., 2., 3., 4., 5., 6.}, {0., 2.});
    const auto zero_weight_ruled = generate(zero_weight_profile, line);
    const auto zwr = BsplineSurface::from_bgfb(*zero_weight_ruled.surface);
    check(zwr.weights() == std::vector<double>{0, 2, 0, 2} && zwr.poles()[0] == zwr.poles()[2],
          "ruled generation retains zero section weights without unweighting");
    const Matrix3 identity{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    TubeBudget identity_budget;
    const auto near_identity = generate_tube_facet_patch(
        section, curve(2, {5e-11, 0., 0., 5e-11, 0., 1.}), identity, false, identity_budget);
    check(BsplineSurface::from_bgfb(*near_identity.surface).poles().front() ==
              section.poles().front(),
          "ruled placement retains native near-identity transform skip");
    // Guarded GePoint3d division keeps extreme endpoint coordinates unchanged.
    const auto huge_line = curve(2, {2e12, 0., 0., 2e12 + 10., 0., 0.}, {2., 2.});
    const auto huge = generate(curve(2, {0., 0., 0., 0., 0., 0.}), huge_line);
    check(BsplineSurface::from_bgfb(*huge.surface).poles().front()[0] == 2e12 &&
              BsplineSurface::from_bgfb(*huge.surface).poles().back()[0] == 2e12 + 10.,
          "ruled endpoint deweighting uses guarded division at equality");
    const auto parabola = curve(3, {0., 0., 0., .5, 0., 0., 1., 1., 0.});
    TubeBudget prep_budget;
    auto prepared = prepare_tube_facet_trace(parabola, prep_budget);
    for (bool rigid : {false, true}) {
        const auto patch = generate(section, parabola, rigid);
        const auto surf = BsplineSurface::from_bgfb(*patch.surface);
        check(patch.success && patch.report["ruled_fallback"] == false &&
                  surf.poles().size() == 2 * prepared.trace->poles().size(),
              "adaptive patch uses all curvature-refined rows");
        check((*patch.surface)["numRulesU"] == 3 &&
                  (*patch.surface)["numPolesV"].get<std::size_t>() > 3,
              "native U rule count comes from original trace, not refined row count");
        // Independent analytic parabola frame. In this plane B=Z and the shared
        // frame follows analytic N,B,T in both transport modes. Normalization
        // and its later multiplication cancel algebraically in this oracle.
        for (std::size_t i = 0; i < prepared.greville.size(); ++i) {
            const double t = prepared.greville[i][0], k = prepared.greville[i][1];
            const double length = std::sqrt(1 + 4 * t * t);
            const auto &p = prepared.trace->poles()[i];
            const Point3 normal{-2 * t / length, 1 / length, 0};
            const Point3 adjusted{normal[0] - (p[0] - t) * k, normal[1] - (p[1] - t * t) * k, 0};
            for (std::size_t j = 0; j < section.poles().size(); ++j) {
                const auto &s = section.poles()[j];
                const Point3 expected{p[0] + adjusted[0] * s[0] + adjusted[1] * s[2],
                                      p[1] + adjusted[1] * s[0] - adjusted[0] * s[2], p[2] + s[1]};
                check(near(surf.poles()[i * 2 + j], expected),
                      "adaptive compensation agrees with independent analytic planar frame");
            }
        }
        const double root5 = std::sqrt(5.);
        check(near(patch.final_frame[0], Point3{-2 / root5, 1 / root5, 0}) &&
                  near(patch.final_frame[2], Point3{1 / root5, 2 / root5, 0}),
              "adaptive final shared frame follows endpoint tangent");
    }
    const auto adaptive = generate(section, parabola);
    const auto weighted_profile = generate(rational_section, parabola);
    const auto ps = BsplineSurface::from_bgfb(*adaptive.surface);
    const auto ws = BsplineSurface::from_bgfb(*weighted_profile.surface);
    for (std::size_t i = 0; i < ws.poles().size(); ++i) {
        Point3 unweighted = ws.poles()[i];
        for (double &x : unweighted)
            x /= ws.weights()[i];
        check(near(unweighted, ps.poles()[i]) && ws.weights()[i] == (i % 2 ? 3. : 2.),
              "adaptive profile weights are removed then reapplied to final grid");
    }
    // With a zero profile, each output row must be the current working path
    // control after its frame query, not the untouched refined control.
    const auto weighted_path = curve(3, {.3, .7, .2, 1.1, 2.3, .4, 3.7, .8, .6}, {1.7, 2.9, 1.3});
    const auto zero_profile = curve(2, {0., 0., 0., 0., 0., 0.});
    auto weighted = generate(zero_profile, weighted_path);
    TubeBudget wb;
    auto wp = prepare_tube_facet_trace(weighted_path, wb);
    auto replay = *wp.trace;
    const auto wsurf = BsplineSurface::from_bgfb(*weighted.surface);
    for (std::size_t i = 0; i < wp.greville.size(); ++i) {
        const auto f = native_bspline_frame_working(replay, wp.greville[i][0]);
        replay = curve_detail::with_poles(replay, f.working_poles);
        Point3 expected = replay.poles()[i];
        const double inverse = 1 / replay.weights()[i];
        for (double &x : expected)
            x = (x * inverse) * replay.weights()[i];
        check(near(wsurf.poles()[2 * i], expected) && wsurf.weights()[2 * i] == replay.weights()[i],
              "adaptive rational grid uses post-query working control and original row weight");
    }
    check(weighted.working_trace_poles == replay.poles(),
          "adaptive private working state retains every row query");
    const double circle_w = std::sqrt(.5);
    const auto circle =
        curve(3, {1., 0., 0., circle_w, circle_w, 0., 0., 1., 0.}, {1., circle_w, 1.});
    TubeBudget circle_preparation_budget;
    const auto circle_prepared = prepare_tube_facet_trace(circle, circle_preparation_budget);
    check(circle_prepared.success && circle_prepared.sampling.sampling.interior.empty() &&
              !circle_prepared.sampling.ruled_fallback,
          "constant nonzero curvature requires tensor generation without inserting knots");
    TubeBudget circle_budget;
    const Matrix3 circle_frame{{{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}}};
    const auto circle_patch =
        generate_tube_facet_patch(zero_profile, circle, circle_frame, false, circle_budget);
    const auto circle_surface = BsplineSurface::from_bgfb(*circle_patch.surface);
    check(circle_surface.poles().size() == 6 && (*circle_patch.surface)["orderV"] == 3,
          "constant curvature is not mistaken for a straight extrusion");
    for (unsigned i = 0; i <= 16; ++i) {
        const auto p = circle_surface.point_at(.3, double(i) / 16);
        check(near(p[0] * p[0] + p[1] * p[1], 1.) && near(p[2], 0.),
              "zero-section rational adaptive surface retains analytic circle geometry");
    }
    const auto spatial = curve(4, {0., 0., 0., 1. / 3, 0., 0., 2. / 3, 1. / 3, 0., 1., 1., 1.});
    const auto spatial_free = generate(section, spatial, false);
    const auto spatial_rigid = generate(section, spatial, true);
    check(spatial_rigid.final_frame[1] == xy_frame[1] &&
              !near(spatial_free.final_frame[1], xy_frame[1]) &&
              spatial_free.surface != spatial_rigid.surface,
          "spatial curve distinguishes transported and rigid frame rules");
    TubeBudget cb;
    const auto callback = tube_facet_patch(section, weighted_path, xy_frame, false, cb);
    const auto initial = native_bspline_frame_working(weighted_path, 0);
    const auto callback_path = curve_detail::with_poles(weighted_path, initial.working_poles);
    Point3 tangent{};
    for (unsigned j = 0; j < 3; ++j)
        tangent[j] = initial.report["frame"][j][0];
    TubeBudget inner_budget;
    const auto inner = generate_tube_facet_patch(
        section, callback_path, advance_tube_frame(xy_frame, tangent, false), false, inner_budget);
    check(callback.surface == inner.surface && callback.final_frame == inner.final_frame &&
              callback.report["callback_working_poles"] == Json(callback_path.poles()),
          "callback advances its own working frame before private adaptive construction");
    check(weighted_path.poles().front() == Point3{.3, .7, .2} &&
              section.poles().front() == Point3{1, 2, 3},
          "facet generation leaves source profile and path unchanged");
    TubeBudget exact;
    exact.max_work = cb.work;
    check(tube_facet_patch(section, weighted_path, xy_frame, false, exact).surface ==
              callback.surface,
          "complete callback works at its exact cumulative work budget");
    TubeBudget short_work;
    short_work.max_work = cb.work - 1;
    rejects([&] { tube_facet_patch(section, weighted_path, xy_frame, false, short_work); },
            "complete callback rejects insufficient cumulative work budget");
    TubeBudget small_grid;
    small_grid.max_control_points = prepared.trace->poles().size();
    rejects([&] { generate_tube_facet_patch(section, parabola, xy_frame, false, small_grid); },
            "adaptive tensor output has its own allocation bound");
    rejects([&] { generate(curve(2, {0., 0., 0., 1., 1., 1.}, {0., 1.}), parabola); },
            "adaptive zero profile weight cannot be silently repaired");
    Matrix3 invalid = xy_frame;
    invalid[0][0] = NAN;
    rejects(
        [&] {
            TubeBudget b;
            generate_tube_facet_patch(section, line, invalid, false, b);
        },
        "facet matrix rejects nonfinite inputs");
    rejects([&] { generate(section, curve(2, {0., 0., 0., 1., 0., 0., 1., 1., 0.})); },
            "facet callback cannot silently discard multi-span path controls");
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return *generate(section, parabola).surface; }));
    for (auto &job : jobs)
        check(job.get() == adaptive.surface, "adaptive facet construction is reentrant");
    return count;
}
