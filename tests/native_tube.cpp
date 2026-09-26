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
bool near(Point3 a, Point3 b, double tol = 1e-11) {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) <= tol;
}
const Matrix3 identity{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
} // namespace
unsigned native_tube_tests() {
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
    // Independent Rodrigues rotation transports an orthonormal basis through
    // the shortest rotation of its tangent, away from native cutoff/degeneracy.
    for (Point3 t : {Point3{1, 0, 0}, Point3{0, 1, 0}, Point3{.6, 0, .8}, Point3{.36, .48, -.8}}) {
        const auto frame = advance_tube_frame(identity, t, false);
        const double sine = std::hypot(t[0], t[1]);
        const Point3 axis{-t[1] / sine, t[0] / sine, 0};
        for (unsigned row = 0; row < 3; ++row) {
            Point3 expected{}, e = identity[row];
            const Point3 cross{axis[1] * e[2] - axis[2] * e[1], axis[2] * e[0] - axis[0] * e[2],
                               axis[0] * e[1] - axis[1] * e[0]};
            for (unsigned a = 0; a < 3; ++a)
                expected[a] = e[a] * t[2] + cross[a] * sine + axis[a] * axis[row] * (1 - t[2]);
            check(near(frame[row], expected), "tube frame matches independent shortest rotation");
        }
    }
    for (double z : {std::nextafter(.99999, 0.), .99999, std::nextafter(.99999, 1.)}) {
        const Point3 t{std::sqrt(1 - z * z), 0, z};
        const auto f = advance_tube_frame(identity, t, false);
        check(z > .99999 ? f == identity : near(f[2], t),
              "native same-direction frame threshold is strict");
    }
    const auto reverse = advance_tube_frame(identity, {0, 0, -1}, false);
    check(reverse[0] == Point3{} && reverse[1] == Point3{} && reverse[2] == Point3{0, 0, -1},
          "antiparallel tangent preserves native degenerate basis instead of inventing an axis");
    const Point3 tilted{0, .6, .8};
    const auto rigid = advance_tube_frame(identity, tilted, true);
    check(rigid[0] == Point3{1, 0, 0} && rigid[1] == identity[1] && rigid[2] == tilted,
          "rigid mode retains previous Y even when it is not perpendicular to tangent");
    const auto zero = advance_tube_frame(identity, {}, false);
    check(zero == Matrix3{}, "zero tangent does not receive a fabricated replacement frame");

    const auto line = curve(2, {0, 0, 0, 0, 0, 5});
    const auto profile = curve(2, {1, 2, 3, 4, 6, 7});
    TubeBudget budget;
    const auto patch = tube_patch(profile, line, identity, false, budget);
    const auto surface = BsplineSurface::from_bgfb(patch.surface);
    check(patch.surface["poles"] == Json({1, 2, 3, 4, 6, 7, 1, 2, 8, 4, 6, 12}) &&
              patch.surface["numRulesU"] == 2 && patch.surface["numRulesV"] == 2,
          "oblique section Z survives and translated control grid keeps native rule counts");
    for (double u : {0., .2, .7, 1.})
        for (double v : {0., .3, .9, 1.}) {
            auto expected = profile.point_at(u);
            expected[2] += 5 * v;
            check(near(surface.point_at(u, v), expected),
                  "straight sweep surface equals independently evaluated profile plus translation");
        }
    const auto periodic = curve(3, {1, 0, 2, 0, 1, 3, -1, 0, 2, 0, -1, 1}, nullptr, true);
    TubeBudget periodic_budget;
    const auto periodic_patch = tube_patch(periodic, line, identity, false, periodic_budget);
    const auto ps = BsplineSurface::from_bgfb(periodic_patch.surface);
    check(ps.u().closed() && periodic_patch.surface["knotsU"] == Json(periodic.knots()),
          "periodic profile keeps its source pole convention and knot storage");
    for (double u : {0., .17, .5, .83, 1.}) {
        auto p = periodic.point_at(u);
        p[2] += 2;
        check(near(ps.point_at(u, .4), p),
              "periodic profile evaluation survives tensor construction");
    }
    const double w = std::sqrt(.5);
    const Matrix3 start{{{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}}};
    const auto section = curve(2, {.2, 0, 0, .4, 0, 0});
    for (double sign : {1., -1.}) {
        const auto arc =
            curve(3, {sign, 0, 0, sign * w, sign * w, 0, 0, sign, 0}, {sign, sign * w, sign});
        TubeBudget arc_budget;
        const auto q = tube_patch(section, arc, start, false, arc_budget);
        const auto s = BsplineSurface::from_bgfb(q.surface);
        for (double u : {0., .3, 1.})
            for (double v : {0., .2, .5, .9, 1.}) {
                const auto p = s.point_at(u, v);
                const double radius = 1 - sign * (.2 + .2 * u);
                check(std::abs(std::hypot(p[0], p[1]) - radius) < 1e-11 && std::abs(p[2]) < 1e-12,
                      "rational circular trace produces native compensated circular offset");
            }
        check(q.report["greville_nodes"] == Json({0., .5, 1.}) &&
                  q.report["weight_compensated_rows"] == (sign > 0 ? 1 : 3),
              "native weight compensation is sensitive to trace weight sign");
    }
    for (double factor : {1. + .5e-8, 1. + 2e-8}) {
        const auto trace = curve(2, {0, 0, 0, 0, 0, 5 / factor}, {1., 1 / factor});
        TubeBudget b;
        const auto q = tube_patch(profile, trace, identity, false, b);
        const double x =
            q.surface["poles"][6].get<double>() / q.surface["weights"][2].get<double>();
        check(std::abs(x - (factor - 1 < 1e-8 ? 1. : factor)) < 1e-14,
              "P3D compensation uses the verified 1e-8 reciprocal-weight threshold");
    }
    const auto negative_profile = curve(2, {-1, -2, -3, -4, -6, -7}, {-1, -1});
    TubeBudget negative_budget;
    const auto ns = BsplineSurface::from_bgfb(
        tube_patch(negative_profile, line, identity, false, negative_budget).surface);
    check(near(ns.point_at(.3, .4), surface.point_at(.3, .4)),
          "negative profile weights retain the same Cartesian section");
    const auto cubic = curve(4, {0, 0, 0, 1. / 3, 0, 0, 2. / 3, 1. / 3, 0, 1, 1, 1});
    TubeBudget cubic_budget;
    const auto spatial = tube_patch(profile, cubic, identity, false, cubic_budget);
    check(spatial.report["greville_nodes"] == Json({0., 1. / 3, 2. / 3, 1.}) &&
              spatial.report["frame_rows"].size() == 4,
          "cubic tube uses each Bezier Greville node, not uniform geometric distance");
    for (std::size_t i = 0; i < 4; ++i) {
        const double t = double(i) / 3, speed = std::sqrt(1 + 4 * t * t + 9 * t * t * t * t);
        const Point3 tangent = spatial.report["frame_rows"][i][2].get<Point3>();
        check(near(tangent, {1 / speed, 2 * t / speed, 3 * t * t / speed}),
              "spatial path row tangents match independent polynomial derivatives");
    }
    auto nonfinite = identity;
    nonfinite[0][0] = std::numeric_limits<double>::infinity();
    rejects([&] { advance_tube_frame(nonfinite, {0, 0, 1}, false); },
            "nonfinite frame is rejected");
    rejects(
        [&] {
            TubeBudget b;
            tube_patch(profile, curve(2, {0, 0, 0, 0, 0, 2}, nullptr, false, {2, 2, 4, 4}),
                       identity, false, b);
        },
        "raw non-unit parameter domain must pass through Bezier callback normalization first");
    rejects(
        [&] {
            TubeBudget b;
            tube_patch(profile, periodic, identity, false, b);
        },
        "periodic source trace cannot bypass required segment decomposition");
    rejects(
        [&] {
            TubeBudget b;
            tube_patch(curve(2, {1, 2, 3, 4, 5, 6}, {0, 1}), line, identity, false, b);
        },
        "zero profile control weight does not escape as a finite partial surface");
    rejects(
        [&] {
            TubeBudget b;
            tube_patch(profile, curve(2, {0, 0, 0, 0, 0, 1}, {0, 1}), identity, false, b);
        },
        "zero trace control weight is not silently replaced");
    rejects(
        [&] {
            TubeBudget b;
            b.max_control_points = 3;
            tube_patch(profile, line, identity, false, b);
        },
        "tensor budget is checked before grid allocation");
    rejects(
        [&] {
            TubeBudget b;
            b.max_work = 27;
            tube_patch(profile, line, identity, false, b);
        },
        "frame evaluation work participates in the shared budget");
    TubeBudget shared;
    shared.max_work = 56;
    tube_patch(profile, line, identity, false, shared);
    tube_patch(profile, line, identity, false, shared);
    rejects([&] { tube_patch(profile, line, identity, false, shared); },
            "successive patches cannot reset the caller's cumulative work budget");
    auto future = std::async(std::launch::async, [&] {
        TubeBudget b;
        return tube_patch(profile, line, identity, false, b);
    });
    check(future.get().surface == patch.surface, "independent tube construction is thread safe");
    return n;
}
