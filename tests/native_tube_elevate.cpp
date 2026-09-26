#include "native_tube.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineCurve make(unsigned order, Json poles, Json knots, Json weights = nullptr,
                  bool closed = false) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"closed", closed},
                                    {"poles", poles},
                                    {"knots", knots},
                                    {"weights", weights}});
}
bool near(const Point3 &a, const Point3 &b, double tol = 2e-10) {
    for (unsigned k = 0; k < 3; ++k)
        if (std::abs(a[k] - b[k]) > tol * std::max({1., std::abs(a[k]), std::abs(b[k])}))
            return false;
    return true;
}
} // namespace
unsigned native_tube_elevate_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto rejects = [&](auto fn, const char *why) {
        bool failed = false;
        try {
            fn();
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, why);
    };
    auto elevate = [](const BsplineCurve &c, unsigned degree) {
        TubeBudget b;
        return elevate_open_tube_curve(c, degree, b);
    };
    auto combine = [](const BsplineCurve &a, const BsplineCurve &b) {
        TubeBudget budget;
        return combine_tube_curves(a, b, false, true, budget);
    };
    auto line = make(2, {0, 0, 0, 6, 3, 0}, {0, 0, 1, 1});
    for (unsigned d : {2u, 3u, 5u, 12u, 25u}) {
        const auto r = elevate(line, d);
        check(r.curve.order() == d + 1 && r.curve.poles().size() == d + 1,
              "line elevation degree and size");
        for (unsigned i = 0; i <= d; ++i)
            check(near(r.curve.poles()[i], {6. * i / d, 3. * i / d, 0}),
                  "line Bernstein elevation golden controls");
        for (unsigned i = 0; i <= 20; ++i) {
            const double t = i / 20.;
            check(near(r.curve.point_at(t), {6 * t, 3 * t, 0}),
                  "elevated line independent analytic evaluation");
        }
    }
    auto quad = make(3, {0, 0, 0, 3, 6, 0, 6, 0, 0}, {2, 2, 2, 7, 7, 7});
    auto cubic = elevate(quad, 3);
    check(cubic.curve.knots() == std::vector<double>({2, 2, 2, 2, 7, 7, 7, 7}) &&
              cubic.report["restored_source_domain"] == true,
          "elevation restores nonunit original knot domain");
    check(cubic.curve.poles() == std::vector<Point3>({{0, 0, 0}, {2, 4, 0}, {4, 4, 0}, {6, 0, 0}}),
          "quadratic-to-cubic exact Bernstein golden controls");
    for (auto weights : {Json(nullptr), Json({1., .6, 1.}), Json({-1., -.6, -1.})}) {
        auto c = make(3, {0, 0, 0, 1, 2, 0, 2, 0, 0}, {0, 0, 0, 1, 1, 1}, weights);
        for (unsigned d : {3u, 5u, 8u}) {
            auto e = elevate(c, d);
            for (unsigned i = 0; i <= 32; ++i)
                check(near(e.curve.point_at(i / 32.), c.point_at(i / 32.)),
                      "homogeneous elevation preserves rational curve across positive/negative "
                      "weights");
        }
    }
    auto broken = make(2, {0, 0, 0, 2, 0, 0, 5, 0, 0, 11, 0, 0}, {0, 0, .5, .5, 1, 1});
    auto eb = elevate(broken, 2);
    check(eb.curve.knots() == std::vector<double>({0, 0, 0, .5, .5, .5, 1, 1, 1}) &&
              eb.curve.poles() ==
                  std::vector<Point3>(
                      {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {5, 0, 0}, {8, 0, 0}, {11, 0, 0}}),
          "full-multiplicity internal discontinuity has independent elevated one-sided controls");
    auto left = native_bspline_evaluate_working(broken, .5, 2, true, broken.poles());
    auto right = native_bspline_evaluate(broken, .5, 2);
    check(left.homogeneous[0][0] == 2 && right.homogeneous[0][0] == 5 &&
              left.homogeneous[1][0] == 4 && right.homogeneous[1][0] == 12 &&
              left.homogeneous[2][0] == 0,
          "shared derivative kernel selects native left/right sides");
    // Near-end knots snap only when normalization actually runs.
    auto nearstart = make(2, {0, 0, 0, 1, 0, 0, 2, 0, 0}, {2, 2, 2 + 1e-13, 3, 3});
    const auto snapped = elevate(nearstart, 2);
    check(
        snapped.curve.knots() == std::vector<double>({2, 2, 2, 2, 3, 3, 3}) &&
            snapped.curve.poles() ==
                std::vector<Point3>({{0, 0, 0}, {1, 0, 0}, {1.5, 0, 0}, {2, 0, 0}}),
        "normalization snap retains extra endpoint multiplicity and native endpoint reassignment");
    auto nearunit = make(2, {0, 0, 0, 1, 0, 0}, {1e-15, 1e-15, 1 + 1e-15, 1 + 1e-15});
    auto en = elevate(nearunit, 2);
    check(en.report["normalized_source_knots"] == false && en.curve.knots().front() == 1e-15,
          "already normalized tolerance does not rewrite near-zero endpoints");
    auto un = make(2, {0, 0, 0, 1, 0, 0}, {-1, 0, 1, 2});
    auto copy = elevate(un, 1);
    check(copy.curve.knots() == un.knots() && copy.curve.poles() == un.poles(),
          "equal degree bypasses normalization and knot allocation");
    rejects([&] { elevate(un, 2); },
            "unwritten native exterior-knot storage is rejected, not invented");
    auto wz = make(2, {0, 0, 0, 1, 0, 0}, {0, 0, 1, 1}, {0, 1});
    check(elevate(wz, 1).curve.weights() == wz.weights(),
          "same-degree zero-weight source is a copy");
    rejects([&] { elevate(wz, 2); },
            "actual elevation tolerance cannot deweight zero control weights");
    rejects([&] { elevate(quad, 1); }, "degree reduction is not elevation");
    rejects([&] { elevate(line, 26); }, "native degree elevation upper bound is 25");
    rejects(
        [&] {
            TubeBudget b;
            b.max_control_points = 2;
            elevate_open_tube_curve(line, 2, b);
        },
        "elevation output budget");
    rejects(
        [&] {
            TubeBudget b;
            b.max_work = 10;
            elevate_open_tube_curve(line, 2, b);
        },
        "elevation cumulative work budget");
    auto a = make(2, {0, 0, 0, 2, 0, 0}, {0, 0, 1, 1});
    auto b = make(3, {2, 0, 0, 3, 1, 0, 4, 0, 0}, {0, 0, 0, 1, 1, 1});
    auto ab = combine(a, b), ba = combine(b, a);
    check(ab.curve.order() == 3 && ab.report["left_elevation"].is_object() &&
              ab.report["right_elevation"].is_null() &&
              ab.report["combination"]["contiguous"] == true,
          "outer wrapper elevates left before checking endpoint connection");
    check(ba.report["right_elevation"].is_object() &&
              ba.report["combination"]["contiguous"] == false,
          "outer wrapper elevates right and retains disconnected join");
    auto periodic = make(3, {0, 0, 0, .5, .8, 0, .8, .5, 0, 0, 0, 0}, {-1, 0, 0, 0, 1, 2, 2, 2, 3},
                         nullptr, true);
    auto pc = combine(periodic, a);
    check(pc.report["left_opening"]["method"] == "strip_exterior_knots" &&
              pc.report["right_elevation"].is_object() && !pc.curve.closed(),
          "closed source opens at native special seam before degree matching and combination");
    auto ordinary = make(2, {0, 0, 0, 1, 0, 0, 1, 1, 0}, {-1, 0, 1, 2, 3, 4}, nullptr, true);
    auto oc = combine(ordinary, a);
    check(oc.report["left_opening"]["method"] == "cyclic_knot_insertion" && !oc.curve.closed(),
          "ordinary periodic curve uses shared native cyclic opening");
    rejects([&] { elevate(periodic, 3); }, "open elevation cannot silently close or open source");
    auto rounding = make(2, {1, 0, 0, 2, 0, 0}, {0, 0, 1, 1}, {49, 49});
    auto er = elevate(rounding, 3);
    check(er.curve.poles().front() == rounding.poles().front(),
          "first homogeneous pole restored before tolerance round trips");
    double last = 2;
    for (unsigned i = 0; i < 5; ++i)
        last = (last * (1. / 49)) * 49;
    check(er.curve.poles().back()[0] == last && rounding.poles().back()[0] == 2,
          "last control retains initial plus one round trip per native derivative query");
    std::vector<std::future<TubeCurve>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(std::async(std::launch::async, [&] { return combine(periodic, a); }));
    for (auto &job : jobs) {
        auto r = job.get();
        check(r.curve.poles() == pc.curve.poles() && r.report == pc.report,
              "concurrent elevation/opening uses no native global scratch state");
    }
    return checks;
}
