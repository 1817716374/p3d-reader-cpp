#include "native_tube.hpp"
#include <future>
#include <random>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
bool near(const Point3 &a, const Point3 &b, double tolerance = 1e-12) {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) <= tolerance;
}
} // namespace
unsigned native_tube_fit_tests() {
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
    auto fit = [&](const std::vector<Point3> &p) {
        TubeBudget b;
        return fit_tube_linestring(p, b);
    };
    const std::vector<Point3> line{{0, 0, 0}, {1, 2, 3}};
    auto r = fit(line);
    check(r.curve.poles() == line && r.curve.knots() == std::vector<double>({0, 0, 1, 1}) &&
              r.report["method"] == "two_point_line" && r.report["attempts"].empty(),
          "two-point native conversion bypasses fit and parameter-length gates");
    r = fit({{0, 0, 0}, {0, 0, 0}});
    check(r.curve.poles()[0] == r.curve.poles()[1] && !r.curve.closed(),
          "two equal points remain an open degenerate native line");
    r = fit({{0, 0, 0}, {2, 0, 0}, {3, 0, 0}, {7, 0, 0}});
    check(r.curve.poles() == std::vector<Point3>({{0, 0, 0}, {7, 0, 0}}) &&
              r.report["source_parameters"] == Json({0, 2. / 7, 3. / 7, 1}) &&
              r.report["control_source_indices"] == Json({0, 3}),
          "collinear samples fit to one line while retaining original chord parameters");
    r = fit({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}});
    check(r.report["attempts"][0]["control_source_index"] == 1 &&
              r.report["attempts"][1]["control_source_index"] == 2,
          "equal zero-error candidates use first-in-native-order tie breaking");
    const std::vector<Point3> corner{{0, 0, 0}, {1, 0, 0}, {1, 2, 0}, {4, 2, 1}};
    r = fit(corner);
    check(r.curve.poles().size() == corner.size() &&
              r.report["control_source_indices"] == Json({0, 1, 2, 3}),
          "ordinary geometric corners are not removed by the native tolerance");
    for (std::size_t i = 0; i < corner.size(); ++i)
        check(near(r.curve.point_at(r.report["source_parameters"][i]), corner[i]),
              "retained nonuniform corners interpolate the input at chord parameters");
    for (double height : {.999e-5, 1e-5, 1.001e-5}) {
        auto x = fit({{0, 0, 0}, {1, height, 0}, {2, 0, 0}});
        check(x.curve.poles().size() == (height <= 1e-5 ? 2 : 3),
              "native accumulated-error threshold is inclusive at the fitting tolerance");
        check(x.report["attempts"][0]["removed"] == (height <= 1e-5),
              "removal report distinguishes accepted and rejected near-threshold samples");
    }
    const std::vector<Point3> zigzag{{0, 0, 0}, {1, 8e-6, 0}, {2, 0, 0}, {3, 8e-6, 0}, {4, 0, 0}};
    r = fit(zigzag);
    check(r.report["control_source_indices"] == Json({0, 2, 4}) &&
              r.report["attempts"].size() == 3 &&
              r.report["attempts"][0]["control_source_index"] == 1 &&
              r.report["attempts"][1]["control_source_index"] == 2 &&
              r.report["attempts"][1]["removed"] == false &&
              r.report["attempts"][2]["control_source_index"] == 3,
          "cumulative error rejects a candidate permanently even after its neighbor is removed");
    check(r.curve.poles() == std::vector<Point3>({{0, 0, 0}, {2, 0, 0}, {4, 0, 0}}),
          "native fit need not remove every geometrically redundant output point");
    for (unsigned cluster : {2u}) {
        std::vector<Point3> input{{0, 0, 0}};
        for (unsigned i = 0; i < cluster; ++i)
            input.push_back({1 + i * 1e-12, 0, 0});
        input.push_back({2, 0, 0});
        const auto x = fit(input);
        check(x.report["attempts"][0]["multiplicity"] == cluster && x.curve.poles().size() == 2,
              "near-multiple internal knots follow the native multiplicity removal branch");
    }
    rejects([&] { fit({{0, 0, 0}, {1, 0, 0}, {1 + 1e-12, 0, 0}, {1 + 2e-12, 0, 0}, {2, 0, 0}}); },
            "dense knot cluster cannot continue through a native zero-multiplicity candidate");
    r = fit({{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 0, 0}});
    check(!r.curve.closed() && r.curve.poles().front() == r.curve.poles().back(),
          "LineString fit does not itself close coincident endpoint geometry");
    std::mt19937 generator(713);
    std::uniform_real_distribution<double> perturb(-4e-6, 4e-6);
    for (unsigned run = 0; run < 25; ++run) {
        std::vector<Point3> input;
        for (unsigned i = 0; i < 9; ++i)
            input.push_back({double(i), perturb(generator), perturb(generator)});
        const auto original = input;
        const auto x = fit(input);
        check(input == original && x.curve.order() == 2 && !x.curve.rational(),
              "fit is immutable and keeps native degree-one nonrational storage");
        for (std::size_t i = 0; i < input.size(); ++i) {
            const double t = x.report["source_parameters"][i];
            check(near(x.curve.point_at(t), input[i], 1e-5 + 1e-12),
                  "independent curve basis evaluation verifies source-point fit residual");
            check(x.report["accumulated_errors"][i].get<double>() <= 1e-5,
                  "committed cumulative error remains within the native tolerance");
        }
    }
    rejects([&] { fit({}); }, "empty LineString fails rather than fabricating a path");
    rejects([&] { fit({{0, 0, 0}}); }, "one-point LineString has no native fitted curve");
    rejects([&] { fit({{0, 0, 0}, {2e-6, 0, 0}, {5e-6, 0, 0}}); },
            "short multi-point path with total length below the native gate is singular");
    const auto section = fit({{1, 0, 0}, {2, 0, 0}});
    const auto trace = fit({{0, 0, 0}, {0, 0, 1}, {0, 0, 4}});
    TubeBudget tube_budget;
    const auto tube = tube_surface(section.curve, trace.curve, false, tube_budget);
    const auto tube_eval = BsplineSurface::from_bgfb(tube.surface);
    const auto middle = tube_eval.point_at(.5, .5);
    check(tube_eval.v().pole_count() == 2 && std::abs(middle[2] - 2) < 1e-12 &&
              std::abs(std::hypot(middle[0], middle[1]) - 1.5) < 1e-12,
          "fitted section and trace feed native tube construction with the expected geometry");
    rejects([&] { fit({{0, 0, 0}, {0, 0, 0}, {1, 0, 0}}); },
            "repeated start point produces singular interpolation");
    rejects([&] { fit({{0, 0, 0}, {1, 0, 0}, {1, 0, 0}, {2, 0, 0}}); },
            "interior duplicates are not silently deduplicated");
    rejects([&] { fit({{0, 0, 0}, {1e-11, 0, 0}, {1, 0, 0}}); },
            "native near-start basis snap makes the interpolation singular");
    rejects([&] { fit({{0, 0, 0}, {1 - 1e-11, 0, 0}, {1, 0, 0}}); },
            "native near-end basis snap makes the interpolation singular");
    rejects([&] { fit({{0, 0, 0}, {std::numeric_limits<double>::infinity(), 0, 0}}); },
            "nonfinite source coordinates are rejected");
    rejects(
        [&] {
            TubeBudget b{3, 10000, 0};
            fit_tube_linestring(corner, b);
        },
        "input control budget applies even when fit could remove points");
    rejects(
        [&] {
            TubeBudget b{100, 50, 0};
            fit_tube_linestring(zigzag, b);
        },
        "shared work budget limits fitting and candidate scans");
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async, [&] {
            TubeBudget b;
            return fit_tube_linestring(zigzag, b).report;
        }));
    const auto expected = jobs.front().get();
    for (std::size_t i = 1; i < jobs.size(); ++i)
        check(jobs[i].get() == expected,
              "concurrent fits retain deterministic candidate order and reports");
    return n;
}
