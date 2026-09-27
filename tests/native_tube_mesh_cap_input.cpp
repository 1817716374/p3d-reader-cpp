#include "native_tube_mesh_cap_input.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_tube_mesh_cap_input_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        ++checks;
        require(value, why);
    };
    auto run = [](const std::vector<std::vector<Point3>> &rings, bool reverse = false,
                  double maximum = 0) {
        TubeBudget budget;
        return prepare_native_tube_mesh_cap_input(rings, reverse, maximum, budget);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "unsafe cap input or exceeded budget rejected");
    };
    const std::vector<Point3> square{{10, 20, 3}, {14, 20, 3}, {14, 22, 3}, {10, 22, 3}};
    const auto original = square;
    auto open = run({square});
    check(open.preparation_succeeded && open.region_type == 2, "single outer region prepared");
    check(open.rings[0].size() == 5 && open.rings[0].front() == open.rings[0].back(),
          "open input explicitly closed before strokes");
    check(open.points == square && open.stroked_points.size() == 5,
          "wrapped polygon compressed only at triangulation boundary");
    check(open.projection.points == std::vector<Point3>{{0, 0, 0}, {4, 0, 0}, {4, 2, 0}, {0, 2, 0}},
          "cap frame uses lower-left unit axes");
    check(open.report["mesh_generated"] == false, "preparation is not cap mesh success");
    auto reversed = run({square}, true);
    auto reverse_points = square;
    std::reverse(reverse_points.begin(), reverse_points.end());
    check(reversed.points == reverse_points, "start cap reverses all points of each ring");
    check(square == original, "source unchanged after reversal and closure");
    auto near_closed = square;
    near_closed.push_back({10 + 1e-11, 20, 3});
    auto snapped = run({near_closed});
    check(snapped.rings[0].size() == 5 && snapped.rings[0].back() == square.front(),
          "near endpoint is replaced, not duplicated");
    check(snapped.report["rings"][0]["closure_snapped"] == true, "snap reported");
    auto exact_closed = square;
    exact_closed.push_back(square.front());
    check(run({exact_closed}).points == square, "exact closure retained then stripped");
    auto repeats = square;
    repeats.insert(repeats.begin() + 1, square[0]);
    repeats.push_back(square[0]);
    repeats.push_back(square[0]);
    check(run({repeats}).points == square, "adjacent and repeated closure points compressed");
    std::vector<Point3> inner{{11, 20.5, 3}, {12, 20.5, 3}, {12, 21, 3}, {11, 21, 3}};
    auto multiple = run({square, inner});
    const double marker = std::numeric_limits<double>::max();
    const Point3 disconnect{marker, marker, marker};
    check(multiple.region_type == 4 && multiple.ring_types == std::vector<std::uint32_t>{2, 3},
          "multi-ring parity region retains outer then inner types");
    check(multiple.stroked_points.size() == 12 && multiple.stroked_points[5] == disconnect &&
              multiple.stroked_points.back() == disconnect,
          "every child appends disconnect");
    check(multiple.points == multiple.stroked_points,
          "native cap input keeps trailing disconnect and per-ring closures");
    check(multiple.preparation_succeeded, "multi-ring frame succeeds with disconnects");
    auto swapped = run({inner, square}, true);
    check(swapped.rings[0].front() == inner.back() && swapped.rings[1].front() == square.back(),
          "no area sorting or geometry-derived outer ring assignment");
    check(swapped.ring_types == multiple.ring_types, "ring roles follow input order");
    check(!run({}).preparation_succeeded && run({}).report["reason"] == "no_rings",
          "zero rings rejected by original helper");
    check(!run({{}}).preparation_succeeded, "single empty ring has no stroked polygon");
    check(run({{{1, 2, 3}}}).stroked_points.empty(), "single point emits no segment");
    auto empty_tail = run({square, {}, {}});
    check(empty_tail.stroked_points.size() == 8 && empty_tail.points.size() == 6,
          "adjacent empty-child disconnects collapse through original predicate");
    auto degenerate = run({{{0, 0, 0}, {1, 0, 0}}});
    check(degenerate.points.size() == 2, "no second point-count gate before frame attempt");
    for (double length : {0.0, -1.0, 4.0, 2.0, 1.0, 0.5}) {
        auto result = run({square}, false, length);
        const auto expected = length <= 0 || length >= 4 ? 4u
                              : length == 2              ? 6u
                              : length == 1              ? 12u
                                                         : 24u;
        check(result.points.size() == expected, "original segment count respects length setting");
        check(result.points.front() == square.front() &&
                  result.stroked_points.back() == square.front(),
              "forced exact endpoint fractions");
    }
    auto capped = run({square}, false, 1e-100);
    check(capped.points.size() == 400, "native signed conversion overflow clamps at 100 per edge");
    check(capped.report["rings"][0]["segment_counts"] == Json::array({100, 100, 100, 100}),
          "subdivision cap preserved");
    auto near_integer = run({{{0, 0, 0}, {2.000001, 0, 0}, {0, 1, 0}}}, false, 1);
    check(near_integer.report["rings"][0]["segment_counts"][0] == 2,
          "original 0.99999 bias differs from exact ceil");
    auto changed_integer = run({{{0, 0, 0}, {2.00002, 0, 0}, {0, 1, 0}}}, false, 1);
    check(changed_integer.report["rings"][0]["segment_counts"][0] == 3,
          "subdivision threshold above original bias");
    auto overflow = square;
    overflow[0][0] = std::numeric_limits<double>::infinity();
    rejects([&] { run({overflow}); });
    rejects([&] { run({square}, false, std::numeric_limits<double>::quiet_NaN()); });
    for (std::size_t maximum : {0u, 3u, 10u}) {
        rejects([&] {
            TubeBudget b;
            b.max_control_points = maximum;
            prepare_native_tube_mesh_cap_input({square}, false, 0, b);
        });
    }
    rejects([&] {
        TubeBudget b;
        b.max_work = 1;
        prepare_native_tube_mesh_cap_input({square}, false, 0, b);
    });
    auto future = std::async(std::launch::async, [&] { return run({square, inner}, true, 0.5); });
    const auto concurrent = run({square, inner}, true, 0.5);
    const auto other = future.get();
    check(concurrent.points == other.points && concurrent.report == other.report,
          "concurrent cap input preparation deterministic");
    return checks;
}
