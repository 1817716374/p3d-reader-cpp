#include "native_builder_coordinates.hpp"
#include <future>
#include <limits>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_builder_coordinates_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "native builder map rejects invalid data or exhausted resources");
    };
    NativeBuilderCoordinateOptions defaults;
    TubeBudget b;
    {
        NativeBuilderCoordinateBatch a, c;
        a.points = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
        a.parameters = {{0, 0}, {1, 0}, {0, 1}};
        a.normals = {{0, 0, 2}, {0, 0, 3}, {0, 0, 4}};
        c.points = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
        c.parameters = {{1, 0}, {1, 1}, {0, 1}};
        c.normals = {{0, 2, 0}, {0, 4, 0}, {0, 8, 0}};
        auto r = map_native_builder_coordinates({a, c}, defaults, b);
        check(r.points == std::vector<Point3>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}},
              "adjacent batches keep first accepted coordinate and insertion order");
        check(r.batches[0].points == std::vector<std::size_t>{0, 1, 2} &&
                  r.batches[1].points == std::vector<std::size_t>{1, 3, 2},
              "shared edges map to existing positions without geometric reordering");
        check(r.batches[1].parameters == std::vector<std::size_t>{1, 3, 2} &&
                  r.batches[0].normals == std::vector<std::size_t>{0, 0, 0} &&
                  r.batches[1].normals == std::vector<std::size_t>{1, 1, 1},
              "position UV and normal maps remain independent at a hard edge");
        check(r.normals == std::vector<Point3>{{0, 0, 1}, {0, 1, 0}} && a.normals[0][2] == 2 &&
                  c.normals[0][1] == 2,
              "builder normalizes derived normals without changing source data");
        check(r.report.at("facet_assembly_performed") == false,
              "coordinate insertion is not reported as full mesh assembly");
    }
    {
        NativeBuilderCoordinateBatch a;
        a.normalize_normals = false;
        a.points = {{1, 0, 0}, {1, 1e-6, 0}, {1, 3e-6, 0}};
        a.normals = a.points;
        a.parameters = {{1, 0}, {1, 1e-6}, {1, 3e-6}};
        auto r = map_native_builder_coordinates({a}, defaults, b);
        check(r.points.size() == 3 && r.parameters.size() == 3 && r.normals.size() == 2 &&
                  r.batches[0].normals == std::vector<std::size_t>{0, 0, 1},
              "native normal relative tolerance is 1e-6, unlike position and UV 1e-12");
        check(r.report["normals"]["relative_tolerance"] == 1e-6 &&
                  r.report["points"]["absolute_tolerance"] == 1e-14,
              "reports state original channel tolerances");
        a.normals = {{0, 0, 0}, {2, 0, 0}, {-2, 0, 0}, {0, 3, 4}};
        a.normalize_normals = true;
        a.reverse_normals = true;
        r = map_native_builder_coordinates({a}, defaults, b);
        check(r.batches[0].normals == std::vector<std::size_t>{0, 1, 0, 2} &&
                  r.normals[0] == Point3{1, 0, 0} && r.normals[1] == Point3{-1, 0, 0},
              "reversal precedes normalization; reversed zero still falls back to plus X");
        check(std::abs(r.normals[2][1] + .6) < 1e-15 && r.normals[2][2] == -.8,
              "normal magnitude uses full XYZ and reciprocal multiplication");
    }
    {
        NativeBuilderCoordinateOptions o;
        o.point_absolute_tolerance = 1;
        o.point_relative_tolerance = 0;
        NativeBuilderCoordinateBatch a;
        a.points = {{0, 0, 0}, {2, 0, 0}, {1, 0, 0}};
        auto r = map_native_builder_coordinates({a}, o, b);
        check(r.batches[0].points == std::vector<std::size_t>{0, 1, 1} &&
                  r.points == std::vector<Point3>{{0, 0, 0}, {2, 0, 0}},
              "non-transitive equivalence uses native descending lower-bound candidate");
        a.points = {{0, 0, 0}, {.75, 0, 0}, {1.5, 0, 0}};
        r = map_native_builder_coordinates({a}, o, b);
        check(r.batches[0].points == std::vector<std::size_t>{0, 0, 1},
              "accepted duplicates do not move the representative or extend a cluster");
        a.points = {{0, 0, 0}, {1, 1, 1}};
        r = map_native_builder_coordinates({a}, o, b);
        check(r.points.size() == 1, "builder uses component comparison, not spherical distance");
        a.points[1][2] = std::nextafter(1., 2.);
        check(map_native_builder_coordinates({a}, o, b).points.size() == 2,
              "inclusive component tolerance changes immediately beyond threshold");
        // An accepted insertion tree: root X=2, left X=4, right X=0.
        // Query X=1 finds X=2, even though X=0 was inserted first.
        a.points = {{0, 0, 0}, {2, 0, 0}, {4, 0, 0}, {1, 0, 0}, {3, 0, 0}};
        r = map_native_builder_coordinates({a}, o, b);
        check(r.batches[0].points == std::vector<std::size_t>{0, 1, 2, 1, 2} &&
                  r.report["points"]["rotations"] == 1,
              "native tree rotation and query candidate determine ambiguous matches");
        o.point_absolute_tolerance = 0;
        o.point_relative_tolerance = .125;
        a.points = {{10, 0, 0}, {12, 0, 0}};
        check(map_native_builder_coordinates({a}, o, b).points.size() == 1,
              "relative tolerance uses absolute coordinates of both keys");
        a.points = {{0, 0, 0}, {2, 0, 0}};
        check(map_native_builder_coordinates({a}, o, b).points.size() == 2,
              "translation changes original relative coordinate comparison");
    }
    {
        NativeBuilderCoordinateBatch a, c, d;
        a.parameters = c.parameters = d.parameters = {{0, 0}, {1, 0}};
        c.parameter_scope = 1;
        d.parameter_scope = 0;
        auto r = map_native_builder_coordinates({a, c, d}, defaults, b);
        check(r.parameters == std::vector<Point2>{{0, 0}, {1, 0}, {0, 0}, {1, 0}} &&
                  r.batches[2].parameters == std::vector<std::size_t>{0, 1},
              "scope separates identical UVs and returning to previous scope finds prior values");
        a.parameter_scope = 1e12;
        r = map_native_builder_coordinates({a}, defaults, b);
        check(r.parameters.size() == 1,
              "scope participates in native three-coordinate relative tolerance");
        NativeBuilderCoordinateOptions o;
        o.parameter_absolute_tolerance = .125;
        o.parameter_relative_tolerance = 0;
        a.parameters = c.parameters = d.parameters = {{2, 3}};
        a.parameter_scope = 0;
        c.parameter_scope = .125;
        d.parameter_scope = std::nextafter(.125, 1.);
        r = map_native_builder_coordinates({a, c, d}, o, b);
        check(r.batches[1].parameters == std::vector<std::size_t>{0} &&
                  r.batches[2].parameters == std::vector<std::size_t>{1},
              "parameter scope uses inclusive tolerance instead of exact integer grouping");
    }
    {
        // Well-separated coordinates have unambiguous exact identities. Exercise
        // ascending, descending and shuffled insertion, including both rotations.
        for (unsigned order = 0; order < 3; ++order) {
            NativeBuilderCoordinateBatch a;
            std::vector<std::size_t> expected;
            for (unsigned i = 0; i < 257; ++i) {
                unsigned j = order == 0 ? i : order == 1 ? 256 - i : (i * 97) % 257;
                a.points.push_back(
                    {double(j % 11) * 10, double(j / 11 % 7) * 10, double(j / 77) * 10});
                expected.push_back(i);
            }
            auto original = a.points;
            for (unsigned i = 0; i < 257; ++i) {
                a.points.push_back(original[(i * 101) % 257]);
                expected.push_back((i * 101) % 257);
            }
            auto r = map_native_builder_coordinates({a}, defaults, b);
            check(r.points == original && r.batches[0].points == expected,
                  "balanced lookup preserves exact identities through rotated 3D trees");
            check(r.report["points"]["rotations"].get<unsigned>() > 0,
                  "larger sequences exercise insertion balancing");
        }
    }
    NativeBuilderCoordinateBatch a;
    a.points = {{0, 0, 0}, {1, 2, 3}, {1, 2, 3}};
    a.parameters = {{0, 0}, {1, 2}};
    a.normals = {{0, 0, 0}, {2, 3, 4}};
    const auto baseline = map_native_builder_coordinates({a}, defaults, b);
    {
        auto bad = a;
        bad.points[0][0] = std::numeric_limits<double>::quiet_NaN();
        rejects([&] { map_native_builder_coordinates({bad}, defaults, b); });
        bad = a;
        bad.parameter_scope = std::numeric_limits<double>::infinity();
        rejects([&] { map_native_builder_coordinates({bad}, defaults, b); });
        bad = a;
        bad.normals[1][2] = 1e308;
        rejects([&] { map_native_builder_coordinates({bad}, defaults, b); });
        bad = a;
        bad.parameters[0][1] = std::numeric_limits<double>::infinity();
        rejects([&] { map_native_builder_coordinates({bad}, defaults, b); });
        bad = a;
        bad.points = {{-1e308, 0, 0}, {1e308, 0, 0}};
        auto extreme = map_native_builder_coordinates({bad}, defaults, b);
        check(extreme.points == std::vector<Point3>{{-1e308, 0, 0}} &&
                  extreme.batches[0].points == std::vector<std::size_t>{0, 0},
              "finite extreme keys retain native infinite-tolerance comparison");
        auto invalid = defaults;
        invalid.point_relative_tolerance = -1;
        rejects([&] { map_native_builder_coordinates({a}, invalid, b); });
        invalid = defaults;
        invalid.parameter_absolute_tolerance = std::numeric_limits<double>::quiet_NaN();
        rejects([&] { map_native_builder_coordinates({}, invalid, b); });
        check(a.points.size() == 3 && a.normals[1] == Point3{2, 3, 4},
              "failure leaves caller's coordinate pools intact");
    }
    {
        TubeBudget measured;
        map_native_builder_coordinates({a}, defaults, measured);
        for (auto limit : {std::size_t(0), std::size_t(1), measured.work / 2, measured.work - 1}) {
            TubeBudget limited;
            limited.max_work = limit;
            rejects([&] { map_native_builder_coordinates({a}, defaults, limited); });
        }
        TubeBudget limited;
        limited.max_control_points = 6;
        rejects([&] { map_native_builder_coordinates({a}, defaults, limited); });
        limited.max_control_points = 7;
        check(map_native_builder_coordinates({a}, defaults, limited).points == baseline.points,
              "aggregate coordinate storage bound is inclusive");
        limited.max_control_points = 1;
        rejects([&] { map_native_builder_coordinates({{}, {}}, defaults, limited); });
        auto empty = map_native_builder_coordinates({}, defaults, b);
        check(empty.points.empty() && empty.batches.empty(),
              "empty coordinate stream remains empty");
    }
    {
        const double marker = std::numeric_limits<double>::max();
        NativeBuilderCoordinateBatch disconnects;
        disconnects.points = {{0, 0, 0}, {1, 0, 0}, {marker, marker, marker}, {1, 1, 0}};
        auto result = map_native_builder_coordinates({disconnects}, defaults, b);
        check(result.points == std::vector<Point3>{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}} &&
                  result.batches[0].points == std::vector<std::size_t>{0, 1, 1, 2},
              "disconnect tolerance overflow preserves original native map lookup");
        disconnects.points = {{marker, marker, marker}, {0, 0, 0}, {1, 1, 1}};
        result = map_native_builder_coordinates({disconnects}, defaults, b);
        check(result.points == std::vector<Point3>{{marker, marker, marker}} &&
                  result.batches[0].points == std::vector<std::size_t>{0, 0, 0},
              "leading disconnect is not silently removed or repaired");
        NativeBuilderCoordinateOptions exact;
        exact.point_relative_tolerance = 0;
        disconnects.points = {{-marker, 0, 0}, {marker, 0, 0}};
        result = map_native_builder_coordinates({disconnects}, exact, b);
        check(result.points.size() == 2, "overflowing difference still orders finite source keys");
    }
    auto run = [&] {
        TubeBudget local;
        return map_native_builder_coordinates({a}, defaults, local);
    };
    auto f1 = std::async(std::launch::async, run), f2 = std::async(std::launch::async, run);
    const auto r1 = f1.get(), r2 = f2.get();
    check(r1.report == baseline.report && r2.report == baseline.report &&
              r1.normals == r2.normals && r1.batches[0].points == baseline.batches[0].points,
          "coordinate maps use isolated deterministic per-call state");
    return checks;
}
