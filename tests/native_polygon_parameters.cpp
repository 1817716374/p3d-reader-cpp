#include "native_polygon_parameters.hpp"
#include "native_tube_mesh_cap_input.hpp"
#include "native_projected_polygon.hpp"
#include "native_polyface_face_data.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polygon_parameters_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        ++checks;
        require(value, why);
    };
    using Range = std::array<Point2, 2>;
    const Range source{{{10, 20}, {14, 22}}};
    const Range distance{{{0, 0}, {4, 2}}};
    Matrix4 identity{};
    for (unsigned k = 0; k < 4; ++k)
        identity[k][k] = 1;
    auto run = [&](std::vector<Point3> points, unsigned mode = 0) {
        TubeBudget budget;
        return prepare_native_polygon_parameters(points, identity, mode, budget);
    };
    const std::vector<Point3> square{{10, 20, 3}, {14, 20, 3}, {14, 22, 3}, {10, 22, 3}};
    auto independent = run(square);
    check(independent.complete && independent.mapping->transform_applied, "unit range mapped");
    check(independent.parameters == std::vector<Point2>{{0, 0}, {1, 0}, {1, 1}, {0, 1}},
          "independent unit axes");
    check(independent.face_distance_range == distance, "distance range retains original lengths");
    check(independent.projected == std::vector<Point2>{{10, 20}, {14, 20}, {14, 22}, {10, 22}},
          "projection retained separately");
    auto common = run(square, 1);
    check(common.parameters == std::vector<Point2>{{0, 0}, {1, 0}, {1, .5}, {0, .5}},
          "common unit preserves aspect ratio");
    auto actual = run(square, 2);
    check(actual.parameters == std::vector<Point2>{{0, 0}, {4, 0}, {4, 2}, {0, 2}},
          "distance mode keeps lengths and removes origin");
    for (unsigned mode : {3u, 999u, UINT32_MAX}) {
        auto unchanged = run(square, mode);
        check(unchanged.complete && !unchanged.mapping->transform_applied,
              "other modes successfully keep original coordinates");
        check(unchanged.parameters == unchanged.projected &&
                  unchanged.face_distance_range == distance,
              "unmapped coordinates still carry distance range");
        check(unchanged.mapping->parameter_range == source, "other mode range is original");
    }
    auto taller = map_native_polygon_parameter_range(source, 1, {1, 4});
    check(taller.distance_range == Range{{{0, 0}, {4, 8}}} &&
              taller.parameter_range == Range{{{0, 0}, {.5, 1}}},
          "axis factors applied before common-unit comparison");
    auto negative = map_native_polygon_parameter_range(source, 2, {-2, 3});
    check(negative.parameter_range == Range{{{-8, 0}, {0, 6}}},
          "range constructor sorts negative extent with origin");
    check(negative.transform[0][0] == 2 && negative.transform[0][3] == -28,
          "negative factor is not a signed mirror shortcut");
    auto zero_target = map_native_polygon_parameter_range(source, 2, {0, 0});
    check(zero_target.transform_applied && zero_target.transform[0][0] == 0 &&
              zero_target.transform[1][1] == 0,
          "zero target succeeds with nonzero source");
    auto collapsed = run({{4, 9, 0}, {4, 11, 0}});
    check(collapsed.complete && !collapsed.mapping->transform_applied &&
              collapsed.parameters == collapsed.projected,
          "collapsed source retains projection");
    check(collapsed.mapping->parameter_range == Range{{{0, 0}, {1, 1}}} &&
              collapsed.face_distance_range == Range{{{0, 0}, {0, 2}}},
          "failed transform still publishes chosen ranges");
    auto equal_zero = map_native_polygon_parameter_range(Range{{{2, 3}, {2, 3}}}, 1, {1, 1});
    check(!equal_zero.transform_applied && equal_zero.parameter_range == Range{{{0, 0}, {1, 1}}},
          "equal zero lengths select unit target but cannot divide");
    auto threshold = map_native_polygon_parameter_range(Range{{{0, 0}, {1e-15, 2}}}, 0, {1, 1});
    check(!threshold.transform_applied && threshold.transform == identity,
          "strict safe-division boundary leaves identity");
    auto above = map_native_polygon_parameter_range(
        Range{{{0, 0}, {std::nextafter(1e-15, 1.0), 2}}}, 0, {1, 1});
    check(above.transform_applied, "one representable step above threshold succeeds");
    auto null_source =
        map_native_polygon_parameter_range(Range{{{1e100, 0}, {2e100, 1}}}, 0, {1, 1});
    check(!null_source.transform_applied, "native range null magnitude guard");
    auto reversed = map_native_polygon_parameter_range(Range{{{4, 2}, {0, 0}}}, 0, {1, 1});
    check(reversed.transform_applied && reversed.transform[0][0] == -.25,
          "reversed bounds are not a native null range");
    const double marker = std::numeric_limits<double>::max();
    auto disconnects =
        run({{marker, 0, 0}, {2, 3, 0}, {0, marker, 0}, {4, 8, 0}, {0, 0, marker}}, 3);
    check(disconnects.projected == std::vector<Point2>{{0, 0}, {2, 3}, {2, 3}, {4, 8}, {4, 8}},
          "all three marker components repeat previous UV; first uses zero");
    check(disconnects.report["disconnect_count"] == 3, "disconnects counted");
    auto negative_marker = run({{-marker, 0, 0}, {-marker, 1, 0}}, 3);
    check(negative_marker.report["disconnect_count"] == 0 &&
              negative_marker.parameters[0][0] == -marker,
          "negative marker is ordinary coordinate");
    for (auto points : {std::vector<Point3>{}, std::vector<Point3>{{8, 9, 10}}}) {
        auto small = run(points);
        check(!small.complete && !small.face_distance_range && !small.mapping,
              "unwritten native face distance is explicit");
        if (!points.empty())
            check(small.parameters == std::vector<Point2>{{0, 0}}, "singleton is set to zero");
    }
    Matrix4 tilted = identity;
    tilted[0] = {1, 0, 0, -10};
    tilted[1] = {0, 0, 1, -30};
    tilted[2] = {0, -1, 0, 20};
    TubeBudget budget;
    auto transformed =
        prepare_native_polygon_parameters({{10, 20, 30}, {14, 20, 32}}, tilted, 0, budget);
    check(transformed.parameters == std::vector<Point2>{{0, 0}, {1, 1}},
          "supplied three-dimensional frame");
    // Actual cap pipeline retains all ring closures, markers, and new slots.
    std::vector<Point3> hole{{11, 20.5, 3}, {12, 20.5, 3}, {12, 21, 3}, {11, 21, 3}};
    auto cap = prepare_native_tube_mesh_cap_input({square, hole}, false, 0, budget);
    auto polygon = triangulate_native_projected_polygon(cap.points, cap.projection.local_to_world,
                                                        cap.projection.world_to_local, 0, budget);
    auto uv =
        prepare_native_polygon_parameters(polygon.points, cap.projection.world_to_local, 0, budget);
    check(polygon.complete && uv.complete && uv.parameters.size() == polygon.points.size(),
          "multi-ring cap projection and UV connected");
    check(uv.parameters[5] == uv.parameters[4] &&
              uv.parameters.back() == uv.parameters[uv.parameters.size() - 2],
          "cap disconnect slot inherits prior ring UV");
    check(uv.face_distance_range == distance, "cap distance axes are unscaled world lengths");
    for (auto token : polygon.indices) {
        if (!token)
            continue;
        const auto p = uv.parameters[static_cast<std::size_t>(std::abs(token) - 1)];
        check(p[0] >= 0 && p[0] <= 1 && p[1] >= 0 && p[1] <= 1,
              "emitted triangle references valid cap UV");
    }
    NativePolyfaceFaceDataState state;
    state.mesh.coordinates.points = square;
    state.mesh.coordinates.parameters = independent.parameters;
    state.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 3, 4, 0};
    state.mesh.indices.indices[parameter_channel] = state.mesh.indices.indices[point_channel];
    state.mesh.indices.active[face_channel] = true;
    state.mesh.indices.active[parameter_channel] = true;
    state.parameter_pool_active = true;
    auto record = native_null_face_data();
    record.parameter_distance_range = *independent.face_distance_range;
    record.source_index = 77;
    auto face = set_native_polyface_face_data(state, record, 0, budget);
    check(face.complete && face.output.mesh.face_data.size() == 1, "end-face consumes UV range");
    check(face.output.mesh.face_data[0].parameter_distance_range == distance &&
              face.output.mesh.face_data[0].parameter_range == Range{{{0, 0}, {1, 1}}} &&
              face.output.mesh.face_data[0].source_index == 77,
          "end-face keeps distance range, computes UV bounds, retains metadata");
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "unsafe parameter input or exhausted budget rejected");
    };
    rejects([&] { run({{NAN, 0, 0}, {1, 1, 1}}); });
    rejects([&] { map_native_polygon_parameter_range(source, 0, {INFINITY, 1}); });
    rejects([&] {
        TubeBudget tiny;
        tiny.max_work = 10;
        prepare_native_polygon_parameters(square, identity, 0, tiny);
    });
    rejects([&] {
        TubeBudget tiny;
        tiny.max_control_points = 3;
        prepare_native_polygon_parameters(square, identity, 0, tiny);
    });
    auto parallel = std::async(std::launch::async, [&] { return run(square).parameters; });
    check(parallel.get() == independent.parameters, "independent concurrent calls");
    return checks;
}
