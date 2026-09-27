#include "native_polyface_attributes.hpp"
#include "native_polygon_projection.hpp"
#include <cmath>
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_attributes_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto near = [](double a, double b) { return std::abs(a - b) < 1e-12; };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "attribute generation rejects unsafe data and exhausted budgets");
    };
    auto rectangle = [] {
        NativePolyfaceFaceDataState s;
        s.mesh.coordinates.points = {{10, 20, 30}, {14, 20, 30}, {14, 22, 30}, {10, 22, 30}};
        s.mesh.indices.indices[point_channel] = {1, -2, 3, 4, 0};
        return s;
    };
    TubeBudget b;
    const auto original = rectangle();
    const auto normals = build_native_polyface_normals(original, b);
    check(normals.complete && normals.native_succeeded && normals.output.normal_pool_active &&
              normals.output.mesh.indices.active[normal_channel],
          "face normal generation activates both native channels");
    check(normals.output.mesh.coordinates.normals == std::vector<Point3>{{0, 0, 1}} &&
              normals.output.mesh.indices.indices[normal_channel] ==
                  std::vector<std::int32_t>{1, 1, 1, 1, 0},
          "one unit normal per face ignores visibility signs and preserves terminators");
    for (int selector : {0, 1, 2, 3, 7, -1}) {
        const auto r = build_native_polyface_parameters(original, selector, b);
        check(r.native_succeeded && r.complete && r.output.parameter_pool_active &&
                  r.output.mesh.indices.active[parameter_channel],
              "all selector branches produce active UV channels");
        const double x = selector == 2 || selector == 3 ? 1 : 4;
        const double y = selector == 2 ? 1 : selector == 3 ? .5 : 2;
        check(r.output.mesh.coordinates.parameters ==
                      std::vector<Point2>{{0, 0}, {x, 0}, {x, y}, {0, y}} &&
                  r.output.mesh.indices.indices[parameter_channel] ==
                      std::vector<std::int32_t>{1, 2, 3, 4, 0},
              "UV corners follow native unit-axis, independent-axis and larger-axis scales");
        auto f = prepare_native_polygon_projection(original.mesh.coordinates.points, b, selector);
        const double z = selector == 2 ? std::sqrt(8.0) : selector == 3 ? 4 : 1;
        check(near(f.local_to_world[2][2], z) && near(f.world_to_local[2][2], 1 / z),
              "frame Z scale uses geometric mean of XY scales");
        for (std::size_t i = 0; i < f.points.size(); ++i) {
            Point3 world{};
            for (unsigned k = 0; k < 3; ++k) {
                const auto &row = f.local_to_world[k];
                world[k] = (f.points[i][1] * row[1] + f.points[i][0] * row[0]) +
                           f.points[i][2] * row[2] + row[3];
            }
            check(world == original.mesh.coordinates.points[i],
                  "adjusted frame remains an inverse pair on analytical rectangle");
        }
    }
    {
        const std::vector<Point3> p{{1, 1, 0}, {2, 1, 0}, {0, 2, 0}};
        auto f = prepare_native_polygon_projection(p, b, 1);
        check(f.frame_succeeded &&
                  f.points == std::vector<Point3>{{1, 0, 0}, {2, 0, 0}, {0, 1, 0}} &&
                  f.local_to_world[0][3] == 0 && f.local_to_world[1][3] == 1,
              "lower-left adjustment changes matrix origins before scaling");
        f = prepare_native_polygon_projection(p, b, 2);
        check(f.points == std::vector<Point3>{{.5, 0, 0}, {1, 0, 0}, {0, 1, 0}},
              "both-axis unit range keeps a displaced first vertex");
        const std::vector<Point3> mixed{{-1, 0, 0}, {0, 0, 0}, {-2, 1, 0}};
        f = prepare_native_polygon_projection(mixed, b, 1);
        check(f.report["origin_shifted"] == false && f.points[2][0] == -1,
              "native mixed-frame AlmostEqual can skip a nonzero local lower-left translation");
        const std::vector<Point3> small{{0, 0, 0}, {2, 0, 0}, {2, 1, 0}, {-1e-12, 1, 0}};
        f = prepare_native_polygon_projection(small, b, 1);
        check(f.report["origin_shifted"] == false && f.points.back()[0] < 0,
              "strict squared native tolerance preserves a sufficiently small lower-left offset");
        const std::vector<Point3> line{{0, 0, 0}, {2, 0, 0}, {4, 0, 0}};
        f = prepare_native_polygon_projection(line, b, 2);
        check(f.frame_succeeded && f.report["range_scale_applied"] == false && f.points == line,
              "zero second span prevents all three scale operations, without failing the frame");
        f = prepare_native_polygon_projection(line, b, 3);
        check(f.frame_succeeded && f.report["range_scale_applied"] == true &&
                  f.points[2] == Point3{1, 0, 0},
              "larger-axis mode can scale an otherwise collapsed second range");
        const double marker = std::numeric_limits<double>::max();
        auto disconnected = original.mesh.coordinates.points;
        disconnected.push_back({marker, 17, 19});
        disconnected.push_back({18, 24, 30});
        f = prepare_native_polygon_projection(disconnected, b, 2);
        check(f.frame_succeeded && f.points[4] == disconnected[4] &&
                  f.points[5] == Point3{1, 1, 0} && f.points[1] == Point3{.5, 0, 0},
              "frame uses first loop normal but range includes later points and preserves "
              "disconnects");
        auto matrices =
            prepare_native_polygon_projection(original.mesh.coordinates.points, b, 0, false);
        check(matrices.frame_succeeded && matrices.points.empty() &&
                  matrices.local_to_world[2][2] == 1,
              "normal caller requests the frame without unnecessary point projection");
    }
    {
        auto s = rectangle();
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 4, 0, 4, 3, 2, 1, 0};
        auto n = build_native_polyface_normals(s, b);
        check(n.complete &&
                  n.output.mesh.coordinates.normals == std::vector<Point3>{{0, 0, 1}, {0, 0, -1}} &&
                  n.output.mesh.indices.indices[normal_channel] ==
                      std::vector<std::int32_t>{1, 1, 1, 1, 0, 2, 2, 2, 2, 0},
              "opposite winding has its own opposite face normal");
        auto uv = build_native_polyface_parameters(n.output, 2, b);
        check(uv.complete && uv.output.mesh.coordinates.parameters.size() == 8 &&
                  uv.output.mesh.indices.indices[parameter_channel] ==
                      std::vector<std::int32_t>{1, 2, 3, 4, 0, 5, 6, 7, 8, 0} &&
                  uv.output.mesh.coordinates.normals == n.output.mesh.coordinates.normals,
              "per-corner UV is deliberately not deduplicated and preserves generated normals");
        auto faces = build_native_polyface_face_data(uv.output, b);
        auto joined = assemble_native_builder_polyfaces({faces.output.mesh}, {}, b);
        check(faces.complete && joined.complete && joined.face_data.size() == 1 &&
                  joined.coordinates.normals.size() == 2,
              "normals, UV, face records and matched assembly form a working prepared-mesh chain");
        s = rectangle();
        s.mesh.coordinates.normals = {{9, 8, 7}};
        s.mesh.indices.indices[normal_channel] = {999};
        s.mesh.coordinates.parameters = {{6, 5}};
        s.mesh.indices.indices[parameter_channel] = {-2};
        // Clear the other unsafe channel; the selected old channel is replaced before visiting.
        auto for_normals = s;
        for_normals.mesh.indices.indices[parameter_channel].clear();
        n = build_native_polyface_normals(for_normals, b);
        check(n.complete &&
                  n.output.mesh.coordinates.normals == normals.output.mesh.coordinates.normals &&
                  n.output.mesh.coordinates.parameters == s.mesh.coordinates.parameters,
              "normal generation discards prior normal indices and values before visiting");
        s.mesh.indices.indices[normal_channel].clear();
        uv = build_native_polyface_parameters(s, 0, b);
        check(uv.complete && uv.output.mesh.coordinates.parameters.size() == 4 &&
                  uv.output.mesh.coordinates.normals == s.mesh.coordinates.normals,
              "parameter generation replaces only its pool and index channel");
    }
    {
        auto s = rectangle();
        s.mesh.coordinates.points = {{7, 8, 9}, {7, 8, 9}, {7, 8, 9}};
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0};
        auto n = build_native_polyface_normals(s, b);
        auto uv = build_native_polyface_parameters(s, 2, b);
        check(
            n.native_succeeded && !n.complete && n.output.mesh.coordinates.normals.empty() &&
                n.output.mesh.indices.indices[normal_channel] ==
                    std::vector<std::int32_t>{0, 0, 0, 0} &&
                n.report["failed_frames"] == 1,
            "failed frame leaves zero normal indices without importing the newer fallback normal");
        check(uv.native_succeeded && !uv.complete &&
                  uv.output.mesh.coordinates.parameters == std::vector<Point2>(3) &&
                  uv.output.mesh.indices.indices[parameter_channel] ==
                      std::vector<std::int32_t>{1, 2, 3, 0},
              "failed UV frame still publishes one zero coordinate per corner and reports the "
              "fallback");
        s.mesh.coordinates.points = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
        n = build_native_polyface_normals(s, b);
        check(n.complete && n.output.mesh.coordinates.normals == std::vector<Point3>{{0, 0, 1}},
              "zero area alone does not imply frame failure in this original coordinateFrame");
        s.mesh.indices.indices[point_channel] = {1, 2, 0};
        n = build_native_polyface_normals(s, b);
        uv = build_native_polyface_parameters(s, 0, b);
        check(!n.complete && !uv.complete && uv.output.mesh.coordinates.parameters.size() == 2,
              "two-corner face follows native failed-frame behavior");
        s = rectangle();
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 4, 0, 1, 2, 99, 0, 1, 2, 3, 0};
        n = build_native_polyface_normals(s, b);
        check(n.native_succeeded && !n.complete && n.output.mesh.coordinates.normals.size() == 1 &&
                  n.report["invalid_point_facets"] == 1 &&
                  n.report["unassigned_nonzero_indices"] == 6,
              "invalid point face ends traversal and does not conceal remaining unassigned faces");
        s = rectangle();
        s.mesh.num_per_face = 4;
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 3, 4, 0};
        n = build_native_polyface_normals(s, b);
        check(n.complete && n.output.mesh.coordinates.normals.size() == 2 &&
                  n.output.mesh.indices.indices[normal_channel] ==
                      std::vector<std::int32_t>{1, 1, 1, 0, 2, 2, 2, 0},
              "fixed padded blocks use actual corner positions for normal assignment");
        s.mesh.indices.indices[point_channel] = {0, 0, 0, 0, 1, 2, 3, 0};
        n = build_native_polyface_normals(s, b);
        check(!n.complete && n.output.mesh.coordinates.normals.empty(),
              "fixed empty block stops native traversal");
        s = rectangle();
        s.mesh.indices.indices[point_channel] = {0, 1, 2, 3, 4, 0, 0, 1, 2, 3, 4};
        uv = build_native_polyface_parameters(s, 1, b);
        check(
            uv.complete && uv.output.mesh.indices.indices[parameter_channel] ==
                               std::vector<std::int32_t>{0, 1, 2, 3, 4, 0, 0, 5, 6, 7, 8},
            "variable traversal preserves repeated zeros and supports an unterminated final face");
    }
    {
        auto s = rectangle();
        s.mesh.face_data = {native_null_face_data(), native_null_face_data()};
        s.mesh.face_data[0].source_index = 999;
        s.mesh.face_data[0].parameter_distance_range = {Point2{7, 8}, Point2{9, 10}};
        s.mesh.indices.active[face_channel] = true;
        s.mesh.indices.indices[face_channel] = {1, 1, 1, 1, 0};
        auto uv = build_native_polyface_parameters(s, 2, b);
        check(uv.complete &&
                  uv.output.mesh.face_data[0].parameter_range ==
                      native_null_face_data().parameter_range &&
                  uv.output.mesh.face_data[1].parameter_range ==
                      std::array<Point2, 2>{Point2{0, 0}, Point2{1, 1}} &&
                  uv.report["face_range_updates"] == 4,
              "native UV writer indexes the second face record directly for stored face value one");
        check(uv.output.mesh.face_data[0].source_index == 999 &&
                  uv.output.mesh.face_data[0].parameter_distance_range ==
                      s.mesh.face_data[0].parameter_distance_range,
              "UV generation clears parameter ranges but preserves metadata and distance bounds");
        s.mesh.indices.indices[face_channel] = {0, 0, 0, 0, 0};
        uv = build_native_polyface_parameters(s, 2, b);
        check(uv.complete && uv.output.mesh.face_data[0].parameter_range[1] == Point2{1, 1},
              "zero stored face value directly updates record zero in the original UV writer");
        s.mesh.indices.indices[face_channel] = {-1, 2, 99, 1, 0};
        uv = build_native_polyface_parameters(s, 0, b);
        check(!uv.complete && uv.report["face_range_updates_skipped"] == 3 &&
                  uv.report["face_range_updates"] == 1,
              "negative and out-of-range face values are skipped without remapping and reported");
        s.mesh.indices.active[face_channel] = false;
        uv = build_native_polyface_parameters(s, 0, b);
        check(
            uv.complete && uv.report["face_range_updates"] == 0 &&
                uv.output.mesh.face_data[1].parameter_range ==
                    native_null_face_data().parameter_range,
            "inactive face indices prevent extension but not unconditional parameter-range reset");
        s.mesh.indices.indices[point_channel].clear();
        s.mesh.coordinates.parameters = {{7, 8}};
        s.mesh.face_data[1].parameter_range = {Point2{9, 10}, Point2{11, 12}};
        uv = build_native_polyface_parameters(s, 2, b);
        check(!uv.native_succeeded && !uv.complete &&
                  uv.output.mesh.coordinates.parameters == s.mesh.coordinates.parameters &&
                  uv.output.mesh.face_data[1].parameter_range ==
                      s.mesh.face_data[1].parameter_range,
              "empty point indices return false before altering old parameters or face ranges");
    }
    {
        for (bool parameters : {false, true}) {
            TubeBudget measured;
            auto run = [&](const NativePolyfaceFaceDataState &s, TubeBudget &limit) {
                return parameters ? build_native_polyface_parameters(s, 2, limit)
                                  : build_native_polyface_normals(s, limit);
            };
            run(original, measured);
            for (auto value :
                 {std::size_t(0), std::size_t(30), measured.work / 2, measured.work - 1}) {
                TubeBudget limited;
                limited.max_work = value;
                rejects([&] { run(original, limited); });
            }
            for (auto value : {std::size_t(0), std::size_t(9), std::size_t(14)}) {
                TubeBudget limited;
                limited.max_control_points = value;
                rejects([&] { run(original, limited); });
            }
            auto s = original;
            s.mesh.coordinates.points[0][0] = std::numeric_limits<double>::infinity();
            rejects([&] { run(s, b); });
        }
        check(original.mesh.coordinates.parameters.empty() &&
                  original.mesh.coordinates.normals.empty(),
              "success and all failures leave source pools untouched");
        auto run = [&] {
            TubeBudget local;
            return build_native_polyface_parameters(original, 2, local);
        };
        auto f1 = std::async(std::launch::async, run), f2 = std::async(std::launch::async, run);
        auto a = f1.get(), c = f2.get();
        check(a.complete && c.complete && a.report == c.report &&
                  a.output.mesh.coordinates.parameters == c.output.mesh.coordinates.parameters,
              "independent attribute operations use local state and deterministic results "
              "concurrently");
    }
    return checks;
}
