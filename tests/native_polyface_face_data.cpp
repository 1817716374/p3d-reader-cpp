#include "native_polyface_face_data.hpp"
#include <cmath>
#include <future>
#include <limits>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_face_data_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto close = [](double a, double b) { return std::abs(a - b) < 1e-12; };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "face data rejects unsafe reads and exhausted budgets");
    };
    auto triangle = [] {
        NativePolyfaceFaceDataState s;
        s.parameter_pool_active = true;
        s.mesh.coordinates.points = {{0, 0, 0}, {2, 0, 0}, {0, 3, 0}};
        s.mesh.coordinates.normals = {{0, 0, 2}, {0, 0, 4}, {0, 0, 8}};
        s.mesh.coordinates.parameters = {{0, 0}, {1, 0}, {0, 1}};
        s.mesh.indices.indices[point_channel] = {1, -2, 3, 0};
        s.mesh.indices.active[face_channel] = true;
        return s;
    };
    TubeBudget b;
    const auto source = triangle();
    const auto base = build_native_polyface_face_data(source, b);
    const auto &data = base.output.mesh.face_data[0];
    check(base.complete && base.output.mesh.face_data.size() == 1,
          "UV triangle produces one face record");
    check(data.parameter_distance_range == std::array<Point2, 2>{Point2{0, 0}, Point2{2, 3}},
          "analytic physical UV scales are two and three");
    check(data.point_range == std::array<Point3, 2>{Point3{0, 0, 0}, Point3{2, 3, 0}} &&
              data.normal_range == std::array<Point3, 2>{Point3{0, 0, 2}, Point3{0, 0, 8}},
          "ranges use raw source coordinates and unnormalized normals");
    check(data.source_index == 0 && data.face_indices == std::array<std::int64_t, 3>{0, 0, -1} &&
              base.output.mesh.indices.indices[face_channel] ==
                  std::vector<std::int32_t>{1, 1, 1, 0},
          "default metadata and signed point references map to one positive face");
    check(base.output.mesh.indices.active[face_channel] && !base.output.face_data_pool_active &&
              base.report["visitor_wrap_count"] == 0,
          "native face-index flag changes without activating face data or wrapping");
    check(source.mesh.face_data.empty() && source.mesh.indices.indices[face_channel].empty(),
          "source remains immutable");
    {
        auto s = triangle();
        s.mesh.coordinates.parameters = {{10, 20}, {12, 20}, {10, 23}};
        auto direct = set_native_polyface_face_data(s, std::nullopt, 0, b);
        auto built = build_native_polyface_face_data(s, b);
        check(direct.output.mesh.face_data[0].parameter_range ==
                      std::array<Point2, 2>{Point2{10, 20}, Point2{12, 23}} &&
                  direct.output.mesh.face_data[0].parameter_distance_range ==
                      data.parameter_distance_range,
              "distance scale multiplies the original UV span before any normalization");
        check(
            built.output.mesh.face_data[0].parameter_range ==
                    std::array<Point2, 2>{Point2{0, 0}, Point2{1, 1}} &&
                built.output.mesh.face_data[0].parameter_distance_range ==
                    data.parameter_distance_range &&
                built.output.mesh.coordinates.parameters == s.mesh.coordinates.parameters,
            "Build changes only the face parameter range, not source UV coordinates or distances");
        NativeBuilderFaceData supplied = native_null_face_data();
        supplied.source_index = 0xfedcba9876543210ULL;
        supplied.face_indices = {-8, 999, 3};
        supplied.parameter_range = {Point2{7, 8}, Point2{9, 11}};
        supplied.parameter_distance_range = {Point2{4, 5}, Point2{40, 50}};
        supplied.point_range = {Point3{7, 7, 7}, Point3{8, 8, 8}};
        supplied.normal_range = supplied.point_range;
        direct = set_native_polyface_face_data(s, supplied, 0, b);
        const auto &f = direct.output.mesh.face_data[0];
        check(f.source_index == supplied.source_index && f.face_indices == supplied.face_indices &&
                  f.parameter_range == supplied.parameter_range &&
                  f.parameter_distance_range == supplied.parameter_distance_range,
              "explicit metadata and nonnull parameter ranges are preserved");
        check(f.point_range == data.point_range && f.normal_range == data.normal_range,
              "supplied XYZ and normal bounds are always recomputed");
        supplied.parameter_range = {Point2{5, 6}, Point2{1, 2}};
        supplied.parameter_distance_range = native_null_face_data().parameter_distance_range;
        direct = set_native_polyface_face_data(source, supplied, 0, b);
        check(direct.output.mesh.face_data[0].parameter_range == supplied.parameter_range &&
                  direct.output.mesh.face_data[0].parameter_distance_range[1] == Point2{-8, -12},
              "reversed finite ranges are not native null ranges and preserve negative spans");
        supplied.parameter_range = {Point2{1e100, 0}, Point2{-1e100, 0}};
        direct = set_native_polyface_face_data(source, supplied, 0, b);
        check(direct.output.mesh.face_data[0].parameter_range ==
                  std::array<Point2, 2>{Point2{0, 0}, Point2{1, 1}},
              "native null threshold includes exactly 1e100 and extends existing bounds");
    }
    {
        auto s = triangle();
        s.mesh.coordinates.points.insert(s.mesh.coordinates.points.end(),
                                         {{0, 0, 0}, {4, 0, 0}, {0, 3, 0}});
        s.mesh.coordinates.parameters.insert(s.mesh.coordinates.parameters.end(),
                                             {{0, 0}, {1, 0}, {0, 1}});
        s.mesh.coordinates.normals.clear();
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 4, 5, 6, 0};
        auto r = build_native_polyface_face_data(s, b);
        check(r.complete && r.output.mesh.face_data.size() == 1 &&
                  r.output.mesh.indices.indices[face_channel] ==
                      std::vector<std::int32_t>{1, 1, 1, 0, 1, 1, 1, 0},
              "UV-bearing facets share one remaining face record");
        check(r.output.mesh.face_data[0].parameter_distance_range[1] == Point2{4, 3} &&
                  r.report["calls"][0]["distance_samples"] == 2,
              "equal-weight samples two and four yield mean plus standard deviation four");
        auto first = set_native_polyface_face_data(s, std::nullopt, 4, b);
        check(!first.complete && first.output.mesh.face_data[0].point_range[1] == Point3{2, 3, 0} &&
                  first.output.mesh.face_data[0].parameter_distance_range[1] == Point2{2, 3},
              "end read-index excludes the next facet from both statistics and bounds");
        auto second = set_native_polyface_face_data(first.output, std::nullopt, 0, b);
        check(second.complete && second.output.mesh.face_data.size() == 2 &&
                  second.output.mesh.face_data[1].parameter_distance_range[1] == Point2{4, 3} &&
                  second.output.mesh.indices.indices[face_channel] ==
                      std::vector<std::int32_t>{1, 1, 1, 0, 2, 2, 2, 0},
              "continuation starts at the existing face-index length");
        s.mesh.coordinates.parameters.clear();
        r = build_native_polyface_face_data(s, b);
        check(r.complete && r.output.mesh.face_data.size() == 2 &&
                  r.output.mesh.indices.indices[face_channel] ==
                      second.output.mesh.indices.indices[face_channel],
              "without UV the original Build assigns one record to each visited facet");
        check(r.output.mesh.face_data[0].parameter_range == native_null_face_data().parameter_range,
              "missing UV leaves native null ranges unchanged");
    }
    {
        // A nonlinear quad distinguishes rolling triples (012,123) from a fan (012,023).
        auto s = triangle();
        s.mesh.coordinates.points = {{0, 0, 0}, {2, 0, 0}, {2, 3, 0}, {0, 6, 0}};
        s.mesh.coordinates.parameters = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
        s.mesh.coordinates.normals.clear();
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 4, 0};
        const auto r = build_native_polyface_face_data(s, b);
        check(close(r.output.mesh.face_data[0].parameter_distance_range[1][0], std::sqrt(13.0)) &&
                  close(r.output.mesh.face_data[0].parameter_distance_range[1][1], 3),
              "consecutive triples retain the native nonfan distance statistics");
        s.mesh.coordinates.parameters = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
        auto zero = build_native_polyface_face_data(s, b);
        check(zero.complete &&
                  zero.output.mesh.face_data[0].parameter_distance_range ==
                      native_null_face_data().parameter_distance_range &&
                  zero.report["calls"][0]["distance_samples"] == 0,
              "exactly degenerate UV triples leave distance unassigned without inventing a scale");
    }
    {
        auto s = triangle();
        s.parameter_pool_active = false;
        s.face_data_pool_active = true;
        auto r = build_native_polyface_face_data(s, b);
        check(r.complete && r.output.face_data_pool_active &&
                  r.output.mesh.face_data[0].parameter_range == data.parameter_range &&
                  r.output.mesh.face_data[0].parameter_distance_range ==
                      native_null_face_data().parameter_distance_range,
              "inactive UV pool still groups facets and normalizes the range but skips distance "
              "calculation");
        s = base.output;
        s.mesh.face_data[0].parameter_range = {Point2{7, 8}, Point2{9, 10}};
        r = build_native_polyface_face_data(s, b);
        check(r.complete && r.output.mesh.face_data.size() == 1 &&
                  r.report["calls"][0]["appended"] == false &&
                  r.output.mesh.face_data[0].parameter_range == data.parameter_range,
              "Build resets existing parameter ranges even when Set has no remaining indices");
        s.mesh.indices.indices[face_channel][0] = 99;
        r = build_native_polyface_face_data(s, b);
        check(!r.complete && !r.report["face_references_valid"].get<bool>() &&
                  r.output.mesh.indices.indices[face_channel][0] == 99,
              "existing invalid references remain visible and cannot claim complete coverage");
        s = triangle();
        s.mesh.indices.active[face_channel] = false;
        r = set_native_polyface_face_data(s, std::nullopt, 999, b);
        check(!r.complete && r.output.mesh.face_data.empty(),
              "inactive Set is a native no-op before checking end");
        NativePolyfaceFaceDataState empty;
        r = build_native_polyface_face_data(empty, b);
        check(r.complete && r.output.mesh.indices.active[face_channel] &&
                  !r.output.face_data_pool_active && r.output.mesh.face_data.empty(),
              "empty Build retains the original flag behavior");
    }
    {
        auto s = triangle();
        s.mesh.coordinates.parameters.clear();
        s.mesh.num_per_face = 4;
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 3, 2, 0};
        auto r = build_native_polyface_face_data(s, b);
        check(!r.complete && r.output.mesh.face_data.size() == 1 &&
                  r.output.mesh.indices.indices[face_channel] == std::vector<std::int32_t>{1, 1, 1},
              "fixed padded face ends before padding and the next Set starts at a leading zero");
        s.mesh.num_per_face = 3;
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 1, 3, 2};
        r = build_native_polyface_face_data(s, b);
        check(r.complete && r.output.mesh.face_data.size() == 2 &&
                  r.output.mesh.indices.indices[face_channel] ==
                      std::vector<std::int32_t>{1, 1, 1, 2, 2, 2},
              "un-padded fixed blocks generate separate records without terminators");
        s.mesh.num_per_face = 0;
        s.mesh.indices.indices[point_channel] = {0, 0, 1, 2, 3, 0, 0, 1, 3, 2, 0};
        r = build_native_polyface_face_data(s, b);
        check(r.complete && r.output.mesh.face_data.size() == 2 &&
                  r.output.mesh.indices.indices[face_channel] ==
                      std::vector<std::int32_t>{0, 0, 1, 1, 1, 0, 0, 2, 2, 2, 0},
              "variable visitor skips leading and repeated terminators while assignments preserve "
              "them");
        s.mesh.num_per_face = 1;
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 3, 2, 0};
        r = build_native_polyface_face_data(s, b);
        check(!r.complete && r.output.mesh.face_data.size() == 2 &&
                  r.output.mesh.indices.indices[face_channel] ==
                      std::vector<std::int32_t>{1, 1, 1, 0, 2, 2, 2},
              "num-per-face one uses variable traversal but omits the last terminator in Set end");
        s = triangle();
        s.mesh.face_data = {native_null_face_data()};
        s.mesh.indices.indices[face_channel] = {1};
        r = set_native_polyface_face_data(s, std::nullopt, 0, b);
        check(r.complete && r.output.mesh.face_data[1].point_range[0] == Point3{0, 0, 0} &&
                  r.report["calls"][0]["distance_samples"] == 0 &&
                  r.output.mesh.indices.indices[face_channel] ==
                      std::vector<std::int32_t>{1, 2, 2, 0},
              "partial existing index list starts inside the source face with no fabricated "
              "closing sample");
        r = set_native_polyface_face_data(s, std::nullopt, 1, b);
        check(!r.complete && r.output.mesh.face_data.size() == 2 &&
                  r.output.mesh.indices.indices[face_channel].size() == 1,
              "native do-while appends a record even when requested end equals the starting index");
    }
    {
        auto s = triangle();
        s.mesh.indices.indices[point_channel] = {1, 2, 99, 0};
        auto r = build_native_polyface_face_data(s, b);
        check(!r.complete && r.output.mesh.face_data.empty() &&
                  r.report["invalid_point_facets"] == 1,
              "invalid point index stops the visitor and is reported");
        s = triangle();
        s.mesh.indices.indices[parameter_channel] = {1, 0, 3, 0};
        rejects([&] { build_native_polyface_face_data(s, b); });
        s = triangle();
        s.mesh.indices.indices[normal_channel] = {1};
        rejects([&] { build_native_polyface_face_data(s, b); });
        s = triangle();
        s.mesh.indices.indices[normal_channel] = {0, 0, 0, 0};
        r = build_native_polyface_face_data(s, b);
        check(!r.complete &&
                  r.output.mesh.face_data[0].normal_range == native_null_face_data().normal_range,
              "empty truncated normal visitor keeps null bounds and reports incompleteness");
        s = triangle();
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 2, 3, 0};
        s.mesh.indices.indices[parameter_channel] = {1, 2, 3, 0, 0, 0, 0, 0};
        r = build_native_polyface_face_data(s, b);
        check(!r.complete && r.report["calls"][0]["distance_samples"] == 1 &&
                  r.report["calls"][0]["distance_range_assigned"] == false &&
                  r.output.mesh.face_data[0].parameter_distance_range ==
                      native_null_face_data().parameter_distance_range,
              "missing UV on a later facet discards previously accumulated distance statistics");
        s = triangle();
        s.mesh.coordinates.parameters.clear();
        s.mesh.coordinates.points[1][0] = std::numeric_limits<double>::max();
        r = build_native_polyface_face_data(s, b);
        check(r.output.mesh.face_data[0].point_range ==
                  std::array<Point3, 2>{Point3{0, 0, 0}, Point3{0, 3, 0}},
              "3D disconnect sentinel skips the entire point during bounds extension");
        s = triangle();
        s.mesh.coordinates.points[0][0] = std::numeric_limits<double>::infinity();
        rejects([&] { build_native_polyface_face_data(s, b); });
        s = triangle();
        rejects([&] { set_native_polyface_face_data(s, std::nullopt, 5, b); });
        s.mesh.coordinates.parameters.clear();
        s.mesh.indices.indices[point_channel].pop_back();
        rejects([&] { build_native_polyface_face_data(s, b); });
    }
    {
        NativeBuilderPolyfaceOptions options;
        auto r =
            assemble_native_builder_polyfaces({base.output.mesh, base.output.mesh}, options, b);
        check(r.complete && r.face_data.size() == 2 && r.coordinates.points.size() == 3 &&
                  r.face_data[0].parameter_distance_range == data.parameter_distance_range &&
                  r.indices.indices[face_channel] ==
                      std::vector<std::int32_t>{1, 1, 1, 0, 2, 2, 2, 0},
              "generated face records pass through native matched assembly with independent "
              "associations");
        TubeBudget measured;
        build_native_polyface_face_data(source, measured);
        for (auto limit : {std::size_t(0), std::size_t(20), measured.work / 2, measured.work - 1}) {
            TubeBudget limited;
            limited.max_work = limit;
            rejects([&] { build_native_polyface_face_data(source, limited); });
        }
        for (auto limit : {std::size_t(0), std::size_t(13), std::size_t(17)}) {
            TubeBudget limited;
            limited.max_control_points = limit;
            rejects([&] { build_native_polyface_face_data(source, limited); });
        }
        check(source.mesh.face_data.empty() && source.mesh.coordinates.normals[2][2] == 8,
              "failed budget attempts preserve source");
    }
    auto run = [&] {
        TubeBudget local;
        return build_native_polyface_face_data(source, local);
    };
    auto f1 = std::async(std::launch::async, run), f2 = std::async(std::launch::async, run);
    const auto r1 = f1.get(), r2 = f2.get();
    check(r1.report == base.report && r2.report == base.report &&
              r1.output.mesh.face_data[0].parameter_distance_range ==
                  r2.output.mesh.face_data[0].parameter_distance_range,
          "independent face-data operations are deterministic and safe to run concurrently");
    return checks;
}
