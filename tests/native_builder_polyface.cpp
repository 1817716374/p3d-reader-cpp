#include "native_builder_polyface.hpp"
#include <future>
#include <limits>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_builder_polyface_tests() {
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
        check(caught, "native builder rejects unsafe access and exhausted budgets");
    };
    auto triangle = [] {
        NativeBuilderPolyface s;
        s.coordinates.points = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
        s.coordinates.normals = {{0, 0, 2}, {0, 0, 4}, {0, 0, 8}};
        s.coordinates.parameters = {{0, 0}, {1, 0}, {0, 1}};
        s.indices.indices[point_channel] = {1, -2, 3, 0};
        s.indices.indices[normal_channel] = {1, -2, 3, 0};
        s.indices.indices[parameter_channel] = {-1, 2, 3, 0};
        s.indices.indices[face_channel] = {1, 1, 1, 0};
        NativeBuilderFaceData d;
        d.parameter_distance_range = {Point2{2, 3}, Point2{4, 5}};
        d.parameter_range = {Point2{6, 7}, Point2{8, 9}};
        d.point_range = {Point3{10, 11, 12}, Point3{13, 14, 15}};
        d.normal_range = {Point3{16, 17, 18}, Point3{19, 20, 21}};
        d.source_index = 0xfedcba9876543210ULL;
        d.face_indices = {-1, 123, 456};
        s.face_data = {d};
        s.edge_chains = {{14, {42, 0xffffffffu}, {1, 2, 3, 1}}};
        return s;
    };
    NativeBuilderPolyfaceOptions defaults;
    TubeBudget b;
    auto a = triangle(), c = triangle();
    c.coordinates.points = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    c.coordinates.parameters = {{1, 0}, {1, 1}, {0, 1}};
    c.coordinates.normals = {{0, 0, -2}, {0, 0, -4}, {0, 0, -8}};
    c.two_sided = true;
    c.face_data[0].source_index = 1234567;
    c.face_data[0].face_indices = {7, 8, 9};
    auto result = assemble_native_builder_polyfaces({a, c}, defaults, b);
    check(result.native_succeeded && result.complete, "two prepared polyfaces assemble completely");
    check(result.coordinates.points ==
                  std::vector<Point3>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}} &&
              result.indices.indices[point_channel] ==
                  std::vector<std::int32_t>{1, -2, 3, 0, 2, -4, 3, 0},
          "assembly connects independent local indices with original point visibility");
    check(result.indices.indices[normal_channel] ==
                  std::vector<std::int32_t>{1, 1, 1, 0, 2, 2, 2, 0} &&
              result.coordinates.normals == std::vector<Point3>{{0, 0, 1}, {0, 0, -1}},
          "opposite normals remain independent while positive magnitude indices are remapped");
    check(result.indices.indices[parameter_channel] ==
              std::vector<std::int32_t>{1, 2, 3, 0, 2, 4, 3, 0},
          "UV signs are discarded and coordinates use their own shared map");
    check(result.indices.indices[face_channel] ==
                  std::vector<std::int32_t>{1, 1, 1, 0, 2, 2, 2, 0} &&
              result.face_data.size() == 2 && result.two_sided,
          "face data are appended instead of deduplicated and two-sided flags accumulate");
    const auto &f = result.face_data[0], &original = a.face_data[0];
    check(f.parameter_distance_range == original.parameter_distance_range &&
              f.parameter_range == original.parameter_range &&
              f.point_range == original.point_range && f.normal_range == original.normal_range &&
              f.source_index == original.source_index && f.face_indices == original.face_indices &&
              result.face_data[1].face_indices == std::array<std::int64_t, 3>{7, 8, 9},
          "all named face-data fields retain exact source values");
    check(result.edge_chains.size() == 2 &&
              result.edge_chains[1].point_indices == std::vector<std::int32_t>{2, 4, 3, 2} &&
              result.edge_chains[1].topology_ids == c.edge_chains[0].topology_ids &&
              result.edge_chains[1].topology_type == 14,
          "edge chains remap vertices but preserve topology identity and ordering");
    check(a.coordinates.normals[0][2] == 2 && a.indices.indices[point_channel][1] == -2 &&
              result.report["attribute_preparation_performed"] == false,
          "source is immutable and matched assembly does not claim attribute generation");
    {
        // Initial requested channels govern explicit pointers, not source flags.
        for (unsigned flags = 0; flags < 4; ++flags) {
            auto s = triangle();
            s.indices.active.fill(false);
            s.indices.indices[normal_channel].clear();
            s.indices.indices[parameter_channel].clear();
            NativeBuilderPolyfaceOptions options;
            options.normals_required = bool(flags & 1);
            options.parameters_required = bool(flags & 2);
            auto r = assemble_native_builder_polyfaces({s}, options, b);
            check(r.native_succeeded && r.complete && r.indices.active[normal_channel] &&
                      r.indices.active[parameter_channel] && r.indices.active[face_channel],
                  "count-matched pointer fallback reactivates unrequested attribute channels");
            check(r.report["sources"][0]["normal_index_source"] == "point_fallback" &&
                      r.report["sources"][0]["parameter_index_source"] == "point_fallback" &&
                      r.indices.indices[parameter_channel] == std::vector<std::int32_t>{1, 2, 3, 0},
                  "fallback uses signed point positions and independently positive attributes");
        }
        auto s = triangle();
        s.coordinates.normals.resize(1);
        s.coordinates.parameters.resize(2);
        NativeBuilderPolyfaceOptions options;
        options.normals_required = options.parameters_required = false;
        auto r = assemble_native_builder_polyfaces({s}, options, b);
        check(r.native_succeeded && !r.complete && r.indices.indices[normal_channel].empty() &&
                  !r.indices.active[normal_channel] && r.normal_pool_active,
              "disabled explicit normal pointers stay absent when point count differs");
        check(r.parameter_pool_active &&
                  r.indices.indices[parameter_channel] == std::vector<std::int32_t>{1, 1, 1, 0} &&
                  r.report["suppressed_attribute_indices"] == 8,
              "inserting UV pool activates global-first-UV fallback even without requested "
              "parameters");
    }
    {
        auto s = triangle();
        s.coordinates.parameters.clear();
        s.indices.indices[parameter_channel].clear();
        auto r = assemble_native_builder_polyfaces({s}, defaults, b);
        check(r.complete && r.coordinates.parameters == std::vector<Point2>{{0, 0}} &&
                  r.indices.indices[parameter_channel] == std::vector<std::int32_t>{1, 1, 1, 0} &&
                  r.report["sources"][0]["default_parameter_inserted"] == true,
              "requested empty UV pool receives native zero coordinate and first-slot indices");
        s.coordinates.parameter_scope = 100;
        r = assemble_native_builder_polyfaces({a, s}, defaults, b);
        check(r.coordinates.parameters == a.coordinates.parameters &&
                  r.indices.indices[parameter_channel] ==
                      std::vector<std::int32_t>{1, 2, 3, 0, 1, 1, 1, 0} &&
                  r.report["sources"][1]["default_parameter_inserted"] == false,
              "existing destination UV pool prevents default insertion despite changed scope");
        s = triangle();
        s.coordinates.parameters.clear();
        s.indices.indices[parameter_channel].clear();
        NativeBuilderPolyfaceOptions options;
        options.parameters_required = false;
        r = assemble_native_builder_polyfaces({s}, options, b);
        check(r.complete && !r.parameter_pool_active && r.coordinates.parameters.empty() &&
                  r.indices.indices[parameter_channel].empty(),
              "unrequested missing UV remains absent without a synthesized pool");
    }
    {
        auto s = triangle();
        s.coordinates.points.resize(1);
        s.coordinates.normals.resize(1);
        s.coordinates.parameters.resize(1);
        s.indices.indices[point_channel] = {0, 1, 0, 0};
        s.indices.indices[normal_channel] = {999, 1, 999, 999};
        s.indices.indices[parameter_channel] = {999, 1, 999, 999};
        s.indices.indices[face_channel] = {999, 1, 999, 999};
        s.edge_chains.clear();
        auto r = assemble_native_builder_polyfaces({s}, defaults, b);
        check(r.native_succeeded && !r.complete &&
                  r.report["missing_remap_keys"] == Json::array({0, 0, 0, 0, 0}),
              "terminators do not read parallel entries but can leave native channels unaligned");
        for (auto channel : {point_channel, normal_channel, parameter_channel})
            check(r.indices.indices[channel] == std::vector<std::int32_t>{1, 0, 0},
                  "leading zero skips empty channels but consecutive later zeros survive");
        check(r.indices.indices[face_channel] == std::vector<std::int32_t>{1, 0} &&
                  r.report["active_index_channels_aligned"] == false,
              "only native face-index termination suppresses repeated zero");
        s = triangle();
        s.indices.indices[normal_channel] = {1};
        rejects([&] { assemble_native_builder_polyfaces({s}, defaults, b); });
        s = triangle();
        s.indices.indices[parameter_channel] = {1};
        rejects([&] { assemble_native_builder_polyfaces({s}, defaults, b); });
    }
    {
        auto first = triangle();
        first.num_per_face = 3;
        for (auto &v : first.indices.indices)
            if (!v.empty())
                v.pop_back();
        auto bad = first;
        bad.num_per_face = 4;
        bad.coordinates.points[0][0] = std::numeric_limits<double>::quiet_NaN();
        auto last = first;
        last.two_sided = true;
        auto r = assemble_native_builder_polyfaces({first, bad, last}, defaults, b);
        check(
            !r.native_succeeded && !r.complete && r.num_per_face == 3 && r.two_sided &&
                r.coordinates.batches.size() == 2 && r.indices.indices[point_channel].size() == 6 &&
                r.report["sources"][1]["reason"] == "incompatible_num_per_face",
            "layout mismatch skips unchanged source before coordinates; later additions continue");
        NativeBuilderPolyface empty;
        empty.num_per_face = 17;
        empty.two_sided = true;
        r = assemble_native_builder_polyfaces({empty, a}, defaults, b);
        check(r.complete && r.num_per_face == 0 && !r.two_sided,
              "empty coordinate and point-index destination accepts new layout and replaces "
              "two-sided flag");
        empty.coordinates.points = {{1, 2, 3}};
        r = assemble_native_builder_polyfaces({empty, a}, defaults, b);
        check(!r.native_succeeded && r.num_per_face == 17 && r.face_data.empty(),
              "nonempty destination point pool prevents layout change even with empty indices");
        empty.coordinates.points.clear();
        empty.indices.indices[point_channel] = {1};
        r = assemble_native_builder_polyfaces({empty, a}, defaults, b);
        check(!r.native_succeeded && r.report["rejected_sources"] == 1,
              "nonempty destination point indices independently prevent layout change");
    }
    {
        auto s = triangle();
        s.indices.indices[face_channel] = {1};
        auto r = assemble_native_builder_polyfaces({s}, defaults, b);
        check(r.native_succeeded && !r.complete && r.face_data.size() == 1 &&
                  r.indices.indices[face_channel].empty() &&
                  r.report["discarded_face_indices"] == 1,
              "mismatched face-index count discards only links and retains face records");
        s = triangle();
        s.indices.indices[color_channel] = {1, 2, 3, 0};
        r = assemble_native_builder_polyfaces({s}, defaults, b);
        check(r.native_succeeded && !r.complete && r.indices.indices[color_channel].empty() &&
                  r.report["ignored_color_indices"] == 4,
              "native matched path does not transfer colors and reports incomplete preservation");
        s = triangle();
        s.indices.indices[point_channel][1] = std::numeric_limits<std::int32_t>::min();
        s.indices.indices[normal_channel][2] = 0;
        s.indices.indices[parameter_channel][2] = 99;
        s.indices.indices[face_channel][2] = -2;
        s.edge_chains[0].point_indices = {0, -2, 999, std::numeric_limits<std::int32_t>::min()};
        r = assemble_native_builder_polyfaces({a, s}, defaults, b);
        check(r.native_succeeded && !r.complete && r.indices.indices[point_channel][5] == -1 &&
                  r.indices.indices[normal_channel][6] == 1 &&
                  r.indices.indices[parameter_channel][6] == 1 &&
                  r.indices.indices[face_channel][6] == 1,
              "unknown remap keys choose global slot zero with explicit incompleteness");
        check(r.edge_chains.back().point_indices == std::vector<std::int32_t>{1, 1, 1, 1} &&
                  r.report["missing_edge_remap_keys"] == 4 &&
                  r.report["missing_remap_keys"] == Json::array({1, 1, 1, 0, 1}),
              "edge chains use signed positive-source keys and report every fallback");
        s = triangle();
        s.coordinates.normals.clear();
        s.indices.indices[normal_channel].clear();
        r = assemble_native_builder_polyfaces({s, a}, defaults, b);
        check(r.native_succeeded && !r.complete && r.indices.indices[normal_channel].size() == 4 &&
                  r.indices.indices[point_channel].size() == 8 &&
                  r.report["active_index_channels_aligned"] == false,
              "later channel activation never backfills earlier faces or hides missing normals");
    }
    {
        TubeBudget measured;
        assemble_native_builder_polyfaces({a, c}, defaults, measured);
        for (auto limit : {std::size_t(0), std::size_t(20), measured.work / 2, measured.work - 1}) {
            TubeBudget limited;
            limited.max_work = limit;
            rejects([&] { assemble_native_builder_polyfaces({a, c}, defaults, limited); });
        }
        TubeBudget limited;
        limited.max_control_points = 12;
        rejects([&] { assemble_native_builder_polyfaces({a, c}, defaults, limited); });
        auto s = triangle();
        s.coordinates.points[0][0] = std::numeric_limits<double>::infinity();
        rejects([&] { assemble_native_builder_polyfaces({a, s}, defaults, b); });
        check(a.face_data[0].source_index == 0xfedcba9876543210ULL &&
                  a.edge_chains[0].point_indices[1] == 2,
              "late failure preserves all source pools and associations");
        auto empty = assemble_native_builder_polyfaces({}, defaults, b);
        check(empty.native_succeeded && empty.complete && empty.coordinates.points.empty(),
              "empty prepared sequence completes assembly without fabricating geometry");
    }
    auto run = [&] {
        TubeBudget local;
        return assemble_native_builder_polyfaces({a, c}, defaults, local);
    };
    auto f1 = std::async(std::launch::async, run), f2 = std::async(std::launch::async, run);
    const auto r1 = f1.get(), r2 = f2.get();
    check(r1.report == result.report && r2.report == result.report &&
              r1.indices.indices == r2.indices.indices &&
              r1.coordinates.points == result.coordinates.points,
          "independent builder assemblies are deterministic and safe to run concurrently");
    return checks;
}
