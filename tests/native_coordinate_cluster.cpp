#include "native_coordinate_cluster.hpp"
#include <future>
#include <limits>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_coordinate_cluster_tests() {
    unsigned checks = 0;
    auto check = [&](bool x, const char *why) {
        ++checks;
        require(x, why);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "coordinate combination rejects invalid data and resource exhaustion");
    };
    TubeBudget b;
    {
        const auto r =
            cluster_native_coordinates(std::vector<Point2>{{0, 0}, {.1, 2}, {.5, 1}}, 1.2, 0, b);
        check(r.source_to_packed == std::vector<std::size_t>{0, 1, 1},
              "later base reassigns an already grouped candidate without union of base groups");
        check(r.coordinates == std::vector<Point2>{{0, 0}, {.5, 1}} &&
                  r.report.at("reassigned_candidates") == 1,
              "packing takes last source member after native candidate reassignment");
        const auto chain = cluster_native_coordinates(
            std::vector<Point3>{{0, 0, 0}, {.75, 0, 0}, {1.5, 0, 0}}, 1, 0, b);
        check(chain.source_to_packed == std::vector<std::size_t>{0, 0, 1} &&
                  chain.coordinates == std::vector<Point3>{{.75, 0, 0}, {1.5, 0, 0}},
              "distance chains do not become a transitive component");
        const auto exact = cluster_native_coordinates(
            std::vector<Point2>{{0, 0}, {1, 0}, {std::nextafter(1., 2.), 0}}, 1, 0, b);
        check(exact.source_to_packed == std::vector<std::size_t>{0, 0, 1},
              "squared-distance tolerance is inclusive without additional epsilon");
        const auto diagonal = cluster_native_coordinates(
            std::vector<Point3>{{0, 0, 0}, {.8, .8, 0}, {0, 0, 2}}, 1, 0, b);
        check(diagonal.coordinates.size() == 3,
              "true multidimensional distance rejects close sort coordinates");
    }
    {
        const std::vector<Point3> points{{100, 200, 300}, {103, 196, 302}};
        const auto r = cluster_native_coordinates(points, .1, .25, b);
        check(r.report.at("relative_extent") == 4 && r.report.at("tolerance") == 1.1,
              "relative tolerance uses maximum coordinate difference from first source");
        auto shifted = points;
        for (auto &p : shifted)
            for (auto &x : p)
                x += 1000000;
        const auto t = cluster_native_coordinates(shifted, .1, .25, b);
        check(t.source_to_packed == r.source_to_packed && t.report.at("tolerance") == 1.1,
              "translation does not inflate native relative tolerance");
        const auto defaults = cluster_native_coordinates(points, -1, -7, b);
        check(defaults.report.at("tolerance") == 5e-8,
              "each negative tolerance argument selects the original 1e-8 default");
        const auto uv =
            cluster_native_coordinates(std::vector<Point2>{{100, 200}, {103, 196}}, -1, -1, b);
        check(uv.report.at("tolerance") == defaults.report.at("tolerance"),
              "2D pools use their own coordinate-relative range");
        const auto empty = cluster_native_coordinates(std::vector<Point3>{}, -1, -1, b);
        check(empty.coordinates.empty() && empty.source_to_packed.empty() &&
                  empty.report.at("tolerance") == 0,
              "empty native context leaves zero tolerance");
    }
    // Spans the original insertion/partition/ninther cutoffs. Distinct x keys
    // have an independently known sorted result regardless of sort implementation.
    for (std::size_t n : {1, 2, 32, 33, 40, 41, 42, 97, 257}) {
        std::vector<Point3> points;
        for (std::size_t i = 0; i < n; ++i)
            points.push_back({double(n - i), 0, 0});
        auto r = cluster_native_coordinates(points, 0, 0, b);
        check(r.coordinates.size() == n, "zero tolerance retains distinct coordinates");
        for (std::size_t i = 0; i < n; ++i)
            check(r.source_to_packed[i] == n - i - 1 &&
                      r.coordinates[i] == Point3{double(i + 1), 0, 0},
                  "native candidate sort preserves expected ordinal mapping across cutoffs");
        if (n > 32)
            check(r.report.at("sort_partitions").get<unsigned>() > 0,
                  "large candidate sets use original partition route");
    }
    {
        std::vector<Point2> p(97, Point2{1, 2});
        auto r = cluster_native_coordinates(p, 0, 0, b);
        check(r.coordinates == std::vector<Point2>{{1, 2}},
              "equal sort keys form one exact cluster");
        for (auto i : r.source_to_packed)
            check(i == 0, "all identical source indices map to one group");
    }
    NativePolyfaceCoordinateState source;
    source.points = {{10, 0, 0}, {0, 0, 0}, {10 + 1e-9, 0, 0}};
    source.normals = {{0, 0, 2}, {0, 0, 2 + 1e-9}, {0, 0, -1}};
    source.parameters = {{1, 1}, {0, 0}, {0, 0}};
    source.indices.indices[point_channel] = {1, -2, 3, 0};
    source.indices.indices[parameter_channel] = {2, -3, 1, 0};
    source.indices.indices[normal_channel] = {-3, 1, 2, 0};
    source.indices.indices[color_channel] = {93, -42, 0};
    source.indices.indices[face_channel] = {-63, 19, 0};
    for (unsigned flags = 0; flags < 16; ++flags) {
        auto s = source;
        s.parameter_pool_active = flags & 1;
        s.indices.active[parameter_channel] = flags & 2;
        s.normal_pool_active = flags & 4;
        s.indices.active[normal_channel] = flags & 8;
        s.indices.active[point_channel] = false;
        auto r = combine_native_polyface_coordinates(s, b);
        check(r.applied && r.point_map == std::vector<std::size_t>{1, 0, 1} &&
                  r.output.indices.indices[point_channel] == std::vector<std::int32_t>{2, -1, 2, 0},
              "native point combination does not require point-index active flag");
        check(r.output.points == std::vector<Point3>{source.points[1], source.points[2]},
              "packed point representative is last original member, not first");
        if ((flags & 3) == 3) {
            check(r.parameter_map == std::vector<std::size_t>{1, 0, 0} &&
                      r.output.indices.indices[parameter_channel] ==
                          std::vector<std::int32_t>{1, -1, 2, 0},
                  "UV mapping is independent and preserves negative index signs");
        } else
            check(r.parameter_map.empty() && r.output.parameters == source.parameters &&
                      r.output.indices.indices[parameter_channel] ==
                          source.indices.indices[parameter_channel],
                  "inactive UV pool or index channel retains original data");
        if ((flags & 12) == 12) {
            check(r.normal_map == std::vector<std::size_t>{1, 1, 0} &&
                      r.output.indices.indices[normal_channel] ==
                          std::vector<std::int32_t>{-1, 2, 2, 0} &&
                      r.output.normals == std::vector<Point3>{source.normals[2], source.normals[1]},
                  "normal vectors cluster independently without renormalization");
        } else
            check(r.normal_map.empty() && r.output.normals == source.normals,
                  "inactive normal data is retained");
        check(r.output.indices.indices[color_channel] == source.indices.indices[color_channel] &&
                  r.output.indices.indices[face_channel] == source.indices.indices[face_channel] &&
                  r.output.indices.active == s.indices.active,
              "combination does not remap color/face data or rewrite activity flags");
    }
    source.parameter_pool_active = source.normal_pool_active = true;
    source.indices.active[parameter_channel] = source.indices.active[normal_channel] = true;
    for (auto style : {0, 2, 3, 4, -1}) {
        auto s = source;
        s.mesh_style = style;
        auto r = combine_native_polyface_coordinates(s, b);
        check(!r.applied && r.output.points == s.points &&
                  r.output.indices.indices == s.indices.indices,
              "mesh styles other than variable signed loops bypass native combination");
    }
    auto no_points = source;
    no_points.points.clear();
    auto bypass = combine_native_polyface_coordinates(no_points, b);
    check(!bypass.applied && bypass.output.parameters == source.parameters,
          "empty point pool bypasses all other channels too");
    for (auto channel : {point_channel, parameter_channel, normal_channel}) {
        for (auto index : {INT32_MIN, INT32_MAX, -4}) {
            auto invalid = source;
            invalid.indices.indices[channel][0] = index;
            rejects([&] { combine_native_polyface_coordinates(invalid, b); });
            check(invalid.indices.indices[channel][0] == index,
                  "invalid remap never changes original buffers");
        }
    }
    check(cluster_native_coordinates(std::vector<Point3>{{0, 0, 0}, {1e308, 1e308, 1e308}}, 0, 0, b)
                  .coordinates.size() == 2,
          "sort-window rejection avoids unneeded overflowing distance squares");
    rejects([&] {
        cluster_native_coordinates(std::vector<Point3>{{-1e308, 0, 0}, {1e308, 0, 0}}, 0, 0, b);
    });
    rejects([&] {
        cluster_native_coordinates(
            std::vector<Point2>{{0, std::numeric_limits<double>::quiet_NaN()}}, 0, 0, b);
    });
    TubeBudget measured;
    combine_native_polyface_coordinates(source, measured);
    for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1}) {
        TubeBudget limited;
        limited.max_work = limit;
        rejects([&] { combine_native_polyface_coordinates(source, limited); });
    }
    {
        TubeBudget limited;
        limited.max_control_points = 3;
        rejects([&] { combine_native_polyface_coordinates(source, limited); });
    }
    auto task = [=] {
        TubeBudget local;
        auto r = combine_native_polyface_coordinates(source, local);
        return Json{{"points", r.output.points},
                    {"normals", r.output.normals},
                    {"uv", r.output.parameters},
                    {"indices", r.output.indices.indices},
                    {"report", r.report}};
    };
    auto reference = task();
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async, task));
    for (auto &j : jobs)
        check(j.get() == reference,
              "native coordinate combination is deterministic across threads");
    return checks;
}
