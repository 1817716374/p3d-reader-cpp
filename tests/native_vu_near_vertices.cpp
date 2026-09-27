#include "native_vu_near_vertices.hpp"
#include "native_vu_cluster.hpp"
#include "native_vu_qsort.hpp"
#include <future>
#include <numeric>
#include <random>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_vu_near_vertices_tests() {
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
        check(caught, "native VU near-vertex validation/budget rejection");
    };
    auto snapshot = [](const NativeVuGraph &g) {
        Json result = Json::array();
        for (const auto &n : g.nodes)
            result.push_back(
                {n.all_next, n.face_next, n.vertex_next, n.mask, n.source_index, n.point});
        return Json{{"tail", g.tail}, {"nodes", result}};
    };
    auto sort = [](const std::vector<int> &keys) {
        std::vector<std::size_t> order(keys.size());
        std::iota(order.begin(), order.end(), 0);
        native_vu_qsort(order,
                        [&](auto a, auto b) { return (keys[a] > keys[b]) - (keys[a] < keys[b]); });
        return order;
    };
    check(sort({2, 1, 2, 1}) == std::vector<std::size_t>{1, 3, 2, 0},
          "small native selection sort preserves exact duplicate exchanges");
    check(sort(std::vector<int>(8)) == std::vector<std::size_t>{1, 2, 3, 4, 5, 6, 7, 0},
          "eight equal keys use selection exchanges");
    check(sort(std::vector<int>(9)) == std::vector<std::size_t>{0, 1, 2, 3, 4, 5, 6, 7, 8},
          "nine equal keys use partition branch instead of small sort");
    std::mt19937 random(737);
    for (unsigned count = 0; count < 320; ++count) {
        std::vector<int> keys(count);
        for (auto &k : keys)
            k = int(random() % 23);
        const auto order = sort(keys);
        auto ids = order;
        std::sort(ids.begin(), ids.end());
        check(std::is_sorted(order.begin(), order.end(),
                             [&](auto a, auto b) { return keys[a] < keys[b]; }),
              "lexical quicksort orders large duplicate-key arrays");
        bool permutation = true;
        for (std::size_t i = 0; i < ids.size(); ++i)
            permutation &= ids[i] == i;
        check(permutation, "lexical sorting preserves every source identity");
    }
    auto collapse = [](std::vector<double> values, double tol, double low = 0, double high = 1) {
        TubeBudget b;
        return collapse_native_vu_fractions(std::move(values), tol, low, high, b);
    };
    check(collapse({.25, .125, .25}, 0) == std::vector<double>{.125, .25},
          "zero tolerance removes equal fractions");
    check(collapse({.375, .125, .25}, .125) == std::vector<double>{.125, .25, .375},
          "transitive projection chain produces equal spacing");
    check(collapse({.125, .25}, .125) == std::vector<double>{.1875},
          "short cluster averages endpoints");
    check(collapse({.125, .1875, .25, .375}, .125) == std::vector<double>{.125, .25, .375},
          "long cluster uses span and truncated subdivision count, not all original members");
    check(collapse({.125, .25, .375}, .125, .125, .25) == std::vector<double>{.125, .25, .375},
          "native upper bound screens only cluster seed, not chained tail");
    check(collapse({.125, .25, .375}, 0, .25, .25) == std::vector<double>{.25},
          "inclusive seed interval");
    check(collapse({.5}, .1, .75, .25).empty(), "inverted seed interval produces no splits");
    rejects([&] { collapse({.25}, -1); });
    rejects([&] { collapse({std::numeric_limits<double>::infinity()}, 0); });
    const auto marker = Point3{std::numeric_limits<double>::max(), 0, 0};
    auto input = [&](std::vector<Point3> points) {
        std::vector<Point3> source{{0, 0, 0}, {10, 0, 20}, {10, 10, 40}};
        for (auto p : points) {
            source.push_back(marker);
            source.push_back(p);
        }
        TubeBudget b;
        return build_native_vu_input(source, 0, b);
    };
    auto run = [](NativeVuGraph &g, double perp = .25, double vertex = .25, double along = .25) {
        TubeBudget b;
        return split_native_vu_edges_near_vertices(g, perp, vertex, along, 0x40000000u, b);
    };
    auto g = input({{5, .125, 100}}).graph;
    const auto before = g;
    auto report = run(g);
    check(report.at("split_count") == 1 && report.at("added_nodes") == 2,
          "nearby off-edge isolated vertex splits just the qualifying triangle edge");
    for (std::size_t i = 0; i < before.nodes.size(); ++i)
        check(g.nodes[i].point == before.nodes[i].point &&
                  g.nodes[i].source_index == before.nodes[i].source_index,
              "original XYZ and source identities are not snapped during edge split");
    for (auto i = before.nodes.size(); i < g.nodes.size(); ++i)
        check(g.nodes[i].point == Point3{5, 0, 10} && g.nodes[i].source_index == 0 &&
                  g.nodes[i].mask == 1,
              "new vertex XYZ interpolates original edge; no candidate height/source/numbered mask "
              "copied");
    for (std::size_t i = 0; i < g.nodes.size(); ++i) {
        auto mate = g.nodes[g.nodes[i].face_next].vertex_next;
        check(g.nodes[g.nodes[mate].face_next].vertex_next == i,
              "split graph keeps edge pairing involution");
    }
    for (double y : {.25, std::nextafter(.25, 0.), std::nextafter(.25, 1.)}) {
        auto a = input({{5, y, 100}}).graph;
        check(run(a).at("split_count") == (y < .25 ? 1 : 0),
              "strict perpendicular distance threshold");
    }
    auto endpoint = input({{.25, 0, 100}}).graph;
    check(run(endpoint, .01, .25, .01).at("split_count") == 0,
          "endpoint distance equality is excluded");
    auto zero = input({{5, 0, 100}}).graph;
    check(run(zero, 0, 0, 0).at("split_count") == 0,
          "zero perpendicular tolerance rejects even exact projection");
    auto duplicate = input({{7.5, 0, 100}, {7.5, 0, 200}}).graph;
    check(run(duplicate, .25, 0, 0).at("split_count") == 1 &&
              duplicate.nodes.back().point == Point3{7.5, 0, 15},
          "equal projection fractions collapse with zero along tolerance; end-based interpolation "
          "retains Z");
    auto ignored = input({{5, .125, 100}}).graph;
    auto range_filtered = ignored;
    check(run(range_filtered, .5, .0625, .0625).at("split_count") == 0,
          "perpendicular tolerance does not expand native edge candidate range");
    ignored.nodes[6].mask |= 2;
    ignored.nodes[7].mask |= 2;
    check(run(ignored).at("split_count") == 0,
          "excluded vertex-loop mask suppresses projection source");
    auto masked = before;
    for (auto &n : masked.nodes)
        n.mask |= 0x40000000;
    check(run(masked).at("split_count") == 1,
          "reused scratch bits are cleared before vertex collection");
    auto height = input({{5, .125, 100}}).graph;
    for (unsigned i = 0; i < 6; ++i)
        height.nodes[i].point[2] = 1e101;
    check(run(height).at("split_count") == 0,
          "native huge-Z null range suppresses tolerance expansion");
    auto coarse = input({{5, .125, 100}}).graph;
    check(run(coarse, .25, .25, 2).at("split_count") == 0,
          "double-square-root coarse edge filter is retained");
    auto chain = input({{2.5, .125, 100}, {3.125, .125, 200}, {3.75, .125, 300}}).graph;
    const auto original_count = chain.nodes.size();
    check(run(chain, .25, .25, .625).at("split_count") == 3,
          "long projection chain creates ordered equally spaced edge splits");
    for (std::size_t i = 0; i < 3; ++i) {
        const auto node = original_count + 2 * i;
        const double x = 2.5 + .625 * double(i);
        check(chain.nodes[node].point == Point3{x, 0, 2 * x} &&
                  chain.nodes[node + 1].point == chain.nodes[node].point,
              "each split uses original edge coordinates and assigns both sides");
        if (i < 2)
            check(chain.nodes[node].face_next == node + 2,
                  "successive splits follow previous left node instead of original base");
    }
    auto working = input({{5, 1e-10, 100}});
    TubeBudget merge_budget;
    prepare_native_vu_merge(working, merge_budget);
    check(working.report.at("merge_preparation").at("near_vertex_splitting").at("split_count") ==
                  1 &&
              working.report.at("merge_preparation").contains("second_consolidation") &&
              working.report.at("merged") == true,
          "merge connects projection splitting and coordinate consolidation");
    auto original = before;
    const auto saved = snapshot(original);
    TubeBudget measured;
    auto measured_graph = original;
    split_native_vu_edges_near_vertices(measured_graph, .25, .25, .25, 0x40000000, measured);
    for (auto limit : {std::size_t(0), measured.work - 1}) {
        rejects([&] {
            TubeBudget b;
            b.max_work = limit;
            split_native_vu_edges_near_vertices(original, .25, .25, .25, 0x40000000, b);
        });
        check(snapshot(original) == saved,
              "early and late budget failures roll back masks and topology");
    }
    rejects([&] {
        TubeBudget b;
        b.max_control_points = original.nodes.size();
        split_native_vu_edges_near_vertices(original, .25, .25, .25, 0x40000000, b);
    });
    check(snapshot(original) == saved, "node limit failure does not publish partial split");
    rejects([&] {
        auto bad = original;
        bad.nodes[0].face_next = bad.nodes.size();
        run(bad);
    });
    rejects([&] {
        auto bad = original;
        bad.nodes[0].vertex_next = bad.nodes[1].vertex_next;
        run(bad);
    });
    rejects([&] {
        auto bad = original;
        bad.nodes[0].point[2] = std::numeric_limits<double>::quiet_NaN();
        run(bad);
    });
    rejects([&] { run(original, -1); });
    auto parallel = [&] {
        auto copy = before;
        auto r = run(copy);
        return std::make_pair(snapshot(copy), r);
    };
    auto task = std::async(std::launch::async, parallel);
    check(task.get() == parallel(), "projection splitting has no shared graph/sort counters");
    NativeVuGraph empty;
    check(run(empty).at("split_count") == 0, "empty input stays empty");
    return checks;
}
