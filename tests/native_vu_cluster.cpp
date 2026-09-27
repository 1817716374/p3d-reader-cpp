#include "native_vu_cluster.hpp"
#include "native_tube_mesh_index_rules.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_vu_cluster_tests() {
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
        check(caught, "native VU consolidation validation/budget rejection");
    };
    auto snapshot = [&](const NativeVuGraph &g) {
        Json a = Json::array();
        for (const auto &n : g.nodes)
            a.push_back({n.all_next, n.face_next, n.vertex_next, n.mask, n.source_index, n.point});
        return a;
    };
    // Isolated original single-point loops keep distinct source identities,
    // allowing consolidation to be checked separately from input XY suppression.
    auto graph = [&](const std::vector<Point3> &positions) {
        std::vector<Point3> p;
        for (const auto &xyz : positions) {
            if (!p.empty())
                p.push_back({std::numeric_limits<double>::max(), 0, 0});
            p.push_back(xyz);
        }
        TubeBudget b;
        return build_native_vu_input(p, 0, b).graph;
    };
    auto consolidate = [&](NativeVuGraph &g, double tol) {
        TubeBudget b;
        return consolidate_native_vu_coordinates(g, tol, 0x40000000, b);
    };
    auto pair = graph({{10, 0, 1}, {10.5, 0, 99}});
    const auto pair_before = pair;
    auto report = consolidate(pair, 1);
    check(report.at("cluster_count") == 1 && report.at("absorbed_vertices") == 1 &&
              report.at("changed_node_coordinates") == 2,
          "two nearby XY vertices form one original cluster");
    for (std::size_t i = 0; i < pair.nodes.size(); ++i) {
        const auto &a = pair.nodes[i], &b = pair_before.nodes[i];
        check(a.point == Point3{10, 0, 1},
              "candidate XYZ including height adopts pivot coordinate");
        check(a.source_index == b.source_index && a.all_next == b.all_next &&
                  a.face_next == b.face_next && a.vertex_next == b.vertex_next &&
                  a.mask == (b.mask | 0x40000000u),
              "coordinate consolidation preserves topology/source labels and retains visited mask");
    }
    for (double x : {1., std::nextafter(1., 0.), std::nextafter(1., 2.)}) {
        auto g = graph({{0, 0, 1}, {x, 0, 2}});
        auto r = consolidate(g, 1);
        check(r.at("absorbed_vertices") == (x < 1 ? 1 : 0),
              "strict squared XY distance at tolerance boundary");
    }
    auto diagonal = graph({{0, 0, 1}, {.75, .75, 2}});
    check(consolidate(diagonal, 1).at("cluster_count") == 2,
          "clustering is Euclidean, unlike per-component input duplicate suppression");
    auto chain = graph({{0, 0, 1}, {.75, 0, 2}, {1.5, 0, 3}});
    auto cr = consolidate(chain, 1);
    check(cr.at("cluster_count") == 2 && chain.nodes[2].point == Point3{0, 0, 1} &&
              chain.nodes[4].point == Point3{1.5, 0, 3},
          "cluster membership compares fixed pivot and is not transitive union-find");
    auto equal = graph({{2, 3, 1}, {2, 3, 2}, {2, 3, 3}});
    auto er = consolidate(equal, 0);
    check(er.at("cluster_count") == 3 && er.at("changed_node_coordinates") == 0,
          "zero consolidation tolerance does not merge even equal XY positions");
    er = consolidate(equal, 1);
    check(er.at("cluster_count") == 1 && equal.nodes[0].point == Point3{2, 3, 3},
          "small equal-key group uses latest allocated seed, not first source index");
    // A stale visit bit must not cause vertices to be skipped next invocation.
    equal.nodes[0].point[2] = 100;
    equal.nodes[1].point[2] = 100;
    er = consolidate(equal, 1);
    check(er.at("vertex_count") == 3 && equal.nodes[0].point[2] == 3,
          "each operation clears logical visits before gathering vertex representatives");
    auto negative = graph({{-7, -2, 10}, {-7.5, -2, 20}});
    consolidate(negative, 1);
    check(negative.nodes[0].point == Point3{-7.5, -2, 20},
          "negative-coordinate sort chooses lowest projection pivot");
    std::vector<Point3> same;
    for (unsigned i = 0; i < 64; ++i)
        same.push_back({5, 7, double(i)});
    auto many = graph(same);
    auto mr = consolidate(many, 1);
    check(mr.at("sort_partitions").get<unsigned>() > 0 && mr.at("cluster_count") == 1,
          "large equal-key group uses native partition route");
    for (const auto &p : many.nodes)
        check(p.point == Point3{5, 7, 63},
              "native all-equal partition retains original newest seed");
    std::vector<Point3> descending;
    for (unsigned i = 0; i < 96; ++i)
        descending.push_back({double((i * 37) % 96) * 2, 0, double(i)});
    auto shuffled = graph(descending);
    auto before = snapshot(shuffled);
    auto sr = consolidate(shuffled, 1);
    check(sr.at("sort_partitions").get<unsigned>() > 0 && sr.at("cluster_count") == 96 &&
              sr.at("changed_node_coordinates") == 0,
          "ninther partitions preserve distinct vertices");
    for (std::size_t i = 0; i < shuffled.nodes.size(); ++i)
        check(shuffled.nodes[i].point == descending[i / 2],
              "sorting does not reorder original graph storage");
    TubeBudget b;
    auto scaled = graph({{-100, 20, 1e200}, {0, 50, -1e200}});
    check(native_vu_merge_tolerance(scaled, 0, 1e-9, b) == 100 * 1e-9,
          "relative tolerance uses max absolute XY, not Z or range span");
    check(native_vu_merge_tolerance(scaled, 2, 1e-9, b) == 2,
          "absolute merge tolerance can dominate");
    auto empty = graph({});
    check(native_vu_merge_tolerance(empty, 3, 1e-9, b) == 3 &&
              consolidate(empty, 0).at("cluster_count") == 0,
          "empty graph preserves absolute tolerance and empty clusters");
    // Actual projected large polygon: near-duplicate coordinates remain in the
    // projection pool while the initial merge operation changes only its graph.
    const std::vector<Point3> polygon{
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1 + 1e-10, 0}};
    TubeBudget pb;
    auto plan = native_facet_index_plan(polygon, pb);
    check(plan.input_graph && plan.projection &&
              plan.input_graph->report.contains("merge_preparation") &&
              plan.input_graph->report.at("merge_preparation")
                      .at("initial_consolidation")
                      .at("absorbed_vertices") == 1,
          "large facet route executes initial native coordinate consolidation");
    check(plan.projection->points[4][1] == 1 + 1e-10 &&
              plan.input_graph->graph.nodes[8].point[1] == 1 && plan.indices.empty() &&
              plan.input_graph->report.at("merged") == false,
          "source projected coordinates remain intact and unfinished merging stays explicit");
    // Shared vertex loops: consolidation must copy pivot XYZ to every candidate
    // vertex use, but must not normalize different coordinates within pivot itself.
    auto shared = graph({{0, 0, 1}, {.1, 0, 2}, {.2, 0, 3}});
    twist_native_vu_vertices(shared, 2, 4, b);
    auto sh = consolidate(shared, 1);
    check(sh.at("vertex_count") == 2 && sh.at("absorbed_vertices") == 1,
          "gather one representative per full vertex ring after original twist");
    for (const auto &node : shared.nodes)
        check(node.point == Point3{0, 0, 1}, "entire candidate vertex ring receives pivot XYZ");
    auto stable = graph({{0, 0, 1}, {.5, 0, 2}});
    const auto saved = snapshot(stable);
    rejects([&] { consolidate(stable, -1); });
    rejects([&] {
        TubeBudget z;
        consolidate_native_vu_coordinates(stable, 1, 0, z);
    });
    rejects([&] {
        TubeBudget z;
        consolidate_native_vu_coordinates(stable, 1, 0x80000000, z);
    });
    rejects([&] {
        TubeBudget z;
        z.max_control_points = 3;
        consolidate_native_vu_coordinates(stable, 1, 0x40000000, z);
    });
    auto measured = stable;
    TubeBudget used;
    consolidate_native_vu_coordinates(measured, 1, 0x40000000, used);
    rejects([&] {
        TubeBudget z;
        z.max_work = used.work - 1;
        consolidate_native_vu_coordinates(stable, 1, 0x40000000, z);
    });
    check(snapshot(stable) == saved,
          "late work-budget failure does not apply planned coordinates or mask bits");
    rejects([&] {
        auto corrupt = stable;
        corrupt.nodes[0].vertex_next = 999;
        consolidate(corrupt, 1);
    });
    rejects([&] {
        auto corrupt = stable;
        corrupt.nodes[0].all_next = 999;
        consolidate(corrupt, 1);
    });
    rejects([&] {
        auto corrupt = stable;
        corrupt.nodes[0].point[2] = std::numeric_limits<double>::infinity();
        consolidate(corrupt, 1);
    });
    rejects([&] {
        auto big = graph({{1e300, 0, 0}, {1e300, 0, 1}});
        consolidate(big, 1e300);
    });
    auto work = [&] {
        auto g = pair_before;
        auto result = consolidate(g, 1);
        return std::make_pair(snapshot(g), result);
    };
    auto task = std::async(std::launch::async, work);
    check(task.get() == work(),
          "parallel consolidation has no shared masks, sort cache or global counters");
    return checks;
}
