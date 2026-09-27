#include "native_vu_graph.hpp"
#include "native_tube_mesh_index_rules.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_vu_graph_tests() {
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
        check(caught, "native VU invalid input or exhausted budget rejects");
    };
    auto build = [&](const std::vector<Point3> &p, double tol = 0) {
        TubeBudget b;
        return build_native_vu_input(p, tol, b);
    };
    auto snapshot = [&](const NativeVuGraph &g) {
        Json a = Json::array();
        for (const auto &n : g.nodes)
            a.push_back({n.all_next, n.face_next, n.vertex_next, n.mask, n.source_index, n.point});
        return Json{{"tail", g.tail}, {"nodes", a}};
    };
    auto cycles = [&](const NativeVuGraph &g, unsigned kind) {
        std::vector<bool> seen(g.nodes.size());
        std::size_t total = 0;
        for (std::size_t i = 0; i < g.nodes.size(); ++i) {
            if (seen[i])
                continue;
            ++total;
            auto p = i;
            do {
                check(p < g.nodes.size() && !seen[p], "VU successors form disjoint closed cycles");
                seen[p] = true;
                const auto &node = g.nodes[p];
                p = kind == 0 ? node.all_next : kind == 1 ? node.face_next : node.vertex_next;
            } while (p != i);
        }
        return total;
    };
    auto topology = [&](const NativeVuGraph &g, std::size_t faces, std::size_t vertices) {
        check(cycles(g, 0) == (g.nodes.empty() ? 0u : 1u),
              "native all-node list is a single cycle");
        check(cycles(g, 1) == faces && cycles(g, 2) == vertices,
              "native face and vertex incidences");
        for (std::size_t i = 0; i < g.nodes.size(); ++i) {
            const auto mate = g.nodes[g.nodes[i].face_next].vertex_next;
            const auto mate2 = g.nodes[g.nodes[mate].face_next].vertex_next;
            check(mate != i && mate2 == i, "edge-mate operation is a fixed-point-free involution");
        }
    };
    const double marker = std::numeric_limits<double>::max();
    const std::vector<Point3> rectangle{{0, 0, 0}, {2, 0, 0}, {2, 3, 0}, {0, 3, 0}, {0, 0, 0}};
    const auto r = build(rectangle);
    topology(r.graph, 2, 4);
    check(r.graph.nodes.size() == 8 && r.first_loop == 6 && r.graph.tail == 0 &&
              r.loops.size() == 1 && r.loops[0].trimmed_end == 4,
          "native allocation, returned last interior node, and closure suppression");
    std::vector<std::int32_t> forward, backward;
    for (auto p = r.first_loop;;) {
        forward.push_back(r.graph.nodes[p].source_index);
        p = r.graph.nodes[p].face_next;
        if (p == r.first_loop)
            break;
    }
    const auto outside = r.graph.nodes[r.first_loop].vertex_next;
    for (auto p = outside;;) {
        backward.push_back(r.graph.nodes[p].source_index);
        p = r.graph.nodes[p].face_next;
        if (p == outside)
            break;
    }
    check(forward == std::vector<std::int32_t>{3, 0, 1, 2} &&
              backward == std::vector<std::int32_t>{3, 2, 1, 0},
          "opposite faces retain original forward and reverse source index cycles");
    auto next = r.graph.nodes[r.graph.tail].all_next;
    for (std::size_t i = 8; i > 0; --i) {
        check(next == i - 1, "original all-node traversal visits newest through oldest allocation");
        next = r.graph.nodes[next].all_next;
    }
    for (const auto &node : r.graph.nodes)
        check(node.mask == 0x80000001u && node.point == rectangle[std::size_t(node.source_index)],
              "both sides have original numbered/boundary bits and full XYZ");
    // Tolerance compares coordinate components, not Euclidean distance, and
    // compares against the last inserted point rather than last input point.
    const std::vector<Point3> duplicates{{0, 0, 1}, {.75, .75, 2}, {1.5, 0, 3}, {2.25, .75, 4},
                                         {3, 2, 5}, {0, 2, 6},     {0, 0, 50},  {0, 0, 60}};
    const auto d = build(duplicates, 1);
    topology(d.graph, 2, 4);
    std::vector<std::int32_t> labels;
    for (std::size_t i = 0; i < d.graph.nodes.size(); i += 2)
        labels.push_back(d.graph.nodes[i].source_index);
    check(labels == std::vector<std::int32_t>{0, 2, 4, 5} &&
              d.report.at("trailing_xy_duplicates") == 2 &&
              d.report.at("adjacent_xy_duplicates") == 2,
          "native XY suppression keeps original indices and ignores Z");
    check(d.graph.nodes[0].point[2] == 1 && duplicates.size() == 8 && duplicates.back()[2] == 60,
          "suppressed points remain in source pool and do not overwrite graph height");
    auto xy_z = build({{0, 0, 1}, {0, 0, 9}, {2, 0, 2}, {2, 2, 3}, {0, 2, 4}, {0, 0, 99}});
    check(xy_z.graph.nodes.size() == 8 && xy_z.graph.nodes[2].source_index == 2 &&
              xy_z.graph.nodes[0].point[2] == 1,
          "zero tolerance still suppresses exact XY duplicates with different heights");
    auto exact = build({{0, 0, 0}, {1, 0, 1}, {1, 1, 2}, {2, 0, 3}}, 1);
    auto beyond = build({{0, 0, 0}, {std::nextafter(1., 2.), 0, 1}, {2, 0, 3}}, 1);
    check(exact.graph.nodes.size() == 4 && beyond.graph.nodes.size() == 4 &&
              exact.graph.nodes[2].source_index == 3 && beyond.graph.nodes[2].source_index == 1,
          "per-component equality is suppressed and one ULP above tolerance is retained");
    for (unsigned axis = 0; axis < 3; ++axis) {
        auto p = rectangle;
        Point3 sep{6, 7, 8};
        sep[axis] = marker;
        p.push_back(sep);
        p.push_back(sep);
        p.insert(p.end(), rectangle.begin(), rectangle.end());
        p.push_back(sep);
        const auto multi = build(p);
        topology(multi.graph, 4, 8);
        check(multi.loops.size() == 2 && multi.first_loop == 6 && multi.loops[1].begin == 7 &&
                  multi.graph.nodes[8].source_index == 7 &&
                  multi.report.at("disconnect_count") == 3,
              "multiple and consecutive separators preserve original whole-array labels");
        check(multi.graph.nodes[0].point == multi.graph.nodes[8].point &&
                  multi.graph.nodes[0].vertex_next != multi.graph.nodes[8].vertex_next,
              "equal source loops remain independent until an original graph operation connects "
              "them");
    }
    auto empty = build({});
    topology(empty.graph, 0, 0);
    check(empty.first_loop == native_vu_null && empty.loops.empty(),
          "empty indexed-loop input stays empty");
    for (const auto &p :
         {std::vector<Point3>{{4, 5, 6}}, std::vector<Point3>{{4, 5, 6}, {4, 5, 9}, {4, 5, 12}}}) {
        auto one = build(p);
        topology(one.graph, 2, 1);
        check(one.graph.nodes.size() == 2 && one.loops[0].inserted_vertices == 1,
              "single unique XY point creates original sling rather than vanishing");
    }
    auto two = build({{0, 0, 0}, {2, 0, 0}, {0, 0, 0}});
    topology(two.graph, 2, 2);
    auto crossed = build({{0, 0, 0}, {2, 2, 0}, {0, 2, 0}, {2, 0, 0}});
    topology(crossed.graph, 2, 4);
    check(crossed.report.at("merged") == false && crossed.report.at("triangulated") == false,
          "self crossing input construction does not claim solved intersections");
    // Split on the opposite face as well, with native copy mask and no vertex
    // label/coordinate inheritance. Then set the new position as a caller does.
    auto split = r.graph;
    TubeBudget budget;
    split.nodes[0].mask = 0xffffffffu;
    const auto old_mate = split.nodes[split.nodes[0].face_next].vertex_next;
    split.nodes[old_mate].mask = 0x80000000u;
    auto inserted = split_native_vu_edge(split, 0, budget);
    topology(split, 2, 5);
    check(split.nodes[inserted.first].mask == 0x54efu && split.nodes[inserted.second].mask == 0 &&
              split.nodes[inserted.first].point == Point3{} &&
              split.nodes[inserted.first].source_index == 0,
          "edge split inherits only original allowed mask bits, no source labels or coordinates");
    auto inserted2 = split_native_vu_edge(split, inserted.second, budget);
    topology(split, 2, 6);
    check(inserted2.first == 10 && inserted2.second == 11,
          "opposite face split appends stable node identities");
    auto multi_points = rectangle;
    multi_points.push_back({marker, 0, 0});
    multi_points.insert(multi_points.end(), rectangle.begin(), rectangle.end());
    auto joined = build(multi_points).graph;
    const auto original = snapshot(joined);
    twist_native_vu_vertices(joined, 0, 8, budget);
    topology(joined, 3, 7);
    check(joined.nodes.size() == 16 && joined.nodes[0].source_index == 0 &&
              joined.nodes[8].source_index == 6,
          "vertex twist changes topology without relabeling source corners");
    twist_native_vu_vertices(joined, 0, 8, budget);
    check(snapshot(joined) == original,
          "repeating original vertex twist restores all links and payloads");
    twist_native_vu_vertices(joined, 0, 0, budget);
    check(snapshot(joined) == original, "same-node twist is an exact no-op");
    // Many splits force vector reallocation. Node indices and face rings must
    // survive growth; no memory address is used as an internal graph handle.
    std::vector<Point3> many;
    for (unsigned i = 0; i < 256; ++i)
        many.push_back({double(i), double(i % 3), double(i % 5)});
    auto grown = build(many);
    topology(grown.graph, 2, 256);
    check(grown.graph.nodes.size() == 512 && grown.graph.nodes[510].source_index == 255,
          "stable topology survives repeated storage growth");
    auto p = rectangle;
    p.insert(p.begin() + 1, p.front());
    TubeBudget plan_budget;
    auto plan = native_facet_index_plan(p, plan_budget);
    check(plan.input_graph && plan.input_graph->graph.nodes.size() == 8 && plan.indices.empty() &&
              plan.input_graph->report.at("triangulated") == false,
          "actual large-facet dispatch connects projection to indexed graph preparation");
    auto failed = native_facet_index_plan(std::vector<Point3>(5, Point3{1, 2, 3}), plan_budget);
    check(failed.projection && !failed.projection->frame_succeeded && !failed.input_graph,
          "failed coordinate frame never publishes an input graph");
    rejects([&] { build({{marker, 0, 0}, {1, 2, 3}}); });
    rejects([&] { build(rectangle, -1); });
    rejects([&] { build(rectangle, std::numeric_limits<double>::infinity()); });
    rejects([&] {
        auto p = rectangle;
        p[0][0] = std::numeric_limits<double>::quiet_NaN();
        build(p);
    });
    rejects([&] {
        TubeBudget b;
        b.max_control_points = 7;
        build_native_vu_input(rectangle, 0, b);
    });
    rejects([&] {
        TubeBudget b;
        b.max_work = 5;
        build_native_vu_input(rectangle, 0, b);
    });
    const auto save = snapshot(joined);
    rejects([&] {
        TubeBudget b;
        b.max_work = 23;
        split_native_vu_edge(joined, 0, b);
    });
    rejects([&] {
        TubeBudget b;
        b.max_control_points = joined.nodes.size() + 1;
        split_native_vu_edge(joined, 0, b);
    });
    rejects([&] {
        TubeBudget b;
        split_native_vu_edge(joined, joined.nodes.size(), b);
    });
    rejects([&] {
        TubeBudget b;
        b.max_work = 11;
        twist_native_vu_vertices(joined, 0, 8, b);
    });
    rejects([&] {
        TubeBudget b;
        twist_native_vu_vertices(joined, 0, joined.nodes.size(), b);
    });
    check(snapshot(joined) == save,
          "primitive validation and budget failures leave graph unchanged");
    auto work = [&] { return build(duplicates, 1); };
    auto task = std::async(std::launch::async, work);
    auto main = work();
    auto parallel = task.get();
    check(snapshot(main.graph) == snapshot(parallel.graph) && main.report == parallel.report,
          "independent graph builds share no allocator or mutable native counters");
    return checks;
}
