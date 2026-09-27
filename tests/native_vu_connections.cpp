#include "native_vu_connections.hpp"
#include "native_vu_cluster.hpp"
#include "native_vu_split_support.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_vu_connections_tests() {
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
        check(caught, "native VU connection invalid input/budget rejection");
    };
    const Point3 mark{std::numeric_limits<double>::max(), 0, 0};
    auto input = [](const std::vector<Point3> &p) {
        TubeBudget b;
        return build_native_vu_input(p, 0, b);
    };
    auto snapshot = [](const NativeVuGraph &g) {
        Json a = Json::array();
        for (auto &n : g.nodes)
            a.push_back({n.all_next, n.face_next, n.vertex_next, n.mask, n.source_index, n.point,
                         n.active, n.internal_data});
        return Json{{"nodes", a}, {"tail", g.tail}, {"free", g.free_head}};
    };
    auto order = [](const NativeVuGraph &g) {
        TubeBudget b;
        return native_vu_all_nodes(g, b);
    };
    auto topology = [&](const NativeVuGraph &g, std::size_t nv, std::size_t nf) {
        TubeBudget b;
        const auto all = validate_native_vu_split_graph(g, b);
        for (bool face : {false, true}) {
            std::vector<bool> seen(g.nodes.size());
            std::size_t cycles = 0;
            for (auto n : all)
                if (!seen[n]) {
                    ++cycles;
                    auto p = n;
                    do {
                        check(!seen[p], "graph loop closes at its own seed");
                        seen[p] = true;
                        p = face ? g.nodes[p].face_next : g.nodes[p].vertex_next;
                    } while (p != n);
                }
            check(cycles == (face ? nf : nv),
                  "native connected graph has expected face/vertex topology");
        }
    };
    auto run = [](NativeVuGraph &g, int type = 1, double tol = 1e-8) {
        TubeBudget b;
        return connect_native_vu_vertices(g, tol, type, 0x40000000, b);
    };
    const std::vector<Point3> triangle{{0, 0, 0}, {4, 0, 1}, {0, 4, 2}};
    auto single = input(triangle).graph;
    auto single_before = single;
    check(run(single).at("vertex_connection_completed") == true,
          "single polygon vertex connection completes");
    topology(single, 3, 2);
    for (std::size_t i = 0; i < single.nodes.size(); ++i)
        check(single.nodes[i].point == single_before.nodes[i].point &&
                  single.nodes[i].source_index == single_before.nodes[i].source_index,
              "ordinary connection retains native payloads and source labels");
    for (unsigned copies : {2u, 3u, 12u})
        for (int type : {0, 1, 2002, 7}) {
            std::vector<Point3> points;
            for (unsigned i = 0; i < copies; ++i) {
                if (i)
                    points.push_back(mark);
                points.insert(points.end(), triangle.begin(), triangle.end());
            }
            auto g = input(points).graph;
            auto r = run(g, type);
            const auto remaining = type == 0 ? (copies % 2) * 6 : type == 2002 ? 6 : copies * 6;
            check(order(g).size() == remaining && g.nodes.size() == copies * 6,
                  "duplicate edge mode controls active nodes without compacting storage");
            if (remaining)
                topology(g, 3, remaining / 2 - 1);
            else
                topology(g, 0, 0);
            if (type == 1 || type == 7)
                check(r.at("null_face_bundles") == 3 * (copies - 1),
                      "preserve mode retains original duplicate edges as null-face bundles");
            TubeBudget b;
            auto saved = snapshot(g);
            check(free_marked_native_vu_edges(g, 0, b) == 0 && snapshot(g) == saved,
                  "zero deletion mask leaves active and free order unchanged");
        }
    auto bow = input({{0, 0, 0}, {4, 4, 8}, {0, 4, 100}, {4, 0, 108}});
    TubeBudget mb;
    prepare_native_vu_merge(bow, mb);
    check(bow.report.at("merged") == true && bow.report.at("triangulated") == false,
          "merge completion remains distinct from final triangulation");
    topology(bow.graph, 5, 3);
    std::size_t degree4 = 0;
    const auto active = order(bow.graph);
    for (auto n : active) {
        std::size_t degree = 0;
        auto p = n;
        do {
            ++degree;
            p = bow.graph.nodes[p].vertex_next;
        } while (p != n);
        if (degree == 4) {
            ++degree4;
            auto q = bow.graph.nodes[n].vertex_next;
            check(bow.graph.nodes[n].point == bow.graph.nodes[q].point,
                  "crossing vertex uses consolidated full XYZ");
        }
    }
    check(degree4 == 4, "two crossing edges become one degree-four vertex");
    // Each isolated point starts as a sling; removing just one marked side must
    // remove its mate too and retain the native survivor/free traversal order.
    auto slings = input({{1, 0, 0}, mark, {2, 0, 0}, mark, {3, 0, 0}}).graph;
    const auto old_order = order(slings);
    slings.nodes[2].mask |= 0x10000;
    TubeBudget db;
    check(free_marked_native_vu_edges(slings, 0x10000, db) == 2,
          "one marked side removes both edge uses");
    std::vector<std::size_t> survivor;
    for (auto n : old_order)
        if (n != 2 && n != 3)
            survivor.push_back(n);
    check(order(slings) == survivor && slings.free_head == 2 && slings.nodes[2].all_next == 3,
          "survivors retain traversal; free slots reverse removed traversal");
    check(!slings.nodes[2].active && !slings.nodes[3].active,
          "deleted slots stay addressable but inactive");
    auto recycled = split_native_vu_edge(slings, native_vu_null, db);
    check(recycled == std::pair<std::size_t, std::size_t>{2, 3} && slings.nodes.size() == 6,
          "native allocation reuses free head slots without growing or deduplicating geometry");
    for (auto n : {2u, 3u})
        check(slings.nodes[n].point == Point3{} && slings.nodes[n].source_index == 0 &&
                  slings.nodes[n].internal_data == 0 && slings.nodes[n].mask == 1 &&
                  slings.nodes[n].active,
              "recycled payload and internal cluster identity reset to native defaults");
    auto allgone = input({{5, 7, 9}}).graph;
    check(run(allgone).at("sling_nodes_removed") == 2 && order(allgone).empty() &&
              allgone.nodes.size() == 2,
          "same-cluster sling removal retains free storage but no active topology");
    // Reuse can succeed at the high-water storage budget, with no growth.
    TubeBudget rb;
    rb.max_control_points = 2;
    auto reused = split_native_vu_edge(allgone, native_vu_null, rb);
    check(reused.first == 0 && reused.second == 1 && allgone.tail == 0,
          "empty active graph restarts using native free order");
    topology(allgone, 1, 2);
    // Deleting a non-sling edge must detach both endpoints while preserving the
    // remaining connected path; repeated deletion does not corrupt free slots.
    auto path = single_before;
    path.nodes[0].mask |= 0x10000;
    TubeBudget pb;
    check(free_marked_native_vu_edges(path, 0x10000, pb) == 2, "ordinary marked edge is removed");
    topology(path, 3, 1);
    auto saved_path = snapshot(path);
    check(free_marked_native_vu_edges(path, 0x10000, pb) == 0 && snapshot(path) == saved_path,
          "deleted masks are not revisited as active edges");
    auto deleting = single_before;
    deleting.nodes[0].mask |= 0x10000;
    const auto deletion_before = snapshot(deleting);
    auto deletion_measure = deleting;
    TubeBudget deletion_budget;
    free_marked_native_vu_edges(deletion_measure, 0x10000, deletion_budget);
    rejects([&] {
        TubeBudget b;
        b.max_work = deletion_budget.work - 1;
        free_marked_native_vu_edges(deleting, 0x10000, b);
    });
    check(snapshot(deleting) == deletion_before,
          "late deletion validation failure restores both list heads and topology");
    auto reused_path = path;
    const auto slot_count = reused_path.nodes.size();
    auto active_path = order(reused_path);
    auto reused_pair = split_native_vu_edge(reused_path, active_path.front(), pb);
    check(reused_path.nodes.size() == slot_count && reused_path.nodes[reused_pair.first].active &&
              reused_path.nodes[reused_pair.second].active,
          "splitting a surviving edge uses freed slots");
    topology(reused_path, 4, 1);
    std::vector<std::size_t> clusters;
    auto cg = input({{0, 0, 0}, mark, {.1, 0, 1}, mark, {2, 0, 2}}).graph;
    TubeBudget cb;
    consolidate_native_vu_coordinates(cg, .5, 0x40000000, cb, &clusters);
    check(clusters == std::vector<std::size_t>{1, 3, native_vu_null, 5, native_vu_null},
          "optional cluster output retains pivot/candidate order and null separators");
    auto original = single_before;
    const auto saved = snapshot(original);
    TubeBudget measure;
    auto measured = original;
    connect_native_vu_vertices(measured, 1e-8, 1, 0x40000000, measure);
    for (auto limit : {std::size_t(0), measure.work - 1}) {
        rejects([&] {
            TubeBudget b;
            b.max_work = limit;
            connect_native_vu_vertices(original, 1e-8, 1, 0x40000000, b);
        });
        check(snapshot(original) == saved,
              "connection failure rolls back topology, cluster ids, masks and free chain");
    }
    auto damaged = single_before;
    damaged.nodes[0].active = false;
    rejects([&] { run(damaged); });
    damaged = single_before;
    damaged.free_head = 0;
    rejects([&] { run(damaged); });
    auto parallel = [&] {
        auto g = single_before;
        auto r = run(g);
        return std::make_pair(snapshot(g), r);
    };
    auto task = std::async(std::launch::async, parallel);
    check(task.get() == parallel(),
          "vertex connection and free-list management have no shared mutable state");
    return checks;
}
