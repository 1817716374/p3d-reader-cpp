#include "native_vu_exterior.hpp"
#include "native_vu_regularize.hpp"
#include "native_vu_cluster.hpp"
#include "native_vu_split_support.hpp"
#include "native_tube_mesh_index_rules.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_vu_exterior_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        ++checks;
        require(value, why);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "native exterior rejects invalid data or exhausted budgets");
    };
    const Point3 marker{std::numeric_limits<double>::max(), 0, 0};
    auto snapshot = [](const NativeVuGraph &g) {
        Json rows = Json::array();
        for (const auto &n : g.nodes)
            rows.push_back({n.all_next, n.face_next, n.vertex_next, n.mask, n.source_index, n.point,
                            n.active, n.internal_data});
        return Json{{"nodes", rows}, {"tail", g.tail}, {"free", g.free_head}};
    };
    auto raw = [](const std::vector<Point3> &p) {
        TubeBudget b;
        return build_native_vu_input(p, 0, b).graph;
    };
    auto prepare = [](const std::vector<Point3> &p) {
        TubeBudget b;
        auto input = build_native_vu_input(p, 0, b);
        prepare_native_vu_merge(input, b);
        auto reg = regularize_native_vu_graph(input.graph, b);
        require(reg.at("regularization_completed") == true, "test setup regularization");
        return std::make_pair(input.graph, reg.at("candidate_array_read_index").get<std::size_t>());
    };
    auto audit = [&](NativeVuGraph &g, std::size_t cursor, double expected_area) {
        const auto before = g;
        TubeBudget b;
        const auto report = mark_native_vu_exterior(g, cursor, b);
        check(report.at("exterior_classification_completed") == true &&
                  report.at("triangulated") == false,
              "classification completes without claiming triangle generation");
        check(g.tail == before.tail && g.free_head == before.free_head &&
                  g.nodes.size() == before.nodes.size(),
              "classification allocates no nodes and changes no allocation links");
        const auto all = validate_native_vu_split_graph(g, b);
        for (auto n : all) {
            const auto &a = g.nodes[n], &z = before.nodes[n];
            check(a.all_next == z.all_next && a.face_next == z.face_next &&
                      a.vertex_next == z.vertex_next && a.point == z.point &&
                      a.source_index == z.source_index && a.internal_data == z.internal_data,
                  "topology XYZ and both source labels stay unchanged");
            check((a.mask & ~0x70000002u) == (z.mask & ~0x70000002u),
                  "nonclassification mask bits survive");
            const auto mate = g.nodes[a.face_next].vertex_next;
            check(bool((a.mask ^ g.nodes[mate].mask) & 2) == bool(a.mask & 1),
                  "parity changes across original boundary, not across added bridges");
        }
        double interior_area = 0;
        std::vector<bool> seen(g.nodes.size());
        for (auto seed : all)
            if (!seen[seed]) {
                auto n = seed;
                double twice_area = 0;
                do {
                    check(!seen[n], "classified face closes on its seed");
                    seen[n] = true;
                    check((g.nodes[n].mask & 2) == (g.nodes[seed].mask & 2),
                          "whole face shares its exterior flag");
                    const auto &p = g.nodes[n].point, &q = g.nodes[g.nodes[n].face_next].point;
                    twice_area += p[0] * q[1] - p[1] * q[0];
                    n = g.nodes[n].face_next;
                } while (n != seed);
                if (!(g.nodes[seed].mask & 2))
                    interior_area += twice_area * .5;
            }
        check(std::abs(interior_area - expected_area) <=
                  std::max(1e-44, std::abs(expected_area) * 1e-12),
              "selected face area matches independent nested-loop parity area");
        return report;
    };
    const std::vector<Point3> square{{0, 0, 0}, {10, 0, 1}, {10, 10, 2}, {0, 10, 3}};
    for (int directions = 0; directions < 8; ++directions) {
        std::vector<Point3> points;
        std::vector<std::vector<Point3>> loops{square,
                                               {{2, 2, 4}, {8, 2, 5}, {8, 8, 6}, {2, 8, 7}},
                                               {{4, 4, 8}, {6, 4, 9}, {6, 6, 10}, {4, 6, 11}}};
        for (int k = 0; k < 3; ++k) {
            if (k)
                points.push_back(marker);
            if (directions & (1 << k))
                std::reverse(loops[k].begin(), loops[k].end());
            points.insert(points.end(), loops[k].begin(), loops[k].end());
        }
        auto state = prepare(points);
        const auto report = audit(state.first, state.second, 68);
        check(report.at("seed_count") == 1 && report.at("unreached_node_count") == 0,
              "outer hole and island classify from one connected exterior seed");
    }
    for (int copies : {2, 3, 12}) {
        std::vector<Point3> points;
        for (int i = 0; i < copies; ++i) {
            if (i)
                points.push_back(marker);
            points.insert(points.end(), square.begin(), square.end());
        }
        auto state = prepare(points);
        audit(state.first, state.second, copies % 2 ? 100 : 0);
    }
    {
        auto state = prepare({{0, 0, 0},
                              {6, 0, 0},
                              {6, 6, 0},
                              {0, 6, 0},
                              marker,
                              {3, 3, 0},
                              {9, 3, 0},
                              {9, 9, 0},
                              {3, 9, 0}});
        audit(state.first, state.second, 54); // Intersection is excluded by crossing parity.
        auto crossing = prepare({{0, 0, 0}, {4, 4, 0}, {0, 4, 0}, {4, 0, 0}});
        audit(crossing.first, crossing.second, 8);
    }
    {
        auto g = raw({{0, 0, 0}, {1e-16, 0, 0}, {0, 1e-16, 0}});
        const auto report = audit(g, 0, 5e-33);
        check(report.at("initial_candidate_count") == 1 && report.at("interior_node_count") == 3,
              "native zero threshold does not classify tiny positive area as exterior");
    }
    {
        auto g = raw({{0, 0, 0},
                      {3, 0, 0},
                      {3, 3, 0},
                      {0, 3, 0},
                      marker,
                      {10, 0, 0},
                      {14, 0, 0},
                      {14, 4, 0},
                      {10, 4, 0}});
        check(audit(g, 0, 25).at("seed_count") == 2,
              "disconnected components each receive an exterior seed");
    }
    for (const auto &p :
         std::vector<std::vector<Point3>>{{}, {{0, 0, 0}}, {{0, 0, 0}, {1, 0, 0}}}) {
        auto g = raw(p);
        for (auto &n : g.nodes)
            n.mask |= 0x70000002u;
        TubeBudget b;
        auto r = mark_native_vu_exterior(g, 0, b);
        check(r.at("exterior_node_count") == 0 && r.at("initial_candidate_count") == 0,
              "empty and one/two-node faces are suppressed as exterior seeds");
        check(r.at("unreached_node_count") == g.nodes.size(),
              "suppressed trivial components report no parity visit");
    }
    {
        auto g = raw({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}});
        TubeBudget b;
        auto r = mark_native_vu_exterior(g, 0, b);
        check(r.at("initial_candidate_count") == 2 && r.at("seed_count") == 1 &&
                  r.at("interior_node_count") == 3 && r.at("exterior_node_count") == 3,
              "zero-area three-node faces are candidates, unlike trivial loops");
    }
    {
        // Deliberately degenerate components expose native stale-cursor array
        // behavior independently of the normal regularized connected route.
        const auto original =
            raw({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, marker, {10, 0, 0}, {11, 0, 0}, {12, 0, 0}});
        for (const auto [cursor, final_cursor] : std::vector<std::pair<std::size_t, std::size_t>>{
                 {0, 0}, {1, 0}, {2, 0}, {3, 2}, {4, 4}, {9, 9}}) {
            auto g = original;
            TubeBudget b;
            auto r = mark_native_vu_exterior(g, cursor, b);
            check(r.at("initial_candidate_count") == 4,
                  "both zero-area sides enter the native candidate array");
            check(r.at("candidate_array_read_index") == final_cursor && r.at("seed_count") == 2 &&
                      r.at("unreached_node_count") == 0,
                  "indexed candidate lookup preserves native stale-cursor removals and seed order");
        }
    }
    {
        auto points = square;
        points.push_back(marker);
        points.push_back({20, 20, 99});
        auto g = raw(points);
        TubeBudget b;
        for (auto &n : g.nodes)
            if (n.source_index == 5)
                n.mask |= 0x10000000u;
        check(free_marked_native_vu_edges(g, 0x10000000u, b) == 2,
              "test setup frees isolated sling");
        const auto saved = snapshot(g);
        audit(g, 0, 100);
        const auto after = snapshot(g);
        for (std::size_t n = 0; n < g.nodes.size(); ++n)
            if (!g.nodes[n].active)
                check(saved.at("nodes").at(n) == after.at("nodes").at(n),
                      "inactive slots retain their payload and masks during classification");
    }
    {
        auto state = prepare(square);
        // Native temporary masks may differ initially; classification resets them.
        auto poisoned = state.first;
        for (auto &n : poisoned.nodes)
            n.mask ^= 0x70000002u;
        audit(state.first, state.second, 100);
        audit(poisoned, state.second, 100);
        check(snapshot(state.first) == snapshot(poisoned),
              "initial exterior and scratch bits do not leak into initialized result");
    }
    auto state = prepare({{0, 0, 0},
                          {10, 0, 0},
                          {10, 10, 0},
                          {0, 10, 0},
                          marker,
                          {3, 3, 0},
                          {7, 3, 0},
                          {7, 7, 0},
                          {3, 7, 0}});
    const auto original = state.first;
    const auto before = snapshot(original);
    TubeBudget done;
    auto reference = original;
    const auto expected = mark_native_vu_exterior(reference, state.second, done);
    for (auto limit : std::vector<std::uint64_t>{0, done.work / 2, done.work - 1}) {
        auto g = original;
        TubeBudget b;
        b.max_work = limit;
        rejects([&] { mark_native_vu_exterior(g, state.second, b); });
        check(snapshot(g) == before, "failed parity search publishes no partial masks");
    }
    {
        auto g = original;
        TubeBudget b;
        rejects([&] { mark_native_vu_exterior(g, b.max_control_points + 1, b); });
        check(snapshot(g) == before, "invalid pooled cursor leaves graph unchanged");
        g.nodes[0].face_next = native_vu_null;
        rejects([&] { mark_native_vu_exterior(g, 0, b); });
        auto huge = raw({{1e200, 1e200, 0}, {-1e200, 1e200, 0}, {0, -1e200, 0}});
        auto huge_before = snapshot(huge);
        rejects([&] { mark_native_vu_exterior(huge, 0, b); });
        check(snapshot(huge) == huge_before, "area overflow does not publish initial masks");
    }
    std::vector<std::future<Json>> jobs;
    for (int i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async, [&] {
            auto g = original;
            TubeBudget b;
            auto report = mark_native_vu_exterior(g, state.second, b);
            return Json{{"report", report}, {"graph", snapshot(g)}};
        }));
    for (auto &job : jobs)
        check(job.get() == Json{{"report", expected}, {"graph", snapshot(reference)}},
              "parity masks and depth-first stacks are local to concurrent calls");
    TubeBudget b;
    auto plan = native_facet_index_plan({{0, 0, 0}, {3, 0, 0}, {4, 2, 0}, {2, 4, 0}, {0, 2, 0}}, b);
    check(plan.completed && !plan.indices.empty() && plan.input_graph->report.at("exterior_classification")
                                          .at("exterior_classification_completed") == true,
          "large-facet route classifies exterior before native triangle source output");
    return checks;
}
