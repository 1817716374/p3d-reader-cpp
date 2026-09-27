#include "native_vu_flip.hpp"
#include "native_vu_triangulate.hpp"
#include "native_vu_cluster.hpp"
#include "native_vu_regularize.hpp"
#include "native_vu_exterior.hpp"
#include "native_vu_split_support.hpp"
#include "native_tube_mesh_index_rules.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_vu_flip_tests() {
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
        check(caught, "flip rejects malformed inputs or exhausted budgets");
    };
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
    const std::vector<Point3> quad{{0, 0, 11}, {4, 0, 22}, {4, 1, 33}, {0, 3, 44}};
    auto bad_diagonal = [&](std::vector<Point3> p) {
        TubeBudget b;
        auto g = raw(p);
        mark_native_vu_exterior(g, 0, b);
        join_native_vu_sectors(g, 2, 6, b);
        return g;
    };
    auto invariant = [&](const NativeVuGraph &before, const NativeVuGraph &g) {
        check(g.nodes.size() == before.nodes.size() && g.tail == before.tail &&
                  g.free_head == before.free_head,
              "flip allocates no nodes or changes allocation identities");
        TubeBudget b;
        validate_native_vu_split_graph(g, b);
        for (std::size_t i = 0; i < g.nodes.size(); ++i) {
            const auto &a = before.nodes[i], &z = g.nodes[i];
            check(a.active == z.active && a.all_next == z.all_next &&
                      a.source_index == z.source_index && a.internal_data == z.internal_data,
                  "flip retains source labels and all-node order");
            check((a.mask & ~0x20000000u) == (z.mask & ~0x20000000u),
                  "flip changes only candidate mask");
            if (a.mask & 0xcef)
                check(a.point == z.point, "fixed-edge endpoint coordinates remain unchanged");
        }
    };
    auto area = [&](const NativeVuGraph &g) {
        TubeBudget b;
        auto all = native_vu_all_nodes(g, b);
        std::vector<bool> seen(g.nodes.size());
        double sum = 0;
        for (auto seed : all) {
            if (seen[seed])
                continue;
            auto n = seed;
            std::size_t count = 0;
            double twice = 0;
            do {
                seen[n] = true;
                ++count;
                auto &p = g.nodes[n].point, &q = g.nodes[g.nodes[n].face_next].point;
                twice += p[0] * q[1] - p[1] * q[0];
                n = g.nodes[n].face_next;
            } while (n != seed);
            if (!(g.nodes[seed].mask & 2)) {
                check(count <= 3, "flip preserves triangular and short interior faces");
                sum += twice * .5;
            }
        }
        return sum;
    };
    {
        auto g = bad_diagonal(quad);
        const auto before = g;
        TubeBudget b;
        check(native_vu_quadratic_flip_test(g, 8, b) && native_vu_quadratic_flip_test(g, 9, b),
              "both orientations of inferior diagonal require flip");
        const auto r = flip_native_vu_triangles(g, b);
        check(r.at("initial_candidate_count") == 1 && r.at("tested_edges") == 1 &&
                  r.at("flipped_edges") == 1 && r.at("candidate_insertions") == 1 &&
                  r.at("native_iterations") == 2 && r.at("edge_adjustment_completed") == true &&
                  r.at("final_indices_ready") == false,
              "one eligible diagonal flips once, preserving native final empty pop attempt");
        check(g.nodes[8].point == quad[0] && g.nodes[9].point == quad[2],
              "flip copies opposite full XYZ corners");
        check(!native_vu_quadratic_flip_test(g, 8, b),
              "better diagonal cannot immediately flip back");
        invariant(before, g);
        check(area(g) == 8, "flip preserves independent quadrilateral area");
        const auto stable = snapshot(g);
        const auto again = flip_native_vu_triangles(g, b);
        check(again.at("flipped_edges") == 0 && snapshot(g) == stable,
              "converged pass preserves complete graph");
    }
    {
        auto g = bad_diagonal(quad);
        g.nodes[8].source_index = 87;
        g.nodes[9].source_index = -64;
        g.nodes[8].internal_data = 17;
        g.nodes[9].internal_data = -19;
        const auto before = g;
        TubeBudget b;
        flip_native_vu_triangles(g, b);
        invariant(before, g);
        check(g.nodes[8].source_index == 87 && g.nodes[9].source_index == -64,
              "fresh graph flags disable both conditional vertex label copies");
    }
    for (std::uint32_t bit : {1u, 2u, 4u, 8u, 0x20u, 0x40u, 0x80u, 0x400u, 0x800u})
        for (std::size_t side : {8u, 9u}) {
            auto g = bad_diagonal(quad);
            g.nodes[side].mask |= bit;
            // The pass clears a scratch bit left by the preceding classifier.
            for (auto &n : g.nodes)
                n.mask &= ~0x20000000u;
            const auto before = snapshot(g);
            TubeBudget b;
            const auto r = flip_native_vu_triangles(g, b);
            check(r.at("initial_candidate_count") == 0 && r.at("flipped_edges") == 0 &&
                      snapshot(g) == before,
                  "either side's original fixed-edge mask excludes the edge");
        }
    for (std::uint32_t bit :
         {0x10u, 0x100u, 0x200u, 0x10000000u, 0x20000000u, 0x40000000u, 0x80000000u}) {
        auto g = bad_diagonal(quad);
        g.nodes[8].mask |= bit;
        TubeBudget b;
        const auto r = flip_native_vu_triangles(g, b);
        check(r.at("flipped_edges") == 1,
              "nonfixed bits and stale candidate masks do not block flip");
    }
    {
        auto square = bad_diagonal({{0, 0, 1}, {1, 0, 2}, {1, 1, 3}, {0, 1, 4}});
        TubeBudget b;
        const auto r = flip_native_vu_triangles(square, b);
        check(r.at("tested_edges") == 1 && r.at("flipped_edges") == 0,
              "equal-quality square preserves chosen diagonal");
        auto nontri = raw(quad);
        check(!native_vu_quadratic_flip_test(nontri, 0, b), "predicate rejects nontriangle pair");
        auto flat = bad_diagonal({{0, 0, 1}, {1, 0, 2}, {2, 0, 3}, {3, 0, 4}});
        check(!native_vu_quadratic_flip_test(flat, 8, b),
              "collinear triangle pair has no improvement");
        auto zero = bad_diagonal(quad);
        for (auto &n : zero.nodes)
            n.point = {0, 0, 0};
        check(!native_vu_quadratic_flip_test(zero, 8, b),
              "zero squared perimeter rejects before division");
        // All ratios collapse to zero below the native strict threshold.
        auto thin = bad_diagonal({{0, 0, 1}, {4, 0, 2}, {4, 1e-7, 3}, {0, 3e-7, 4}});
        check(!native_vu_quadratic_flip_test(thin, 8, b),
              "near-zero signed ratios use native threshold");
        auto almost = bad_diagonal({{0, 0, 1}, {4, 0, 2}, {4, 1e-4, 3}, {0, 3e-4, 4}});
        check(native_vu_quadratic_flip_test(almost, 8, b),
              "resolved thin triangle ratios can improve");
    }
    auto prepare = [](const std::vector<Point3> &p) {
        TubeBudget b;
        auto in = build_native_vu_input(p, 0, b);
        prepare_native_vu_merge(in, b);
        const auto reg = regularize_native_vu_graph(in.graph, b);
        mark_native_vu_exterior(in.graph, reg.at("candidate_array_read_index").get<std::size_t>(),
                                b);
        const auto tri = triangulate_native_vu_interiors(in.graph, b);
        require(tri.at("interior_triangle_faces_completed") == true, "flip setup triangulates");
        return in.graph;
    };
    const Point3 marker{std::numeric_limits<double>::max(), 0, 0};
    for (const auto &p : std::vector<std::vector<Point3>>{
             {{0, 0, 1}, {4, 0, 2}, {6, 3, 3}, {4, 6, 4}, {0, 6, 5}, {-2, 3, 6}},
             {{0, 0, 0},
              {10, 0, 1},
              {10, 10, 2},
              {0, 10, 3},
              marker,
              {2, 2, 4},
              {8, 2, 5},
              {8, 8, 6},
              {2, 8, 7}},
             {{0, 0, 0}, {4, 4, 1}, {0, 4, 2}, {4, 0, 3}}}) {
        auto g = prepare(p);
        const auto before = g;
        const auto expected_area = area(g);
        TubeBudget b;
        const auto r = flip_native_vu_triangles(g, b);
        invariant(before, g);
        check(std::abs(area(g) - expected_area) < 1e-10 && r.at("remaining_candidates") == 0,
              "complete pipeline preserves concavity, holes and self-intersection area");
        for (auto n : native_vu_all_nodes(g, b)) {
            auto m = g.nodes[g.nodes[n].face_next].vertex_next;
            if (!((g.nodes[n].mask | g.nodes[m].mask) & 0xcef))
                check(!native_vu_quadratic_flip_test(g, n, b),
                      "all remaining eligible edges satisfy native quality predicate");
        }
    }
    {
        // Blocked large graphs isolate the native cap's three size regimes.
        for (std::size_t points : {10u, 601u, 1801u}) {
            std::vector<Point3> p;
            for (std::size_t i = 0; i < points; ++i)
                p.push_back({double(i), double(i % 3), 0});
            auto g = raw(p);
            TubeBudget b;
            const auto r = flip_native_vu_triangles(g, b);
            const double n = 2.0 * points, expected = points == 10    ? 20 * n
                                                      : points == 601 ? n * (n / 60)
                                                                      : 60 * n;
            check(r.at("native_iteration_limit").get<double>() == expected &&
                      r.at("native_iterations") == 1 && r.at("initial_candidate_count") == 0,
                  "native iteration cap counts all nodes including fixed edges");
        }
    }
    auto baseline = bad_diagonal(quad);
    auto complete = baseline;
    TubeBudget measured;
    flip_native_vu_triangles(complete, measured);
    for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1}) {
        auto g = baseline;
        const auto before = snapshot(g);
        TubeBudget b;
        b.max_work = limit;
        rejects([&] { flip_native_vu_triangles(g, b); });
        check(snapshot(g) == before, "budget exhaustion rolls back coordinates masks and topology");
    }
    {
        auto g = baseline;
        g.nodes[0].vertex_next = native_vu_null;
        const auto before = snapshot(g);
        TubeBudget b;
        rejects([&] { flip_native_vu_triangles(g, b); });
        check(snapshot(g) == before, "invalid topology stays unchanged");
        rejects([&] { native_vu_quadratic_flip_test(baseline, native_vu_null, b); });
        auto huge = baseline;
        for (auto &n : huge.nodes) {
            n.point[0] *= 1e200;
            n.point[1] *= 1e200;
        }
        const auto unchanged = snapshot(huge);
        rejects([&] { flip_native_vu_triangles(huge, b); });
        check(snapshot(huge) == unchanged, "nonfinite quality arithmetic rolls back graph");
        auto inactive = baseline;
        const auto pair = make_native_vu_pair(inactive, b);
        inactive.nodes[pair.first].mask = 0x10;
        free_marked_native_vu_edges(inactive, 0x10, b);
        const auto old = inactive;
        flip_native_vu_triangles(inactive, b);
        invariant(old, inactive);
        NativeVuGraph empty;
        const auto r = flip_native_vu_triangles(empty, b);
        check(r.at("native_iterations") == 0 && r.at("edge_adjustment_completed") == true,
              "empty graph needs no pop attempt");
        auto many = quad;
        many.push_back(many.front());
        const auto plan = native_facet_index_plan(many, b);
        check(plan.input_graph &&
                  plan.input_graph->report.at("edge_adjustment").at("edge_adjustment_completed") ==
                      true &&
                  plan.completed && plan.indices.size() == 8,
              "large-face path performs edge adjustment before final source indices");
    }
    auto task = [baseline, snapshot] {
        auto g = baseline;
        TubeBudget b;
        const auto r = flip_native_vu_triangles(g, b);
        return Json{{"report", r}, {"graph", snapshot(g)}};
    };
    const auto expected = task();
    std::vector<std::future<Json>> futures;
    for (unsigned i = 0; i < 4; ++i)
        futures.push_back(std::async(std::launch::async, task));
    for (auto &f : futures)
        check(f.get() == expected,
              "concurrent flip calls have independent graph and candidate state");
    return checks;
}
