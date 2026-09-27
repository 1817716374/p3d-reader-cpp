#include "native_vu_triangulate.hpp"
#include "native_vu_cluster.hpp"
#include "native_vu_regularize.hpp"
#include "native_vu_exterior.hpp"
#include "native_vu_split_support.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_vu_triangulate_tests() {
    unsigned checks = 0;
    auto check = [&](bool v, const char *why) {
        ++checks;
        require(v, why);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "triangulation rejects invalid data or exhausted budgets");
    };
    auto raw = [](const std::vector<Point3> &p) {
        TubeBudget b;
        return build_native_vu_input(p, 0, b).graph;
    };
    auto snapshot = [](const NativeVuGraph &g) {
        Json rows = Json::array();
        for (const auto &n : g.nodes)
            rows.push_back({n.all_next, n.face_next, n.vertex_next, n.mask, n.source_index, n.point,
                            n.active, n.internal_data});
        return Json{{"nodes", rows}, {"tail", g.tail}, {"free", g.free_head}};
    };
    auto prepare = [](const std::vector<Point3> &p) {
        TubeBudget b;
        auto in = build_native_vu_input(p, 0, b);
        prepare_native_vu_merge(in, b);
        const auto r = regularize_native_vu_graph(in.graph, b);
        require(r.at("regularization_completed") == true, "triangulation setup regularizes");
        mark_native_vu_exterior(in.graph, r.at("candidate_array_read_index").get<std::size_t>(), b);
        return in.graph;
    };
    // Audit the published topology independently of the sweep's diagnostics.
    auto triangles = [&](const NativeVuGraph &g, std::size_t expected, double area) {
        TubeBudget b;
        const auto all = validate_native_vu_split_graph(g, b);
        std::vector<bool> seen(g.nodes.size());
        std::size_t count = 0;
        double sum = 0;
        for (auto seed : all) {
            if (seen[seed])
                continue;
            std::size_t n = seed, edges = 0;
            double twice = 0;
            do {
                check(!seen[n], "triangulated faces are disjoint cycles");
                seen[n] = true;
                ++edges;
                const auto &p = g.nodes[n].point, &q = g.nodes[g.nodes[n].face_next].point;
                check((g.nodes[n].mask & 2) == (g.nodes[seed].mask & 2),
                      "face exterior mask is uniform");
                twice += p[0] * q[1] - p[1] * q[0];
                n = g.nodes[n].face_next;
            } while (n != seed);
            if (g.nodes[seed].mask & 2)
                continue;
            check(edges <= 3, "interior faces have at most three corners");
            if (edges == 3)
                ++count;
            check(twice >= -1e-12, "interior triangles do not reverse orientation");
            sum += twice * .5;
        }
        check(count == expected, "independently counted triangle faces");
        check(std::abs(sum - area) <= 1e-10 * std::max(1.0, std::abs(area)),
              "triangle area preserves parity region");
    };
    auto original_payloads = [&](const NativeVuGraph &before, const NativeVuGraph &g) {
        for (std::size_t i = 0; i < before.nodes.size(); ++i) {
            const auto &a = before.nodes[i], &b = g.nodes[i];
            if (!a.active)
                continue;
            check(b.active && a.point == b.point && a.source_index == b.source_index &&
                      a.internal_data == b.internal_data,
                  "original XYZ and source labels survive triangulation");
            check((a.mask & ~0x60000000u) == (b.mask & ~0x60000000u),
                  "only triangulation scratch masks change on old nodes");
        }
        for (std::size_t i = 0; i < g.nodes.size(); ++i) {
            if (!g.nodes[i].active || (i < before.nodes.size() && before.nodes[i].active))
                continue;
            check(g.nodes[i].mask == 0 && g.nodes[i].source_index == 0 &&
                      g.nodes[i].internal_data == 0,
                  "new diagonal nodes remain unnumbered with zero payload masks");
        }
    };
    const std::vector<Point3> hex{{0, 0, 1}, {4, 0, 2}, {6, 3, 3},
                                  {4, 6, 4}, {0, 6, 5}, {-2, 3, 6}};
    {
        auto g = raw(hex);
        TubeBudget b;
        for (std::size_t i = 0; i < hex.size(); ++i) {
            const auto c = native_vu_face_centroid(g, 2 * i, b);
            check(c.succeeded && c.point == Point2{2, 3} && c.reported_area == 18 &&
                      c.positive_count == 4 && c.nonpositive_count == 0,
                  "centroid preserves native extra half factor and fan counts");
        }
        auto reversed = hex;
        std::reverse(reversed.begin(), reversed.end());
        auto negative = raw(reversed);
        const auto c = native_vu_face_centroid(negative, 0, b);
        check(c.succeeded && c.point == Point2{2, 3} && c.reported_area == -18 &&
                  c.positive_count == 0 && c.nonpositive_count == 4,
              "clockwise centroid keeps signed area");
        auto flat = raw({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}});
        const auto z = native_vu_face_centroid(flat, 0, b);
        check(!z.succeeded && z.point == Point2{0, 0} && z.reported_area == 0 &&
                  z.positive_count == 0 && z.nonpositive_count == 4,
              "zero-area fan counts zero terms as nonpositive");
        for (const auto &p :
             std::vector<std::vector<Point3>>{{{2, 3, 4}}, {{2, 3, 4}, {5, 6, 7}}}) {
            auto short_graph = raw(p);
            const auto s = native_vu_face_centroid(short_graph, 0, b);
            check(!s.succeeded && s.point == Point2{2, 3} && s.positive_count == 0 &&
                      s.nonpositive_count == 0,
                  "one and two corner centroids keep origin without fan terms");
        }
    }
    {
        auto g = raw(hex);
        TubeBudget b;
        mark_native_vu_exterior(g, 0, b);
        const auto before = g;
        const auto r = triangulate_native_vu_face(g, 0, b);
        check(r.at("monotone_check_passed") == true && r.at("added_edges") == 3 &&
                  r.at("left_runs").get<unsigned>() > 0 && r.at("right_runs").get<unsigned>() > 0 &&
                  r.at("guard_stopped") == false,
              "convex sweep exercises both chains");
        triangles(g, 4, 36);
        original_payloads(before, g);
        for (std::size_t i = before.nodes.size(); i < g.nodes.size(); ++i)
            check(std::find(hex.begin(), hex.end(), g.nodes[i].point) != hex.end(),
                  "normal diagonals copy full endpoint XYZ");
    }
    {
        auto g = raw(hex);
        TubeBudget b;
        mark_native_vu_exterior(g, 0, b);
        const auto before = g;
        const auto r = triangulate_native_vu_face(g, 2, b);
        check(r.at("monotone_check_passed") == false && r.at("fallback_status") == "constructed" &&
                  r.at("added_edges") == 6,
              "nonminimum seed takes native centroid fallback");
        triangles(g, 6, 36);
        original_payloads(before, g);
        check(g.nodes[12].point == Point3{2, 3, 0} && g.nodes[13].point == Point3{4, 0, 0},
              "first fallback pair initializes both Z values to zero");
        std::size_t center_count = 0;
        for (const auto &n : g.nodes)
            if (n.point == Point3{2, 3, 0})
                ++center_count;
        check(center_count == 6, "centroid spokes share six vertex sectors");
        auto top = before;
        const auto top_before = snapshot(top);
        const auto stop = triangulate_native_vu_face(top, 6, b);
        check(stop.at("monotone_check_passed") == true && stop.at("guard_stopped") == true &&
                  stop.at("centroid_fallback_attempted") == false && snapshot(top) == top_before,
              "native local maximum guard stops without an invented fallback");
    }
    {
        for (auto p : std::vector<std::vector<Point3>>{
                 {{0, 0, 1}, {4, 0, 2}, {4, 4, 3}, {0, 4, 4}},
                 {{0, 0, 1}, {4, 0, 2}, {6, 3, 3}, {3, 6, 4}, {0, 4, 5}}}) {
            auto g = raw(p);
            const auto before = snapshot(g);
            TubeBudget b;
            const auto r = triangulate_native_vu_face(g, 2, b);
            check(r.at("fallback_status") == "insufficient_fan_triangles" && snapshot(g) == before,
                  "native fallback refuses fewer than six polygon corners");
        }
        // The mathematical centroid sees every edge, but the native first dv
        // uses start.X: here successor.Y == start.X makes its first cross zero.
        auto g = raw({{0, 0, 1}, {6, 2, 2}, {6, 6, 3}, {3, 5, 4}, {0, 8, 5}, {-2, 3, 6}});
        const auto before = snapshot(g);
        TubeBudget b;
        const auto center = native_vu_face_centroid(g, 2, b);
        check(center.nonpositive_count > 0, "zero fan term enables native visibility test");
        std::size_t edge = 2;
        do {
            const auto &a = g.nodes[edge].point;
            const auto &z = g.nodes[g.nodes[edge].face_next].point;
            check((a[0] - center.point[0]) * (z[1] - center.point[1]) -
                          (a[1] - center.point[1]) * (z[0] - center.point[0]) >
                      0,
                  "corrected XY visibility would accept this star-shaped face");
            edge = g.nodes[edge].face_next;
        } while (edge != 2);
        const auto r = triangulate_native_vu_face(g, 2, b);
        check(r.at("fallback_status") == "visibility_rejected" && snapshot(g) == before,
              "native first visibility coordinate is preserved rather than repaired");
        auto flat = raw({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}});
        const auto flat_before = snapshot(flat);
        const auto zero = triangulate_native_vu_face(flat, 2, b);
        check(zero.at("fallback_status") == "zero_area" && snapshot(flat) == flat_before,
              "zero-area fallback does not publish a center pair");
    }
    const Point3 marker{std::numeric_limits<double>::max(), 0, 0};
    const std::vector<Point3> square{{0, 0, 0}, {10, 0, 1}, {10, 10, 2}, {0, 10, 3}};
    auto pipeline = [&](const std::vector<Point3> &p, std::size_t count, double area) {
        auto g = prepare(p);
        const auto before = g;
        TubeBudget b;
        const auto r = triangulate_native_vu_interiors(g, b);
        check(r.at("interior_triangle_faces_completed") == true &&
                  r.at("final_indices_ready") == false &&
                  r.at("remaining_large_interior_faces") == 0,
              "interior triangles do not claim final indices");
        original_payloads(before, g);
        triangles(g, count, area);
        const auto again = triangulate_native_vu_interiors(g, b);
        check(again.at("added_edges") == 0 && again.at("processed_large_faces") == 0 &&
                  again.at("candidate_array_read_index") == 0,
              "existing triangles are removed from candidate array");
    };
    pipeline(hex, 4, 36);
    pipeline(square, 2, 100);
    pipeline({{0, 0, 0}, {4, 4, 0}, {0, 4, 0}, {4, 0, 0}}, 2, 8);
    pipeline({{0, 0, 1}, {6, 2, 2}, {6, 6, 3}, {3, 4, 4}, {0, 8, 5}, {-2, 3, 6}}, 4, 35);
    for (int direction = 0; direction < 8; ++direction) {
        std::vector<std::vector<Point3>> loops{square,
                                               {{2, 2, 4}, {8, 2, 5}, {8, 8, 6}, {2, 8, 7}},
                                               {{4, 4, 8}, {6, 4, 9}, {6, 6, 10}, {4, 6, 11}}};
        std::vector<Point3> p;
        for (unsigned k = 0; k < loops.size(); ++k) {
            if (k)
                p.push_back(marker);
            if (direction & (1 << k))
                std::reverse(loops[k].begin(), loops[k].end());
            p.insert(p.end(), loops[k].begin(), loops[k].end());
        }
        pipeline(p, 10, 68);
    }
    for (int copies : {2, 3}) {
        std::vector<Point3> p;
        for (int i = 0; i < copies; ++i) {
            if (i)
                p.push_back(marker);
            p.insert(p.end(), square.begin(), square.end());
        }
        pipeline(p, copies % 2 ? 2 : 0, copies % 2 ? 100 : 0);
    }
    {
        NativeVuGraph g;
        TubeBudget b;
        const auto p = make_native_vu_pair(g, b);
        check(p.first == 0 && p.second == 1 && g.nodes[0].face_next == 1 &&
                  g.nodes[1].face_next == 0 && g.nodes[0].vertex_next == 0 &&
                  g.nodes[1].vertex_next == 1,
              "isolated pair has crossed face successors and self vertex successors");
        g.nodes[0].mask = 8;
        g.nodes[0].point = {1, 2, 3};
        g.nodes[0].source_index = 99;
        check(free_marked_native_vu_edges(g, 8, b) == 2,
              "isolated edge can enter native free list");
        const auto free = g.free_head;
        b.max_control_points = 2;
        const auto reused = make_native_vu_pair(g, b);
        check(reused.first == free && g.nodes.size() == 2 && g.free_head == native_vu_null,
              "pair creation reuses freed stable slots at storage limit");
        for (const auto &n : g.nodes)
            check(n.point == Point3{} && n.mask == 0 && n.source_index == 0 && n.internal_data == 0,
                  "reused pair clears old payload");
        validate_native_vu_split_graph(g, b);
    }
    for (bool fallback : {false, true}) {
        auto baseline = raw(hex);
        TubeBudget measured;
        auto success = baseline;
        triangulate_native_vu_face(success, fallback ? 2 : 0, measured);
        for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1}) {
            auto g = baseline;
            const auto before = snapshot(g);
            TubeBudget b;
            b.max_work = limit;
            rejects([&] { triangulate_native_vu_face(g, fallback ? 2 : 0, b); });
            check(snapshot(g) == before,
                  "direct sweep budget failure never publishes partial joins");
        }
        auto g = baseline;
        const auto before = snapshot(g);
        TubeBudget b;
        b.max_control_points = 14;
        rejects([&] { triangulate_native_vu_face(g, fallback ? 2 : 0, b); });
        check(snapshot(g) == before, "partial edge allocation rolls back on point limit");
    }
    {
        auto baseline = prepare(hex);
        auto complete = baseline;
        TubeBudget measured;
        triangulate_native_vu_interiors(complete, measured);
        for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1}) {
            auto g = baseline;
            const auto before = snapshot(g);
            TubeBudget b;
            b.max_work = limit;
            rejects([&] { triangulate_native_vu_interiors(g, b); });
            check(snapshot(g) == before,
                  "whole interior pass rolls back masks and topology on failure");
        }
        auto bad = baseline;
        bad.nodes[0].face_next = native_vu_null;
        const auto before = snapshot(bad);
        TubeBudget b;
        rejects([&] { triangulate_native_vu_interiors(bad, b); });
        check(snapshot(bad) == before, "malformed topology is not modified");
        rejects([&] { native_vu_face_centroid(baseline, native_vu_null, b); });
        auto huge = raw({{0, 0, 0}, {1e200, 0, 0}, {1e200, 1e200, 0}, {0, 1e200, 0}});
        rejects([&] { native_vu_face_centroid(huge, 0, b); });
        NativeVuGraph empty;
        const auto r = triangulate_native_vu_interiors(empty, b);
        check(r.at("triangle_face_count") == 0 && r.at("input_interior_face_count") == 0,
              "empty graph is a completed empty interior pass");
    }
    auto concurrent = [=] {
        auto g = prepare(hex);
        TubeBudget b;
        const auto report = triangulate_native_vu_interiors(g, b);
        return Json{{"report", report}, {"graph", snapshot(g)}};
    };
    const auto expected = concurrent();
    std::vector<std::future<Json>> workers;
    for (unsigned i = 0; i < 4; ++i)
        workers.push_back(std::async(std::launch::async, concurrent));
    for (auto &worker : workers)
        check(worker.get() == expected, "triangulation has no shared mutable state");
    return checks;
}
