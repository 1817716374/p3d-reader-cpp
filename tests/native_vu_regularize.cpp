#include "native_vu_regularize.hpp"
#include "native_vu_cluster.hpp"
#include "native_vu_split_support.hpp"
#include "native_tube_mesh_index_rules.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_vu_regularize_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        ++checks;
        require(value, why);
    };
    auto rejects = [&](auto fn) {
        bool failed = false;
        try {
            fn();
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, "regularization rejects invalid data or exhausted budget");
    };
    const Point3 mark{std::numeric_limits<double>::max(), 0, 0};
    auto build = [](const std::vector<Point3> &points) {
        TubeBudget b;
        return build_native_vu_input(points, 0, b).graph;
    };
    auto merged = [](const std::vector<Point3> &points) {
        TubeBudget b;
        auto input = build_native_vu_input(points, 0, b);
        prepare_native_vu_merge(input, b);
        return input.graph;
    };
    auto snapshot = [](const NativeVuGraph &g) {
        Json nodes = Json::array();
        for (const auto &n : g.nodes)
            nodes.push_back({n.all_next, n.face_next, n.vertex_next, n.mask, n.source_index,
                             n.point, n.active, n.internal_data});
        return Json{{"nodes", nodes}, {"tail", g.tail}, {"free", g.free_head}};
    };
    auto sector = [](const NativeVuGraph &g, int source, int next_source) {
        for (std::size_t n = 0; n < g.nodes.size(); ++n)
            if (g.nodes[n].active && g.nodes[n].source_index == source &&
                g.nodes[g.nodes[n].face_next].source_index == next_source)
                return n;
        throw std::runtime_error("missing test sector");
    };
    for (const auto &q : std::vector<Point3>{{1, 1, 0}, {2, 0, 0}, {-1, 1, 0}, {0, 0, 0}}) {
        auto g = build({{0, 0, 0}, {4, 0, 0}, {0, 4, 0}, mark, q});
        TubeBudget b;
        const auto query = sector(g, 4, 4);
        check(native_vu_node_in_sector(g, query, sector(g, 0, 1), b) == (q[0] == 1),
              "convex sector excludes boundary and exterior");
        check(native_vu_node_in_sector(g, query, sector(g, 0, 2), b) == (q[0] == -1),
              "reflex sector selects the exterior wedge");
    }
    {
        auto g = build({{0, 0, 0}, {4, 0, 0}});
        const auto s = sector(g, 0, 1), opposite = g.nodes[s].face_next;
        TubeBudget b;
        check(native_vu_node_in_sector(g, opposite, s, b), "null face accepts its own other node");
        check(!native_vu_node_in_sector(g, g.nodes[opposite].vertex_next, s, b),
              "equal coordinate with different identity is outside null face");
        auto input = build({{0, 0, 0}, {4, 0, 0}, {0, 4, 0}});
        auto before = snapshot(input);
        auto limited = b;
        limited.max_control_points = input.nodes.size();
        rejects([&] { join_native_vu_sectors(input, 0, 2, limited); });
        check(snapshot(input) == before, "join allocation failure leaves graph unchanged");
    }
    const std::vector<Point3> square{{0, 0, -0.0}, {10, 0, 1}, {10, 10, 2}, {0, 10, 3}};
    {
        TubeBudget b;
        NativeVuGraph isolated;
        const auto pair = split_native_vu_edge(isolated, native_vu_null, b);
        twist_native_vu_vertices(isolated, pair.first, pair.second, b);
        check(native_vu_node_in_sector(isolated, pair.first, pair.first, b),
              "full-circle singleton vertex sector accepts coincident node");
        for (double x : {1.0, -3.0}) {
            auto g = build({{0, 0, 0}, {-2, 0, 0}, {-1, 0, 0}, mark, {x, 0, 0}});
            check(native_vu_node_in_sector(g, sector(g, 4, 4), sector(g, 0, 1), b) == (x > 0),
                  "collinear non-null sector uses forward dot-product condition");
        }
    }
    {
        auto g = build(square);
        TubeBudget b;
        const auto a = sector(g, 0, 1), z = sector(g, 2, 3);
        const auto edge = join_native_vu_sectors(g, a, z, b);
        validate_native_vu_split_graph(g, b);
        check(g.nodes[edge.first].point == square[0] && g.nodes[edge.second].point == square[2],
              "join copies both endpoint XYZ");
        check(g.nodes[edge.first].source_index == 0 && g.nodes[edge.second].source_index == 0 &&
                  g.nodes[edge.first].mask == 0 && g.nodes[edge.second].mask == 0,
              "join does not inherit source labels or numbered/boundary masks");
        check(g.nodes[g.nodes[edge.first].face_next].vertex_next == edge.second,
              "joined nodes are mates after the two vertex twists");
        const auto prior_slots = g.nodes.size();
        g.nodes[edge.first].mask |= 0x10000000;
        check(free_marked_native_vu_edges(g, 0x10000000, b) == 2,
              "bridge removal reclaims both nodes");
        auto free_head = g.free_head;
        auto free_next = g.nodes[free_head].all_next;
        b.max_control_points = prior_slots;
        auto reused = join_native_vu_sectors(g, a, z, b);
        check(reused.first == free_head && reused.second == free_next &&
                  g.nodes.size() == prior_slots,
              "join consumes free slots in original order at storage limit");
        validate_native_vu_split_graph(g, b);
    }
    auto audit = [&](NativeVuGraph &g) {
        auto before = g;
        TubeBudget b;
        auto result = regularize_native_vu_graph(g, b);
        check(result.at("regularization_completed") == true && result.at("triangulated") == false,
              "two sweeps finish without claiming triangulation");
        const auto all = validate_native_vu_split_graph(g, b);
        for (std::size_t i = 0; i < before.nodes.size(); ++i) {
            if (!before.nodes[i].active)
                continue;
            check(g.nodes[i].point == before.nodes[i].point &&
                      g.nodes[i].source_index == before.nodes[i].source_index &&
                      g.nodes[i].internal_data == before.nodes[i].internal_data,
                  "regularization preserves original points and both labels");
            check((g.nodes[i].mask & ~0x60000000u) == (before.nodes[i].mask & ~0x60000000u),
                  "original nonscratch mask bits survive both sweeps");
            for (int axis = 0; axis < 3; ++axis)
                check(std::signbit(g.nodes[i].point[axis]) ==
                          std::signbit(before.nodes[i].point[axis]),
                      "double rotation restores signed zero and leaves Z unchanged");
        }
        std::vector<bool> seen(g.nodes.size());
        std::size_t faces = 0, vertices = 0;
        for (auto seed : all)
            if (!seen[seed]) {
                ++faces;
                auto n = seed;
                double area2 = 0;
                std::size_t minima = 0, maxima = 0;
                auto below = [&](std::size_t a, std::size_t c) {
                    const auto &p = g.nodes[a].point, &q = g.nodes[c].point;
                    return p[1] < q[1] || (p[1] == q[1] && p[0] < q[0]);
                };
                do {
                    check(!seen[n], "each regularized face closes at its own seed");
                    seen[n] = true;
                    auto next = g.nodes[n].face_next, second = g.nodes[next].face_next;
                    const auto &p = g.nodes[n].point, &q = g.nodes[next].point;
                    area2 += p[0] * q[1] - p[1] * q[0];
                    bool u = below(n, next), v = below(next, second);
                    minima += !u && v;
                    maxima += u && !v;
                    n = next;
                } while (n != seed);
                if (area2 > 0)
                    check(minima == 1 && maxima == 1,
                          "bounded positive-area face is lexically monotone");
            }
        std::fill(seen.begin(), seen.end(), false);
        for (auto seed : all)
            if (!seen[seed]) {
                ++vertices;
                auto n = seed;
                do {
                    check(!seen[n], "each vertex ring closes at its own seed");
                    seen[n] = true;
                    n = g.nodes[n].vertex_next;
                } while (n != seed);
            }
        check(vertices + faces == all.size() / 2 + 2,
              "connected planar graph satisfies Euler identity");
        return result;
    };
    auto plain = merged(square);
    check(audit(plain).at("joined_edges") == 0, "monotone rectangle requires no diagonal");
    const std::vector<Point3> hole{{0, 0, 0}, {10, 0, 1}, {10, 10, 2}, {0, 10, 3}, mark,
                                   {3, 3, 4}, {3, 7, 5},  {7, 7, 6},   {7, 3, 7}};
    auto holed = merged(hole);
    auto hole_report = audit(holed);
    check(hole_report.at("joined_edges") == 2, "rectangular hole receives lower and upper bridges");
    check(holed.nodes[16].point == hole[5] && holed.nodes[17].point == hole[1] &&
              holed.nodes[18].point == hole[7] && holed.nodes[19].point == hole[3],
          "both sweeps use the original right-boundary endpoints, not nearest coordinates");
    check((holed.nodes[16].mask & 0x80000001u) == 0 && (holed.nodes[19].mask & 0x80000001u) == 0,
          "new hole bridges are unnumbered interior edges");
    const std::vector<Point3> concave{{0, 0, 0}, {10, 0, 1}, {10, 10, 2}, {7, 10, 3},
                                      {7, 3, 4}, {3, 3, 5},  {3, 10, 6},  {0, 10, 7}};
    auto concave_graph = merged(concave);
    check(audit(concave_graph).at("downward_sweep").at("left_joins") == 1,
          "concave upper sector connects through the left-boundary path");
    auto separate = merged({{0, 0, 0},
                            {3, 0, 0},
                            {3, 3, 0},
                            {0, 3, 0},
                            mark,
                            {5, 5, 0},
                            {8, 5, 0},
                            {8, 8, 0},
                            {5, 8, 0}});
    check(audit(separate).at("upward_sweep").at("peak_joins") == 1,
          "disconnected rings connect through highest completed peak");
    for (bool reverse : {false, true}) {
        std::vector<Point3> many{{0, 0, 0}, {40, 0, 0}, {40, 30, 0}, {0, 30, 0}};
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 4; ++col) {
                many.push_back(mark);
                double x = 2 + 8 * col, y = 2 + 8 * row;
                std::vector<Point3> ring{
                    {x, y, 1}, {x, y + 3, 2}, {x + 3, y + 3, 3}, {x + 3, y, 4}};
                if (reverse)
                    std::reverse(ring.begin(), ring.end());
                many.insert(many.end(), ring.begin(), ring.end());
            }
        auto g = merged(many);
        audit(g); // More than eight minima exercises the native qsort partition.
    }
    for (double height : {2.0, std::nextafter(1.0, 2.0)}) {
        auto g = build({{0, 0, 0},
                        {12, 0, 0},
                        {12, 3, 0},
                        {0, 3, 0},
                        mark,
                        {2, 1, 0},
                        {2, height, 0},
                        {4, height, 0},
                        {4, 1, 0}});
        audit(g); // One-ULP-high ring and horizontal scan ties, without clustering it away.
    }
    for (bool reverse : {false, true}) {
        std::vector<Point3> star;
        for (int i = 0; i < 48; ++i) {
            double angle = i * 6.2831853071795864769 / 48, radius = i % 2 ? 4.0 : 10.0;
            star.push_back({radius * std::cos(angle), radius * std::sin(angle), double(i)});
        }
        if (reverse)
            std::reverse(star.begin(), star.end());
        auto g = merged(star);
        audit(g);
    }
    {
        NativeVuGraph g;
        TubeBudget b;
        check(regularize_native_vu_graph(g, b).at("active_node_count") == 0,
              "empty graph completes without allocation");
    }
    auto original = merged(hole);
    const auto saved = snapshot(original);
    TubeBudget complete;
    auto reference = original;
    const auto reference_report = regularize_native_vu_graph(reference, complete);
    for (auto limit : std::vector<std::uint64_t>{0, 48, complete.work / 2, complete.work - 1}) {
        auto copy = original;
        TubeBudget b;
        b.max_work = limit;
        rejects([&] { regularize_native_vu_graph(copy, b); });
        check(snapshot(copy) == saved, "failed sweep publishes no partial bridges or rotation");
    }
    {
        auto copy = original;
        TubeBudget b;
        b.max_control_points = copy.nodes.size();
        rejects([&] { regularize_native_vu_graph(copy, b); });
        check(snapshot(copy) == saved, "node budget failure restores pre-sweep graph");
        copy.nodes[0].face_next = native_vu_null;
        rejects([&] { regularize_native_vu_graph(copy, b); });
        copy = original;
        copy.nodes[0].point[0] = std::numeric_limits<double>::infinity();
        rejects([&] { regularize_native_vu_graph(copy, b); });
    }
    std::vector<std::future<Json>> jobs;
    for (int i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async, [&] {
            auto copy = original;
            TubeBudget b;
            auto r = regularize_native_vu_graph(copy, b);
            return Json{{"report", r}, {"graph", snapshot(copy)}};
        }));
    for (auto &job : jobs)
        check(job.get() == Json{{"report", reference_report}, {"graph", snapshot(reference)}},
              "scratch masks and equal-key comparator state are local to each call");
    TubeBudget b;
    auto plan = native_facet_index_plan(concave, b);
    check(plan.indices.empty() &&
              plan.input_graph->report.at("regularization").at("regularization_completed") == true,
          "large facet route consumes regularization but keeps unfinished triangle output empty");
    return checks;
}
