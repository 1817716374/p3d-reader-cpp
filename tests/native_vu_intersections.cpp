#include "native_vu_intersections.hpp"
#include "native_vu_cluster.hpp"
#include "native_vu_split_support.hpp"
#include <future>
#include <set>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_vu_intersections_tests() {
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
        check(caught, "native VU intersection validation/budget rejection");
    };
    auto solve = [](double a, double b, double c, double d, double x, double y) {
        TubeBudget budget;
        return solve_native_vu_2x2(a, b, c, d, x, y, budget);
    };
    check(solve(1, 0, 0, 1, 2, 3).value == Point2{2, 3}, "identity scaled solve");
    check(solve(0, 1, 1, 0, 2, 3).value == Point2{3, 2}, "negative determinant scaled solve");
    check(solve(4, 4, 4, -4, 4, 0).value == Point2{0x1.ffffffffffffep-2, 0x1.ffffffffffffep-2},
          "native singular-value product keeps its rounding instead of direct determinant result");
    check(!solve(0, 1, 0, 2, 3, 4).solved && !solve(1, 2, 2, 4, 3, 6).solved,
          "zero-column and rank-one systems have no solution");
    check(!solve(1, 1, 1, 1 + 1e-14, 2, 2).solved && solve(1, 1, 1, 1 + 1e-10, 2, 2).solved,
          "relative singular-value threshold rejects nearly parallel equations");
    for (double scale : {1e-150, 1e-12, 1., 1e12, 1e150}) {
        auto r = solve(2 * scale, -1 / scale, scale, 3 / scale, 1, 7);
        check(r.solved, "column scaling handles disparate magnitudes");
        const double x = r.value[0] * scale, y = r.value[1] / scale;
        check(std::abs(2 * x - y - 1) < 1e-13 && std::abs(x + 3 * y - 7) < 1e-13,
              "scaled solutions satisfy original two equations");
    }
    rejects([&] { solve(std::numeric_limits<double>::infinity(), 0, 0, 1, 1, 1); });
    rejects([&] {
        TubeBudget b;
        b.max_work = 47;
        solve_native_vu_2x2(1, 0, 0, 1, 1, 1, b);
    });
    const Point3 marker{std::numeric_limits<double>::max(), 0, 0};
    auto input = [](const std::vector<Point3> &points) {
        TubeBudget b;
        return build_native_vu_input(points, 0, b);
    };
    auto run = [](NativeVuGraph &g, double vertex = 0, double along = 0) {
        TubeBudget b;
        return split_native_vu_edges_at_intersections(g, vertex, along, b);
    };
    auto snapshot = [](const NativeVuGraph &g) {
        Json a = Json::array();
        for (const auto &n : g.nodes)
            a.push_back({n.all_next, n.face_next, n.vertex_next, n.mask, n.source_index, n.point});
        return Json{{"tail", g.tail}, {"nodes", a}};
    };
    const std::vector<Point3> bow{{0, 0, 0}, {4, 4, 8}, {0, 4, 100}, {4, 0, 108}};
    auto g = input(bow).graph;
    const auto before = g;
    auto r = run(g);
    check(r.at("interior_pairs") == 1 && r.at("record_count") == 2 && r.at("split_count") == 2 &&
              r.at("added_nodes") == 4,
          "bow-tie splits both crossing edges exactly once");
    std::set<double> heights;
    for (std::size_t i = before.nodes.size(); i < g.nodes.size(); ++i) {
        const auto &n = g.nodes[i];
        check(std::abs(n.point[0] - 2) < 1e-14 && std::abs(n.point[1] - 2) < 1e-14,
              "crossing split has expected XY");
        heights.insert(n.point[2]);
        check(n.source_index == 0 && n.mask == 1,
              "crossing nodes keep native default labels and copied boundary mask");
        check(g.nodes[n.vertex_next].vertex_next == i,
              "each new edge retains separate two-node vertex ring");
    }
    check(heights.size() == 2 && std::abs(*heights.begin() - 4) < 1e-12 &&
              std::abs(*heights.rbegin() - 104) < 1e-12,
          "crossing edges retain independent heights until later coordinate connection");
    for (std::size_t i = 0; i < before.nodes.size(); ++i)
        check(g.nodes[i].point == before.nodes[i].point &&
                  g.nodes[i].source_index == before.nodes[i].source_index &&
                  g.nodes[i].mask == before.nodes[i].mask,
              "intersection pass does not rewrite original payloads or masks");
    TubeBudget topology;
    check(validate_native_vu_split_graph(g, topology).size() == g.nodes.size(),
          "crossing graph retains valid topology");
    auto blocked = before;
    check(run(blocked, 3, 0).at("split_count") == 0,
          "vertex tolerance excludes intersection fractions near endpoints");
    auto no_length = before;
    check(run(no_length, 0, 8).at("split_count") == 0,
          "edge shorter than along tolerance remains unsplit");
    auto endpoints = input({{0, 0, 0}, {1, 1, 1}, marker, {1, 1, 2}, {2, 0, 3}}).graph;
    check(run(endpoints).at("interior_pairs") == 0,
          "endpoint contact is not an interior transverse intersection");
    auto overlap = input({{0, 0, 0}, {4, 4, 4}, marker, {1, 1, 9}, {3, 3, 11}}).graph;
    auto overlap_report = run(overlap);
    check(overlap_report.at("split_count") == 0 &&
              overlap_report.at("singular_pairs").get<unsigned>() > 0,
          "collinear overlapping edges are rejected by singular solve");
    auto horizontal = input({{0, 1, 0}, {4, 1, 4}, marker, {1, 1, 10}, {3, 1, 12}}).graph;
    check(run(horizontal).at("tested_pairs") == 0,
          "Y boundary equality breaks pair scan before solving");
    auto vertical = input({{1, 0, 0}, {1, 4, 4}, marker, {1, 1, 10}, {1, 3, 12}}).graph;
    check(run(vertical).at("tested_pairs") == 0, "X boundary equality filters pair before solving");
    // Standalone two-point loops contain two distinct original edges per line.
    // Repeated crossings must collapse per edge, without deduplicating lines.
    std::vector<Point3> grid;
    auto line = [&](Point3 a, Point3 b) {
        if (!grid.empty())
            grid.push_back(marker);
        grid.push_back(a);
        grid.push_back(b);
    };
    for (unsigned i = 1; i <= 4; ++i)
        line({0, double(i), double(i)}, {5, double(i), double(i)});
    for (unsigned i = 1; i <= 4; ++i)
        line({double(i), 0, 100 + double(i)}, {double(i), 5, 100 + double(i)});
    auto mesh = input(grid).graph;
    const auto original_size = mesh.nodes.size();
    auto grid_report = run(mesh);
    check(grid_report.at("interior_pairs") == 64 && grid_report.at("record_count") == 128 &&
              grid_report.at("split_count") == 64 && mesh.nodes.size() == original_size + 128,
          "large record sort groups crossings by original edge and collapses repeated fractions");
    TubeBudget mesh_topology;
    check(validate_native_vu_split_graph(mesh, mesh_topology).size() == mesh.nodes.size(),
          "many successive splits preserve complete graph permutations");
    for (double f : {1e-14, 1e-12, 1 - 1e-14, 1 - 1e-12}) {
        auto near_end = input({{0, 0, 0}, {1, 0, 10}, marker, {f, -1, 100}, {f, 1, 110}}).graph;
        check(run(near_end).at("split_count") == ((f > 1e-13 && f < .9999999999999) ? 4 : 0),
              "strict native interior-fraction band suppresses near-end transverse crossings");
    }
    // The public preparation chain still reports incomplete angle connection.
    auto prepared = input(bow);
    TubeBudget merge;
    prepare_native_vu_merge(prepared, merge);
    const auto &prep = prepared.report.at("merge_preparation");
    check(prep.at("edge_splitting_completed") == true && prep.at("merge_completed") == false &&
              prep.at("intersection_splitting").at("split_count") == 2,
          "merge preparation reaches actual crossing splits without claiming complete vertex "
          "connection");
    auto original = before;
    const auto saved = snapshot(original);
    TubeBudget measure;
    auto measured = before;
    split_native_vu_edges_at_intersections(measured, 0, 0, measure);
    for (auto limit : {std::size_t(0), measure.work - 1}) {
        rejects([&] {
            TubeBudget b;
            b.max_work = limit;
            split_native_vu_edges_at_intersections(original, 0, 0, b);
        });
        check(snapshot(original) == saved, "work limit failure leaves original graph untouched");
    }
    rejects([&] {
        TubeBudget b;
        b.max_control_points = before.nodes.size();
        split_native_vu_edges_at_intersections(original, 0, 0, b);
    });
    check(snapshot(original) == saved, "allocation limit failure rolls back crossing splits");
    rejects([&] { run(original, -1, 0); });
    rejects([&] {
        auto bad = before;
        bad.nodes[0].face_next = 999;
        run(bad);
    });
    rejects([&] {
        auto bad = before;
        bad.nodes[0].point[0] = std::numeric_limits<double>::quiet_NaN();
        run(bad);
    });
    auto parallel = [&] {
        auto copy = before;
        auto report = run(copy);
        return std::make_pair(snapshot(copy), report);
    };
    auto task = std::async(std::launch::async, parallel);
    check(task.get() == parallel(), "parallel crossing work has no shared state");
    NativeVuGraph empty;
    check(run(empty).at("split_count") == 0, "empty graph has no crossing records");
    return checks;
}
