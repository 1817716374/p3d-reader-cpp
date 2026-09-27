#include "native_projected_polygon.hpp"
#include "native_tube_mesh_cap_input.hpp"
#include <future>
#include <set>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_projected_polygon_tests() {
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
        check(caught, "projected polygon invalid input or budget rejected");
    };
    Matrix4 identity{};
    for (unsigned i = 0; i < 4; ++i)
        identity[i][i] = 1;
    auto run = [&](const std::vector<Point3> &p) {
        TubeBudget b;
        return triangulate_native_projected_polygon(p, identity, identity, 0, b);
    };
    auto area = [&](const NativeProjectedPolygon &r) {
        double total = 0;
        std::vector<std::size_t> face;
        for (auto index : r.indices) {
            if (index) {
                const auto id = std::size_t(std::abs(index) - 1);
                check(id < r.points.size(), "all output indices resolve");
                face.push_back(id);
            } else {
                check(face.size() == 3, "output face is triangular");
                const auto &a = r.points[face[0]], &b = r.points[face[1]], &c = r.points[face[2]];
                total +=
                    std::abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) * 0.5;
                face.clear();
            }
        }
        check(face.empty(), "all faces terminated");
        return total;
    };
    const std::vector<Point3> cross{{0, 0, 0}, {4, 4, 0}, {0, 4, 0}, {4, 0, 0}};
    auto result = run(cross);
    check(result.native_succeeded && result.complete,
          "self-crossing polygon has coordinate output");
    check(result.points.size() == 5 && std::abs(result.points.back()[0] - 2) < 1e-12 &&
              std::abs(result.points.back()[1] - 2) < 1e-12 && result.points.back()[2] == 0,
          "crossing appended once at native graph location");
    check(std::equal(cross.begin(), cross.end(), result.points.begin()),
          "complete source prefix retained");
    check(area(result) == 8, "self-crossing parity area correct");
    check(result.report["index_output"]["new_vertices_created"] == 1, "new vertex diagnosed");
    check(std::find(result.indices.begin(), result.indices.end(), 5) != result.indices.end(),
          "first new vertex index is positive after promotion");
    const double max = std::numeric_limits<double>::max();
    const Point3 marker{max, max, max};
    std::vector<Point3> holes{{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}, marker,
                              {3, 3, 0}, {7, 3, 0},  {7, 7, 0},   {3, 7, 0},  marker};
    auto hole = run(holes);
    check(hole.complete && hole.points == holes,
          "hole output preserves all markers and source values");
    check(area(hole) == 84, "hole parity area is outer minus inner");
    check(hole.report["index_output"]["new_vertices_created"] == 0,
          "holes reuse source identities");
    auto opposite = holes;
    std::reverse(opposite.begin() + 5, opposite.begin() + 9);
    check(area(run(opposite)) == 84, "hole winding does not replace parity classification");
    std::vector<Point3> duplicate{{0, 0, 0}, {3, 0, 0}, {3, 0, 0}, {3, 2, 0}, {0, 2, 0}, {0, 0, 0}};
    auto repeated = run(duplicate);
    check(repeated.points == duplicate && area(repeated) == 6,
          "unused duplicate source points remain in prefix");
    std::vector<Point3> leading{marker, {0, 0, 0}, {3, 0, 0}, {0, 2, 0}, marker};
    auto lead = run(leading);
    check(lead.complete && area(lead) == 3, "general signed loop scan accepts leading disconnect");
    check(lead.points == leading, "leading and trailing source markers retained");
    rejects([&] {
        TubeBudget b;
        build_native_vu_input(leading, 0, b);
    });
    {
        TubeBudget b;
        const auto g = build_native_vu_polygon_input({{0, 0, max}, {1, 0, 0}, {0, 1, 0}}, 0, b);
        check(g.loops.size() == 1 && g.loops[0].inserted_vertices == 3,
              "general input disconnect test excludes Z");
    }
    Matrix4 forward = identity, inverse = identity;
    forward[0] = {1, 0, 0, 10};
    forward[1] = {0, 0, -1, 20};
    forward[2] = {0, 1, 0, 30};
    inverse[0] = {1, 0, 0, -10};
    inverse[1] = {0, 0, 1, -30};
    inverse[2] = {0, -1, 0, 20};
    const std::vector<Point3> tilted{{10, 20, 30}, {14, 20, 34}, {10, 20, 34}, {14, 20, 30}};
    TubeBudget b;
    auto world = triangulate_native_projected_polygon(tilted, forward, inverse, 0, b);
    check(world.complete && world.points.back() == Point3{12, 20, 32},
          "crossing restored to tilted world plane");
    check(std::equal(tilted.begin(), tilted.end(), world.points.begin()),
          "source world coordinates round trip");
    check(world.report["output_coordinate_space"] == "world", "coordinate space explicit");
    {
        TubeBudget prep_budget, tri_budget;
        auto input = prepare_native_tube_mesh_cap_input(
            {{{10, 20, 3}, {14, 20, 3}, {14, 24, 3}, {10, 24, 3}},
             {{11, 21, 3}, {13, 21, 3}, {13, 23, 3}, {11, 23, 3}}},
            true, 0, prep_budget);
        auto cap =
            triangulate_native_projected_polygon(input.points, input.projection.local_to_world,
                                                 input.projection.world_to_local, 0, tri_budget);
        check(input.preparation_succeeded && cap.complete && area(cap) == 12,
              "native reversed multi-ring cap input joins projected triangle path");
    }
    auto snapshot = [](const NativeVuGraph &g) {
        Json nodes = Json::array();
        for (const auto &n : g.nodes)
            nodes.push_back({n.all_next, n.face_next, n.vertex_next, n.mask, n.source_index,
                             n.point, n.active, n.internal_data});
        return Json{{"nodes", nodes}, {"tail", g.tail}, {"free", g.free_head}};
    };
    {
        TubeBudget q;
        auto g = build_native_vu_polygon_input(duplicate, 0, q).graph;
        const auto all = native_vu_all_nodes(g, q);
        const auto first = all.front(), next = g.nodes[first].vertex_next;
        g.nodes[first].source_index = 1;
        g.nodes[next].source_index = 2;
        const auto original = snapshot(g);
        TubeBudget full;
        auto out = collect_native_vu_mesh_indices(g, duplicate, full);
        check(g.nodes[first].source_index == 1 && g.nodes[next].source_index == 1,
              "first numbered seed propagates label without geometrical lookup");
        check(out.points == duplicate && out.faces.report["vertex_label_writes"] > 0,
              "label propagation does not compact coordinate pool");
        auto invalid = g;
        invalid.nodes[first].source_index = INT32_MAX;
        const auto before = snapshot(invalid);
        rejects([&] {
            TubeBudget limited;
            collect_native_vu_mesh_indices(invalid, duplicate, limited);
        });
        check(snapshot(invalid) == before, "invalid label fails transactionally");
        check(original != snapshot(g), "general path exposes original label writes");
    }
    {
        auto g = result.input_graph.graph;
        const auto seed =
            result.report["index_output"]["appended_vertex_nodes"][0].get<std::size_t>();
        auto node = seed;
        do {
            g.nodes[node].mask &= ~(native_vu_numbered_mask | native_vu_boundary_mask);
            node = g.nodes[node].vertex_next;
        } while (node != seed);
        const auto before = snapshot(g);
        auto trial = g;
        TubeBudget full;
        auto out = collect_native_vu_mesh_indices(trial, cross, full);
        check(out.points.size() == 5, "unmarked crossing gets one shared vertex index");
        check(std::find(out.faces.indices.begin(), out.faces.indices.end(), 5) !=
                      out.faces.indices.end() &&
                  std::find(out.faces.indices.begin(), out.faces.indices.end(), -5) !=
                      out.faces.indices.end(),
              "only first new sector is promoted before index sign selection");
        rejects([&] {
            TubeBudget limited;
            limited.max_work = full.work - 1;
            collect_native_vu_mesh_indices(g, cross, limited);
        });
        check(snapshot(g) == before, "late budget failure leaves graph unchanged");
    }
    check(result.input_graph.report["merge_preparation"]["tolerance"].get<double>() >= 1e-14,
          "general polygon uses native absolute merge tolerance");
    {
        const auto tiny = run({{0, 0, 0}, {1e-15, 0, 0}, {0, 1e-15, 0}});
        check(tiny.input_graph.report["merge_preparation"]["tolerance"] == 1e-14,
              "absolute tolerance remains when larger than relative scale");
        check(tiny.points.size() == 3, "merged-away tiny graph does not discard source pool");
        TubeBudget b;
        auto short_graph = build_native_vu_polygon_input({{0, 0, 0}}, 0, b).graph;
        auto short_output = collect_native_vu_mesh_indices(short_graph, {{0, 0, 0}}, b);
        check(short_output.faces.succeeded && short_output.faces.indices.empty(),
              "short faces skipped without emitting separators");
        for (const auto &node : short_graph.nodes)
            check((node.mask & 2) != 0, "general route marks skipped short faces exterior");
    }
    rejects([&] {
        auto bad = cross;
        bad[0][0] = std::numeric_limits<double>::infinity();
        run(bad);
    });
    rejects([&] {
        TubeBudget small;
        small.max_control_points = 3;
        triangulate_native_projected_polygon(cross, identity, identity, 0, small);
    });
    rejects([&] {
        TubeBudget small;
        small.max_work = 1;
        triangulate_native_projected_polygon(cross, identity, identity, 0, small);
    });
    auto future = std::async(std::launch::async, [&] { return run(holes); });
    auto concurrent = run(holes), other = future.get();
    check(concurrent.points == other.points && concurrent.indices == other.indices &&
              concurrent.report == other.report,
          "independent calls deterministic");
    return checks;
}
