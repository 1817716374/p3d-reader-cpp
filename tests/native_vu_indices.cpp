#include "native_vu_indices.hpp"
#include "native_vu_flip.hpp"
#include "native_vu_triangulate.hpp"
#include "native_vu_cluster.hpp"
#include "native_vu_regularize.hpp"
#include "native_vu_exterior.hpp"
#include "native_vu_split_support.hpp"
#include "native_tube_mesh_index_rules.hpp"
#include <future>
#include <set>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_vu_indices_tests() {
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
        check(caught, "source index collection rejects invalid data or exhausted budget");
    };
    auto snapshot = [](const NativeVuGraph &g) {
        Json a = Json::array();
        for (const auto &n : g.nodes)
            a.push_back({n.all_next, n.face_next, n.vertex_next, n.mask, n.source_index, n.point,
                         n.active, n.internal_data});
        return Json{{"nodes", a}, {"tail", g.tail}, {"free", g.free_head}};
    };
    auto raw = [](const std::vector<Point3> &p) {
        TubeBudget b;
        return build_native_vu_input(p, 0, b).graph;
    };
    auto prepare = [&](const std::vector<Point3> &p) {
        TubeBudget b;
        auto in = build_native_vu_input(p, 0, b);
        prepare_native_vu_merge(in, b);
        const auto r = regularize_native_vu_graph(in.graph, b);
        mark_native_vu_exterior(in.graph, r.at("candidate_array_read_index").get<std::size_t>(), b);
        triangulate_native_vu_interiors(in.graph, b);
        flip_native_vu_triangles(in.graph, b);
        return in.graph;
    };
    auto invariants = [&](const NativeVuGraph &a, const NativeVuGraph &g) {
        check(a.nodes.size() == g.nodes.size() && a.tail == g.tail && a.free_head == g.free_head,
              "source lookup does not allocate graph nodes or change allocation links");
        for (std::size_t i = 0; i < a.nodes.size(); ++i) {
            const auto &x = a.nodes[i], &y = g.nodes[i];
            check(x.all_next == y.all_next && x.face_next == y.face_next &&
                      x.vertex_next == y.vertex_next && x.point == y.point &&
                      x.source_index == y.source_index && x.internal_data == y.internal_data &&
                      x.active == y.active && (x.mask & ~0x20000000u) == (y.mask & ~0x20000000u),
                  "source output changes only the face collection scratch mask");
        }
    };
    const std::vector<Point3> quad{{0, 0, 11}, {4, 0, 22}, {4, 1, 33}, {0, 3, 44}};
    auto fixture = [&] {
        TubeBudget b;
        auto g = raw(quad);
        mark_native_vu_exterior(g, 0, b);
        join_native_vu_sectors(g, 2, 6, b);
        flip_native_vu_triangles(g, b);
        return g;
    };
    {
        auto g = fixture();
        const auto before = g;
        TubeBudget b;
        const auto r = collect_native_vu_source_indices(g, b);
        check(r.succeeded && r.indices.size() == 8 && r.report.at("new_vertices_created") == 0,
              "two triangles resolve existing sectors without appending coordinates");
        invariants(before, g);
        std::set<std::set<int>> triangles;
        std::multiset<std::pair<int, int>> hidden;
        for (std::size_t i = 0; i < 8; i += 4) {
            std::set<int> corners;
            for (std::size_t k = 0; k < 3; ++k) {
                const int a = std::abs(r.indices[i + k]), z = std::abs(r.indices[i + (k + 1) % 3]);
                check(a >= 1 && a <= 4, "source indices address the original corner array");
                corners.insert(a);
                if (r.indices[i + k] < 0)
                    hidden.emplace(std::min(a, z), std::max(a, z));
            }
            check(r.indices[i + 3] == 0, "native face is zero terminated");
            triangles.insert(corners);
        }
        check(triangles == std::set<std::set<int>>{{1, 2, 3}, {1, 3, 4}} &&
                  hidden == std::multiset<std::pair<int, int>>{{1, 3}, {1, 3}},
              "flipped diagonal resolves new endpoint identities and both hidden edge signs");
        for (auto &n : g.nodes)
            if (n.mask & native_vu_numbered_mask)
                n.mask &= ~native_vu_boundary_mask;
        const auto numbered_signs = collect_native_vu_source_indices(g, b);
        check(numbered_signs.indices == r.indices,
              "numbered bit alone makes an original corner edge positive");
    }
    {
        TubeBudget b;
        auto g = raw({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}});
        mark_native_vu_exterior(g, 0, b);
        auto seed = native_vu_null;
        for (auto n : native_vu_all_nodes(g, b))
            if (!(g.nodes[n].mask & 2)) {
                seed = n;
                break;
            }
        const auto other = g.nodes[seed].vertex_next;
        check(other != seed && (g.nodes[seed].mask & native_vu_numbered_mask) &&
                  (g.nodes[other].mask & native_vu_numbered_mask),
              "test vertex has two independent numbered sectors");
        g.nodes[seed].source_index = 23;
        g.nodes[other].source_index = 37;
        const auto before = g;
        const auto r = collect_native_vu_source_indices(g, b);
        check(r.succeeded && r.indices.front() == 24,
              "native chooses first numbered sector from current corner");
        invariants(before, g);
    }
    auto area = [&](const std::vector<Point3> &points, const std::vector<std::int32_t> &indices) {
        check(indices.size() % 4 == 0, "published index buffer contains complete triangles");
        double sum = 0;
        for (std::size_t i = 0; i < indices.size(); i += 4) {
            check(indices[i + 3] == 0, "triangle output delimiter");
            std::array<Point3, 3> p;
            for (std::size_t k = 0; k < 3; ++k) {
                auto n = std::abs(indices[i + k]);
                check(n > 0 && std::size_t(n) <= points.size(),
                      "triangle index preserves original array extent");
                p[k] = points[n - 1];
            }
            sum += ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) -
                    (p[1][1] - p[0][1]) * (p[2][0] - p[0][0])) *
                   .5;
        }
        return sum;
    };
    const std::vector<Point3> hex{{0, 0, 1}, {4, 0, 2}, {6, 3, 3},
                                  {4, 6, 4}, {0, 6, 5}, {-2, 3, 6}};
    {
        auto g = prepare(hex);
        TubeBudget b;
        const auto r = collect_native_vu_source_indices(g, b);
        check(r.succeeded && r.indices.size() == 16 && area(hex, r.indices) == 36,
              "full native graph pipeline preserves convex source polygon area");
        auto plan = native_facet_index_plan(hex, b);
        check(plan.completed && plan.route == NativeFacetIndexPlan::Route::projected_loops &&
                  plan.input_graph->report.at("triangulated") == true &&
                  area(hex, plan.indices) == 36,
              "large-face entry publishes fully resolved native source triangles");
        auto spaced = hex;
        spaced.insert(spaced.begin() + 1, spaced[0]);
        auto repeated = native_facet_index_plan(spaced, b);
        check(repeated.completed && area(spaced, repeated.indices) == 36,
              "duplicate suppression retains source index gaps");
        for (auto index : repeated.indices)
            check(std::abs(index) != 2, "suppressed duplicate source corner is not renumbered");
    }
    {
        const Point3 marker{std::numeric_limits<double>::max(), 0, 0};
        const std::vector<Point3> hole{{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}, marker,
                                       {2, 2, 0}, {8, 2, 0},  {8, 8, 0},   {2, 8, 0}};
        auto g = prepare(hole);
        TubeBudget b;
        const auto r = collect_native_vu_source_indices(g, b);
        check(r.succeeded && r.indices.size() == 32 && area(hole, r.indices) == 64,
              "hole triangles retain original indices across disconnect marker");
        for (auto index : r.indices)
            check(std::abs(index) != 5, "disconnect marker cannot become a corner");
    }
    {
        auto g = fixture();
        TubeBudget b;
        auto all = native_vu_all_nodes(g, b);
        auto seed = native_vu_null;
        for (auto n : all)
            if (!(g.nodes[n].mask & 2)) {
                seed = n;
                break;
            }
        // Remove numbering from the second output vertex, leaving the first
        // resolvable. No coordinate search may repair the missing association.
        auto n = g.nodes[seed].face_next;
        const auto lost = n;
        do {
            g.nodes[n].mask &= ~native_vu_numbered_mask;
            n = g.nodes[n].vertex_next;
        } while (n != lost);
        const auto r = collect_native_vu_source_indices(g, b);
        check(!r.succeeded && r.indices.size() == 2 && r.indices[0] != 0 && r.indices[1] == 0 &&
                  r.report.at("unresolved_vertex_node") == lost &&
                  r.report.at("completed_face_count") == 0,
              "native lookup failure retains only its terminated partial first face and stops");
        auto crossing = prepare({{0, 0, 0}, {4, 4, 0}, {0, 4, 0}, {4, 0, 0}});
        const auto c = collect_native_vu_source_indices(crossing, b);
        check(!c.succeeded && c.report.at("unresolved_vertex_node") != nullptr,
              "new crossing point cannot be assigned invented source attributes");
        const std::vector<Point3> crossed{{0, 0, 0}, {4, 4, 0}, {0, 4, 0}, {4, 0, 0}, {0, 0, 0}};
        const auto plan = native_facet_index_plan(crossed, b);
        check(!plan.completed && plan.indices.empty() && plan.input_graph &&
                  plan.input_graph->report.at("source_index_output").at("native_succeeded") ==
                      false,
              "large-face caller reports failed numbering without publishing partial indices");
    }
    {
        TubeBudget b;
        auto g = raw(quad);
        mark_native_vu_exterior(g, 0, b);
        const auto r = collect_native_vu_source_indices(g, b);
        check(r.succeeded && r.indices.size() == 5 &&
                  r.report.at("all_emitted_faces_triangular") == false,
              "native source lookup can emit residual polygon without claiming triangulation");
        for (const auto &p :
             std::vector<std::vector<Point3>>{{{1, 2, 3}}, {{1, 2, 3}, {4, 5, 6}}}) {
            auto s = raw(p);
            const auto out = collect_native_vu_source_indices(s, b);
            check(out.succeeded && out.indices.empty() && out.report.at("skipped_short_faces") == 2,
                  "source output omits short faces instead of fabricating triangles");
        }
        NativeVuGraph empty;
        const auto out = collect_native_vu_source_indices(empty, b);
        check(out.succeeded && out.indices.empty(), "empty graph source output succeeds");
    }
    const auto baseline = fixture();
    auto good = baseline;
    TubeBudget measured;
    collect_native_vu_source_indices(good, measured);
    for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1}) {
        auto g = baseline;
        const auto before = snapshot(g);
        TubeBudget b;
        b.max_work = limit;
        rejects([&] { collect_native_vu_source_indices(g, b); });
        check(snapshot(g) == before, "resource failure rolls back source output masks");
    }
    {
        auto g = baseline;
        g.nodes[0].face_next = native_vu_null;
        TubeBudget b;
        rejects([&] { collect_native_vu_source_indices(g, b); });
        auto bad = baseline;
        for (auto &n : bad.nodes)
            if (n.mask & native_vu_numbered_mask)
                n.source_index = INT32_MAX;
        const auto before = snapshot(bad);
        rejects([&] { collect_native_vu_source_indices(bad, b); });
        check(snapshot(bad) == before,
              "source one-based integer overflow cannot publish wrapped indices");
    }
    auto task = [baseline] {
        auto g = baseline;
        TubeBudget b;
        auto r = collect_native_vu_source_indices(g, b);
        return Json{{"report", r.report}, {"indices", r.indices}};
    };
    const auto expected = task();
    std::vector<std::future<Json>> futures;
    for (unsigned i = 0; i < 4; ++i)
        futures.push_back(std::async(std::launch::async, task));
    for (auto &f : futures)
        check(f.get() == expected, "source lookup has no cross-call mutable state");
    return checks;
}
