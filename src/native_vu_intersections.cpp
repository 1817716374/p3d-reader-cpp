#include "native_vu_intersections.hpp"
// Intersection sweep and scaled 2x2 solve adapted from Bentley imodel-native
// vumerge2.cpp and bsibasegeom_for_vu_static.c, with P3D arithmetic retained.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_attribute_sort.hpp"
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
#include "native_vu_split_support.hpp"
#include <numeric>
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
using Work = curve_detail::BezierWork;
struct Range {
    double x0, x1, y0, y1;
};
Range edge_range(const NativeVuGraph &g, std::size_t edge) {
    const auto &a = g.nodes[edge].point;
    // Range uses the edge mate; the actual intersection below uses face-next.
    const auto &b = g.nodes[g.nodes[g.nodes[edge].face_next].vertex_next].point;
    return {std::min(a[0], b[0]), std::max(a[0], b[0]), std::min(a[1], b[1]), std::max(a[1], b[1])};
}
struct Crossing {
    Point3 point;
    std::size_t edge;
    double fraction;
};
bool interior(double f) {
    return f > 1e-13 && f < 0.9999999999999;
}
} // namespace
NativeVuLinearSolution solve_native_vu_2x2(double a, double b, double c, double d, double x,
                                           double y, TubeBudget &budget) {
    const Work work{budget.work, budget.max_work};
    work.charge(48);
    for (auto v : {a, b, c, d, x, y})
        finite(v);
    const auto c0 = std::max(std::abs(c), std::abs(a));
    const auto c1 = std::max(std::abs(b), std::abs(d));
    if (c0 == 0 || c1 == 0)
        return {};
    const auto r0 = finite(1 / c0), r1 = finite(1 / c1);
    a = finite(a * r0);
    c = finite(c * r0);
    b = finite(b * r1);
    d = finite(d * r1);
    const auto ad_sum = finite(a + d), bc_diff = finite(b - c);
    const auto ad_diff = finite(a - d), bc_sum = finite(b + c);
    const auto h0 =
        finite(.5 * std::sqrt(finite(finite(bc_diff * bc_diff) + finite(ad_sum * ad_sum))));
    const auto h1 =
        finite(.5 * std::sqrt(finite(finite(bc_sum * bc_sum) + finite(ad_diff * ad_diff))));
    const auto sum = finite(h1 + h0), difference = finite(h0 - h1);
    if (!(finite(sum * 1e-12) < std::abs(difference)))
        return {};
    // Original denominator is the product of singular-value sum/difference,
    // rather than recomputing a*d-b*c after the condition test.
    const auto determinant = finite(difference * sum);
    const auto u = finite(finite(finite(d * x) - finite(b * y)) / determinant);
    const auto v = finite(finite(finite(a * y) - finite(c * x)) / determinant);
    return {true, {finite(u * r0), finite(r1 * v)}};
}
Json split_native_vu_edges_at_intersections(NativeVuGraph &graph, double vertex, double along,
                                            TubeBudget &budget) {
    require(std::isfinite(vertex) && vertex >= 0 && std::isfinite(along) && along >= 0,
            "native VU intersection tolerance");
    const auto all = validate_native_vu_split_graph(graph, budget);
    auto edges = collect_native_vu_up_edges(graph, all, budget);
    sort_native_vu_nodes(graph, edges, budget);
    const Work work{budget.work, budget.max_work};
    std::vector<Crossing> crossings;
    std::size_t tested = 0, singular = 0, interior_pairs = 0;
    auto append = [&](std::size_t edge, double f, const Point3 &point) {
        work.charge(1);
        require(crossings.size() < budget.max_control_points && crossings.size() < INT32_MAX,
                "native VU intersection record budget");
        crossings.push_back({point, edge, f});
    };
    for (std::size_t i = 0; i < edges.size(); ++i) {
        work.charge(1);
        const auto range_a = edge_range(graph, edges[i]);
        for (auto j = i + 1; j < edges.size(); ++j) {
            work.charge(1);
            const auto range_b = edge_range(graph, edges[j]);
            if (range_b.y0 >= range_a.y1)
                break;
            if (range_b.x1 <= range_a.x0 || range_b.x0 >= range_a.x1)
                continue;
            ++tested;
            const auto &a = graph.nodes[edges[i]].point;
            const auto &b = graph.nodes[graph.nodes[edges[i]].face_next].point;
            const auto &c = graph.nodes[edges[j]].point;
            const auto &d = graph.nodes[graph.nodes[edges[j]].face_next].point;
            Point3 da, db;
            for (std::size_t k = 0; k < 3; ++k) {
                da[k] = finite(b[k] - a[k]);
                db[k] = finite(d[k] - c[k]);
            }
            const auto solution = solve_native_vu_2x2(
                da[0], -db[0], da[1], -db[1], finite(c[0] - a[0]), finite(c[1] - a[1]), budget);
            if (!solution.solved) {
                ++singular;
                continue;
            }
            const auto fa = solution.value[0], fb = solution.value[1];
            if (!interior(fa) || !interior(fb))
                continue;
            Point3 pa, pb;
            for (std::size_t k = 0; k < 3; ++k) {
                pa[k] = finite(finite(da[k] * fa) + a[k]);
                pb[k] = finite(finite(db[k] * fb) + c[k]);
            }
            ++interior_pairs;
            append(i, fa, pa);
            append(j, fb, pb);
        }
    }
    work.charge(crossings.size());
    std::vector<std::size_t> order(crossings.size());
    std::iota(order.begin(), order.end(), 0);
    auto less = [&](std::size_t a, std::size_t b) {
        work.charge(1);
        return crossings[a].edge < crossings[b].edge ||
               (crossings[a].edge == crossings[b].edge &&
                crossings[a].fraction < crossings[b].fraction);
    };
    NativeAttributeSort<decltype(less)> sorter(order, less);
    sorter.sort(0, order.size(), order.size());
    work.charge(graph.nodes.size());
    auto g = graph;
    std::size_t splits = 0, split_edges = 0;
    for (std::size_t i = 0; i < order.size();) {
        const auto edge = crossings[order[i]].edge;
        std::vector<double> fractions;
        do {
            work.charge(1);
            fractions.push_back(crossings[order[i++]].fraction);
        } while (i < order.size() && crossings[order[i]].edge == edge);
        const auto count = split_native_vu_at_fractions(g, edges[edge], std::move(fractions),
                                                        vertex, along, budget);
        splits += count;
        if (count)
            ++split_edges;
    }
    Json report = {{"scope", "native_vu_transverse_intersection_splits"},
                   {"upward_edge_count", edges.size()},
                   {"tested_pairs", tested},
                   {"singular_pairs", singular},
                   {"interior_pairs", interior_pairs},
                   {"record_count", crossings.size()},
                   {"split_count", splits},
                   {"split_edge_count", split_edges},
                   {"added_nodes", g.nodes.size() - graph.nodes.size()},
                   {"vertex_connection_completed", false},
                   {"work_used", budget.work}};
    graph = std::move(g);
    return report;
}
} // namespace p3d::swept_detail
