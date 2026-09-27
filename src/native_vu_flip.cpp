#include "native_vu_flip.hpp"
// Adapted from Bentley imodel-native vutriang.cpp and vusubset.cpp.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
#include "native_vu_split_support.hpp"
namespace p3d::swept_detail {
namespace {
using Node = std::size_t;
using curve_detail::bezier_support::finite;
Node next(const NativeVuGraph &g, Node n) {
    return g.nodes[n].face_next;
}
Node mate(const NativeVuGraph &g, Node n) {
    return g.nodes[next(g, n)].vertex_next;
}
bool ratio(const Point3 &a, const Point3 &b, const Point3 &c, double &out) {
    const double ux = finite(b[0] - a[0]), uy = finite(b[1] - a[1]);
    const double vx = finite(c[0] - b[0]), vy = finite(c[1] - b[1]);
    const double wx = finite(a[0] - c[0]), wy = finite(a[1] - c[1]);
    // Native denominator evaluates V, then U, then W, with Y^2 before X^2.
    const double v = finite(finite(vy * vy) + finite(vx * vx));
    const double u = finite(finite(uy * uy) + finite(ux * ux));
    const double w = finite(finite(wy * wy) + finite(wx * wx));
    const double perimeter = finite(finite(v + u) + w);
    out = 0;
    if (perimeter <= 0)
        return false;
    out = finite(finite(finite(vy * ux) - finite(vx * uy)) / perimeter);
    return true;
}
bool flip_test(const NativeVuGraph &g, Node a, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(8);
    const auto b = next(g, a), c = next(g, b), d = g.nodes[b].vertex_next, e = next(g, d),
               f = next(g, e);
    if (next(g, c) != a || next(g, f) != d)
        return false;
    const auto &pa = g.nodes[a].point, &pb = g.nodes[b].point, &pc = g.nodes[c].point,
               &pf = g.nodes[f].point;
    double q[4]{};
    work.charge(96);
    if (!ratio(pa, pb, pc, q[0]) || !ratio(pb, pa, pf, q[1]) || !ratio(pc, pa, pf, q[2]) ||
        !ratio(pc, pf, pb, q[3]))
        return false;
    for (double &x : q)
        if (std::abs(x) < 1e-6)
            x = 0;
    const double current = std::min(q[0], q[1]), flipped = std::min(q[2], q[3]);
    return flipped > (current < 1e-6 ? finite(current * 1.25) : current);
}
} // namespace
bool native_vu_quadratic_flip_test(const NativeVuGraph &g, Node a, TubeBudget &budget) {
    validate_native_vu_split_graph(g, budget);
    require(a < g.nodes.size() && g.nodes[a].active, "native VU flip seed");
    return flip_test(g, a, budget);
}
Json flip_native_vu_triangles(NativeVuGraph &graph, TubeBudget &budget) {
    const auto all = validate_native_vu_split_graph(graph, budget);
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(graph.nodes.size());
    auto g = graph;
    constexpr std::uint32_t fixed = 0xcef, queued = 0x20000000;
    std::vector<Node> candidates;
    for (auto n : all) {
        work.charge(1);
        g.nodes[n].mask &= ~queued;
    }
    std::size_t peak = 0, enqueued = 0;
    auto add = [&](Node n) {
        work.charge(4);
        if (g.nodes[n].mask & (fixed | queued))
            return;
        const auto m = mate(g, n);
        if (g.nodes[m].mask & (fixed | queued))
            return;
        g.nodes[n].mask |= queued;
        g.nodes[m].mask |= queued;
        require(candidates.size() < budget.max_control_points, "native VU flip candidate budget");
        candidates.push_back(n);
        ++enqueued;
        peak = std::max(peak, candidates.size());
    };
    for (auto n : all)
        add(n);
    const auto initial = candidates.size();
    const double cap =
        finite(double(all.size()) * std::min(60.0, std::max(double(all.size()) / 60.0, 20.0)));
    double iterations = 0;
    std::size_t tested = 0, flipped = 0;
    while (iterations < cap) {
        work.charge(1);
        iterations += 1;
        if (candidates.empty())
            break;
        const auto a = candidates.back();
        candidates.pop_back();
        g.nodes[a].mask &= ~queued;
        g.nodes[mate(g, a)].mask &= ~queued;
        ++tested;
        if (!flip_test(g, a, budget))
            continue;
        const auto b = next(g, a), d = g.nodes[b].vertex_next, e = next(g, d), c = next(g, b),
                   f = next(g, e);
        twist_native_vu_vertices(g, a, e, budget);
        twist_native_vu_vertices(g, b, d, budget);
        // Enqueue the exposed quadrilateral before reinserting the diagonal.
        auto n = e;
        do {
            add(n);
            n = next(g, n);
        } while (n != e);
        twist_native_vu_vertices(g, a, c, budget);
        twist_native_vu_vertices(g, d, f, budget);
        g.nodes[a].point = g.nodes[c].point;
        g.nodes[d].point = g.nodes[f].point;
        // Fresh graph disables both conditional user-data copy flags.
        ++flipped;
    }
    validate_native_vu_split_graph(g, budget);
    Json report{{"edge_adjustment_completed", candidates.empty()},
                {"iteration_limit_reached", !candidates.empty()},
                {"native_iteration_limit", cap},
                {"native_iterations", iterations},
                {"initial_candidate_count", initial},
                {"candidate_insertions", enqueued},
                {"tested_edges", tested},
                {"flipped_edges", flipped},
                {"remaining_candidates", candidates.size()},
                {"candidate_stack_peak", peak},
                {"final_indices_ready", false}};
    graph = std::move(g);
    return report;
}
} // namespace p3d::swept_detail
