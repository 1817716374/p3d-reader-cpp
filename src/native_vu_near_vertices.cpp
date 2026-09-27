#include "native_vu_near_vertices.hpp"
// Projection and fraction collapse adapted from Bentley imodel-native
// vumerge2.cpp, preserving native P3D arithmetic and graph ordering.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
#include "native_vu_qsort.hpp"
#include "native_vu_split_support.hpp"
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
using Work = curve_detail::BezierWork;
void tolerance(double value) {
    require(std::isfinite(value) && value >= 0, "native VU split tolerance");
}
double distance_squared(const Point3 &a, const Point3 &b) {
    const auto dx = finite(a[0] - b[0]), dy = finite(a[1] - b[1]);
    return finite(finite(dy * dy) + finite(dx * dx));
}
} // namespace
std::vector<std::size_t> validate_native_vu_split_graph(const NativeVuGraph &g,
                                                        TubeBudget &budget) {
    require(g.nodes.size() <= budget.max_control_points && g.nodes.size() <= INT32_MAX,
            "native VU split graph size");
    const Work work{budget.work, budget.max_work};
    std::vector<std::size_t> all;
    if (g.nodes.empty()) {
        require(g.tail == native_vu_null, "native VU empty tail");
        return all;
    }
    require(g.tail < g.nodes.size(), "native VU split tail");
    work.charge(g.nodes.size() * 4);
    std::vector<bool> seen(g.nodes.size()), faces(g.nodes.size()), vertices(g.nodes.size());
    auto p = g.nodes[g.tail].all_next;
    for (;;) {
        work.charge(1);
        require(p < g.nodes.size() && !seen[p], "native VU split all cycle");
        seen[p] = true;
        const auto &n = g.nodes[p];
        require(n.face_next < g.nodes.size() && n.vertex_next < g.nodes.size(),
                "native VU split successor");
        require(!faces[n.face_next] && !vertices[n.vertex_next], "native VU split permutations");
        faces[n.face_next] = vertices[n.vertex_next] = true;
        for (double x : n.point)
            finite(x);
        all.push_back(p);
        if (p == g.tail)
            break;
        p = n.all_next;
    }
    require(all.size() == g.nodes.size(), "native VU split disconnected list");
    for (auto n : all) {
        work.charge(1);
        auto mate = g.nodes[g.nodes[n].face_next].vertex_next;
        require(g.nodes[g.nodes[mate].face_next].vertex_next == n, "native VU split edge pairing");
    }
    return all;
}
std::vector<std::size_t> collect_native_vu_up_edges(const NativeVuGraph &g,
                                                    const std::vector<std::size_t> &all,
                                                    TubeBudget &budget) {
    const Work work{budget.work, budget.max_work};
    std::vector<std::size_t> edges;
    for (auto n : all) {
        work.charge(1);
        const auto &a = g.nodes[n].point, &b = g.nodes[g.nodes[n].face_next].point;
        if (a[1] < b[1] || (a[1] == b[1] && a[0] < b[0]))
            edges.push_back(n);
    }
    return edges;
}
void sort_native_vu_nodes(const NativeVuGraph &g, std::vector<std::size_t> &nodes,
                          TubeBudget &budget) {
    const Work work{budget.work, budget.max_work};
    native_vu_qsort(nodes, [&](std::size_t a, std::size_t b) {
        work.charge(1);
        for (auto axis : {1, 0}) {
            if (g.nodes[a].point[axis] < g.nodes[b].point[axis])
                return -1;
            if (g.nodes[a].point[axis] > g.nodes[b].point[axis])
                return 1;
        }
        return 0;
    });
}
std::vector<double> collapse_native_vu_fractions(std::vector<double> values, double tol,
                                                 double minimum, double maximum,
                                                 TubeBudget &budget) {
    tolerance(tol);
    finite(minimum);
    finite(maximum);
    require(values.size() <= budget.max_control_points && values.size() <= INT32_MAX,
            "native VU projection fraction count");
    const Work work{budget.work, budget.max_work};
    work.charge(values.size());
    for (double x : values)
        finite(x);
    // Only scalar values survive this sort; equal keys carry no source identity.
    std::sort(values.begin(), values.end(), [&](double a, double b) {
        work.charge(1);
        return a < b;
    });
    std::vector<double> result;
    auto add = [&](double f) {
        work.charge(1);
        require(result.size() < budget.max_control_points, "native VU collapsed fraction budget");
        result.push_back(finite(f));
    };
    for (std::size_t i = 0; i < values.size();) {
        work.charge(1);
        const auto first = values[i++];
        if (first < minimum || first > maximum)
            continue;
        auto last = first;
        auto limit = finite(first + tol);
        // The range check applies only to the first member, not the chain tail.
        while (i < values.size() && values[i] <= limit) {
            work.charge(1);
            last = values[i++];
            limit = finite(last + tol);
        }
        const auto span = finite(last - first);
        if (span <= tol)
            add(finite(0.5 * finite(first + last)));
        else {
            const auto ratio = finite(span / tol);
            require(ratio >= 1 && ratio < double(INT32_MAX),
                    "native VU fraction subdivision extent");
            const auto count = static_cast<std::int32_t>(ratio);
            const auto step = finite(span / double(count));
            for (std::int32_t j = 0; j <= count; ++j)
                add(finite(first + finite(double(j) * step)));
        }
    }
    return result;
}
std::size_t split_native_vu_at_fractions(NativeVuGraph &g, std::size_t edge,
                                         std::vector<double> fractions, double vertex, double along,
                                         TubeBudget &budget) {
    tolerance(vertex);
    tolerance(along);
    require(edge < g.nodes.size() && g.nodes[edge].face_next < g.nodes.size(),
            "native VU fraction split edge");
    const Work work{budget.work, budget.max_work};
    const auto a = g.nodes[edge].point, b = g.nodes[g.nodes[edge].face_next].point;
    const auto length = std::sqrt(distance_squared(a, b));
    if (fractions.empty() || !(length > along))
        return 0;
    const auto vertex_fraction = finite(vertex / length);
    fractions = collapse_native_vu_fractions(std::move(fractions), finite(along / length),
                                             vertex_fraction, finite(1 - vertex_fraction), budget);
    auto base = edge;
    for (auto f : fractions) {
        work.charge(10);
        Point3 point;
        for (std::size_t k = 0; k < 3; ++k) {
            const auto delta = finite(b[k] - a[k]);
            point[k] = f <= .5 ? finite(a[k] + finite(delta * f))
                               : finite(b[k] + finite(delta * finite(f - 1)));
        }
        const auto pair = split_native_vu_edge(g, base, budget);
        g.nodes[pair.first].point = g.nodes[pair.second].point = point;
        base = pair.first;
    }
    return fractions.size();
}
Json split_native_vu_edges_near_vertices(NativeVuGraph &graph, double perp, double vertex,
                                         double along, std::uint32_t visit_mask,
                                         TubeBudget &budget) {
    tolerance(perp);
    tolerance(vertex);
    tolerance(along);
    require(visit_mask && !(visit_mask & (visit_mask - 1)) &&
                !(visit_mask & (native_vu_numbered_mask | 3u)),
            "native VU split scratch mask");
    const auto all = validate_native_vu_split_graph(graph, budget);
    const Work work{budget.work, budget.max_work};
    work.charge(graph.nodes.size());
    auto g = graph; // One transaction for the entire pass, not one copy per edge.
    std::vector<std::size_t> vertices;
    for (auto n : all)
        g.nodes[n].mask &= ~visit_mask;
    for (auto n : all) {
        work.charge(1);
        if (!(g.nodes[n].mask & (visit_mask | 2u))) {
            vertices.push_back(n);
            auto p = n;
            do {
                work.charge(1);
                g.nodes[p].mask |= visit_mask;
                p = g.nodes[p].vertex_next;
            } while (p != n);
        }
    }
    auto edges = collect_native_vu_up_edges(g, all, budget);
    sort_native_vu_nodes(g, vertices, budget);
    sort_native_vu_nodes(g, edges, budget);
    const auto perp2 = finite(perp * perp), vertex2 = finite(vertex * vertex);
    // Native range expansion omits perpendicular tolerance from the maximum.
    const auto max_tol = std::max(along, vertex);
    std::size_t lowest = 0, projections = 0, splits = 0, split_edges = 0;
    for (auto edge : edges) {
        work.charge(1);
        const auto a = g.nodes[edge].point, b = g.nodes[g.nodes[edge].face_next].point;
        const auto dx = finite(b[0] - a[0]), dy = finite(b[1] - a[1]);
        // Retain the original double square root in the coarse rejection only.
        const auto coarse_length = std::sqrt(std::sqrt(finite(finite(dx * dx) + finite(dy * dy))));
        if (coarse_length <= vertex || coarse_length < finite(along + along))
            continue;
        Point3 low, high;
        bool null_range = false;
        for (std::size_t k = 0; k < 3; ++k) {
            low[k] = std::min(a[k], b[k]);
            high[k] = std::max(a[k], b[k]);
            null_range |= std::abs(low[k]) >= 1e100 || std::abs(high[k]) >= 1e100;
        }
        if (!null_range)
            for (std::size_t k = 0; k < 3; ++k) {
                low[k] = finite(low[k] - max_tol);
                high[k] = finite(high[k] + max_tol);
            }
        while (lowest < vertices.size() && g.nodes[vertices[lowest]].point[1] < low[1]) {
            work.charge(1);
            ++lowest;
        }
        const auto uu = finite(finite(dy * dy) + finite(dx * dx));
        std::vector<double> fractions;
        for (auto i = lowest; i < vertices.size(); ++i) {
            work.charge(1);
            const auto &v = g.nodes[vertices[i]].point;
            if (v[1] > high[1])
                break;
            if (v[0] < low[0] || v[0] > high[0])
                continue;
            work.charge(24);
            const auto uv =
                finite(finite(finite(v[1] - a[1]) * dy) + finite(finite(v[0] - a[0]) * dx));
            const auto f = std::abs(uu) > finite(std::abs(uv) * 1e-15) ? finite(uv / uu) : 0;
            if (!(f > 0 && f < 1))
                continue;
            const Point3 projected{finite(a[0] + finite(dx * f)), finite(a[1] + finite(dy * f)), 0};
            if (distance_squared(projected, v) < perp2 && distance_squared(a, v) > vertex2 &&
                distance_squared(b, v) > vertex2)
                fractions.push_back(f);
        }
        projections += fractions.size();
        const auto count =
            split_native_vu_at_fractions(g, edge, std::move(fractions), vertex, along, budget);
        if (count)
            ++split_edges;
        splits += count;
    }
    Json report = {{"scope", "native_vu_near_vertex_edge_splits"},
                   {"vertex_count", vertices.size()},
                   {"upward_edge_count", edges.size()},
                   {"projection_count", projections},
                   {"split_edge_count", split_edges},
                   {"split_count", splits},
                   {"added_nodes", g.nodes.size() - graph.nodes.size()},
                   {"intersection_splitting_completed", false},
                   {"work_used", budget.work}};
    graph = std::move(g);
    return report;
}
} // namespace p3d::swept_detail
