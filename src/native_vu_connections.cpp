#include "native_vu_connections.hpp"
// Clustered edge collection and vertex connection adapted from Bentley
// imodel-native vucluster.cpp, retaining P3D merge types and node order.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_vu_cluster.hpp"
#include "native_vu_split_support.hpp"
#include "native_attribute_sort.hpp"
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
#include <numeric>
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
struct Edge {
    std::int32_t c0, c1;
    std::size_t n0, n1;
    double theta;
};
std::size_t mate(const NativeVuGraph &g, std::size_t n) {
    return g.nodes[g.nodes[n].face_next].vertex_next;
}
std::size_t pred(const NativeVuGraph &g, std::size_t n) {
    return g.nodes[mate(g, n)].face_next;
}
} // namespace
Json connect_native_vu_vertices(NativeVuGraph &graph, double tolerance, int merge_type,
                                std::uint32_t mask, TubeBudget &budget) {
    const auto initial = validate_native_vu_split_graph(graph, budget);
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(graph.nodes.size());
    auto g = graph;
    std::vector<std::size_t> members;
    auto clusters = consolidate_native_vu_coordinates(g, tolerance, mask, budget, &members);
    for (auto n : initial)
        g.nodes[n].mask &= ~mask;
    std::int32_t cluster = 0;
    for (auto n : members) {
        work.charge(1);
        if (n == native_vu_null) {
            ++cluster;
            continue;
        }
        auto current = n;
        do {
            work.charge(1);
            g.nodes[current].internal_data = cluster;
            current = g.nodes[current].vertex_next;
        } while (current != n);
    }
    std::vector<Edge> edges;
    for (auto n : initial) {
        work.charge(8);
        const auto next = g.nodes[n].face_next, m = mate(g, n);
        const auto dx = finite(g.nodes[next].point[0] - g.nodes[n].point[0]);
        const auto dy = finite(g.nodes[next].point[1] - g.nodes[n].point[1]);
        const auto theta = dx == 0 && dy == 0 ? 0 : finite(std::atan2(dy, dx));
        const auto c0 = g.nodes[n].internal_data, c1 = g.nodes[m].internal_data;
        if (c0 == c1) {
            g.nodes[n].mask |= mask;
            g.nodes[m].mask |= mask;
        } else
            edges.push_back({c0, c1, n, m, theta});
    }
    const auto slings = free_marked_native_vu_edges(g, mask, budget);
    const auto all = native_vu_all_nodes(g, budget);
    for (auto n : all)
        twist_native_vu_vertices(g, n, pred(g, n), budget);
    work.charge(edges.size());
    std::vector<std::size_t> sorted(edges.size());
    std::iota(sorted.begin(), sorted.end(), 0);
    auto by_clusters = [&](std::size_t a, std::size_t b) {
        work.charge(1);
        return edges[a].c0 < edges[b].c0 ||
               (edges[a].c0 == edges[b].c0 && edges[a].c1 < edges[b].c1);
    };
    NativeAttributeSort<decltype(by_clusters)> cluster_sort(sorted, by_clusters);
    cluster_sort.sort(0, sorted.size(), sorted.size());
    for (auto n : all)
        g.nodes[n].mask &= ~mask;
    std::size_t to_delete = 0, bundled = 0;
    auto marked = [&](std::size_t n) { return (g.nodes[n].mask & mask) != 0; };
    for (std::size_t i = 0; i < sorted.size();) {
        work.charge(1);
        const auto a = edges[sorted[i]];
        auto exposed = a.n0;
        if (marked(a.n0) || marked(a.n1)) {
            ++i;
            continue;
        }
        auto j = i + 1;
        for (; j < sorted.size(); ++j) {
            work.charge(1);
            const auto b = edges[sorted[j]];
            if (a.c0 != b.c0 || a.c1 != b.c1)
                break;
            const auto b1 = g.nodes[b.n0].face_next;
            if (marked(b.n0) || marked(b1))
                continue;
            if (g.nodes[b.n0].vertex_next != b.n0 || g.nodes[b1].vertex_next != b1)
                continue;
            if (merge_type == 0) {
                for (auto n : {a.n0, a.n1, b.n0, b1})
                    g.nodes[n].mask |= mask;
                to_delete += 2;
                break;
            } else if (merge_type == 2002) {
                g.nodes[b.n0].mask |= mask;
                g.nodes[b1].mask |= mask;
                ++to_delete;
            } else {
                twist_native_vu_vertices(g, exposed, b.n0, budget);
                twist_native_vu_vertices(g, b1, a.n1, budget);
                g.nodes[exposed].mask |= mask;
                g.nodes[b1].mask |= mask;
                exposed = b.n0;
                ++bundled;
            }
        }
        i = j;
    }
    // Sort the already cluster-pair-sorted array. Its equal-key permutation is
    // part of the native angular tie order; do not reset ordinals here.
    auto by_angle = [&](std::size_t a, std::size_t b) {
        work.charge(1);
        return edges[a].c0 < edges[b].c0 ||
               (edges[a].c0 == edges[b].c0 && edges[a].theta < edges[b].theta);
    };
    NativeAttributeSort<decltype(by_angle)> angle_sort(sorted, by_angle);
    angle_sort.sort(0, sorted.size(), sorted.size());
    for (std::size_t i = 0; i < sorted.size();) {
        work.charge(1);
        const auto a = edges[sorted[i]];
        auto exposed = marked(a.n0) ? native_vu_null : a.n0;
        auto j = i + 1;
        for (; j < sorted.size() && edges[sorted[j]].c0 == a.c0; ++j) {
            work.charge(1);
            const auto n = edges[sorted[j]].n0;
            if (marked(n))
                continue;
            if (exposed != native_vu_null)
                twist_native_vu_vertices(g, n, exposed, budget);
            exposed = n;
        }
        i = j;
    }
    const auto duplicates = to_delete ? free_marked_native_vu_edges(g, mask, budget) : 0;
    const auto active = validate_native_vu_split_graph(g, budget);
    Json report = {{"scope", "native_vu_clustered_vertex_connection"},
                   {"merge_type", merge_type},
                   {"coordinate_consolidation", std::move(clusters)},
                   {"edge_record_count", edges.size()},
                   {"sling_nodes_removed", slings},
                   {"duplicate_nodes_removed", duplicates},
                   {"null_face_bundles", bundled},
                   {"active_node_count", active.size()},
                   {"storage_slot_count", g.nodes.size()},
                   {"vertex_connection_completed", true},
                   {"triangulated", false},
                   {"work_used", budget.work}};
    graph = std::move(g);
    return report;
}
} // namespace p3d::swept_detail
