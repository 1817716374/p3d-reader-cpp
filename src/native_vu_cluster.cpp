#include "native_vu_cluster.hpp"
// Coordinate clustering adapted from Bentley imodel-native vucluster.cpp;
// native P3D distance, sort permutation, visitation and XYZ rules preserved.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_attribute_sort.hpp"
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
#include "native_vu_near_vertices.hpp"
#include "native_vu_intersections.hpp"
#include "native_vu_connections.hpp"
#include <numeric>
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
std::vector<std::size_t> all_nodes(const NativeVuGraph &g, TubeBudget &budget) {
    auto order = native_vu_all_nodes(g, budget);
    for (auto n : order)
        for (auto x : g.nodes[n].point)
            finite(x);
    return order;
}
struct Key {
    std::size_t node;
    Point2 xy;
    double coordinate;
    bool clustered = false;
};
} // namespace
double native_vu_merge_tolerance(const NativeVuGraph &g, double absolute_tolerance,
                                 double relative_tolerance, TubeBudget &budget) {
    require(std::isfinite(absolute_tolerance) && std::isfinite(relative_tolerance) &&
                absolute_tolerance >= 0 && relative_tolerance >= 0,
            "native VU merge tolerance inputs");
    const auto order = all_nodes(g, budget);
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    double largest = 0;
    for (auto node : order) {
        work.charge(2);
        largest = std::max(std::abs(g.nodes[node].point[0]), largest);
        largest = std::max(std::abs(g.nodes[node].point[1]), largest);
    }
    return std::max(absolute_tolerance, finite(largest * relative_tolerance));
}
Json consolidate_native_vu_coordinates(NativeVuGraph &g, double tolerance, std::uint32_t visit_mask,
                                       TubeBudget &budget,
                                       std::vector<std::size_t> *cluster_nodes) {
    require(std::isfinite(tolerance) && tolerance >= 0, "native VU cluster tolerance");
    require(visit_mask && !(visit_mask & (visit_mask - 1)) &&
                !(visit_mask & (native_vu_numbered_mask | native_vu_boundary_mask)),
            "native VU cluster scratch mask");
    const auto all = all_nodes(g, budget);
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(g.nodes.size());
    std::vector<bool> visited(g.nodes.size());
    std::vector<Point3> replacement;
    replacement.reserve(g.nodes.size());
    for (const auto &node : g.nodes)
        replacement.push_back(node.point);
    std::vector<Key> keys;
    const double x = 0.13363333323, y = 1.41423;
    const double reciprocal = 1 / std::sqrt((x * x + y * y) + 0.0);
    const double sx = x * reciprocal, sy = y * reciprocal;
    const double band = finite(std::sqrt(3.0) * tolerance);
    for (auto seed : all) {
        work.charge(1);
        if (visited[seed])
            continue;
        auto node = seed;
        do {
            work.charge(1);
            require(node < g.nodes.size() && g.nodes[node].active && !visited[node],
                    "native VU cluster vertex cycle");
            visited[node] = true;
            node = g.nodes[node].vertex_next;
        } while (node != seed);
        const auto &p = g.nodes[seed].point;
        keys.push_back({seed, {p[0], p[1]}, finite(finite(p[1] * sy) + finite(p[0] * sx)), false});
    }
    work.charge(keys.size());
    std::vector<std::size_t> sorted(keys.size());
    std::iota(sorted.begin(), sorted.end(), 0);
    auto less = [&](std::size_t a, std::size_t b) {
        work.charge(1);
        return keys[a].coordinate < keys[b].coordinate;
    };
    // Base 32ad0/33310/338b0 uses the same 32-entry insertion cutoff,
    // ninther/equal partition, depth decay and heap tie selection as this core.
    NativeAttributeSort<decltype(less)> sorter(sorted, less);
    sorter.sort(0, sorted.size(), sorted.size());
    std::size_t clusters = 0, absorbed = 0;
    std::vector<std::size_t> members;
    for (std::size_t i = 0; i < sorted.size(); ++i) {
        work.charge(1);
        auto &pivot = keys[sorted[i]];
        if (pivot.clustered)
            continue;
        ++clusters;
        pivot.clustered = true;
        if (cluster_nodes)
            members.push_back(pivot.node);
        const double limit = finite(band + pivot.coordinate);
        const auto xyz = g.nodes[pivot.node].point;
        for (std::size_t j = i + 1; j < sorted.size(); ++j) {
            work.charge(1);
            auto &candidate = keys[sorted[j]];
            if (candidate.coordinate > limit)
                break;
            if (candidate.clustered)
                continue;
            work.charge(6);
            const double dx = finite(candidate.xy[0] - pivot.xy[0]),
                         dy = finite(candidate.xy[1] - pivot.xy[1]);
            // Native keysInSameCluster uses a strict Euclidean XY test.
            if (!(finite(tolerance * tolerance) > finite(finite(dy * dy) + finite(dx * dx))))
                continue;
            candidate.clustered = true;
            if (cluster_nodes)
                members.push_back(candidate.node);
            ++absorbed;
            auto node = candidate.node;
            do {
                work.charge(1);
                replacement[node] = xyz; // Includes Z. Original source pool is separate.
                node = g.nodes[node].vertex_next;
            } while (node != candidate.node);
        }
        if (cluster_nodes)
            members.push_back(native_vu_null);
    }
    std::size_t changed = 0;
    for (auto i : all) {
        work.charge(1);
        if (replacement[i] != g.nodes[i].point)
            ++changed;
    }
    // Construct diagnostics before committing any graph mutation.
    Json report = {{"scope", "native_vu_coordinate_consolidation"},
                   {"tolerance", tolerance},
                   {"vertex_count", keys.size()},
                   {"cluster_count", clusters},
                   {"absorbed_vertices", absorbed},
                   {"changed_node_coordinates", changed},
                   {"visit_mask", visit_mask},
                   {"sort_partitions", sorter.partition_count},
                   {"sort_heap_ranges", sorter.heap_count},
                   {"topology_changed", false},
                   {"work_used", budget.work}};
    for (auto i : all) {
        g.nodes[i].point = replacement[i];
        // The original scratch bit remains set after returning it to the pool.
        g.nodes[i].mask |= visit_mask;
    }
    if (cluster_nodes)
        *cluster_nodes = std::move(members);
    return report;
}
void prepare_native_vu_merge(NativeVuInput &input, TubeBudget &budget) {
    const double tolerance = native_vu_merge_tolerance(input.graph, 0, 1e-9, budget);
    auto report = consolidate_native_vu_coordinates(input.graph, tolerance, 0x40000000u, budget);
    auto splits = split_native_vu_edges_near_vertices(input.graph, tolerance, tolerance, tolerance,
                                                      0x40000000u, budget);
    auto second = consolidate_native_vu_coordinates(input.graph, tolerance, 0x40000000u, budget);
    auto intersections =
        split_native_vu_edges_at_intersections(input.graph, tolerance, tolerance, budget);
    auto connections = connect_native_vu_vertices(input.graph, tolerance, 1, 0x40000000u, budget);
    input.report["merge_preparation"] = {{"tolerance", tolerance},
                                         {"initial_consolidation", std::move(report)},
                                         {"near_vertex_splitting", std::move(splits)},
                                         {"second_consolidation", std::move(second)},
                                         {"intersection_splitting", std::move(intersections)},
                                         {"edge_splitting_completed", true},
                                         {"vertex_connection", std::move(connections)},
                                         {"merge_completed", true}};
    input.report["merged"] = true;
}
} // namespace p3d::swept_detail
