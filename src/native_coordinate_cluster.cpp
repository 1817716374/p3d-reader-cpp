#include "native_coordinate_cluster.hpp"
// Adapted from Bentley imodel-native DPoint3dOps.cpp.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Native sort order, floating-point sequence and independent channel gates.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_attribute_sort.hpp"
#include "native_bezier_support.hpp"
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
template <std::size_t N>
NativeCoordinateCluster<N> cluster(const std::vector<std::array<double, N>> &source, double abs_tol,
                                   double rel_tol, TubeBudget &budget) {
    static_assert(N == 2 || N == 3);
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    require(source.size() <= budget.max_control_points, "native coordinate cluster input budget");
    finite(abs_tol);
    finite(rel_tol);
    if (abs_tol < 0)
        abs_tol = 1e-8;
    if (rel_tol < 0)
        rel_tol = 1e-8;
    NativeCoordinateCluster<N> out;
    double relative_extent = 0, tolerance = 0;
    std::vector<double> keys;
    std::vector<std::size_t> order, parents;
    std::size_t reassigned = 0, tests = 0, clusters = 0;
    std::array<double, N> direction{};
    direction[0] = 1.45;
    direction[1] = .13;
    double square_length = (1.45 * 1.45) + (.13 * .13);
    if constexpr (N == 3) {
        direction[2] = .2;
        square_length += .2 * .2;
    }
    const double inverse = 1 / std::sqrt(square_length);
    for (auto &x : direction)
        x *= inverse;
    work.charge(source.size());
    out.source_to_packed.resize(source.size());
    keys.reserve(source.size());
    order.reserve(source.size());
    parents.reserve(source.size());
    for (std::size_t i = 0; i < source.size(); ++i) {
        std::array<double, N> delta{};
        work.charge(N * 4 + 3);
        for (std::size_t j = 0; j < N; ++j) {
            delta[j] = finite(finite(source[i][j]) - finite(source.front()[j]));
            relative_extent = std::max(relative_extent, std::abs(delta[j]));
        }
        double key = finite(finite(delta[1] * direction[1]) + finite(delta[0] * direction[0]));
        if constexpr (N == 3)
            key = finite(key + finite(delta[2] * direction[2]));
        keys.push_back(key);
        order.push_back(i);
        parents.push_back(i);
    }
    if (!source.empty())
        tolerance = finite(abs_tol + finite(rel_tol * relative_extent));
    const auto tolerance_squared = finite(tolerance * tolerance),
               sort_tolerance = finite(tolerance + tolerance);
    auto less = [&](std::size_t a, std::size_t b) {
        work.charge(1);
        return keys[a] < keys[b];
    };
    // Same MSVC introsort exchanges as the original 24-byte candidate array:
    // 32-item insertion cutoff, median/ninther, equal range, heap fallback.
    NativeAttributeSort sorter(order, less);
    sorter.sort(0, order.size(), order.size());
    for (std::size_t base = 0; base < order.size(); ++base) {
        work.charge(1);
        if (parents[base] != base)
            continue;
        out.source_to_packed[order[base]] = clusters;
        for (auto test = base + 1; test < order.size(); ++test) {
            work.charge(1);
            if (std::abs(finite(keys[order[base]] - keys[order[test]])) > sort_tolerance)
                break;
            ++tests;
            std::array<double, N> delta{};
            work.charge(N * 3 + 2);
            for (std::size_t j = 0; j < N; ++j)
                delta[j] = finite(source[order[test]][j] - source[order[base]][j]);
            double squared = finite(finite(delta[1] * delta[1]) + finite(delta[0] * delta[0]));
            if constexpr (N == 3)
                squared = finite(squared + finite(delta[2] * delta[2]));
            if (squared <= tolerance_squared) {
                // Native sweep retests even candidates assigned by an earlier
                // base. It may replace their group; this is not union-find.
                reassigned += parents[test] != test;
                parents[test] = base;
                out.source_to_packed[order[test]] = clusters;
            }
        }
        ++clusters;
    }
    work.charge(clusters + source.size());
    out.coordinates.resize(clusters);
    // Last original source member wins, not the sorted cluster base/centroid.
    for (std::size_t i = 0; i < source.size(); ++i)
        out.coordinates[out.source_to_packed[i]] = source[i];
    out.report = {{"scope", "native_coordinate_sort_sweep"},
                  {"dimensions", N},
                  {"input_count", source.size()},
                  {"cluster_count", clusters},
                  {"relative_extent", relative_extent},
                  {"tolerance", tolerance},
                  {"candidate_tests", tests},
                  {"reassigned_candidates", reassigned},
                  {"sort_partitions", sorter.partition_count},
                  {"sort_heap_ranges", sorter.heap_count}};
    return out;
}
} // namespace
NativeCoordinateCluster<2> cluster_native_coordinates(const std::vector<Point2> &p, double a,
                                                      double r, TubeBudget &b) {
    return cluster(p, a, r, b);
}
NativeCoordinateCluster<3> cluster_native_coordinates(const std::vector<Point3> &p, double a,
                                                      double r, TubeBudget &b) {
    return cluster(p, a, r, b);
}
NativePolyfaceCoordinateCombination
combine_native_polyface_coordinates(const NativePolyfaceCoordinateState &source,
                                    TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    std::size_t stored = 0;
    auto count = [&](std::size_t n) {
        require(n <= budget.max_control_points - stored, "native polyface combine input budget");
        stored += n;
        work.charge(n);
    };
    count(source.points.size());
    count(source.normals.size());
    count(source.parameters.size());
    for (const auto &v : source.indices.indices)
        count(v.size());
    NativePolyfaceCoordinateCombination out;
    out.output = source;
    out.report = {{"scope", "native_polyface_coordinate_combination"}, {"applied", false}};
    if (source.points.empty() || source.mesh_style != 1)
        return out;
    auto remap = [&](std::vector<std::int32_t> &indices, const auto &map) {
        for (auto &i : indices) {
            work.charge(1);
            require(i != INT32_MIN, "native combine signed index magnitude");
            if (!i)
                continue;
            const auto old = std::size_t(std::abs(i) - 1);
            require(old < map.size() && map[old] < INT32_MAX, "native combine index range");
            const auto packed = std::int32_t(map[old] + 1);
            i = i < 0 ? -packed : packed;
        }
    };
    auto p = cluster_native_coordinates(source.points, -1, -1, budget);
    out.output.points = std::move(p.coordinates);
    out.point_map = std::move(p.source_to_packed);
    remap(out.output.indices.indices[point_channel], out.point_map);
    out.report["points"] = std::move(p.report);
    if (source.parameter_pool_active && source.indices.active[parameter_channel]) {
        auto uv = cluster_native_coordinates(source.parameters, -1, -1, budget);
        out.output.parameters = std::move(uv.coordinates);
        out.parameter_map = std::move(uv.source_to_packed);
        remap(out.output.indices.indices[parameter_channel], out.parameter_map);
        out.report["parameters"] = std::move(uv.report);
    }
    if (source.normal_pool_active && source.indices.active[normal_channel]) {
        auto n = cluster_native_coordinates(source.normals, -1, -1, budget);
        out.output.normals = std::move(n.coordinates);
        out.normal_map = std::move(n.source_to_packed);
        remap(out.output.indices.indices[normal_channel], out.normal_map);
        out.report["normals"] = std::move(n.report);
    }
    out.applied = true;
    out.report["applied"] = true;
    return out;
}
} // namespace p3d::swept_detail
