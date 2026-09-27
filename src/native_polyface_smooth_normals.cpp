// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from polyfaceAddNormals.cpp. Preserves original P3D descending
// traversal, edge-angle margin, arithmetic order and publication behavior.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_polyface_smooth_normals.hpp"
#include "native_polyface_connectivity.hpp"
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
using Work = curve_detail::BezierWork;
double angle(const Point3 &a, const Point3 &b, Work work) {
    work.charge(40);
    const Point3 cross{finite(finite(a[1] * b[2]) - finite(a[2] * b[1])),
                       finite(finite(b[0] * a[2]) - finite(a[0] * b[2])),
                       finite(finite(a[0] * b[1]) - finite(b[0] * a[1]))};
    const double square = finite(finite(finite(cross[1] * cross[1]) + finite(cross[0] * cross[0])) +
                                 finite(cross[2] * cross[2]));
    const double dot =
        finite(finite(finite(a[0] * b[0]) + finite(b[1] * a[1])) + finite(a[2] * b[2]));
    const double magnitude = std::sqrt(square);
    return dot == 0 && magnitude == 0 ? 0 : finite(std::atan2(magnitude, dot));
}
Point3 normalized(Point3 value, Work work) {
    work.charge(15);
    const double length =
        std::sqrt(finite(finite(finite(value[0] * value[0]) + finite(value[1] * value[1])) +
                         finite(value[2] * value[2])));
    if (length <= 0)
        return {1, 0, 0};
    const double inverse = finite(1 / length);
    for (auto &x : value)
        x = finite(x * inverse);
    return value;
}
} // namespace
NativePolyfaceAttributes
build_native_polyface_approximate_normals(const NativePolyfaceFaceDataState &input,
                                          double max_single, double max_accumulated,
                                          bool mark_transitions_visible, TubeBudget &budget) {
    finite(max_single);
    finite(max_accumulated);
    const Work work{budget.work, budget.max_work};
    auto result = build_native_polyface_normals(input, budget);
    Json report{{"operation", "native_build_approximate_normals"},
                {"per_face", std::move(result.report)},
                {"max_single_edge_angle", max_single},
                {"max_accumulated_angle", max_accumulated},
                {"mark_transitions_visible", mark_transitions_visible},
                {"node_order", "descending"},
                {"edge_angle_margin", 1e-5},
                {"average_normal_dot_threshold", 0.92},
                {"normal_pool_replaced", false}};
    auto finish = [&](bool native, bool complete, const char *reason) {
        result.native_succeeded = native;
        result.complete = complete;
        report["native_succeeded"] = native;
        report["complete"] = complete;
        report["reason"] = reason;
        result.report = std::move(report);
        return std::move(result);
    };
    if (!result.native_succeeded)
        return finish(false, false, "per_face_generation_failed");
    const bool per_face_complete = result.complete;
    auto graph = build_native_polyface_connectivity(result.output, false, budget);
    report["connectivity"] = std::move(graph.report);
    if (!graph.native_succeeded)
        return finish(false, false, "connectivity_failed");
    auto &mesh = result.output.mesh;
    const auto &old_normals = mesh.coordinates.normals;
    auto &old_indices = mesh.indices.indices[normal_channel];
    auto &points = mesh.indices.indices[point_channel];
    const auto &nodes = graph.nodes;
    std::size_t storage = 0;
    auto grow = [&](std::size_t n) {
        require(n <= budget.max_control_points - storage, "native smooth normal storage budget");
        storage += n;
        work.charge(n);
    };
    grow(mesh.coordinates.points.size());
    grow(old_normals.size());
    grow(mesh.coordinates.parameters.size());
    for (const auto &v : mesh.indices.indices)
        grow(v.size());
    grow(mesh.face_data.size());
    grow(mesh.edge_chains.size());
    for (const auto &e : mesh.edge_chains) {
        grow(e.point_indices.size());
        grow(e.topology_ids.size());
    }
    grow(graph.points.size());
    grow(nodes.size());
    grow(graph.half_edges.size());
    grow(graph.read_to_half_edge.size());
    grow(old_indices.size());
    std::vector<std::int32_t> new_indices = old_indices;
    for (auto &i : new_indices) {
        work.charge(1);
        require(i != INT32_MIN, "native smooth normal signed index overflow");
        i = -std::abs(i);
    }
    grow(nodes.size());
    grow(nodes.size());
    std::vector<std::uint8_t> barrier(nodes.size()), visited(nodes.size());
    std::vector<std::size_t> bases;
    std::vector<Point3> new_normals;
    std::size_t boundary_bases = 0, angle_bases = 0, interior_bases = 0, averaged_sectors = 0,
                separate_sectors = 0, hidden = 0;
    auto exterior = [&](std::size_t n) { return bool(nodes[n].mask & native_mtg_exterior); };
    auto get_normal = [&](std::size_t n, Point3 &normal) {
        work.charge(1);
        const auto read = nodes[n].read_index;
        if (read < 0 || static_cast<std::size_t>(read) > points.size() || old_indices.empty() ||
            old_normals.empty())
            return false;
        require(static_cast<std::size_t>(read) < old_indices.size(),
                "native normal query outside index buffer");
        const auto signed_index =
            static_cast<std::int64_t>(old_indices[static_cast<std::size_t>(read)]);
        const auto index =
            static_cast<std::uint64_t>(signed_index < 0 ? -signed_index : signed_index) - 1;
        if (index >= old_normals.size())
            return false;
        normal = old_normals[static_cast<std::size_t>(index)];
        for (auto x : normal)
            finite(x);
        return true;
    };
    auto add_base = [&](std::size_t n) {
        grow(1);
        bases.push_back(n);
    };
    for (std::size_t a = nodes.size(); a-- > 0;) {
        work.charge(1);
        const auto c = nodes[nodes[a].face_successor].vertex_successor;
        const auto d = nodes[c].face_successor;
        if (exterior(a))
            barrier[a] = 1;
        else if (exterior(c)) {
            barrier[a] = 1;
            add_base(a);
            ++boundary_bases;
        } else {
            Point3 normal_a{}, normal_d{};
            if (!get_normal(a, normal_a) || !get_normal(d, normal_d))
                return finish(false, false, "missing_edge_normal");
            const double theta = angle(normal_a, normal_d, work);
            // Original performs two AngleTo calls and checks theta+1e-5 too.
            if (theta > max_single || finite(angle(normal_a, normal_d, work) + 1e-5) > max_single) {
                barrier[a] = 1;
                add_base(a);
                ++angle_bases;
            }
        }
    }
    for (std::size_t a = nodes.size(); a-- > 0;)
        if (!visited[a]) {
            std::size_t n = a, barriers = 0, steps = 0;
            do {
                work.charge(1);
                require(++steps <= nodes.size(), "native vertex ring does not close");
                visited[n] = 1;
                if (barrier[n])
                    ++barriers;
                n = nodes[n].vertex_successor;
            } while (n != a);
            if (!barriers) {
                add_base(a);
                ++interior_bases;
            }
        }
    report["base_nodes"] = bases;
    report["boundary_base_count"] = boundary_bases;
    report["angle_base_count"] = angle_bases;
    report["interior_base_count"] = interior_bases;
    Json sectors = Json::array();
    std::vector<std::size_t> sector_nodes;
    std::vector<Point3> sector_normals;
    std::size_t sector_capacity = 0;
    auto append = [&](const Point3 &normal) {
        grow(1);
        require(new_normals.size() < static_cast<std::size_t>(INT32_MAX),
                "native smooth normal output index overflow");
        new_normals.push_back(normal);
        return static_cast<std::int32_t>(new_normals.size());
    };
    auto assign = [&](std::size_t node, std::int32_t index) {
        work.charge(1);
        const auto read = nodes[node].read_index;
        if (read < 0 || static_cast<std::size_t>(read) >= new_indices.size())
            return false;
        new_indices[static_cast<std::size_t>(read)] = index;
        return true;
    };
    for (auto base : bases) {
        sector_nodes.clear();
        sector_normals.clear();
        std::size_t n = base;
        do {
            work.charge(1);
            require(sector_nodes.size() < nodes.size(), "native normal sector does not terminate");
            Point3 normal{};
            if (!get_normal(n, normal))
                return finish(false, false, "missing_sector_normal");
            if (sector_nodes.size() == sector_capacity) {
                grow(2);
                ++sector_capacity;
            }
            sector_nodes.push_back(n);
            sector_normals.push_back(normal);
            n = nodes[n].vertex_successor;
        } while (n != base && !barrier[n]);
        Point3 accumulated = sector_normals[0];
        double accumulated_angle = 0;
        for (std::size_t k = 1; k < sector_normals.size(); ++k) {
            accumulated_angle =
                finite(accumulated_angle + angle(sector_normals[k - 1], sector_normals[k], work));
            work.charge(3);
            for (unsigned i = 0; i < 3; ++i)
                accumulated[i] = finite(sector_normals[k][i] + accumulated[i]);
        }
        const auto unit = normalized(accumulated, work);
        const double inverse = 1 / static_cast<double>(sector_normals.size());
        Point3 mean{};
        for (unsigned i = 0; i < 3; ++i)
            mean[i] = finite(accumulated[i] * inverse);
        work.charge(9);
        const double dot = finite(finite(finite(mean[1] * unit[1]) + finite(mean[0] * unit[0])) +
                                  finite(mean[2] * unit[2]));
        const bool average = dot > 0.92 && accumulated_angle < max_accumulated;
        sectors.push_back({{"base_node", base},
                           {"corner_count", sector_nodes.size()},
                           {"accumulated_angle", accumulated_angle},
                           {"average_normal_dot", dot},
                           {"averaged", average}});
        if (average) {
            const auto index = append(unit);
            barrier[base] = 1;
            ++averaged_sectors;
            for (auto node : sector_nodes)
                if (!assign(node, index))
                    return finish(false, false, "invalid_sector_read_index");
        } else {
            ++separate_sectors;
            for (std::size_t k = 0; k < sector_nodes.size(); ++k) {
                const auto index = append(sector_normals[k]);
                barrier[sector_nodes[k]] = 1;
                if (!assign(sector_nodes[k], index))
                    return finish(false, false, "invalid_sector_read_index");
            }
        }
    }
    report["sectors"] = std::move(sectors);
    report["averaged_sectors"] = averaged_sectors;
    report["separate_sectors"] = separate_sectors;
    for (auto i : new_indices) {
        work.charge(1);
        if (i < 0)
            return finish(false, false, "untouched_normal_index");
    }
    mesh.coordinates.normals = std::move(new_normals);
    old_indices = std::move(new_indices);
    report["normal_pool_replaced"] = true;
    if (mark_transitions_visible) {
        for (auto &i : points) {
            work.charge(1);
            require(i != INT32_MIN, "native visibility signed index overflow");
            i = std::abs(i);
        }
        for (std::size_t z = nodes.size(); z-- > 0;)
            if (!exterior(z)) {
                work.charge(1);
                const auto a = nodes[nodes[nodes[z].face_successor].vertex_successor]
                                   .face_successor; // F-V-F = VPred
                if (!exterior(a)) {
                    const auto ra = nodes[a].read_index, rz = nodes[z].read_index;
                    if (ra >= 0 && rz >= 0 && static_cast<std::size_t>(ra) < old_indices.size() &&
                        static_cast<std::size_t>(rz) < old_indices.size() &&
                        old_indices[ra] == old_indices[rz] &&
                        static_cast<std::size_t>(rz) < points.size()) {
                        points[rz] = -std::abs(points[rz]);
                        ++hidden;
                    }
                }
            }
    }
    std::size_t missing = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        work.charge(1);
        if (points[i] && old_indices[i] == 0)
            ++missing;
    }
    report["hidden_half_edges"] = hidden;
    report["unassigned_nonzero_indices"] = missing;
    return finish(true, per_face_complete && graph.complete && missing == 0, "completed");
}
} // namespace p3d::swept_detail
