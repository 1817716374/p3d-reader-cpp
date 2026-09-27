#include "native_vu_graph.hpp"
// Indexed loops and topology primitives adapted from Bentley imodel-native
// VuOps.cpp and vu.cpp, with native P3D masks and traversal order.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
#include "native_vu_split_support.hpp"
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
void node_index(const NativeVuGraph &g, std::size_t index) {
    require(index < g.nodes.size() && g.nodes[index].active, "native VU node index");
}
std::size_t mate(const NativeVuGraph &g, std::size_t index) {
    node_index(g, index);
    const auto next = g.nodes[index].face_next;
    node_index(g, next);
    const auto result = g.nodes[next].vertex_next;
    node_index(g, result);
    return result;
}
void twist_nodes(NativeVuGraph &g, std::size_t a, std::size_t b) {
    node_index(g, a);
    node_index(g, b);
    const auto va = g.nodes[a].vertex_next, vb = g.nodes[b].vertex_next;
    const auto fa = mate(g, va), fb = mate(g, vb);
    g.nodes[a].vertex_next = vb;
    g.nodes[b].vertex_next = va;
    g.nodes[fa].face_next = b;
    g.nodes[fb].face_next = a;
}
void reserve_pair(NativeVuGraph &g, TubeBudget &budget) {
    const auto count = g.nodes.size();
    std::size_t needed = 2, free = g.free_head;
    while (needed && free != native_vu_null) {
        require(free < count && !g.nodes[free].active, "native VU free slot");
        const auto next = g.nodes[free].all_next;
        require(next != free && (needed == 2 || next != g.free_head), "native VU free cycle");
        free = next;
        --needed;
    }
    require(count <= budget.max_control_points && budget.max_control_points - count >= needed &&
                count <= std::size_t(INT32_MAX) - needed,
            "native VU node allocation budget");
    // Reserve both before mutating links. Growth is bounded and amortized,
    // rather than reallocating the entire graph for every split.
    if (g.nodes.capacity() - count < needed) {
        const auto extra = std::max<std::size_t>(count / 2, 16);
        const auto growth = std::min(extra, budget.max_control_points - count);
        g.nodes.reserve(count + growth);
    }
}
std::size_t append_node(NativeVuGraph &g) {
    const auto id = g.free_head == native_vu_null ? g.nodes.size() : g.free_head;
    NativeVuNode node;
    node.all_next = node.face_next = node.vertex_next = id;
    if (id == g.nodes.size())
        g.nodes.push_back(node);
    else {
        g.free_head = g.nodes[id].all_next;
        g.nodes[id] = node;
    }
    if (g.tail == native_vu_null)
        g.tail = id;
    else {
        g.nodes[id].all_next = g.nodes[g.tail].all_next;
        g.nodes[g.tail].all_next = id;
    }
    return id;
}
bool disconnect(const Point3 &p) {
    const auto mark = std::numeric_limits<double>::max();
    return p[0] == mark || p[1] == mark || p[2] == mark;
}
bool distinct_xy(const Point3 &a, const Point3 &b, double tolerance) {
    return std::abs(finite(a[0] - b[0])) > tolerance || std::abs(finite(a[1] - b[1])) > tolerance;
}
void mask_face(NativeVuGraph &g, std::size_t base, std::uint32_t mask,
               const curve_detail::BezierWork &work) {
    auto current = base;
    std::size_t count = 0;
    do {
        work.charge(1);
        node_index(g, current);
        require(++count <= g.nodes.size(), "native VU face cycle");
        g.nodes[current].mask |= mask;
        current = g.nodes[current].face_next;
    } while (current != base);
}
} // namespace
std::vector<std::size_t> native_vu_all_nodes(const NativeVuGraph &g, TubeBudget &budget) {
    require(g.nodes.size() <= budget.max_control_points && g.nodes.size() <= INT32_MAX,
            "native VU storage extent");
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(g.nodes.size());
    std::vector<bool> seen(g.nodes.size());
    std::vector<std::size_t> order;
    if (g.tail != native_vu_null) {
        node_index(g, g.tail);
        auto n = g.nodes[g.tail].all_next;
        for (;;) {
            work.charge(1);
            node_index(g, n);
            require(!seen[n], "native VU active cycle");
            seen[n] = true;
            order.push_back(n);
            if (n == g.tail)
                break;
            n = g.nodes[n].all_next;
        }
    }
    auto n = g.free_head;
    while (n != native_vu_null) {
        work.charge(1);
        require(n < g.nodes.size() && !g.nodes[n].active && !seen[n], "native VU free chain");
        seen[n] = true;
        n = g.nodes[n].all_next;
    }
    for (bool present : seen)
        require(present, "native VU orphan slot");
    return order;
}
std::size_t free_marked_native_vu_edges(NativeVuGraph &graph, std::uint32_t mask,
                                        TubeBudget &budget) {
    const auto all = validate_native_vu_split_graph(graph, budget);
    if (!mask)
        return 0;
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(graph.nodes.size());
    auto g = graph;
    bool found = false;
    for (auto n : all) {
        work.charge(1);
        if (!(g.nodes[n].mask & mask))
            continue;
        found = true;
        const auto m = mate(g, n);
        g.nodes[m].mask |= mask;
        auto pred = g.nodes[m].face_next;
        if (pred != n)
            twist_native_vu_vertices(g, n, pred, budget);
        pred = g.nodes[mate(g, m)].face_next;
        // Native second comparison is against the original node, not mate.
        if (pred != n)
            twist_native_vu_vertices(g, m, pred, budget);
    }
    if (!found)
        return 0;
    g.tail = native_vu_null;
    std::size_t first = native_vu_null, removed = 0;
    for (auto n : all) {
        work.charge(1);
        if (g.nodes[n].mask & mask) {
            g.nodes[n].active = false;
            g.nodes[n].all_next = g.free_head;
            g.free_head = n;
            ++removed;
        } else {
            if (first == native_vu_null)
                first = n;
            else
                g.nodes[g.tail].all_next = n;
            g.tail = n;
        }
    }
    if (g.tail != native_vu_null)
        g.nodes[g.tail].all_next = first;
    validate_native_vu_split_graph(g, budget);
    graph = std::move(g);
    return removed;
}
std::pair<std::size_t, std::size_t> split_native_vu_edge(NativeVuGraph &g, std::size_t base,
                                                         TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(24);
    if (g.tail != native_vu_null) {
        node_index(g, g.tail);
        node_index(g, g.nodes[g.tail].all_next);
    } else
        require(g.nodes.empty() || g.free_head != native_vu_null, "native VU missing list tail");
    auto opposite = native_vu_null;
    if (base != native_vu_null) {
        opposite = mate(g, base);
        node_index(g, g.nodes[opposite].face_next);
    }
    reserve_pair(g, budget);
    const auto left = append_node(g), right = append_node(g);
    if (base != native_vu_null) {
        g.nodes[left].face_next = g.nodes[base].face_next;
        g.nodes[base].face_next = left;
        g.nodes[right].face_next = g.nodes[opposite].face_next;
        g.nodes[opposite].face_next = right;
        g.nodes[left].mask |= g.nodes[base].mask & 0x54efu;
        g.nodes[right].mask |= g.nodes[opposite].mask & 0x54efu;
    } else {
        g.nodes[left].mask |= 1;
        g.nodes[right].mask |= 1;
    }
    g.nodes[left].vertex_next = right;
    g.nodes[right].vertex_next = left;
    return {left, right};
}
void twist_native_vu_vertices(NativeVuGraph &g, std::size_t a, std::size_t b, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(12);
    twist_nodes(g, a, b);
}
std::pair<std::size_t, std::size_t> make_native_vu_pair(NativeVuGraph &g, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(24);
    if (g.tail != native_vu_null) {
        node_index(g, g.tail);
        node_index(g, g.nodes[g.tail].all_next);
    } else
        require(g.nodes.empty() || g.free_head != native_vu_null, "native VU missing list tail");
    reserve_pair(g, budget);
    const auto start = append_node(g), end = append_node(g);
    g.nodes[start].face_next = end;
    g.nodes[end].face_next = start;
    return {start, end};
}
std::pair<std::size_t, std::size_t> join_native_vu_sectors(NativeVuGraph &g, std::size_t a,
                                                       std::size_t b, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(48);
    node_index(g, a);
    node_index(g, b);
    mate(g, g.nodes[a].vertex_next);
    mate(g, g.nodes[b].vertex_next);
    for (double x : g.nodes[a].point)
        finite(x);
    for (double x : g.nodes[b].point)
        finite(x);
    reserve_pair(g, budget);
    const auto start = append_node(g), end = append_node(g);
    g.nodes[start].face_next = end;
    g.nodes[end].face_next = start;
    twist_nodes(g, start, a);
    g.nodes[start].point = g.nodes[a].point;
    twist_nodes(g, end, b);
    g.nodes[end].point = g.nodes[b].point;
    return {start, end};
}
static NativeVuInput build_input(const std::vector<Point3> &points, double tolerance,
                                 TubeBudget &budget, bool xy_only) {
    require(points.size() <= budget.max_control_points && points.size() <= INT32_MAX,
            "native VU source point extent");
    require(std::isfinite(tolerance) && tolerance >= 0, "native VU XY tolerance");
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(points.size());
    for (const auto &p : points)
        for (double x : p)
            finite(x);
    // 134630 computes unsigned end = first_marker - 1. At index zero this
    // underflows; the space-loop caller normally rejects the short prefix.
    require(xy_only || points.empty() || !disconnect(points.front()),
            "native VU initial disconnect underflow");
    auto is_disconnect = [&](const Point3 &p) {
        const auto marker = std::numeric_limits<double>::max();
        return xy_only ? p[0] == marker || p[1] == marker : disconnect(p);
    };
    NativeVuInput out;
    std::size_t pos = 0, markers = 0, trimmed = 0, adjacent = 0;
    while (pos < points.size()) {
        const auto begin = pos;
        while (pos < points.size() && !is_disconnect(points[pos])) {
            work.charge(1);
            ++pos;
        }
        const auto end = pos;
        if (pos < points.size()) {
            work.charge(1);
            ++pos;
            ++markers;
        }
        if (begin == end)
            continue;
        NativeVuInputLoop loop;
        loop.begin = begin;
        loop.end = end;
        loop.trimmed_end = end;
        while (loop.trimmed_end > begin + 1) {
            work.charge(4);
            if (distinct_xy(points[loop.trimmed_end - 1], points[begin], tolerance))
                break;
            --loop.trimmed_end;
            ++trimmed;
        }
        Point3 last = points[loop.trimmed_end - 1];
        for (auto i = begin; i < loop.trimmed_end; ++i) {
            work.charge(4);
            if (i == begin || distinct_xy(points[i], last, tolerance)) {
                const auto pair = split_native_vu_edge(out.graph, loop.base, budget);
                loop.base = pair.first;
                for (auto node : {pair.first, pair.second}) {
                    out.graph.nodes[node].point = points[i];
                    out.graph.nodes[node].source_index = std::int32_t(i);
                }
                last = points[i];
                ++loop.inserted_vertices;
            } else
                ++adjacent;
        }
        const auto mask = native_vu_numbered_mask | native_vu_boundary_mask;
        mask_face(out.graph, loop.base, mask, work);
        mask_face(out.graph, out.graph.nodes[loop.base].vertex_next, mask, work);
        if (out.first_loop == native_vu_null)
            out.first_loop = loop.base;
        work.charge(1);
        out.loops.push_back(loop);
    }
    out.report = {{"scope", "native_vu_indexed_input_loops"},
                  {"source_point_count", points.size()},
                  {"loop_count", out.loops.size()},
                  {"node_count", out.graph.nodes.size()},
                  {"disconnect_count", markers},
                  {"trailing_xy_duplicates", trimmed},
                  {"adjacent_xy_duplicates", adjacent},
                  {"xy_tolerance", tolerance},
                  {"numbered_node_mask", native_vu_numbered_mask},
                  {"merged", false},
                  {"triangulated", false},
                  {"work_used", budget.work}};
    if (xy_only)
        out.report["scope"] = "native_vu_general_polygon_input_loops";
    return out;
}
NativeVuInput build_native_vu_input(const std::vector<Point3> &points, double tolerance,
                                    TubeBudget &budget) {
    return build_input(points, tolerance, budget, false);
}
NativeVuInput build_native_vu_polygon_input(const std::vector<Point3> &points, double tolerance,
                                           TubeBudget &budget) {
    return build_input(points, tolerance, budget, true);
}
} // namespace p3d::swept_detail
