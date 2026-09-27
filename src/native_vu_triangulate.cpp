#include "native_vu_triangulate.hpp"
// Adapted from Bentley imodel-native vumod2.cpp and vucoord.cpp, with P3D's
// floating-point order, fallback visibility coordinates and new-node payloads.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
#include "native_vu_exterior.hpp"
#include "native_vu_split_support.hpp"
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
using Node = std::size_t;
const NativeVuNode &node(const NativeVuGraph &g, Node n) {
    require(n < g.nodes.size() && g.nodes[n].active, "native VU triangulation node");
    return g.nodes[n];
}
Node pred(const NativeVuGraph &g, Node n) {
    return g.nodes[g.nodes[g.nodes[n].vertex_next].face_next].vertex_next;
}
bool below(const NativeVuGraph &g, Node a, Node b) {
    const auto &p = g.nodes[a].point, &q = g.nodes[b].point;
    return p[1] < q[1] || (p[1] == q[1] && p[0] < q[0]);
}
double cross(const Point3 &a, const Point3 &b, const Point3 &c) {
    return finite(finite(finite(b[0] - a[0]) * finite(c[1] - b[1])) -
                  finite(finite(b[1] - a[1]) * finite(c[0] - b[0])));
}
NativeVuCentroid centroid(const NativeVuGraph &g, Node start, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    const auto origin = node(g, start).point;
    NativeVuCentroid out;
    out.point = {origin[0], origin[1]};
    double sum_x = 0, sum_y = 0, area = 0;
    auto a = g.nodes[start].face_next, b = g.nodes[a].face_next;
    while (b != start) {
        work.charge(24);
        const auto &p = g.nodes[a].point, &q = g.nodes[b].point;
        const double x = finite(finite(finite(p[0] + origin[0]) + q[0]) / 3.0);
        const double y = finite(finite(finite(p[1] + origin[1]) + q[1]) / 3.0);
        const double da = finite(cross(origin, p, q) * .5);
        if (da > 0)
            ++out.positive_count;
        else
            ++out.nonpositive_count;
        area = finite(area + da);
        sum_y = finite(sum_y + finite(y * da));
        sum_x = finite(sum_x + finite(x * da));
        a = b;
        b = g.nodes[b].face_next;
    }
    if (area != 0) {
        out.point = {finite(sum_x / area), finite(sum_y / area)};
        area = finite(area * .5);
        out.succeeded = true;
    }
    out.reported_area = area;
    return out;
}
struct Sweep {
    NativeVuGraph &g;
    TubeBudget &budget;
    curve_detail::BezierWork work;
    std::size_t added_edges = 0, left_runs = 0, right_runs = 0;
    bool monotone = true, centroid_used = false, centroid_accepted = false, guard_stopped = false;
    std::string fallback_status = "not_needed";
    Sweep(NativeVuGraph &graph, TubeBudget &b) : g(graph), budget(b), work{b.work, b.max_work} {}
    Node next(Node n) const {
        return g.nodes[n].face_next;
    }
    Node previous(Node n) const {
        return pred(g, n);
    }
    double turn(Node a, Node b, Node c) {
        work.charge(10);
        return cross(g.nodes[a].point, g.nodes[b].point, g.nodes[c].point);
    }
    Node join(Node a, Node b) {
        const auto result = join_native_vu_sectors(g, a, b, budget);
        ++added_edges;
        return result.first;
    }
    void from_centroid(Node start) {
        centroid_used = true;
        const auto c = centroid(g, start, budget);
        if (!c.succeeded) {
            fallback_status = "zero_area";
            return;
        }
        const auto count = c.positive_count + c.nonpositive_count;
        if (count <= 3) {
            fallback_status = "insufficient_fan_triangles";
            return;
        }
        if (c.nonpositive_count > 0) {
            double du0 = finite(g.nodes[start].point[0] - c.point[0]);
            // The native first dv uses start.X, not start.Y. Preserve this
            // older-version visibility rule rather than silently correcting it.
            double dv0 = finite(g.nodes[start].point[0] - c.point[1]);
            const auto first = next(start);
            auto n = first;
            std::size_t rejected = 0;
            do {
                work.charge(10);
                const double du1 = finite(g.nodes[n].point[0] - c.point[0]);
                const double dv1 = finite(g.nodes[n].point[1] - c.point[1]);
                if (finite(finite(du0 * dv1) - finite(du1 * dv0)) <= 0)
                    ++rejected;
                du0 = du1;
                dv0 = dv1;
                n = next(n);
            } while (n != first);
            if (rejected) {
                fallback_status = "visibility_rejected";
                return;
            }
        }
        const auto pair = make_native_vu_pair(g, budget);
        ++added_edges;
        Node center = pair.first, fringe = pair.second;
        g.nodes[center].point = {c.point[0], c.point[1], 0};
        g.nodes[fringe].point = {g.nodes[start].point[0], g.nodes[start].point[1], 0};
        twist_native_vu_vertices(g, fringe, start, budget);
        // nEdge=count+2; native pre-decrement loop performs count+1 joins.
        for (std::size_t i = 0; i < count + 1; ++i) {
            work.charge(1);
            center = join(center, next(next(center)));
        }
        centroid_accepted = true;
        fallback_status = "constructed";
    }
    Json run(Node start) {
        node(g, start);
        Node left = previous(start), right = next(start), l = start, r = start;
        while (below(g, l, previous(l))) {
            work.charge(3);
            l = previous(l);
        }
        while (below(g, r, next(r))) {
            work.charge(3);
            r = next(r);
        }
        monotone = l == r;
        if (!monotone)
            from_centroid(start);
        else {
            while (left != right && right != start && next(right) != left) {
                work.charge(6);
                if (turn(left, start, right) <= 0 || !below(g, start, left) ||
                    !below(g, start, right)) {
                    guard_stopped = true;
                    break;
                }
                Node p0 = left, p1 = start, p2 = right;
                if (below(g, left, right)) {
                    ++left_runs;
                    while (p0 != p2 && below(g, p0, right)) {
                        work.charge(4);
                        while (p2 != right && p2 != p0 && p2 != p1 && turn(p0, p1, p2) > 0) {
                            p0 = join(p0, p2);
                            p1 = next(p0);
                            p2 = next(p1);
                        }
                        p2 = p1;
                        p1 = p0;
                        p0 = previous(p0);
                    }
                    left = p1;
                    p2 = right;
                    p1 = previous(p2);
                    p0 = previous(p1);
                    while (next(p2) != p0 && p0 != left) {
                        work.charge(2);
                        p1 = join(p0, p2);
                        p0 = previous(p1);
                    }
                    if (next(p2) != p0)
                        p0 = join(p0, p2);
                    start = p0;
                } else {
                    ++right_runs;
                    while (p0 != p2 && below(g, p2, left)) {
                        work.charge(4);
                        while (p0 != left && p2 != p0 && p2 != p1 && turn(p0, p1, p2) > 0) {
                            p1 = join(p0, p2);
                            p0 = previous(p1);
                        }
                        p0 = p1;
                        p1 = p2;
                        p2 = next(p2);
                    }
                    right = p1;
                    p0 = left;
                    p1 = next(p0);
                    p2 = next(p1);
                    while (next(p2) != p0 && p2 != right) {
                        work.charge(2);
                        p0 = join(p0, p2);
                        p1 = p2;
                        p2 = next(p2);
                    }
                    if (next(p2) != p0)
                        join(p0, p2);
                    start = right;
                }
                right = next(start);
                left = previous(start);
            }
        }
        return {{"monotone_check_passed", monotone},
                {"centroid_fallback_attempted", centroid_used},
                {"centroid_fallback_constructed", centroid_accepted},
                {"fallback_status", fallback_status},
                {"guard_stopped", guard_stopped},
                {"added_edges", added_edges},
                {"left_runs", left_runs},
                {"right_runs", right_runs}};
    }
};
} // namespace
NativeVuCentroid native_vu_face_centroid(const NativeVuGraph &g, Node start, TubeBudget &budget) {
    validate_native_vu_split_graph(g, budget);
    return centroid(g, start, budget);
}
Json triangulate_native_vu_face(NativeVuGraph &graph, Node start, TubeBudget &budget) {
    validate_native_vu_split_graph(graph, budget);
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(graph.nodes.size());
    auto g = graph;
    Sweep state(g, budget);
    auto report = state.run(start);
    validate_native_vu_split_graph(g, budget);
    graph = std::move(g);
    return report;
}
Json triangulate_native_vu_interiors(NativeVuGraph &graph, TubeBudget &budget) {
    auto all = validate_native_vu_split_graph(graph, budget);
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(graph.nodes.size());
    auto g = graph;
    constexpr std::uint32_t original_mask = 0x20000000u, collect_mask = 0x40000000u;
    std::vector<Node> faces;
    for (auto n : all) {
        work.charge(1);
        g.nodes[n].mask &= ~collect_mask;
    }
    for (auto seed : all) {
        work.charge(1);
        if (g.nodes[seed].mask & collect_mask)
            continue;
        if (!(g.nodes[seed].mask & native_vu_exterior_mask))
            faces.push_back(seed);
        auto n = seed;
        do {
            work.charge(1);
            g.nodes[n].mask |= collect_mask;
            n = g.nodes[n].face_next;
        } while (n != seed);
    }
    for (auto n : all) {
        work.charge(1);
        g.nodes[n].mask |= original_mask;
    }
    const auto input_face_count = faces.size();
    std::size_t processed = 0, added = 0, fallbacks = 0, accepted = 0, guard_stops = 0;
    for (std::size_t i = 0; i < faces.size();) {
        auto n = faces[i];
        std::size_t count = 0;
        do {
            work.charge(1);
            ++count;
            n = g.nodes[n].face_next;
        } while (n != faces[i]);
        if (count <= 3) {
            faces[i] = faces.back();
            faces.pop_back();
            continue;
        }
        Node low = faces[i];
        do {
            work.charge(3);
            if (below(g, n, low))
                low = n;
            n = g.nodes[n].face_next;
        } while (n != faces[i]);
        Sweep state(g, budget);
        state.run(low);
        ++processed;
        added += state.added_edges;
        fallbacks += state.centroid_used;
        accepted += state.centroid_accepted;
        guard_stops += state.guard_stopped;
        ++i;
    }
    all = validate_native_vu_split_graph(g, budget);
    std::vector<bool> seen(g.nodes.size());
    std::size_t triangles = 0, short_faces = 0, remaining = 0;
    for (auto seed : all) {
        work.charge(1);
        if (seen[seed])
            continue;
        auto n = seed;
        std::size_t count = 0;
        do {
            work.charge(1);
            seen[n] = true;
            ++count;
            n = g.nodes[n].face_next;
        } while (n != seed);
        if (g.nodes[seed].mask & native_vu_exterior_mask)
            continue;
        if (count == 3)
            ++triangles;
        else if (count < 3)
            ++short_faces;
        else
            ++remaining;
    }
    Json report{{"interior_triangle_faces_completed", remaining == 0},
                {"final_indices_ready", false},
                {"input_interior_face_count", input_face_count},
                {"processed_large_faces", processed},
                {"added_edges", added},
                {"centroid_fallback_attempts", fallbacks},
                {"centroid_fallback_constructions", accepted},
                {"guard_stops", guard_stops},
                {"triangle_face_count", triangles},
                {"short_interior_face_count", short_faces},
                {"remaining_large_interior_faces", remaining},
                {"candidate_array_read_index", faces.size()}};
    graph = std::move(g);
    return report;
}
} // namespace p3d::swept_detail
