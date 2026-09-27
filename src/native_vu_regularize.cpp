#include "native_vu_regularize.hpp"
// Adapted from Bentley imodel-native vureg.cpp and vucoord.cpp, with the P3D
// comparator, scratch-mask order and native floating-point evaluation rules.
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
using Node = std::size_t;
constexpr auto none = native_vu_null;
const NativeVuNode &node(const NativeVuGraph &g, Node n) {
    require(n < g.nodes.size() && g.nodes[n].active, "native VU regularization node");
    return g.nodes[n];
}
Node pred(const NativeVuGraph &g, Node n) {
    return node(g, node(g, node(g, n).vertex_next).face_next).vertex_next;
}
bool below(const NativeVuGraph &g, Node a, Node b) {
    const auto &x = node(g, a).point, &y = node(g, b).point;
    return x[1] < y[1] || (x[1] == y[1] && x[0] < y[0]);
}
double cross(const Point3 &a, const Point3 &b, const Point3 &c) {
    // Native vu_cross uses consecutive differences, not a common origin.
    return finite(finite(finite(b[0] - a[0]) * finite(c[1] - b[1])) -
                  finite(finite(b[1] - a[1]) * finite(c[0] - b[0])));
}
double crossing(const Point3 &b, const Point3 &t, double y, double fallback) {
    const double dy = finite(t[1] - b[1]);
    if (dy == 0)
        return fallback;
    const double f = finite(finite(y - b[1]) / dy);
    if (f < .5)
        return finite(b[0] + finite(f * finite(t[0] - b[0])));
    const double reverse = finite(finite(t[1] - y) / dy);
    return finite(t[0] + finite(reverse * finite(b[0] - t[0])));
}
int horizontal(const NativeVuGraph &g, Node a, Node at, Node b, Node bt) {
    const auto &a0 = g.nodes[a].point, &a1 = g.nodes[at].point;
    const auto &b0 = g.nodes[b].point, &b1 = g.nodes[bt].point;
    const double low = std::max(a0[1], b0[1]), high = std::min(a1[1], b1[1]);
    if (high < low)
        return a0[1] < b0[1] ? -1 : (a0[1] > b0[1] ? 1 : 0);
    const double middle = finite(.5 * finite(low + high));
    double x0, x1;
    if (middle > low && middle < high) {
        x0 = crossing(a0, a1, middle, finite(.5 * finite(a0[0] + a1[0])));
        x1 = crossing(b0, b1, middle, finite(.5 * finite(b0[0] + b1[0])));
    } else {
        // Adjacent representable Y values may have no representable midpoint.
        double base0, base1, top0, top1;
        if (a0[1] < b0[1]) {
            base0 = crossing(a0, a1, b0[1], a0[0]);
            base1 = b0[0];
        } else {
            base0 = a0[0];
            base1 = crossing(b0, b1, a0[1], b0[0]);
        }
        if (b1[1] < a1[1]) {
            top0 = crossing(a0, a1, b1[1], a1[0]);
            top1 = b1[0];
        } else {
            top0 = a1[0];
            top1 = crossing(b0, b1, a1[1], b1[0]);
        }
        x0 = finite(.5 * finite(top0 + base0));
        x1 = finite(.5 * finite(top1 + base1));
    }
    return x0 < x1 ? -1 : (x0 > x1 ? 1 : 0);
}
struct Sweep {
    NativeVuGraph &g;
    TubeBudget &budget;
    curve_detail::BezierWork work;
    std::uint32_t up, downward;
    std::vector<Node> minima, left, right, peaks;
    std::size_t joins = 0, rejections = 0, peak_joins = 0, right_joins = 0, left_joins = 0;
    Sweep(NativeVuGraph &graph, TubeBudget &b, std::uint32_t u, std::uint32_t d)
        : g(graph), budget(b), work{b.work, b.max_work}, up(u), downward(d) {}
    void add(std::vector<Node> &a, Node n) {
        work.charge(1);
        require(a.size() < budget.max_control_points, "native VU sweep array budget");
        a.push_back(n);
    }
    bool marked(Node n, std::uint32_t mask) const {
        return (g.nodes[n].mask & mask) != 0;
    }
    void collect() {
        const auto all = native_vu_all_nodes(g, budget);
        for (auto n : all) {
            work.charge(3);
            g.nodes[n].mask &= ~(up | downward);
            if (below(g, n, g.nodes[n].face_next))
                g.nodes[n].mask |= up;
        }
        for (auto n : all) {
            work.charge(12);
            const auto next = g.nodes[n].face_next, second = g.nodes[next].face_next;
            if (!marked(n, up) && marked(next, up)) {
                const double turn =
                    cross(g.nodes[n].point, g.nodes[next].point, g.nodes[second].point);
                if (n == second && g.nodes[n].vertex_next != n)
                    continue; // A two-node null face cannot accept an edge.
                add(minima, next);
                if (g.nodes[next].vertex_next == next || turn <= 0)
                    g.nodes[next].mask |= downward;
            } else if (marked(n, up) && !marked(next, up) && g.nodes[next].vertex_next != next &&
                       cross(g.nodes[n].point, g.nodes[next].point, g.nodes[second].point) > 0)
                g.nodes[next].mask |= downward;
        }
        native_vu_qsort(minima, [&](Node a, Node b) {
            work.charge(4);
            if (below(g, a, b))
                return -1;
            if (below(g, b, a))
                return 1;
            if (a == b)
                return 0;
            // Native qsort comparator returns -1 even when BOTH distinct,
            // colocated nodes are downward. Do not pass it to std::sort.
            if (marked(a, downward))
                return -1;
            return marked(b, downward) ? 1 : 0;
        });
    }
    Node advance_right(Node minimum) {
        Node first = none;
        for (std::size_t i = 0; i < right.size();) {
            work.charge(32);
            Node b = right[i], t = g.nodes[b].face_next;
            while (below(g, t, minimum) && marked(t, up)) {
                work.charge(3);
                b = t;
                t = g.nodes[t].face_next;
            }
            if (below(g, t, minimum)) {
                if (!marked(t, downward))
                    add(peaks, t);
                // Native removeCurrent moves the last entry to this position.
                right[i] = right.back();
                right.pop_back();
            } else {
                right[i++] = b;
                const double x = crossing(g.nodes[b].point, g.nodes[t].point,
                                          g.nodes[minimum].point[1], g.nodes[b].point[0]);
                if (g.nodes[minimum].point[0] < x &&
                    (first == none || horizontal(g, b, t, first, g.nodes[first].face_next) == -1))
                    first = b;
            }
        }
        return first;
    }
    Node advance_left(Node minimum) {
        Node first = none;
        for (std::size_t i = 0; i < left.size();) {
            work.charge(32);
            Node b = left[i], t = pred(g, b), far = pred(g, t);
            while (below(g, t, minimum) && !marked(far, up)) {
                work.charge(7);
                b = t;
                t = pred(g, t);
                far = pred(g, t);
            }
            if (below(g, t, minimum)) {
                left[i] = left.back();
                left.pop_back();
            } else {
                left[i++] = b;
                const double x = crossing(g.nodes[b].point, g.nodes[t].point,
                                          g.nodes[minimum].point[1], g.nodes[b].point[0]);
                if (g.nodes[minimum].point[0] > x &&
                    (first == none || horizontal(g, b, t, first, pred(g, first)) == 1))
                    first = b;
            }
        }
        return first;
    }
    Node highest_peak(Node l, Node r) {
        Node top = none;
        for (auto p : peaks) {
            work.charge(32);
            if ((l != none && !below(g, l, p)) || (r != none && !below(g, r, p)) ||
                (top != none && !below(g, top, p)))
                continue;
            const auto &point = g.nodes[p].point;
            if (l != none && crossing(g.nodes[l].point, g.nodes[pred(g, l)].point, point[1],
                                      point[0]) >= point[0])
                continue;
            if (r != none && crossing(g.nodes[r].point, g.nodes[g.nodes[r].face_next].point,
                                      point[1], point[0]) <= point[0])
                continue;
            top = p;
        }
        return top;
    }
    std::pair<Node, Node> join(Node minimum, Node far) {
        if (!native_vu_node_in_sector(g, minimum, far, budget) ||
            !native_vu_node_in_sector(g, far, minimum, budget)) {
            ++rejections;
            return {none, none};
        }
        const auto edge = join_native_vu_sectors(g, minimum, far, budget);
        g.nodes[edge.second].mask |= up;
        ++joins;
        return edge;
    }
    void insert(Node minimum) {
        Node r = advance_right(minimum), l = advance_left(minimum);
        const Node peak = highest_peak(l, r);
        if (peak != none) {
            const auto e = join(minimum, peak);
            if (e.first != none) {
                ++peak_joins;
                add(left, e.first);
                add(right, minimum);
            }
            return;
        }
        if (l != none && r != none) {
            if (below(g, l, r))
                l = none;
            else
                r = none;
        }
        if (r != none) {
            const auto e = join(minimum, r);
            if (e.first != none) {
                ++right_joins;
                add(right, minimum);
                add(left, e.first);
                for (auto &n : left) {
                    work.charge(1);
                    if (n == r)
                        n = e.second;
                }
            }
        } else if (l != none) {
            const auto e = join(minimum, l);
            if (e.first != none) {
                ++left_joins;
                add(right, minimum);
                add(left, e.second);
            }
        } else {
            add(right, minimum);
            add(left, minimum);
        }
    }
    Json run() {
        collect();
        for (auto n : minima) {
            work.charge(1);
            if (marked(n, downward))
                insert(n);
            else {
                add(right, n);
                add(left, n);
            }
        }
        // In the native callback-free graph, a rejected sector emits no panic
        // callback and does not increment the retry error field. No extra pass.
        return {{"minimum_count", minima.size()},       {"joined_edges", joins},
                {"sector_join_rejections", rejections}, {"peak_joins", peak_joins},
                {"right_joins", right_joins},           {"left_joins", left_joins}};
    }
};
void rotate(NativeVuGraph &g, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    for (auto n : native_vu_all_nodes(g, budget)) {
        work.charge(2);
        g.nodes[n].point[0] = -g.nodes[n].point[0];
        g.nodes[n].point[1] = -g.nodes[n].point[1];
    }
}
} // namespace
bool native_vu_node_in_sector(const NativeVuGraph &g, Node n, Node s, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(40);
    const auto &sector = node(g, s);
    const Node successor = sector.face_next, predecessor = pred(g, s);
    const auto &p = node(g, n).point, &q = sector.point;
    const auto &before = node(g, predecessor).point, &after = node(g, successor).point;
    for (const auto *v : {&p, &q, &before, &after})
        for (double x : *v)
            finite(x);
    const double sc = cross(q, after, p), pc = cross(before, q, p), cc = cross(before, q, after);
    if (sector.vertex_next == s || (pc > 0 && sc > 0))
        return true;
    if (pc <= 0 && sc <= 0) {
        if (pc == 0 && sc == 0 && cc == 0) {
            if (predecessor == successor)
                return n == predecessor;
            return finite(finite(finite(p[1] - q[1]) * finite(q[1] - before[1])) +
                          finite(finite(p[0] - q[0]) * finite(q[0] - before[0]))) > 0;
        }
        return false;
    }
    if (cc == 0 && pc != 0 && sc != 0)
        return predecessor != successor;
    return cc < 0;
}
Json regularize_native_vu_graph(NativeVuGraph &graph, TubeBudget &budget) {
    validate_native_vu_split_graph(graph, budget);
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(graph.nodes.size());
    auto g = graph;
    auto sweep = [&](std::uint32_t up, std::uint32_t down) {
        Sweep state(g, budget, up, down);
        return state.run();
    };
    const auto up = sweep(0x40000000u, 0x20000000u);
    rotate(g, budget);
    // returnMask(up), then returnMask(downward) reverses the mask-stack order.
    const auto down = sweep(0x20000000u, 0x40000000u);
    rotate(g, budget);
    const auto all = validate_native_vu_split_graph(g, budget);
    Json report{{"regularization_completed",
                 up.at("sector_join_rejections") == 0 && down.at("sector_join_rejections") == 0},
                {"native_sweeps_completed", true},
                {"triangulated", false},
                {"upward_sweep", up},
                {"downward_sweep", down},
                {"joined_edges", up.at("joined_edges").get<std::size_t>() +
                                     down.at("joined_edges").get<std::size_t>()},
                {"active_node_count", all.size()},
                {"storage_slot_count", g.nodes.size()}};
    graph = std::move(g);
    return report;
}
} // namespace p3d::swept_detail
