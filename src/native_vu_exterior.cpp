#include "native_vu_exterior.hpp"
// Adapted from Bentley imodel-native vumerge.cpp and vusubset.cpp, with P3D's
// zero area threshold, pooled array cursor and native traversal order.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
#include "native_vu_split_support.hpp"
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
constexpr std::uint32_t pass0 = 0x40000000u, pass1 = 0x20000000u, collect_mask = 0x10000000u;
constexpr auto none = native_vu_null;
struct Exterior {
    NativeVuGraph &g;
    TubeBudget &budget;
    curve_detail::BezierWork work;
    std::size_t candidate_cursor, seed_count = 0, within_faces = 0;
    std::size_t within_peak = 0, outside_peak = 0;
    std::vector<std::size_t> candidates;
    Exterior(NativeVuGraph &graph, TubeBudget &b, std::size_t cursor)
        : g(graph), budget(b), work{b.work, b.max_work}, candidate_cursor(cursor) {}
    template <class F> void face(std::size_t seed, F fn) {
        auto n = seed;
        do {
            work.charge(1);
            fn(n);
            n = g.nodes[n].face_next;
        } while (n != seed);
    }
    std::size_t mate(std::size_t n) const {
        return g.nodes[g.nodes[n].face_next].vertex_next;
    }
    bool marked(std::size_t n, std::uint32_t mask) const {
        return (g.nodes[n].mask & mask) != 0;
    }
    // vu_markFaceAndComputeArea sums even trivial loops; vu_area has a special
    // two-successor return of zero. Both use the native trapezoid accumulation.
    double area(std::size_t seed, bool mark) {
        if (!mark && g.nodes[g.nodes[seed].face_next].face_next == seed)
            return 0;
        double sum = 0;
        face(seed, [&](std::size_t n) {
            work.charge(5);
            const auto &p = g.nodes[n].point, &q = g.nodes[g.nodes[n].face_next].point;
            sum = finite(sum - finite(finite(q[1] + p[1]) * finite(q[0] - p[0])));
            if (mark)
                g.nodes[n].mask |= pass0;
        });
        return finite(sum * .5);
    }
    void initialize(const std::vector<std::size_t> &all) {
        for (auto n : all) {
            work.charge(1);
            g.nodes[n].mask &= ~(pass0 | native_vu_exterior_mask);
        }
        for (auto n : all) {
            work.charge(1);
            if (marked(n, pass0))
                continue;
            const double a = area(n, true);
            if (a <= 0 && g.nodes[g.nodes[n].face_next].face_next != n)
                face(n, [&](std::size_t p) { g.nodes[p].mask |= native_vu_exterior_mask; });
        }
        for (auto n : all) {
            work.charge(1);
            g.nodes[n].mask &= ~(pass0 | pass1 | collect_mask);
        }
        for (auto n : all) {
            work.charge(1);
            if (marked(n, collect_mask))
                continue;
            if (marked(n, native_vu_exterior_mask)) {
                require(candidates.size() < budget.max_control_points,
                        "native exterior candidates budget");
                candidates.push_back(n);
            }
            face(n, [&](std::size_t p) { g.nodes[p].mask |= collect_mask; });
        }
    }
    std::size_t select() {
        std::size_t best = none, index = none;
        double best_area = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            work.charge(1);
            const auto n = candidates[i];
            if (marked(n, pass0)) {
                // Native getVuP(i) does not advance the cursor. removeCurrent
                // therefore uses the retained cursor, not the current i.
                if (candidate_cursor > 0 && candidate_cursor <= candidates.size()) {
                    --candidate_cursor;
                    candidates[candidate_cursor] = candidates.back();
                    candidates.pop_back();
                }
            } else {
                const double a = area(n, false);
                if (best == none || a < best_area) {
                    best = n;
                    index = i;
                    best_area = a;
                }
            }
        }
        if (best != none) {
            const auto last = candidates.empty() ? none : candidates.back();
            if (!candidates.empty())
                candidates.pop_back();
            if (last != best && index < candidates.size())
                candidates[index] = last;
        }
        return best;
    }
    struct Frame {
        std::size_t seed;
        bool exterior;
    };
    void push(std::vector<Frame> &stack, std::size_t n, bool ext, std::size_t &peak) {
        work.charge(1);
        require(stack.size() < budget.max_control_points, "native exterior search stack budget");
        stack.push_back({n, ext});
        peak = std::max(peak, stack.size());
    }
    void within(std::size_t seed, bool ext) {
        std::vector<Frame> stack;
        push(stack, seed, ext, within_peak);
        while (!stack.empty()) {
            work.charge(1);
            const auto f = stack.back();
            stack.pop_back();
            if (marked(f.seed, pass0))
                continue;
            ++within_faces;
            face(f.seed, [&](std::size_t n) {
                g.nodes[n].mask = (g.nodes[n].mask & ~native_vu_exterior_mask) | pass0 |
                                  (f.exterior ? native_vu_exterior_mask : 0);
            });
            face(f.seed, [&](std::size_t n) {
                if (!marked(n, native_vu_boundary_mask) && !marked(mate(n), pass0))
                    push(stack, mate(n), f.exterior, within_peak);
            });
        }
    }
    void outside(std::size_t seed) {
        std::vector<Frame> stack;
        push(stack, seed, false, outside_peak);
        while (!stack.empty()) {
            work.charge(1);
            const auto f = stack.back();
            stack.pop_back();
            // Native does not skip duplicate already-queued pass1 frames.
            face(f.seed, [&](std::size_t n) { g.nodes[n].mask |= pass1; });
            face(f.seed, [&](std::size_t n) {
                const auto other = mate(n);
                if (marked(n, native_vu_boundary_mask)) {
                    if (!marked(other, pass0)) {
                        within(other, f.exterior);
                        push(stack, other, !f.exterior, outside_peak);
                    }
                } else if (!marked(other, pass1))
                    push(stack, other, f.exterior, outside_peak);
            });
        }
    }
};
} // namespace
Json mark_native_vu_exterior(NativeVuGraph &graph, std::size_t candidate_read_index,
                             TubeBudget &budget) {
    const auto all = validate_native_vu_split_graph(graph, budget);
    require(candidate_read_index <= budget.max_control_points,
            "native exterior retained cursor budget");
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(graph.nodes.size());
    auto g = graph;
    Exterior state(g, budget, candidate_read_index);
    state.initialize(all);
    const auto candidate_count = state.candidates.size();
    for (auto seed = state.select(); seed != none; seed = state.select()) {
        work.charge(1);
        if ((g.nodes[seed].mask & native_vu_exterior_mask) && !(g.nodes[seed].mask & pass0)) {
            ++state.seed_count;
            state.within(seed, true);
            state.outside(seed);
        }
    }
    std::size_t interior = 0, exterior = 0, unvisited = 0;
    for (auto n : all) {
        work.charge(1);
        if (g.nodes[n].mask & native_vu_exterior_mask)
            ++exterior;
        else
            ++interior;
        if (!(g.nodes[n].mask & pass0))
            ++unvisited;
    }
    Json report{{"exterior_classification_completed", true},
                {"triangulated", false},
                {"initial_candidate_count", candidate_count},
                {"seed_count", state.seed_count},
                {"classified_face_count", state.within_faces},
                {"interior_node_count", interior},
                {"exterior_node_count", exterior},
                {"unreached_node_count", unvisited},
                {"candidate_array_read_index", state.candidate_cursor},
                {"within_stack_peak", state.within_peak},
                {"outside_stack_peak", state.outside_peak}};
    // Topology and coordinates were validated and never changed. Publish masks
    // only after every allocation and report construction has succeeded.
    for (auto n : all)
        graph.nodes[n].mask = g.nodes[n].mask;
    return report;
}
} // namespace p3d::swept_detail
