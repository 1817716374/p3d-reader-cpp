#include "native_vu_indices.hpp"
// Adapted from Bentley imodel-native vupoly.cpp/vusubset.cpp, preserving
// P3D's first numbered-sector lookup without the newer label propagation.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_bezier.hpp"
#include "native_vu_split_support.hpp"
namespace p3d::swept_detail {
NativeVuIndices collect_native_vu_source_indices(NativeVuGraph &graph, TubeBudget &budget) {
    const auto all = validate_native_vu_split_graph(graph, budget);
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(graph.nodes.size());
    auto g = graph;
    constexpr std::uint32_t visit = 0x20000000u;
    std::vector<std::size_t> faces;
    for (auto n : all) {
        work.charge(1);
        g.nodes[n].mask &= ~visit;
    }
    for (auto seed : all) {
        work.charge(1);
        if (g.nodes[seed].mask & visit)
            continue;
        if (!(g.nodes[seed].mask & 2))
            faces.push_back(seed);
        auto n = seed;
        do {
            work.charge(1);
            g.nodes[n].mask |= visit;
            n = g.nodes[n].face_next;
        } while (n != seed);
    }
    NativeVuIndices out;
    out.succeeded = true;
    std::size_t skipped = 0, completed = 0, nontriangles = 0, lookups = 0, failed = native_vu_null;
    auto append = [&](std::int32_t index) {
        work.charge(1);
        require(out.indices.size() < budget.max_control_points, "native VU source index budget");
        out.indices.push_back(index);
    };
    for (auto seed : faces) {
        auto n = seed;
        std::size_t count = 0;
        do {
            work.charge(1);
            ++count;
            n = g.nodes[n].face_next;
        } while (n != seed);
        if (count < 3) {
            ++skipped;
            continue;
        }
        do {
            auto source = n;
            bool found = false;
            do {
                work.charge(1);
                ++lookups;
                if (g.nodes[source].mask & native_vu_numbered_mask) {
                    found = true;
                    break;
                }
                source = g.nodes[source].vertex_next;
            } while (source != n);
            if (!found) {
                out.succeeded = false;
                failed = n;
                break;
            }
            const auto label = g.nodes[source].source_index;
            require(label >= 0 && label < INT32_MAX, "native VU source index range");
            const auto index = std::int32_t(label + 1);
            // Native sign tests the corner, using numbered|boundary together.
            append(g.nodes[n].mask & (native_vu_numbered_mask | native_vu_boundary_mask) ? index
                                                                                         : -index);
            n = g.nodes[n].face_next;
        } while (n != seed);
        append(0); // Native retains a terminated partial face when lookup fails.
        if (!out.succeeded)
            break;
        ++completed;
        if (count != 3)
            ++nontriangles;
    }
    out.report = {
        {"native_succeeded", out.succeeded},
        {"interior_face_count", faces.size()},
        {"completed_face_count", completed},
        {"skipped_short_faces", skipped},
        {"completed_nontriangle_faces", nontriangles},
        {"vertex_sector_visits", lookups},
        {"index_count", out.indices.size()},
        {"new_vertices_created", 0},
        {"unresolved_vertex_node", failed == native_vu_null ? Json(nullptr) : Json(failed)},
        {"all_emitted_faces_triangular", out.succeeded && nontriangles == 0}};
    graph = std::move(g);
    return out;
}
} // namespace p3d::swept_detail
