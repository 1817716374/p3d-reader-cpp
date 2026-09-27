// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from pf_halfEdgeArray.h and MTGGraph.cpp. Original P3D half-edge
// sorting, degeneracy pairing, labels and node twists. See THIRD_PARTY.md.
#include "native_polyface_connectivity.hpp"
#include "native_polyface_visitor.hpp"
#include "native_attribute_sort.hpp"
#include "native_bezier.hpp"
namespace p3d::swept_detail {
NativePolyfaceConnectivity
assemble_native_polyface_half_edges(const std::vector<Point3> &points,
                                    std::vector<NativePolyfaceHalfEdge> edges, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    std::size_t storage = 0;
    auto grow = [&](std::size_t n) {
        require(n <= budget.max_control_points - storage, "native connectivity storage budget");
        storage += n;
        work.charge(n);
    };
    grow(points.size());
    grow(edges.size());
    require(points.size() <= std::size_t(INT32_MAX), "native connectivity vertex label overflow");
    for (const auto &e : edges) {
        work.charge(1);
        require(e.vertex0 > 0 && e.vertex0 <= points.size() && e.vertex1 > 0 &&
                    e.vertex1 <= points.size(),
                "native connectivity invalid prepared vertex identity");
    }
    NativePolyfaceConnectivity out;
    auto &nodes = out.nodes;
    // Sort whole half-edge identities with the original MSVC exchange order.
    // Stable sorting equal keys would change pairing and subsequent node IDs.
    grow(edges.size());
    std::vector<std::size_t> order(edges.size());
    for (std::size_t i = 0; i < order.size(); ++i)
        order[i] = i;
    auto key = [](const NativePolyfaceHalfEdge &e) {
        return std::pair{std::min(e.vertex0, e.vertex1), std::max(e.vertex0, e.vertex1)};
    };
    auto less = [&](std::size_t a, std::size_t z) {
        work.charge(1);
        return key(edges[a]) < key(edges[z]);
    };
    NativeAttributeSort sorter(order, less);
    sorter.sort(0, order.size(), order.size());
    grow(edges.size());
    std::vector<NativePolyfaceHalfEdge> sorted;
    sorted.reserve(edges.size());
    for (auto i : order)
        sorted.push_back(edges[i]);
    edges = std::move(sorted);
    std::size_t boundary_edges = 0, paired_edges = 0, special_groups = 0, same_direction_pairs = 0;
    if (!edges.empty()) {
        std::size_t max_read = 0;
        for (const auto &e : edges) {
            work.charge(1);
            max_read = std::max({max_read, e.read_index, e.successor_read_index});
        }
        require(max_read < static_cast<std::size_t>(INT32_MAX),
                "native connectivity read label overflow");
        grow(max_read + 1);
        out.read_to_half_edge.assign(max_read + 1, SIZE_MAX);
        for (std::size_t i = 0; i < edges.size(); ++i)
            out.read_to_half_edge[edges[i].read_index] = i;
        auto make_edge = [&](std::size_t a, std::size_t z, bool primary_a, bool primary_z,
                             std::int32_t read_a, std::int32_t read_z, bool boundary) {
            grow(2);
            require(nodes.size() <= static_cast<std::size_t>(INT32_MAX) - 2,
                    "native connectivity node label overflow");
            const auto n = nodes.size();
            nodes.push_back({n, n + 1, primary_a ? native_mtg_primary : 0,
                             static_cast<std::int32_t>(a - 1), read_a});
            nodes.push_back({n + 1, n, primary_z ? native_mtg_primary : 0,
                             static_cast<std::int32_t>(z - 1), read_z});
            if (boundary) {
                nodes[n].mask |= native_mtg_boundary;
                nodes[n + 1].mask |= native_mtg_boundary | native_mtg_exterior;
                if (a == z)
                    nodes[n + 1].mask |= native_mtg_polar_loop;
                ++boundary_edges;
            }
            return n;
        };
        auto pair = [&](std::size_t a, std::size_t z) {
            auto &x = edges[a];
            auto &y = edges[z];
            const auto n = make_edge(x.vertex0, y.vertex0, x.visible, y.visible,
                                     static_cast<std::int32_t>(x.read_index),
                                     static_cast<std::int32_t>(y.read_index), false);
            x.node = n;
            y.node = n + 1;
            ++paired_edges;
            if (x.vertex0 != y.vertex1 || x.vertex1 != y.vertex0)
                ++same_direction_pairs;
        };
        auto successor = [&](std::size_t i) {
            work.charge(1);
            return out.read_to_half_edge[edges[i].successor_read_index];
        };
        for (std::size_t first = 0; first < edges.size();) {
            std::size_t last = first + 1;
            while (last < edges.size() && key(edges[last]) == key(edges[first])) {
                work.charge(1);
                ++last;
            }
            bool coupled = false;
            if (last - first == 2) {
                pair(first, first + 1);
                coupled = true;
            } else if (last - first == 4) {
                auto partner = [&](std::size_t a, std::size_t excluded) {
                    for (std::size_t i = first; i < last; ++i) {
                        work.charge(1);
                        if (i != a && i != excluded && edges[i].vertex0 == edges[a].vertex1 &&
                            edges[i].vertex1 == edges[a].vertex0)
                            return i;
                    }
                    return SIZE_MAX;
                };
                for (std::size_t a = first; a < last && !coupled; ++a) {
                    const auto z = successor(a);
                    if (z == SIZE_MAX)
                        continue;
                    const auto c = successor(z);
                    if (c == SIZE_MAX)
                        continue;
                    const auto d = successor(c);
                    if (edges[z].vertex0 == edges[z].vertex1 && c >= first && c < last && d == a) {
                        const auto pa = partner(a, c), pc = partner(c, a);
                        if (pa != SIZE_MAX && pc != SIZE_MAX && pa != pc) {
                            pair(a, pa);
                            pair(c, pc);
                            coupled = true;
                            ++special_groups;
                        }
                    }
                }
            }
            if (!coupled)
                for (std::size_t i = first; i < last; ++i) {
                    auto &e = edges[i];
                    e.node = make_edge(e.vertex0, e.vertex1, true, true,
                                       static_cast<std::int32_t>(e.read_index), -1, true);
                }
            first = last;
        }
        auto twist = [&](std::size_t a, std::size_t z) {
            work.charge(8);
            // V-F-V reaches the face predecessor in the original MTG node chain.
            const auto pa = nodes[nodes[nodes[a].vertex_successor].face_successor].vertex_successor;
            const auto pz = nodes[nodes[nodes[z].vertex_successor].face_successor].vertex_successor;
            std::swap(nodes[a].vertex_successor, nodes[z].vertex_successor);
            std::swap(nodes[pa].face_successor, nodes[pz].face_successor);
        };
        for (std::size_t i = 0; i < edges.size(); ++i) {
            const auto j = successor(i);
            require(j < edges.size(), "native connectivity missing face successor");
            twist(nodes[edges[i].node].face_successor, edges[j].node);
        }
        out.native_succeeded = true;
    }
    std::size_t label_mismatches = 0;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        work.charge(1);
        const auto &n = nodes[i];
        require(n.face_successor < nodes.size() && n.vertex_successor < nodes.size(),
                "native connectivity broken successor");
        require(nodes[n.face_successor].vertex_successor == (i ^ 1),
                "native connectivity broken edge mate");
        if (n.vertex_index != nodes[n.vertex_successor].vertex_index)
            ++label_mismatches;
    }
    out.complete = out.native_succeeded && !label_mismatches;
    out.report = {{"operation", "native_polyface_half_edge_assembly"},
                  {"native_succeeded", out.native_succeeded},
                  {"complete", out.complete},
                  {"paired_edges", paired_edges},
                  {"boundary_edges", boundary_edges},
                  {"four_edge_degenerate_couplings", special_groups},
                  {"same_direction_pairs", same_direction_pairs},
                  {"vertex_label_mismatches", label_mismatches},
                  {"vertex_label_tag", -1},
                  {"read_index_label_tag", -10004}};
    out.points = points;
    out.half_edges = std::move(edges);
    return out;
}
NativePolyfaceConnectivity build_native_polyface_mesh_connectivity(const NativePolyfaceMesh &mesh,
                                                                   bool ignore_degeneracies,
                                                                   TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    require(mesh.data.coordinates.points.size() <= std::size_t(INT32_MAX),
            "native connectivity vertex label overflow");
    std::vector<NativePolyfaceHalfEdge> edges;
    std::size_t filtered = 0, skipped = 0, corners = 0;
    const auto visited = consume_native_polyface(
        mesh, budget,
        [&](std::size_t, const NativePolyfaceVisitorFacet &f, const NativePolyfaceVisitedData &d) {
            const auto count = f.points.size();
            corners += count;
            require(count <= budget.max_control_points - edges.size(),
                    "native typed connectivity temporary corner budget");
            const auto &ids = f.client_indices[point_channel];
            require(ids.size() >= count && f.visible.size() >= count,
                    "native connectivity incomplete point visitor");
            std::vector<std::size_t> pack;
            std::size_t end = count;
            if (ignore_degeneracies && count) {
                while (end > 1 && ids[end - 1] == ids[0]) {
                    work.charge(1);
                    --end;
                }
                pack.push_back(0);
                for (std::size_t i = 1; i < end; ++i) {
                    work.charge(1);
                    if (ids[i] != ids[pack.back()])
                        pack.push_back(i);
                }
                filtered += count - pack.size();
                if (pack.size() == 2) {
                    filtered += 2;
                    ++skipped;
                    return;
                }
            } else {
                for (std::size_t i = 0; i < count; ++i) {
                    work.charge(1);
                    pack.push_back(i);
                }
            }
            require(pack.size() <= budget.max_control_points - edges.size(),
                    "native typed connectivity half-edge budget");
            for (std::size_t i = 0; i < pack.size(); ++i) {
                work.charge(1);
                const auto a = pack[i], b = pack[(i + 1) % pack.size()];
                require(a < d.index_positions.size() && b < d.index_positions.size(),
                        "native connectivity reads missing visitor index position");
                require(ids[a] >= 0 && ids[b] >= 0, "native connectivity negative client identity");
                edges.push_back({std::size_t(ids[a]) + 1, std::size_t(ids[b]) + 1,
                                 d.index_positions[a], d.index_positions[b], 0,
                                 bool(f.visible[a])});
            }
        });
    auto out =
        assemble_native_polyface_half_edges(mesh.data.coordinates.points, std::move(edges), budget);
    out.complete = out.complete && visited.complete;
    out.report["operation"] = "native_typed_polyface_to_mtg_connectivity";
    out.report["mesh_style"] = mesh.mesh_style;
    out.report["complete"] = out.complete;
    out.report["ignore_degeneracies"] = ignore_degeneracies;
    out.report["filtered_corners"] = filtered;
    out.report["skipped_degenerate_faces"] = skipped;
    out.report["visited_corners"] = corners;
    out.report["visitor"] = visited.report;
    return out;
}
} // namespace p3d::swept_detail
