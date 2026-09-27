// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from PolyfaceEdgeChain.cpp with original P3D two-pass visitation,
// topology IDs, facet numbering and single-edge emission. See THIRD_PARTY.md.
#include "native_polyface_edge_chains.hpp"
#include "native_bezier.hpp"
#include <map>
namespace p3d::swept_detail {
NativePolyfaceMeshEdgeChains build_native_polyface_mesh_edge_chains(const NativePolyfaceMesh &input,
                                                                    std::size_t draw_method,
                                                                    TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    std::size_t storage = 0;
    auto grow = [&](std::size_t n) {
        require(n <= budget.max_control_points - storage, "native typed edge-chain storage budget");
        storage += n;
        work.charge(n);
    };
    for (auto n : {input.data.coordinates.points.size(), input.data.coordinates.normals.size(),
                   input.data.coordinates.parameters.size(), input.data.face_data.size(),
                   input.data.edge_chains.size(), input.double_colors.size(),
                   input.float_colors.size(), input.integer_colors.size(), input.color_table.size(),
                   input.face_uv_points.size(), input.face_material_ids.size(),
                   input.face_smooth_groups.size(), input.illumination_name.size()})
        grow(n);
    for (const auto &i : input.data.indices.indices)
        grow(i.size());
    for (const auto &e : input.data.edge_chains) {
        grow(e.topology_ids.size());
        grow(e.point_indices.size());
    }
    NativePolyfaceMeshEdgeChains out;
    out.output = input;
    out.report = {{"operation", "native_typed_build_edge_chains"},
                  {"mesh_style", input.mesh_style},
                  {"draw_method_index", draw_method},
                  {"draw_method_used", false},
                  {"visitor_wrap_count", 1}};
    if (!input.data.edge_chains.empty()) {
        out.report["reason"] = "edge_chains_already_exist";
        out.report["native_status"] = 1;
        out.report["native_succeeded"] = false;
        out.report["complete"] = false;
        return out;
    }
    using Key = std::pair<std::size_t, std::size_t>;
    struct Faces {
        std::size_t first, last, occurrences;
    };
    auto less = [&](const Key &a, const Key &b) {
        work.charge(1);
        return a < b;
    };
    std::map<Key, Faces, decltype(less)> registered(less);
    auto key = [](std::size_t a, std::size_t b) { return a < b ? Key{a, b} : Key{b, a}; };
    auto vertex = [&](const NativePolyfaceVisitorFacet &f, std::size_t i) {
        work.charge(1);
        const auto &ids = f.client_indices[point_channel];
        require(i < ids.size(), "native edge-chain reads missing wrapped client point");
        require(ids[i] >= 0 && ids[i] < INT32_MAX, "native edge-chain client index overflow");
        return std::size_t(ids[i]);
    };
    std::size_t qualified[2]{}, skipped[2]{}, facet_id = 0;
    std::size_t visible = 0, unique = 0, shared = 0, boundary = 0, over_two = 0;
    const auto first = consume_native_polyface(
        input, budget,
        [&](std::size_t, const NativePolyfaceVisitorFacet &f, const NativePolyfaceVisitedData &d) {
            if (d.edge_count < 3) {
                ++skipped[0];
                return;
            }
            ++qualified[0];
            ++facet_id;
            require(f.visible.size() >= d.edge_count, "native edge-chain missing visibility");
            for (std::size_t i = 0; i < d.edge_count; ++i) {
                work.charge(1);
                if (!f.visible[i])
                    continue;
                ++visible;
                const auto k = key(vertex(f, i), vertex(f, i + 1));
                auto at = registered.find(k);
                if (at == registered.end()) {
                    grow(1);
                    registered.emplace(k, Faces{facet_id, facet_id, 1});
                    ++unique;
                } else {
                    auto &v = at->second;
                    ++v.occurrences;
                    if (facet_id < v.first) {
                        v.last = v.first;
                        v.first = facet_id;
                    } else
                        v.last = facet_id;
                }
            }
            ++facet_id;
        },
        true, 1);
    Json emitted = Json::array();
    const auto second = consume_native_polyface(
        input, budget,
        [&](std::size_t read, const NativePolyfaceVisitorFacet &f,
            const NativePolyfaceVisitedData &d) {
            if (d.edge_count < 3) {
                ++skipped[1];
                return;
            }
            ++qualified[1];
            for (std::size_t i = 0; i < d.edge_count; ++i) {
                const auto a = vertex(f, i), b = vertex(f, i + 1);
                auto at = registered.find(key(a, b));
                if (at == registered.end())
                    continue;
                const auto &v = at->second;
                NativeBuilderEdgeChain e;
                if (v.first == v.last) {
                    e.topology_type = 24;
                    e.topology_ids = {std::uint32_t(a), std::uint32_t(b)};
                    ++boundary;
                } else {
                    e.topology_type = 4;
                    e.topology_ids = {std::uint32_t(v.first), std::uint32_t(v.last)};
                    ++shared;
                }
                e.point_indices = {std::int32_t(a + 1), std::int32_t(b + 1)};
                over_two += v.occurrences > 2;
                grow(5);
                out.output.data.edge_chains.push_back(std::move(e));
                require(i < f.visible.size(), "native edge-chain missing second-pass visibility");
                emitted.push_back(
                    {{"read_index",
                      i < d.index_positions.size() ? Json(d.index_positions[i]) : Json(nullptr)},
                     {"source_read_index", read},
                     {"corner_index", i},
                     {"visible_occurrences", v.occurrences},
                     {"first_visible_facet", v.first},
                     {"last_visible_facet", v.last},
                     {"emitted_from_hidden_half_edge", !f.visible[i]}});
                registered.erase(at);
                --storage;
            }
        },
        true, 1);
    Json passes = Json::array();
    for (unsigned pass = 0; pass < 2; ++pass) {
        const auto &v = pass ? second : first;
        Json r = {{"visited_facets", v.read_indices.size()},
                  {"qualified_facets", qualified[pass]},
                  {"skipped_short_facets", skipped[pass]},
                  {"visitor", v.report}};
        if (input.mesh_style == 1) {
            r["unvisited_nonzero_indices"] = v.report["source_corners"].get<std::size_t>() -
                                             v.report["covered_point_corners"].get<std::size_t>();
            r["invalid_point_facets"] = v.report["invalid_point_facets"];
            std::size_t truncated = 0;
            for (const auto &n : v.report["truncated_attribute_facets"])
                truncated += n.get<std::size_t>();
            r["truncated_attribute_visits"] = truncated;
        }
        passes.push_back(std::move(r));
    }
    out.native_status = 0;
    out.native_succeeded = true;
    out.complete = first.complete && second.complete && registered.empty();
    out.report["native_status"] = 0;
    out.report["native_succeeded"] = true;
    out.report["complete"] = out.complete;
    out.report["reason"] = "completed";
    out.report["passes"] = std::move(passes);
    out.report["emitted"] = std::move(emitted);
    out.report["visible_half_edge_occurrences"] = visible;
    out.report["registered_edges"] = unique;
    out.report["unemitted_edges"] = registered.size();
    out.report["shared_edge_records"] = shared;
    out.report["vertex_edge_records"] = boundary;
    out.report["edges_with_over_two_visible_occurrences"] = over_two;
    // Native pushes records without activating their vector.
    return out;
}
} // namespace p3d::swept_detail
