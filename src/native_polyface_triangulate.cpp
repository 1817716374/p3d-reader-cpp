#include "native_polyface_triangulate.hpp"
// Adapted from Bentley imodel-native Polyface.cpp.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// P3D channel order, missing-normal/color distinction and partial publication.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_bezier.hpp"
namespace p3d::swept_detail {
NativePolyfaceTriangulation
triangulate_native_polyface_facets(const std::vector<NativePolyfaceVisitorFacet> &facets,
                                   const NativePolyfaceIndexState &original, TubeBudget &budget,
                                   std::size_t max_edges_per_face) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    require(facets.size() <= budget.max_control_points, "native polyface facet budget");
    std::size_t input_count = 0;
    auto extent = [&](std::size_t n) {
        require(n <= budget.max_control_points - input_count,
                "native polyface input storage budget");
        input_count += n;
        work.charge(n);
    };
    for (const auto &a : original.indices)
        extent(a.size());
    for (const auto &f : facets) {
        extent(f.points.size());
        extent(f.visible.size());
        for (const auto &a : f.client_indices)
            extent(a.size());
    }
    NativePolyfaceTriangulation out;
    out.output = original;
    std::array<std::vector<std::int32_t>, polyface_channel_count> generated;
    std::size_t output_count = 0, errors = 0, failed_faces = 0, normal_drops = 0, color_drops = 0,
                nontriangular = 0, retained_polygons = 0;
    Json face_reports = Json::array();
    bool all_output_triangular = true;
    auto append = [&](std::size_t channel, std::int32_t index) {
        work.charge(1);
        require(output_count < budget.max_control_points, "native polyface output index budget");
        ++output_count;
        generated[channel].push_back(index);
    };
    for (std::size_t fi = 0; fi < facets.size(); ++fi) {
        const auto &f = facets[fi];
        auto plan = native_facet_index_plan(f.points, budget, max_edges_per_face);
        if (plan.route == NativeFacetIndexPlan::Route::passthrough && f.points.size() > 3)
            ++retained_polygons;
        Json status{{"source_facet", fi},
                    {"native_succeeded", plan.native_succeeded},
                    {"triangulation_completed", plan.completed}};
        if (plan.input_graph)
            status["source_index_output"] = plan.input_graph->report.at("source_index_output");
        face_reports.push_back(std::move(status));
        if (!plan.native_succeeded) {
            ++failed_faces;
            ++errors;
            continue;
        }
        if (!plan.completed ||
            (plan.route == NativeFacetIndexPlan::Route::passthrough && f.points.size() != 3))
            all_output_triangular = false;
        if (!plan.completed)
            ++nontriangular;
        const auto &local = plan.completed ? plan.indices : plan.incomplete_indices;
        for (auto index : local) {
            work.charge(1);
            if (index == 0) {
                for (std::size_t c = 0; c < polyface_channel_count; ++c)
                    if (out.output.active[c])
                        append(c, 0);
                continue;
            }
            require(index != INT32_MIN, "native polyface local index magnitude");
            const auto k = std::size_t(std::abs(index) - 1);
            auto mapped = [&](std::size_t channel) {
                require(k < f.client_indices[channel].size(),
                        "native polyface visitor channel index");
                const auto v = f.client_indices[channel][k];
                require(v >= 0 && v < INT32_MAX, "native polyface one-based index range");
                return std::int32_t(v + 1);
            };
            auto point = mapped(point_channel);
            // Original visibility is read only for positive local edges.
            if (index < 0)
                point = -point;
            else {
                require(k < f.visible.size(), "native polyface visitor visibility");
                if (!f.visible[k])
                    point = -point;
            }
            if (out.output.active[point_channel])
                append(point_channel, point);
            if (out.output.active[parameter_channel])
                append(parameter_channel, mapped(parameter_channel));
            if (out.output.active[normal_channel]) {
                if (k < f.client_indices[normal_channel].size())
                    append(normal_channel, mapped(normal_channel));
                else {
                    out.output.active[normal_channel] = false;
                    out.output.indices[normal_channel].clear();
                    ++normal_drops;
                }
            }
            if (out.output.active[color_channel]) {
                if (k < f.client_indices[color_channel].size())
                    append(color_channel, mapped(color_channel));
                else {
                    out.output.active[color_channel] = false;
                    out.output.indices[color_channel].clear();
                    ++color_drops;
                    ++errors;
                }
            }
            if (out.output.active[face_channel])
                append(face_channel, mapped(face_channel));
        }
    }
    // Point indices are replaced unconditionally, even when inactive. Other
    // inactive channels retain originals (or their explicit cleared state).
    out.output.indices[point_channel] = std::move(generated[point_channel]);
    for (std::size_t c = 1; c < polyface_channel_count; ++c)
        if (out.output.active[c])
            out.output.indices[c] = std::move(generated[c]);
    out.native_succeeded = errors == 0;
    out.complete = out.native_succeeded && normal_drops == 0 && nontriangular == 0;
    out.report = {{"scope", "native_prepared_polyface_visitor_triangulation"},
                  {"native_succeeded", out.native_succeeded},
                  {"complete", out.complete},
                  {"source_facets", facets.size()},
                  {"failed_facets", failed_faces},
                  {"normal_channel_drops", normal_drops},
                  {"color_channel_drops", color_drops},
                  {"nontriangular_facets", nontriangular},
                  {"retained_polygon_facets", retained_polygons},
                  {"max_edges_per_face", std::max(std::size_t(3), max_edges_per_face)},
                  {"all_output_faces_triangular", all_output_triangular},
                  {"native_error_count", errors},
                  {"facet_results", std::move(face_reports)},
                  {"active_channels", out.output.active},
                  {"generated_index_count", output_count},
                  {"num_per_face", 0},
                  {"mesh_style", 1}};
    return out;
}
} // namespace p3d::swept_detail
