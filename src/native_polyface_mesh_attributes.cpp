// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from polyfaceAddNormals.cpp with original P3D visitor ordering,
// index-position writes and direct stored face-range indices.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_polyface_mesh_attributes.hpp"
#include "native_polygon_projection.hpp"
#include "native_polyface_smooth_normals.hpp"
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"

namespace p3d::swept_detail {
namespace {
NativePolyfaceMeshAttributes build(const NativePolyfaceMesh &input, bool parameters, int selector,
                                   TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    std::size_t storage = 0;
    auto grow = [&](std::size_t n) {
        require(n <= budget.max_control_points - storage, "native typed attribute storage budget");
        storage += n;
        work.charge(n);
    };
    for (auto n :
         {input.data.coordinates.points.size(), input.data.coordinates.normals.size(),
          input.data.coordinates.parameters.size(), input.data.face_data.size(),
          input.double_colors.size(), input.float_colors.size(), input.integer_colors.size(),
          input.color_table.size(), input.face_uv_points.size(), input.face_material_ids.size(),
          input.face_smooth_groups.size(), input.illumination_name.size(),
          input.data.edge_chains.size()})
        grow(n);
    for (const auto &indices : input.data.indices.indices)
        grow(indices.size());
    for (const auto &edge : input.data.edge_chains) {
        grow(edge.topology_ids.size());
        grow(edge.point_indices.size());
    }
    NativePolyfaceMeshAttributes out;
    out.output = input;
    auto &mesh = out.output;
    auto &coordinates = mesh.data.coordinates;
    const auto &points = mesh.data.indices.indices[point_channel];
    const auto channel = parameters ? parameter_channel : normal_channel;
    auto &indices = mesh.data.indices.indices[channel];
    out.report = {{"operation", parameters ? "native_typed_per_face_parameters"
                                           : "native_typed_per_face_normals"},
                  {"mesh_style", mesh.mesh_style},
                  {"coordinate_selector", parameters ? selector : 0},
                  {"visitor_wrap_count", 0}};
    if (points.empty()) {
        out.report["reason"] = "empty_point_index_array";
        out.report["native_succeeded"] = false;
        out.report["complete"] = false;
        return out;
    }
    require(points.size() < UINT32_MAX, "native typed attribute index extent");
    // Native constructs the visitor before clearing the selected arrays. The
    // visitor keeps a query reference; pool counts are queried on each advance.
    require(mesh.mesh_style == 1 || mesh.mesh_style == 3 || mesh.mesh_style == 4 ||
                mesh.mesh_style == 5 || mesh.mesh_style == 6,
            "native typed attribute unsupported visitor style");
    storage -= indices.size();
    storage -= parameters ? coordinates.parameters.size() : coordinates.normals.size();
    grow(points.size());
    indices.assign(points.size(), 0);
    mesh.data.indices.active[channel] = true;
    if (parameters) {
        coordinates.parameters.clear();
        mesh.pool_active[native_parameter_pool] = true;
        for (auto &face : mesh.data.face_data) {
            work.charge(1);
            face.parameter_range[0].fill(std::numeric_limits<double>::max());
            face.parameter_range[1].fill(-std::numeric_limits<double>::max());
        }
    } else {
        coordinates.normals.clear();
        mesh.pool_active[native_normal_pool] = true;
    }
    std::size_t failed_frames = 0, skipped_writes = 0, range_updates = 0, range_skipped = 0;
    Json frames = Json::array();
    const auto visited = visit_native_polyface_attribute_updates(
        mesh, budget, channel,
        [&](std::size_t read, const NativePolyfaceVisitorFacet &f,
            const NativePolyfaceVisitedData &d) {
            auto frame = prepare_native_polygon_projection(f.points, budget,
                                                           parameters ? selector : 0, parameters);
            frames.push_back({{"read_index", read}, {"frame", std::move(frame.report)}});
            failed_frames += !frame.frame_succeeded;
            if (!parameters && !frame.frame_succeeded)
                return;
            if (!parameters) {
                grow(1);
                require(coordinates.normals.size() < std::size_t(INT32_MAX),
                        "native typed normal index overflow");
                coordinates.normals.push_back({frame.local_to_world[0][2],
                                               frame.local_to_world[1][2],
                                               frame.local_to_world[2][2]});
            }
            for (std::size_t i = 0; i < f.points.size(); ++i) {
                work.charge(1);
                Point2 uv{};
                if (parameters) {
                    grow(1);
                    require(coordinates.parameters.size() < std::size_t(INT32_MAX),
                            "native typed parameter index overflow");
                    if (frame.frame_succeeded)
                        uv = {frame.points[i][0], frame.points[i][1]};
                    coordinates.parameters.push_back(uv);
                }
                // Native reads IndexPosition first, then bounds-checks the
                // resulting destination index. Grid visitors have no positions.
                require(i < d.index_positions.size(),
                        "native attribute reads missing visitor index position");
                const auto position = d.index_positions[i];
                if (position < indices.size())
                    indices[position] = static_cast<std::int32_t>(
                        parameters ? coordinates.parameters.size() : coordinates.normals.size());
                else
                    ++skipped_writes;
                const auto &face_indices = mesh.data.indices.indices[face_channel];
                if (parameters && mesh.data.indices.active[face_channel] &&
                    position < face_indices.size()) {
                    const auto key = face_indices[position];
                    if (key >= 0 && std::size_t(key) < mesh.data.face_data.size()) {
                        auto &range = mesh.data.face_data[std::size_t(key)].parameter_range;
                        for (std::size_t k = 0; k < 2; ++k) {
                            const auto x = curve_detail::bezier_support::finite(uv[k]);
                            if (x < range[0][k])
                                range[0][k] = x;
                            if (x > range[1][k])
                                range[1][k] = x;
                        }
                        ++range_updates;
                    } else
                        ++range_skipped;
                }
            }
        });
    std::size_t missing = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        work.charge(1);
        missing += points[i] != 0 && indices[i] == 0;
    }
    out.native_succeeded = true;
    out.complete =
        visited.complete && !missing && !failed_frames && !skipped_writes && !range_skipped;
    out.report["native_succeeded"] = out.native_succeeded;
    out.report["complete"] = out.complete;
    out.report["visitor"] = visited.report;
    out.report["frames"] = std::move(frames);
    out.report["failed_frames"] = failed_frames;
    out.report["skipped_index_writes"] = skipped_writes;
    out.report["unassigned_nonzero_indices"] = missing;
    if (parameters) {
        out.report["face_range_indexing"] = "native_direct_stored_index";
        out.report["face_range_updates"] = range_updates;
        out.report["face_range_updates_skipped"] = range_skipped;
    }
    return out;
}
} // namespace
NativePolyfaceMeshAttributes build_native_polyface_mesh_normals(const NativePolyfaceMesh &input,
                                                                TubeBudget &budget) {
    return build(input, false, 0, budget);
}
NativePolyfaceMeshAttributes build_native_polyface_mesh_parameters(const NativePolyfaceMesh &input,
                                                                   int selector,
                                                                   TubeBudget &budget) {
    return build(input, true, selector, budget);
}
NativePolyfaceMeshAttributes
build_native_polyface_mesh_approximate_normals(const NativePolyfaceMesh &input, double max_single,
                                               double max_accumulated,
                                               bool mark_transitions_visible, TubeBudget &budget) {
    curve_detail::bezier_support::finite(max_single);
    curve_detail::bezier_support::finite(max_accumulated);
    auto out = build_native_polyface_mesh_normals(input, budget);
    NativePolyfaceConnectivity graph;
    if (out.native_succeeded)
        graph = build_native_polyface_mesh_connectivity(out.output, false, budget);
    NativePolyfaceAttributes prepared;
    prepared.native_succeeded = out.native_succeeded;
    prepared.complete = out.complete;
    prepared.report = std::move(out.report);
    prepared.output.mesh = std::move(out.output.data);
    prepared.output.normal_pool_active = out.output.pool_active[native_normal_pool];
    prepared.output.parameter_pool_active = out.output.pool_active[native_parameter_pool];
    prepared.output.face_data_pool_active = out.output.pool_active[native_face_data_pool];
    auto smoothed =
        smooth_native_polyface_prepared_normals(std::move(prepared), std::move(graph), max_single,
                                                max_accumulated, mark_transitions_visible, budget);
    out.output.data = std::move(smoothed.output.mesh);
    out.output.pool_active[native_normal_pool] = smoothed.output.normal_pool_active;
    out.native_succeeded = smoothed.native_succeeded;
    out.complete = smoothed.complete;
    out.report = std::move(smoothed.report);
    out.report["operation"] = "native_typed_build_approximate_normals";
    out.report["mesh_style"] = out.output.mesh_style;
    return out;
}
} // namespace p3d::swept_detail
