// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from PolyfaceVisitor.cpp and BlockedVector.cpp. P3D independent
// color count/pointer selection, wrap behavior and index limits are preserved.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_polyface_visitor.hpp"
#include "native_bezier.hpp"

namespace p3d::swept_detail {
namespace {
std::array<std::size_t, native_polyface_pool_count> pool_counts(const NativePolyfaceMesh &m) {
    return {m.data.coordinates.points.size(),
            m.data.coordinates.parameters.size(),
            m.data.coordinates.normals.size(),
            m.double_colors.size(),
            m.float_colors.size(),
            m.integer_colors.size(),
            m.color_table.size(),
            m.data.face_data.size()};
}
struct Context {
    TubeBudget &budget;
    curve_detail::BezierWork work;
    std::size_t storage = 0;
    Context(const NativePolyfaceMesh &m, TubeBudget &b) : budget(b), work{b.work, b.max_work} {
        for (auto n : pool_counts(m))
            grow(n);
        for (const auto &v : m.data.indices.indices)
            grow(v.size());
        grow(m.face_uv_points.size());
        grow(m.face_material_ids.size());
        grow(m.face_smooth_groups.size());
        grow(m.illumination_name.size());
        grow(m.data.edge_chains.size());
        for (const auto &e : m.data.edge_chains) {
            grow(e.topology_ids.size());
            grow(e.point_indices.size());
        }
    }
    void grow(std::size_t n) {
        require(n <= budget.max_control_points - storage, "native visitor storage budget");
        storage += n;
        work.charge(n);
    }
};
NativePolyfaceLayout layout(const NativePolyfaceMesh &m) {
    NativePolyfaceLayout out;
    const auto counts = pool_counts(m);
    for (std::size_t i = 0; i < counts.size(); ++i)
        out.pools[i] = {counts[i], m.pool_rows[i], m.pool_active[i]};
    out.indices = m.data.indices;
    out.index_structs_per_row = m.index_rows;
    out.mesh_style = m.mesh_style;
    out.num_per_face = m.data.num_per_face;
    out.num_per_row = m.num_per_row;
    return out;
}
void apply_layout(NativePolyfaceMesh &m, NativePolyfaceLayout &&l) {
    m.data.indices = std::move(l.indices);
    m.data.num_per_face = l.num_per_face;
    m.mesh_style = l.mesh_style;
    m.num_per_row = l.num_per_row;
    m.index_rows = l.index_structs_per_row;
    for (std::size_t i = 0; i < l.pools.size(); ++i) {
        m.pool_rows[i] = l.pools[i].structs_per_row;
        m.pool_active[i] = l.pools[i].active;
    }
}
} // namespace
NativePolyfaceVisit visit_native_polyface(const NativePolyfaceMesh &mesh, TubeBudget &budget,
                                          bool all_data, std::uint32_t wrap) {
    Context context(mesh, budget);
    require(mesh.mesh_style == 1, "native indexed visitor requires converted mesh style");
    const auto &m = mesh.data;
    const auto &indices = m.indices.indices;
    const auto &points = m.coordinates.points;
    const auto &pi = indices[point_channel];
    require(pi.size() < UINT32_MAX, "native visitor cursor extent");
    require(wrap <= budget.max_control_points, "native visitor wrap budget");
    const auto counts = pool_counts(mesh);
    // Native color pointers depend on presence, while ColorCount consults
    // activity, with float/double in the opposite priority order.
    std::size_t color_pool = native_polyface_pool_count, color_count_pool = color_pool;
    for (auto p : {native_double_color_pool, native_float_color_pool, native_integer_color_pool,
                   native_color_table_pool})
        if (counts[p]) {
            color_pool = p;
            break;
        }
    for (auto p : {native_float_color_pool, native_double_color_pool, native_integer_color_pool,
                   native_color_table_pool})
        if (mesh.pool_active[p]) {
            color_count_pool = p;
            break;
        }
    const auto color_count =
        color_count_pool == native_polyface_pool_count ? 0 : counts[color_count_pool];
    NativePolyfaceVisit out;
    std::size_t cursor = 0, covered = 0, expected = 0, invalid_points = 0, incomplete_channels = 0;
    std::array<std::size_t, polyface_channel_count> truncated{};
    Json reports = Json::array();
    for (auto i : pi) {
        context.work.charge(1);
        expected += i != 0;
    }
    while (cursor < pi.size()) {
        context.work.charge(1);
        std::size_t first = cursor, count = 0, next = 0;
        if (m.num_per_face > 1) {
            while (count < m.num_per_face && first + count < pi.size() && pi[first + count]) {
                context.work.charge(1);
                ++count;
            }
            require(m.num_per_face <= UINT32_MAX - first, "native visitor next cursor overflow");
            next = first + m.num_per_face;
        } else {
            while (first < pi.size() && !pi[first]) {
                context.work.charge(1);
                ++first;
            }
            while (first + count < pi.size() && pi[first + count]) {
                context.work.charge(1);
                ++count;
            }
            next = first + count + 1;
        }
        if (!count)
            break;
        NativePolyfaceVisitorFacet f;
        std::array<std::size_t, polyface_channel_count> collected{};
        auto collect = [&](std::size_t channel, const std::vector<std::int32_t> &source,
                           std::size_t native_index_count, std::size_t native_pool_count,
                           std::size_t actual_pool_count) {
            auto &target = f.client_indices[channel];
            if (source.empty())
                return;
            for (std::size_t i = 0; i < count && first + i < native_index_count; ++i) {
                context.work.charge(1);
                require(first + i < source.size(),
                        "native visitor attribute index read outside source");
                const auto signed_index = static_cast<std::int64_t>(source[first + i]);
                if (!signed_index)
                    break;
                const auto k = std::size_t(signed_index < 0 ? -signed_index : signed_index) - 1;
                if (k >= native_pool_count)
                    break;
                require(k < actual_pool_count, "native visitor pool query exceeds selected array");
                require(k <= INT32_MAX, "native visitor client index extent");
                context.grow(1);
                target.push_back(static_cast<std::int32_t>(k));
            }
            collected[channel] = target.size();
            if (!target.empty()) {
                context.grow(wrap);
                // Reads from the growing output: wrap may exceed the base size.
                for (std::size_t i = 0; i < wrap; ++i) {
                    const auto v = target[i];
                    target.push_back(v);
                }
            }
        };
        collect(point_channel, pi, pi.size(), points.size(), points.size());
        if (collected[point_channel] != count)
            ++invalid_points;
        // The original checks after appending the wrap. A missing final point
        // can therefore pass; preserve the prefix but diagnose incomplete input.
        if (f.client_indices[point_channel].size() < count)
            break;
        context.grow(collected[point_channel]);
        for (std::size_t i = 0; i < collected[point_channel]; ++i)
            f.points.push_back(points[std::size_t(f.client_indices[point_channel][i])]);
        context.grow(count);
        context.grow(wrap);
        for (std::size_t i = 0; i < count; ++i)
            f.visible.push_back(pi[first + i] > 0);
        for (std::size_t i = 0; i < wrap; ++i)
            f.visible.push_back(pi[first + i % count] > 0);
        if (all_data) {
            for (auto channel : {normal_channel, parameter_channel}) {
                const auto pool =
                    channel == normal_channel ? native_normal_pool : native_parameter_pool;
                const auto &src = indices[channel].empty() && counts[pool] == points.size()
                                      ? pi
                                      : indices[channel];
                collect(channel, src, pi.size(), counts[pool], counts[pool]);
            }
            collect(face_channel, indices[face_channel], indices[face_channel].size(),
                    counts[native_face_data_pool], counts[native_face_data_pool]);
            if (color_pool != native_polyface_pool_count)
                collect(color_channel, indices[color_channel], pi.size(), color_count,
                        counts[color_pool]);
            for (auto channel : {parameter_channel, normal_channel, color_channel, face_channel}) {
                const auto pool_count = channel == parameter_channel ? counts[native_parameter_pool]
                                        : channel == normal_channel  ? counts[native_normal_pool]
                                        : channel == face_channel    ? counts[native_face_data_pool]
                                        : color_pool == native_polyface_pool_count
                                            ? 0
                                            : counts[color_pool];
                if ((pool_count || m.indices.active[channel] || !indices[channel].empty()) &&
                    collected[channel] < count) {
                    ++truncated[channel];
                    ++incomplete_channels;
                }
            }
        }
        covered += collected[point_channel];
        context.grow(2); // facet and source-read records
        reports.push_back({{"read_index", first},
                           {"source_corner_count", count},
                           {"collected_corner_counts", collected}});
        out.read_indices.push_back(first);
        out.facets.push_back(std::move(f));
        cursor = next;
    }
    out.complete = covered == expected && invalid_points == 0 && incomplete_channels == 0;
    out.report = {{"scope", "native_indexed_polyface_visitor"},
                  {"complete", out.complete},
                  {"all_data", all_data},
                  {"wrap", wrap},
                  {"source_corners", expected},
                  {"covered_point_corners", covered},
                  {"invalid_point_facets", invalid_points},
                  {"truncated_attribute_facets", truncated},
                  {"color_value_pool", color_pool},
                  {"color_count_pool", color_count_pool},
                  {"color_query_count", color_count},
                  {"facet_results", std::move(reports)}};
    return out;
}
NativePolyfaceMeshTriangulation triangulate_native_polyface_mesh(const NativePolyfaceMesh &input,
                                                                 TubeBudget &budget,
                                                                 std::size_t max_edges) {
    Context context(input, budget); // Check all typed pools before copying anything.
    auto converted = convert_native_polyface_layout(layout(input), budget);
    require(converted.output.mesh_style == 1,
            "native triangulation cannot attach visitor for unsupported mesh style");
    auto activated = activate_native_polyface_layout(converted.output, budget);
    NativePolyfaceMeshTriangulation out;
    out.output = input;
    apply_layout(out.output, std::move(activated.output));
    auto visited = visit_native_polyface(out.output, budget, true, 1);
    auto triangulated = triangulate_native_polyface_facets(visited.facets, out.output.data.indices,
                                                           budget, max_edges);
    out.output.data.indices = std::move(triangulated.output);
    out.output.data.num_per_face = 0;
    out.output.mesh_style = 1;
    // Native writes a full qword at meshStyle, also clearing global numPerRow.
    out.output.num_per_row = 0;
    out.native_succeeded = triangulated.native_succeeded;
    out.complete = converted.complete && visited.complete && triangulated.complete;
    out.report = {{"scope", "native_typed_polyface_triangulation"},
                  {"native_succeeded", out.native_succeeded},
                  {"complete", out.complete},
                  {"face_extension_mapping_recomputed", false},
                  {"layout_conversion", std::move(converted.report)},
                  {"activation", std::move(activated.report)},
                  {"visitor", std::move(visited.report)},
                  {"triangulation", std::move(triangulated.report)}};
    return out;
}
} // namespace p3d::swept_detail
