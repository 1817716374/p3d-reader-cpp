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
NativePolyfaceVisit visit_unindexed(const NativePolyfaceMesh &mesh, TubeBudget &budget,
                                    bool all_data, std::uint32_t requested_wrap,
                                    const NativePolyfaceFacetConsumer *consumer,
                                    std::size_t ignored_channel) {
    Context context(mesh, budget);
    const bool blocked = mesh.mesh_style == 3 || mesh.mesh_style == 4;
    require(blocked || mesh.mesh_style == 5 || mesh.mesh_style == 6,
            "native visitor unsupported mesh style");
    require(requested_wrap <= budget.max_control_points && requested_wrap <= INT32_MAX,
            "native raw visitor wrap extent");
    const auto wrap = blocked ? requested_wrap : std::min(requested_wrap, std::uint32_t(10));
    const auto &points = mesh.data.coordinates.points;
    require(points.size() < INT32_MAX, "native raw visitor point index extent");
    const auto counts = pool_counts(mesh);
    std::size_t color_count = 0;
    for (auto p : {native_float_color_pool, native_double_color_pool, native_integer_color_pool,
                   native_color_table_pool})
        if (mesh.pool_active[p]) {
            color_count = counts[p];
            break;
        }
    if (!blocked)
        require(mesh.num_per_row != 1, "native grid visitor divides by zero at row width one");
    std::vector<std::uint8_t> used(points.size(), 0);
    context.grow(used.size());
    NativePolyfaceVisit out;
    std::size_t incomplete_attributes = 0;
    const bool ignored_float = !mesh.float_colors.empty() && (!blocked || all_data);
    const bool ignored_faces = blocked && all_data && !mesh.data.face_data.empty();
    const bool ignored_table =
        !blocked && !mesh.integer_colors.empty() && !mesh.color_table.empty();
    for (std::size_t ordinal = 0;; ++ordinal) {
        context.work.charge(1);
        const auto retained_storage = context.storage;
        require(ordinal < UINT32_MAX, "native raw visitor facet ordinal overflow");
        std::vector<std::size_t> indices;
        std::size_t last = 0, first = 0;
        if (blocked) {
            const auto count = mesh.mesh_style == 3 ? 3u : 4u;
            require(ordinal <= (UINT32_MAX - count) / count,
                    "native blocked visitor offset overflow");
            first = ordinal * count;
            if (first + count > points.size())
                break;
            for (std::size_t i = 0; i < count; ++i)
                indices.push_back(first + i);
            last = first + count - 1;
        } else {
            const auto quad = mesh.mesh_style == 5 ? ordinal >> 1 : ordinal;
            const auto width = std::size_t(mesh.num_per_row);
            // Width zero retains native unsigned width-1. Width one was rejected.
            first = quad + quad / (width - 1);
            const auto lower = first + width;
            last = lower + 1;
            // Original tests >, not >=; actual reads below still need bounds.
            if (last > points.size())
                break;
            indices.push_back(mesh.mesh_style == 5 && (ordinal & 1) ? lower : first);
            indices.push_back(first + 1);
            indices.push_back(mesh.mesh_style == 6 || (ordinal & 1) ? last : lower);
            if (mesh.mesh_style == 6)
                indices.push_back(lower);
        }
        const auto corners = indices.size();
        context.grow(corners);
        context.grow(wrap);
        for (std::size_t i = 0; i < wrap; ++i) {
            const auto k = indices[i];
            indices.push_back(k); // Growing-array wrap, including repeated cycles.
        }
        NativePolyfaceVisitorFacet f;
        NativePolyfaceVisitedData d;
        context.grow(corners);
        context.grow(indices.size());
        context.grow(indices.size());
        for (std::size_t i = 0; i < indices.size(); ++i) {
            const auto k = indices[i];
            require(k < points.size(), "native grid visitor point read outside source");
            f.client_indices[point_channel].push_back(static_cast<std::int32_t>(k));
            f.visible.push_back(1);
            if (i < corners) {
                f.points.push_back(points[k]);
                used[k] = 1;
            }
        }
        if (blocked) {
            context.grow(indices.size());
            for (std::size_t i = 0; i < indices.size(); ++i)
                d.index_positions.push_back(first + i); // Native does not wrap these positions.
            if (all_data) {
                auto values = [&](auto &dest, const auto &pool, std::size_t query_count,
                                  std::size_t channel) {
                    if (pool.empty() || !query_count)
                        return;
                    for (std::size_t i = 0; i < corners && first + i < query_count; ++i) {
                        require(first + i < pool.size(),
                                "native blocked visitor pool read outside source");
                        context.grow(1);
                        dest.push_back(pool[first + i]);
                    }
                    if (dest.size() != corners && channel != ignored_channel)
                        ++incomplete_attributes;
                    context.grow(wrap);
                    for (std::size_t i = 0; i < wrap; ++i) {
                        require(i < dest.size(),
                                "native blocked visitor wraps an empty attribute prefix");
                        const auto value = dest[i];
                        dest.push_back(value);
                    }
                };
                values(d.normals, mesh.data.coordinates.normals,
                       mesh.data.coordinates.normals.size(), normal_channel);
                values(d.parameters, mesh.data.coordinates.parameters,
                       mesh.data.coordinates.parameters.size(), parameter_channel);
                values(d.double_colors, mesh.double_colors, color_count, color_channel);
                values(d.integer_colors, mesh.integer_colors, color_count, color_channel);
                values(d.color_table, mesh.color_table, color_count, color_channel);
                if (!color_count && (!mesh.double_colors.empty() || !mesh.integer_colors.empty() ||
                                     !mesh.color_table.empty()))
                    ++incomplete_attributes;
                // Constructor activates only PointIndex. AddSequentialBlock
                // is activity-gated, so all four attribute index arrays stay empty.
            }
        } else {
            // No allData condition in this original grid advance implementation.
            auto values = [&](auto &dest, const auto &pool, std::size_t query_count,
                              std::size_t channel) {
                if (last > query_count || pool.empty()) {
                    if (!pool.empty() && channel != ignored_channel)
                        ++incomplete_attributes;
                    return;
                }
                context.grow(indices.size());
                context.grow(indices.size());
                for (const auto k : indices) {
                    require(k < pool.size(), "native grid visitor attribute read outside source");
                    dest.push_back(pool[k]);
                    f.client_indices[channel].push_back(static_cast<std::int32_t>(k));
                }
            };
            values(d.normals, mesh.data.coordinates.normals, mesh.data.coordinates.normals.size(),
                   normal_channel);
            values(d.parameters, mesh.data.coordinates.parameters,
                   mesh.data.coordinates.parameters.size(), parameter_channel);
            values(d.face_data, mesh.data.face_data, counts[native_face_data_pool], face_channel);
            values(d.double_colors, mesh.double_colors, color_count, color_channel);
            // Double RGB is independent; integer RGB takes priority over table.
            if (last <= color_count && !mesh.integer_colors.empty())
                values(d.integer_colors, mesh.integer_colors, color_count, color_channel);
            else
                values(d.color_table, mesh.color_table, color_count, color_channel);
            if (!mesh.integer_colors.empty() && last > color_count)
                ++incomplete_attributes;
        }
        context.grow(3);
        if (consumer)
            (*consumer)(ordinal, f, d);
        out.read_indices.push_back(ordinal);
        if (consumer) {
            context.storage = retained_storage;
            context.grow(1); // Retain the read ordinal, release this face's values.
        } else {
            out.facets.push_back(std::move(f));
            out.data.push_back(std::move(d));
        }
    }
    std::size_t unused = 0;
    for (auto u : used) {
        context.work.charge(1);
        unused += !u;
    }
    out.complete =
        !unused && !incomplete_attributes && !ignored_float && !ignored_faces && !ignored_table;
    out.report = {{"scope", "native_unindexed_polyface_visitor"},
                  {"mesh_style", mesh.mesh_style},
                  {"complete", out.complete},
                  {"all_data_requested", all_data},
                  {"all_data_effective", blocked ? all_data : true},
                  {"requested_wrap", requested_wrap},
                  {"actual_wrap", wrap},
                  {"read_index_kind", "facet_ordinal"},
                  {"unused_source_points", unused},
                  {"incomplete_attribute_reads", incomplete_attributes},
                  {"ignored_float_color_values", ignored_float ? mesh.float_colors.size() : 0},
                  {"ignored_face_data_values", ignored_faces ? mesh.data.face_data.size() : 0},
                  {"ignored_color_table_values", ignored_table ? mesh.color_table.size() : 0},
                  {"color_query_count", color_count},
                  {"global_num_per_row", mesh.num_per_row},
                  {"face_count", out.read_indices.size()},
                  {"values_consumed_incrementally", consumer != nullptr}};
    return out;
}
} // namespace
static NativePolyfaceVisit visit_impl(const NativePolyfaceMesh &mesh, TubeBudget &budget,
                                      bool all_data, std::uint32_t wrap,
                                      const NativePolyfaceFacetConsumer *consumer,
                                      std::size_t ignored_channel) {
    if (mesh.mesh_style != 1)
        return visit_unindexed(mesh, budget, all_data, wrap, consumer, ignored_channel);
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
        const auto retained_storage = context.storage;
        const auto current_counts = pool_counts(mesh);
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
        NativePolyfaceVisitedData data;
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
        context.grow(count);
        context.grow(wrap);
        for (std::size_t i = 0; i < count; ++i)
            data.index_positions.push_back(first + i);
        for (std::size_t i = 0; i < wrap; ++i)
            data.index_positions.push_back(first + i % count);
        if (all_data) {
            for (auto channel : {normal_channel, parameter_channel}) {
                const auto pool =
                    channel == normal_channel ? native_normal_pool : native_parameter_pool;
                const auto &src = indices[channel].empty() && current_counts[pool] == points.size()
                                      ? pi
                                      : indices[channel];
                collect(channel, src, pi.size(), current_counts[pool], current_counts[pool]);
            }
            collect(face_channel, indices[face_channel], indices[face_channel].size(),
                    counts[native_face_data_pool], counts[native_face_data_pool]);
            if (color_pool != native_polyface_pool_count)
                collect(color_channel, indices[color_channel], pi.size(), color_count,
                        counts[color_pool]);
            for (auto channel : {parameter_channel, normal_channel, color_channel, face_channel}) {
                const auto pool_count =
                    channel == parameter_channel ? mesh.data.coordinates.parameters.size()
                    : channel == normal_channel  ? mesh.data.coordinates.normals.size()
                    : channel == face_channel    ? counts[native_face_data_pool]
                    : color_pool == native_polyface_pool_count ? 0
                                                               : counts[color_pool];
                if ((pool_count || m.indices.active[channel] || !indices[channel].empty()) &&
                    collected[channel] < count && channel != ignored_channel) {
                    ++truncated[channel];
                    ++incomplete_channels;
                }
            }
            auto values = [&](auto &dest, const auto &pool, std::size_t channel) {
                context.grow(f.client_indices[channel].size());
                for (auto k : f.client_indices[channel])
                    dest.push_back(pool[std::size_t(k)]);
            };
            values(data.normals, m.coordinates.normals, normal_channel);
            values(data.parameters, m.coordinates.parameters, parameter_channel);
            values(data.face_data, m.face_data, face_channel);
            if (color_pool == native_double_color_pool)
                values(data.double_colors, mesh.double_colors, color_channel);
            else if (color_pool == native_float_color_pool)
                values(data.float_colors, mesh.float_colors, color_channel);
            else if (color_pool == native_integer_color_pool)
                values(data.integer_colors, mesh.integer_colors, color_channel);
            else if (color_pool == native_color_table_pool)
                values(data.color_table, mesh.color_table, color_channel);
        }
        covered += collected[point_channel];
        context.grow(3); // facet, values and source-read records
        if (consumer)
            (*consumer)(first, f, data);
        reports.push_back({{"read_index", first},
                           {"source_corner_count", count},
                           {"collected_corner_counts", collected}});
        out.read_indices.push_back(first);
        if (consumer) {
            context.storage = retained_storage;
            context.grow(2); // Read position and per-face diagnostic, no value copies retained.
        } else {
            out.facets.push_back(std::move(f));
            out.data.push_back(std::move(data));
        }
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
                  {"values_consumed_incrementally", consumer != nullptr},
                  {"facet_results", std::move(reports)}};
    return out;
}
NativePolyfaceVisit visit_native_polyface(const NativePolyfaceMesh &mesh, TubeBudget &budget,
                                          bool all_data, std::uint32_t wrap) {
    return visit_impl(mesh, budget, all_data, wrap, nullptr, polyface_channel_count);
}
NativePolyfaceVisit
visit_native_polyface_attribute_updates(NativePolyfaceMesh &mesh, TubeBudget &budget,
                                        std::size_t channel,
                                        const NativePolyfaceFacetConsumer &consumer) {
    require(channel == normal_channel || channel == parameter_channel,
            "native attribute visitor requires normal or parameter channel");
    return visit_impl(mesh, budget, true, 0, &consumer, channel);
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
