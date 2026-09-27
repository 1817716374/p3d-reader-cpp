// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from Polyface.cpp and BlockedVector.cpp. Preserves original P3D
// activity gates, untouched row metadata, color pools and missing-terminator behavior.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_polyface_layout.hpp"
#include "native_bezier.hpp"
namespace p3d::swept_detail {
namespace {
struct Context {
    curve_detail::BezierWork work;
    TubeBudget &budget;
    std::size_t storage = 0;
    NativePolyfaceLayout output;
    Context(const NativePolyfaceLayout &input, TubeBudget &b)
        : work{b.work, b.max_work}, budget(b) {
        for (const auto &p : input.pools)
            grow(p.count);
        for (const auto &i : input.indices.indices)
            grow(i.size());
        output = input;
    }
    void grow(std::size_t n) {
        require(n <= budget.max_control_points - storage, "native polyface layout storage budget");
        storage += n;
        work.charge(n);
    }
    void append(std::vector<std::int32_t> &a, std::int32_t i) {
        grow(1);
        a.push_back(i);
    }
    void clear(std::vector<std::int32_t> &a) {
        storage -= a.size();
        a.clear();
    }
};
constexpr std::array<std::size_t, 5> native_channel_order{
    point_channel, normal_channel, parameter_channel, color_channel, face_channel};
} // namespace
NativePolyfaceLayoutResult convert_native_polyface_layout(const NativePolyfaceLayout &input,
                                                          TubeBudget &budget) {
    Context context(input, budget);
    auto &out = context.output;
    Json changes = Json::array();
    std::size_t dropped_indices = 0, unused_points = 0;
    bool native = true;
    if (out.mesh_style == 1) {
        if (out.num_per_face > 1)
            for (auto channel : native_channel_order) {
                const auto width = out.index_structs_per_row[channel];
                if (!out.indices.active[channel] || width <= 1)
                    continue;
                auto &a = out.indices.indices[channel];
                const auto old_size = a.size(), rows = old_size / width,
                           source_count = rows * width;
                const auto omitted = old_size - source_count;
                dropped_indices += omitted;
                // The native buffer temporarily retains its partial tail and appends
                // one zero per complete row before shifting the complete prefix.
                context.grow(rows);
                a.resize(old_size + rows, 0);
                std::size_t read = source_count, write = source_count + rows, in_row = 0;
                while (read) {
                    context.work.charge(1);
                    if (in_row == 0)
                        a[--write] = 0;
                    a[--write] = a[--read];
                    if (++in_row == width)
                        in_row = 0;
                }
                write = 0;
                read = 0;
                const auto expanded = source_count + rows;
                std::size_t removed_zeros = 0;
                while (read < expanded) {
                    context.work.charge(1);
                    const auto value = a[read++];
                    a[write++] = value;
                    if (value == 0)
                        while (read < expanded && a[read] == 0) {
                            context.work.charge(1);
                            ++read;
                            ++removed_zeros;
                        }
                }
                context.storage -= a.size() - write;
                a.resize(write);
                out.index_structs_per_row[channel] = 0;
                changes.push_back({{"channel", channel},
                                   {"operation", "blocked_to_terminated"},
                                   {"source_block_width", width},
                                   {"complete_rows", rows},
                                   {"discarded_tail_indices", omitted},
                                   {"collapsed_zero_count", removed_zeros},
                                   {"output_index_count", a.size()}});
            }
    } else if (out.mesh_style >= 3 && out.mesh_style <= 6) {
        const auto point_count = out.pools[native_point_pool].count;
        const bool grid = out.mesh_style >= 5, triangulated = out.mesh_style == 5;
        const std::size_t width = grid ? out.pools[native_point_pool].structs_per_row
                                  : out.mesh_style == 3 ? 3
                                                        : 4;
        const std::size_t rows =
            grid && width <= 1 ? point_count : point_count / (width ? width : 1);
        if (grid && (width < 2 || rows < 2))
            unused_points = point_count;
        else
            unused_points = point_count - rows * width;
        const bool colors = out.pools[native_double_color_pool].active ||
                            out.pools[native_integer_color_pool].active ||
                            out.pools[native_color_table_pool].active;
        for (auto channel : native_channel_order) {
            const bool generate = channel == point_channel    ? out.pools[native_point_pool].active
                                  : channel == normal_channel ? out.pools[native_normal_pool].active
                                  : channel == parameter_channel
                                      ? out.pools[native_parameter_pool].active
                                  : channel == color_channel ? colors
                                                             : false;
            if (!generate)
                continue;
            auto &a = out.indices.indices[channel];
            context.clear(a);
            require(point_count <= static_cast<std::size_t>(INT32_MAX),
                    "native generated index exceeds signed range");
            auto append = [&](std::size_t index) {
                require(index <= static_cast<std::size_t>(INT32_MAX),
                        "native generated index overflow");
                context.append(a, static_cast<std::int32_t>(index));
            };
            if (grid) {
                for (std::size_t r = 1; r < rows; ++r) {
                    context.work.charge(1);
                    for (std::size_t c = 1; c < width; ++c) {
                        const auto i00 = 1 + (r - 1) * width + c - 1, i01 = i00 + 1,
                                   i10 = i00 + width, i11 = i10 + 1;
                        append(i00);
                        append(i01);
                        if (triangulated) {
                            append(i10);
                            append(0);
                            append(i01);
                            append(i11);
                            append(i10);
                            append(0);
                        } else {
                            append(i11);
                            append(i10);
                            append(0);
                        }
                    }
                }
            } else {
                std::size_t index = 1;
                for (std::size_t r = 0; r < rows; ++r) {
                    context.work.charge(1);
                    for (std::size_t c = 0; c < width; ++c)
                        append(index++);
                    append(0);
                }
            }
            // Original helpers do not activate the index array or reset any
            // row metadata, unlike the newer Bentley reference implementation.
            changes.push_back({{"channel", channel},
                               {"operation", grid ? "grid_blocks" : "sequential_blocks"},
                               {"row_count", rows},
                               {"row_width", width},
                               {"triangulated", triangulated},
                               {"output_index_count", a.size()}});
        }
    } else
        native = false;
    if (native) {
        out.mesh_style = 1;
        out.num_per_face = 0;
    }
    NativePolyfaceLayoutResult result;
    result.native_succeeded = native;
    result.complete = native && dropped_indices == 0 && unused_points == 0;
    result.report = {{"operation", "native_convert_polyface_layout"},
                     {"source_mesh_style", input.mesh_style},
                     {"native_succeeded", native},
                     {"complete", result.complete},
                     {"discarded_tail_indices", dropped_indices},
                     {"unreferenced_partial_row_points", unused_points},
                     {"index_activity_changed", false},
                     {"pool_values_changed", false},
                     {"references_checked", false},
                     {"changes", std::move(changes)}};
    result.output = std::move(out);
    return result;
}
NativePolyfaceLayoutResult activate_native_polyface_layout(const NativePolyfaceLayout &input,
                                                           TubeBudget &budget) {
    Context context(input, budget);
    auto &out = context.output;
    for (auto &p : out.pools) {
        context.work.charge(1);
        p.active = p.count != 0;
    }
    const bool colors =
        out.pools[native_double_color_pool].active || out.pools[native_float_color_pool].active ||
        out.pools[native_integer_color_pool].active || out.pools[native_color_table_pool].active;
    for (auto channel : native_channel_order) {
        context.work.charge(1);
        const bool available = channel == point_channel    ? out.pools[native_point_pool].active
                               : channel == normal_channel ? out.pools[native_normal_pool].active
                               : channel == parameter_channel
                                   ? out.pools[native_parameter_pool].active
                               : channel == color_channel ? colors
                                                          : out.pools[native_face_data_pool].active;
        out.indices.active[channel] = !out.indices.indices[channel].empty() && available;
    }
    NativePolyfaceLayoutResult result;
    result.native_succeeded = true;
    result.complete = true;
    result.report = {{"operation", "native_activate_available_polyface_data"},
                     {"native_succeeded", true},
                     {"complete", true},
                     {"references_checked", false},
                     {"active_indices", out.indices.active}};
    result.output = std::move(out);
    return result;
}
} // namespace p3d::swept_detail
