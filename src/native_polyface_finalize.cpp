#include "native_polyface_finalize.hpp"
#include "native_coordinate_cluster.hpp"
#include "native_bezier.hpp"
#include <map>
#include <set>
namespace p3d::swept_detail {
NativePolyfaceEdgeVisibility
resolve_native_polyface_edge_visibility(const NativePolyfaceIndexState &source,
                                        TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    std::size_t storage = 0;
    auto count = [&](std::size_t n) {
        require(n <= budget.max_control_points - storage, "native edge visibility storage budget");
        storage += n;
        work.charge(n);
    };
    for (const auto &v : source.indices)
        count(v.size());
    require(source.indices[point_channel].size() <= INT32_MAX,
            "native edge visibility signed count");
    using Edge = std::pair<std::int32_t, std::int32_t>;
    struct Less {
        curve_detail::BezierWork work;
        bool operator()(const Edge &a, const Edge &b) const {
            work.charge(1);
            return a < b;
        }
    };
    std::set<Edge, Less> visible(Less{work});
    std::map<Edge, std::size_t, Less> first_hidden(Less{work});
    NativePolyfaceEdgeVisibility out;
    out.output = source;
    auto &indices = out.output.indices[point_channel];
    std::int32_t first = 0;
    bool at_start = true;
    std::size_t examined = 0, promoted_current = 0, promoted_previous = 0, repeated_hidden = 0;
    auto magnitude = [](std::int32_t value) {
        require(value != INT32_MIN, "native edge visibility signed magnitude");
        return value < 0 ? -value : value;
    };
    // Last slot is only a possible next endpoint. The native routine neither
    // validates terminators nor repairs an open final index run.
    for (std::size_t i = 0; i + 1 < indices.size(); ++i) {
        work.charge(1);
        if (at_start) {
            first = indices[i];
            at_start = false;
        }
        if (!indices[i]) {
            at_start = true;
            continue;
        }
        ++examined;
        const auto a = magnitude(indices[i]);
        const auto b = magnitude(indices[i + 1] ? indices[i + 1] : first);
        const Edge edge{std::min(a, b), std::max(a, b)};
        if (visible.find(edge) != visible.end()) {
            if (indices[i] < 0) {
                indices[i] = -indices[i];
                ++promoted_current;
            }
        } else if (indices[i] > 0) {
            count(1);
            visible.insert(edge);
            auto previous = first_hidden.find(edge);
            if (previous != first_hidden.end()) {
                indices[previous->second] = -indices[previous->second];
                ++promoted_previous;
            }
        } else {
            const auto previous = first_hidden.find(edge);
            if (previous == first_hidden.end()) {
                count(1);
                first_hidden.emplace(edge, i);
            } else {
                ++repeated_hidden;
            }
        }
    }
    out.report = {{"scope", "native_post_combination_edge_visibility"},
                  {"examined_edges", examined},
                  {"visible_edge_keys", visible.size()},
                  {"remembered_hidden_edge_keys", first_hidden.size()},
                  {"promoted_current", promoted_current},
                  {"promoted_previous", promoted_previous},
                  {"repeated_hidden_occurrences", repeated_hidden},
                  {"coordinate_comparison_performed", false},
                  {"topology_repair_performed", false}};
    return out;
}
NativePolyfaceFinalization finalize_native_polyface_mesh(const NativePolyfaceMesh &source,
                                                         TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    std::size_t metadata = 0;
    auto count = [&](std::size_t n) {
        require(n <= budget.max_control_points - metadata, "native finalization metadata budget");
        metadata += n;
        work.charge(n);
    };
    count(source.data.coordinates.points.size());
    count(source.data.coordinates.normals.size());
    count(source.data.coordinates.parameters.size());
    for (const auto &i : source.data.indices.indices)
        count(i.size());
    count(source.data.face_data.size());
    count(source.data.edge_chains.size());
    for (const auto &e : source.data.edge_chains) {
        count(e.point_indices.size());
        count(e.topology_ids.size());
    }
    count(source.double_colors.size());
    count(source.float_colors.size());
    count(source.integer_colors.size());
    count(source.color_table.size());
    count(source.face_uv_points.size());
    count(source.face_material_ids.size());
    count(source.face_smooth_groups.size());
    count(source.illumination_name.size());
    NativePolyfaceCoordinateState input;
    // combineCoordinate only compares this original 32-bit setting with 1.
    input.mesh_style = source.mesh_style == 1 ? 1 : 0;
    input.parameter_pool_active = source.pool_active[native_parameter_pool];
    input.normal_pool_active = source.pool_active[native_normal_pool];
    input.points = source.data.coordinates.points;
    input.normals = source.data.coordinates.normals;
    input.parameters = source.data.coordinates.parameters;
    input.indices = source.data.indices;
    auto combined = combine_native_polyface_coordinates(input, budget);
    auto visibility = resolve_native_polyface_edge_visibility(combined.output.indices, budget);
    NativePolyfaceFinalization out;
    out.output = source;
    auto &data = out.output.data;
    data.coordinates.points = std::move(combined.output.points);
    data.coordinates.normals = std::move(combined.output.normals);
    data.coordinates.parameters = std::move(combined.output.parameters);
    data.indices = std::move(visibility.output);
    out.point_map = std::move(combined.point_map);
    out.parameter_map = std::move(combined.parameter_map);
    out.normal_map = std::move(combined.normal_map);
    out.combination_applied = combined.applied;
    out.report = {{"scope", "native_sweep_polyface_finalization"},
                  {"coordinate_combination", std::move(combined.report)},
                  {"edge_visibility", std::move(visibility.report)},
                  {"face_ranges_recomputed", false},
                  {"edge_chains_remapped", false},
                  {"mesh_validity_evaluated", false}};
    return out;
}
} // namespace p3d::swept_detail
