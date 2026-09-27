#include "native_tube_mesh_group.hpp"
namespace p3d::swept_detail {
TubeMeshGroupOutput emit_tube_mesh_group(const TubeMeshGridPreparation &input,
                                         const TubeMeshGroupOptions &options, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    require(input.success && input.section.success, "native mesh group preparation required");
    const auto np = input.patches.size(), ns = input.section.interval_samples.size();
    auto state =
        make_tube_mesh_edge_state(np, ns, options.profile_closed, options.path_closed, budget);
    require(np <= budget.max_control_points / ns, "native mesh group strip output budget");
    work.charge(ns);
    std::vector<bool> discontinuities(ns, false);
    for (auto i : input.section.discontinuity_intervals) {
        work.charge(1);
        require(i < ns, "native mesh group discontinuity interval");
        discontinuities[i] = true;
    }
    TubeMeshGroupOutput out;
    out.report = {{"scope", "native_swept_prepared_group_strips"},
                  {"caps_generated", false},
                  {"coordinate_combination_applied", false},
                  {"strip_results", Json::array()}};
    std::size_t stored = 0, incomplete = 0;
    auto count = [&](std::size_t n) {
        work.charge(n);
        require(n <= budget.max_control_points - stored,
                "native mesh group cumulative output budget");
        stored += n;
    };
    auto finish = [&]() {
        count(state.start_points.size());
        count(state.end_points.size());
        out.start_boundary = std::move(state.start_points);
        out.end_boundary = std::move(state.end_points);
        out.strips_complete = out.traversal_completed && incomplete == 0;
        out.report["traversal_completed"] = out.traversal_completed;
        out.report["strips_complete"] = out.strips_complete;
        out.report["emitted_strips"] = out.strips.size();
        out.report["incomplete_strips"] = incomplete;
        out.report["start_boundary_points"] = out.start_boundary.size();
        out.report["end_boundary_points"] = out.end_boundary.size();
        return std::move(out);
    };
    auto invalid_attributes = [&](const TubeMeshRegularMesh &m) {
        std::size_t bad = 0;
        auto check = [&](const auto &indices, std::size_t size) {
            for (auto i : indices) {
                work.charge(1);
                require(i != INT32_MIN, "native mesh group attribute index magnitude");
                bad += std::size_t(std::abs(i)) > size;
            }
        };
        check(m.normal_indices, m.normals.size());
        check(m.parameter_indices, m.parameters.size());
        return bad;
    };
    for (std::size_t pi = 0; pi < np; ++pi) {
        const auto &patch = input.patches[pi];
        require(patch.preparation.success && patch.preparation.boundaries.success &&
                    patch.path.success && patch.preparation.strips.size() == ns,
                "native mesh group patch correspondence");
        for (std::size_t si = 0; si < ns; ++si) {
            work.charge(1);
            const bool first_patch = pi == 0, last_patch = pi + 1 == np;
            TubeMeshEdgeOptions edge{options.normals, options.parameters,
                                     options.collect_cap_boundaries && first_patch,
                                     options.collect_cap_boundaries && last_patch};
            // 8b687..8b6a3 removes the previous strip endpoint before the next
            // strip adds its first point; no coordinate-equality test is made.
            auto remove_endpoint = [&](std::vector<Point3> &v) {
                require(!v.empty(), "native cap boundary predecessor endpoint");
                work.charge(1);
                v.pop_back();
            };
            if (si > 0) {
                if (edge.collect_start)
                    remove_endpoint(state.start_points);
                if (edge.collect_end)
                    remove_endpoint(state.end_points);
            }
            TubeMeshVisibilityLayout layout;
            layout.u_count = input.section.interval_samples[si].size();
            layout.v_count = patch.path.parameters.size();
            layout.first_column_visible = discontinuities[si];
            layout.last_column_visible = si + 1 == ns && input.section.end_discontinuity;
            layout.trimmed =
                bool(patch.preparation.boundaries.lower || patch.preparation.boundaries.upper);
            TubeMeshRegularMesh mesh;
            bool complete = true;
            if (layout.trimmed) {
                auto vertices = evaluate_tube_mesh_trim_vertices(patch, input.section, si, budget);
                if (!vertices.plan.success) {
                    out.report["failure"] = {{"patch", pi},
                                             {"strip", si},
                                             {"step", "trim_column_preparation"},
                                             {"details", std::move(vertices.plan.report)}};
                    return finish();
                }
                require(!vertices.plan.columns.empty() &&
                            vertices.plan.columns.front().count <= INT32_MAX &&
                            vertices.plan.columns.back().count <= INT32_MAX,
                        "native mesh group trim visibility column lengths");
                layout.first_column_count = std::int32_t(vertices.plan.columns.front().count);
                layout.last_column_count = std::int32_t(vertices.plan.columns.back().count);
                auto connected = connect_tube_mesh_trim_vertices(vertices, patch.path.parameters,
                                                                 state, edge, budget);
                layout.coordinate_count = connected.points.size();
                auto emitted = emit_tube_mesh_trimmed_strip(connected, edge, layout, budget);
                complete = emitted.complete;
                mesh = std::move(emitted.mesh);
            } else {
                auto vertices =
                    evaluate_tube_mesh_regular_vertices(patch, input.section, si, pi, np, budget);
                mesh = connect_tube_mesh_regular_vertices(vertices, state, edge, budget);
                layout.coordinate_count = mesh.points.size();
                auto visibility =
                    apply_tube_mesh_edge_visibility(mesh.point_indices, layout, budget);
                mesh.point_indices = std::move(visibility.indices);
                mesh.report["edge_visibility"] = std::move(visibility.report);
                const auto invalid = invalid_attributes(mesh);
                mesh.report["invalid_attribute_indices"] = invalid;
                complete = invalid == 0;
            }
            count(mesh.points.size());
            count(mesh.normals.size());
            count(mesh.parameters.size());
            count(mesh.point_indices.size());
            count(mesh.normal_indices.size());
            count(mesh.parameter_indices.size());
            incomplete += !complete;
            out.report["strip_results"].push_back(
                {{"patch", pi},
                 {"strip", si},
                 {"trimmed", layout.trimmed},
                 {"complete", complete},
                 {"first_column_visible", layout.first_column_visible},
                 {"last_column_visible", layout.last_column_visible}});
            out.strips.push_back(std::move(mesh));
        }
    }
    out.traversal_completed = true;
    return finish();
}
} // namespace p3d::swept_detail
