#include "native_tube_mesh_assembly.hpp"
namespace p3d::swept_detail {
NativeTubeMeshAssembly
assemble_native_tube_mesh_groups(const std::vector<TubeMeshGridPreparation> &input,
                                 const NativeTubeMeshAssemblyOptions &options, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    std::size_t storage = 0;
    auto count = [&](std::size_t n) {
        require(n <= budget.max_control_points - storage, "native sweep assembly storage budget");
        storage += n;
        work.charge(n);
    };
    count(input.size());
    NativeTubeMeshAssembly out;
    out.complete = true;
    out.report = {{"scope", "native_prepared_sweep_mesh_assembly"},
                  {"source_conditions_resolved_by_caller", true},
                  {"file_dispatch_performed", false}};
    std::vector<std::vector<Point3>> starts, ends;
    TubeMeshGroupOptions group_options{options.normals, options.parameters, options.profile_closed,
                                       options.path_closed, options.cap_eligible};
    for (std::size_t gi = 0; gi < input.size(); ++gi) {
        auto group = emit_tube_mesh_group(input[gi], group_options, budget);
        out.complete = out.complete && group.strips_complete;
        const auto &details = group.report.at("strip_results");
        for (std::size_t si = 0; si < group.strips.size(); ++si) {
            const auto &strip = group.strips[si];
            const auto interval = details.at(si).at("strip").get<std::size_t>();
            const bool trimmed = details.at(si).at("trimmed").get<bool>();
            const auto width = input[gi].section.interval_samples.at(interval).size();
            require(width >= 2 && width <= INT32_MAX, "native strip point row width");
            for (auto n : {strip.points.size(), strip.normals.size(), strip.parameters.size(),
                           strip.point_indices.size(), strip.normal_indices.size(),
                           strip.parameter_indices.size()})
                count(n);
            auto mesh = make_native_polyface_mesh(trimmed ? 0 : 3, trimmed ? 1 : 5);
            mesh.data.coordinates.points = strip.points;
            mesh.data.coordinates.normals = strip.normals;
            mesh.data.coordinates.parameters = strip.parameters;
            mesh.pool_rows[native_point_pool] = static_cast<std::uint32_t>(width);
            mesh.pool_active[native_point_pool] = !strip.points.empty();
            mesh.pool_active[native_normal_pool] = !strip.normals.empty();
            mesh.pool_active[native_parameter_pool] = !strip.parameters.empty();
            // Triangulation replaces only index buffers, retaining row/tag
            // metadata. Attribute index assignment then copies point tags too.
            mesh.data.num_per_face = 0;
            mesh.mesh_style = 1;
            mesh.data.indices.indices[point_channel] = strip.point_indices;
            mesh.data.indices.active[point_channel] = strip.point_index_active;
            auto copy_index = [&](std::size_t channel, const auto &indices, bool requested) {
                if (!requested)
                    return;
                mesh.data.indices.indices[channel] = indices;
                mesh.data.indices.active[channel] = mesh.data.indices.active[point_channel];
                mesh.index_tags[channel] = mesh.index_tags[point_channel];
                mesh.index_rows[channel] = mesh.index_rows[point_channel];
            };
            copy_index(normal_channel, strip.normal_indices, options.normals);
            copy_index(parameter_channel, strip.parameter_indices, options.parameters);
            out.sources.push_back(std::move(mesh));
        }
        if (options.cap_eligible && group.traversal_completed) {
            count(group.start_boundary.size());
            count(group.end_boundary.size());
            starts.push_back(group.start_boundary);
            ends.push_back(group.end_boundary);
        }
        const bool traversed = group.traversal_completed;
        out.groups.push_back(std::move(group));
        if (!traversed) {
            out.report["failure"] = "group_traversal_incomplete";
            out.report["complete"] = false;
            return out;
        }
    }
    out.caps = emit_native_tube_mesh_cap_pair(
        starts, ends, options.cap_eligible,
        {options.normals, options.parameters, options.parameter_mode}, budget);
    if (options.cap_eligible)
        out.complete = out.complete && out.caps.complete;
    if (out.caps.published) {
        out.sources.push_back(out.caps.start->mesh);
        out.sources.push_back(out.caps.end->mesh);
    }
    NativePolyfacePreparationOptions preparation;
    preparation.normals_required = options.normals;
    preparation.parameters_required = options.parameters;
    // Only equality with 0/1 is used by the native parameter selector.
    preparation.parameter_mode = options.parameter_mode == 0   ? 0
                                 : options.parameter_mode == 1 ? 1
                                                               : 2;
    preparation.attributes_enabled_at_construction = false;
    out.assembly = assemble_native_prepared_polyfaces(out.sources, preparation, {}, budget);
    // Query-copy intentionally retains the fresh destination's row widths and
    // vector tags. For these generated strips that is an established conversion,
    // not an unresolved reconstruction step. Preserve both source layouts and
    // the generic copy-fidelity diagnostic; never excuse omitted values or a
    // failed visitor/attribute/triangulation operation.
    bool processing_complete = out.assembly.assembled.complete;
    Json conversions = Json::array();
    for (std::size_t i = 0; i < out.assembly.sources.size(); ++i) {
        const auto &prepared = out.assembly.sources[i];
        bool ready = prepared.report.at("source_query").at("complete").get<bool>() &&
                     prepared.output.mesh_style == 1;
        for (const auto &step : prepared.report.at("steps")) {
            const auto &r = step.at("result");
            if (step.at("step") == "query_copy") {
                ready = ready && r.at("omitted_source_values") == 0 &&
                        r.at("discarded_name_code_units") == 0 &&
                        r.at("source_texture_id_differs") == false;
                conversions.push_back({{"source", i},
                                       {"row_width_changes", r.at("different_source_row_widths")},
                                       {"tag_changes", r.at("different_source_vector_tags")}});
            } else {
                ready = ready && r.at("complete").get<bool>();
            }
        }
        processing_complete = processing_complete && ready;
    }
    auto mesh = make_native_polyface_mesh();
    const auto &assembled = out.assembly.assembled;
    mesh.data.coordinates.points = assembled.coordinates.points;
    mesh.data.coordinates.normals = assembled.coordinates.normals;
    mesh.data.coordinates.parameters = assembled.coordinates.parameters;
    mesh.data.indices = assembled.indices;
    mesh.data.face_data = assembled.face_data;
    mesh.data.edge_chains = assembled.edge_chains;
    mesh.data.num_per_face = assembled.num_per_face;
    if (!out.sources.empty())
        mesh.data.two_sided = assembled.two_sided;
    mesh.pool_active[native_normal_pool] = assembled.normal_pool_active;
    mesh.pool_active[native_parameter_pool] = assembled.parameter_pool_active;
    out.finalized = finalize_native_polyface_mesh(mesh, budget);
    out.complete = out.complete && processing_complete;
    out.report["native_layout_conversions"] = std::move(conversions);
    out.report["source_layouts_retained"] = true;
    out.report["complete"] = out.complete;
    out.report["group_count"] = out.groups.size();
    out.report["source_mesh_count"] = out.sources.size();
    out.report["caps"] = out.caps.report;
    out.report["assembly"] = out.assembly.report;
    out.report["finalization"] = out.finalized->report;
    return out;
}
} // namespace p3d::swept_detail
