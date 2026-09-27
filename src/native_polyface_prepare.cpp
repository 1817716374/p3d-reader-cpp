#include "native_polyface_prepare.hpp"
#include "native_polyface_smooth_normals.hpp"
#include "native_polyface_mesh_attributes.hpp"
#include "native_polyface_mesh_face_data.hpp"
#include "native_polyface_edge_chains.hpp"
#include "native_polygon_convexity.hpp"
#include "native_bezier.hpp"

namespace p3d::swept_detail {
namespace {
std::int64_t signed_setting(std::uint32_t value) {
    return value <= INT32_MAX ? std::int64_t(value) : std::int64_t(value) - 0x100000000LL;
}
void count_mesh(const NativePolyfaceMesh &m, std::size_t &count, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    auto add = [&](std::size_t n) {
        require(n <= budget.max_control_points - count, "native outer preparation storage budget");
        count += n;
        work.charge(n);
    };
    add(m.data.coordinates.points.size());
    add(m.data.coordinates.normals.size());
    add(m.data.coordinates.parameters.size());
    add(m.data.face_data.size());
    add(m.data.edge_chains.size());
    for (const auto &e : m.data.edge_chains) {
        add(e.topology_ids.size());
        add(e.point_indices.size());
    }
    for (const auto &i : m.data.indices.indices)
        add(i.size());
    add(m.double_colors.size());
    add(m.float_colors.size());
    add(m.integer_colors.size());
    add(m.color_table.size());
    add(m.face_uv_points.size());
    add(m.face_material_ids.size());
    add(m.face_smooth_groups.size());
    add(m.illumination_name.size());
}
bool has_untransferred_metadata(const NativePolyfaceMesh &m) {
    return !m.double_colors.empty() || !m.float_colors.empty() || !m.integer_colors.empty() ||
           !m.color_table.empty() || !m.face_uv_points.empty() || !m.face_material_ids.empty() ||
           !m.face_smooth_groups.empty() || m.texture_id != 0 || !m.illumination_name.empty();
}
} // namespace
NativePolyfacePreparation
prepare_native_polyface_for_builder(const NativePolyfaceMesh &source,
                                    const NativePolyfacePreparationOptions &options,
                                    TubeBudget &budget) {
    require(source.mesh_style == 1 || source.mesh_style == 3 || source.mesh_style == 4 ||
                source.mesh_style == 5 || source.mesh_style == 6,
            "native outer preparation unsupported query visitor");
    std::size_t storage = 0;
    count_mesh(source, storage, budget);
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    const bool normals = options.normals_required && source.data.coordinates.normals.empty();
    const bool parameters =
        options.parameters_required && source.data.coordinates.parameters.empty();
    const bool faces = options.parameters_required && source.data.face_data.empty();
    const bool edges = options.edge_chains_required && source.data.edge_chains.empty();
    // Wrap zero / allData false, exactly as the outer max-face query. Keep the
    // immutable facets for the later source-convexity query only if it is used.
    const auto visited = visit_native_polyface(source, budget, false, 0);
    std::size_t maximum = 0;
    for (const auto &f : visited.facets) {
        work.charge(1);
        maximum = std::max(maximum, f.points.size());
    }
    require(maximum <= UINT32_MAX, "native outer maximum face count extent");
    const auto limit = signed_setting(options.max_edges_per_face);
    const bool oversized = limit < signed_setting(static_cast<std::uint32_t>(maximum));
    NativePolyfacePreparation out;
    out.complete = visited.complete;
    out.copied = normals || parameters || faces || edges || oversized;
    Json steps = Json::array();
    out.report = {{"scope", "native_builder_preparation"},
                  {"copied", out.copied},
                  {"max_facet_size", maximum},
                  {"signed_edge_limit", limit},
                  {"needs",
                   {{"normals", normals},
                    {"parameters", parameters},
                    {"face_data", faces},
                    {"edge_chains", edges},
                    {"edge_limit", oversized}}},
                  {"source_query", visited.report},
                  {"source_convexity_tested", false}};
    if (!out.copied) {
        out.output = source;
        out.report["complete"] = out.complete;
        out.report["steps"] = std::move(steps);
        return out;
    }
    auto copied = copy_native_polyface_query(source, make_native_polyface_mesh(), budget);
    out.complete = out.complete && copied.complete;
    steps.push_back({{"step", "query_copy"}, {"result", std::move(copied.report)}});
    out.output = std::move(copied.output);
    // These are caller-provided builder controls, not source native mesh fields.
    out.output.data.coordinates.parameter_scope = source.data.coordinates.parameter_scope;
    out.output.data.coordinates.normalize_normals = source.data.coordinates.normalize_normals;
    out.output.data.coordinates.reverse_normals = source.data.coordinates.reverse_normals;
    if (normals) {
        // Base DegreesToRadians is one multiplication by this exact double.
        constexpr double degrees_to_radians = 0x1.1df46a2529d39p-6;
        auto result = build_native_polyface_mesh_approximate_normals(
            out.output, 30.0 * degrees_to_radians, 90.0 * degrees_to_radians,
            options.hide_smooth_edges, budget);
        out.complete = out.complete && result.complete;
        steps.push_back({{"step", "approximate_normals"}, {"result", std::move(result.report)}});
        out.output = std::move(result.output);
    }
    if (parameters) {
        const auto selector = options.parameter_mode == 0 ? 2 : options.parameter_mode == 1 ? 3 : 1;
        auto result = build_native_polyface_mesh_parameters(out.output, selector, budget);
        out.complete = out.complete && result.complete;
        steps.push_back({{"step", "parameters"}, {"result", std::move(result.report)}});
        out.output = std::move(result.output);
    }
    if (faces) {
        auto result = build_native_polyface_mesh_face_data(out.output, budget);
        out.complete = out.complete && result.complete;
        steps.push_back({{"step", "face_data"}, {"result", std::move(result.report)}});
        out.output = std::move(result.output);
    }
    bool convex = true;
    for (const auto &f : visited.facets) {
        work.charge(1);
        if (f.points.size() > 3 && !native_polygon_convexity(f.points, budget).convex) {
            convex = false;
            break;
        }
    }
    out.report["source_convexity_tested"] = true;
    out.report["source_has_convex_facets"] = convex;
    const bool force_triangles = !convex && options.convex_facets_required;
    if (force_triangles || oversized) {
        // Preserve sign extension for negative original 32-bit settings.
        const auto max_edges = force_triangles ? std::size_t(3) : static_cast<std::size_t>(limit);
        auto result = triangulate_native_polyface_mesh(out.output, budget, max_edges);
        out.complete = out.complete && result.complete;
        steps.push_back({{"step", "triangulation"},
                         {"reason", force_triangles ? "source_nonconvex" : "edge_limit"},
                         {"max_edges", max_edges},
                         {"result", std::move(result.report)}});
        out.output = std::move(result.output);
    }
    if (edges) {
        auto result =
            build_native_polyface_mesh_edge_chains(out.output, options.draw_method_index, budget);
        out.complete = out.complete && result.complete;
        steps.push_back({{"step", "edge_chains"}, {"result", std::move(result.report)}});
        out.output = std::move(result.output);
    }
    out.report["complete"] = out.complete;
    out.report["steps"] = std::move(steps);
    return out;
}
NativePreparedPolyfaceAssembly assemble_native_prepared_polyfaces(
    const std::vector<NativePolyfaceMesh> &sources, const NativePolyfacePreparationOptions &options,
    const NativeBuilderCoordinateOptions &coordinate_options, TubeBudget &budget) {
    require(sources.size() <= budget.max_control_points, "native outer source count budget");
    std::size_t storage = sources.size();
    for (const auto &s : sources)
        count_mesh(s, storage, budget);
    NativePreparedPolyfaceAssembly out;
    out.complete = true;
    std::vector<NativeBuilderPolyface> prepared;
    std::vector<std::size_t> untransferred, untransferred_layouts;
    for (std::size_t i = 0; i < sources.size(); ++i) {
        auto result = prepare_native_polyface_for_builder(sources[i], options, budget);
        out.complete = out.complete && result.complete;
        if (has_untransferred_metadata(sources[i]))
            untransferred.push_back(i);
        if (result.output.mesh_style != 1)
            untransferred_layouts.push_back(i);
        prepared.push_back(result.output.data);
        out.sources.push_back(std::move(result));
    }
    NativeBuilderPolyfaceOptions matched;
    matched.coordinates = coordinate_options;
    matched.normals_required = options.normals_required;
    matched.parameters_required = options.parameters_required;
    out.assembled = assemble_native_builder_polyfaces(prepared, matched, budget);
    out.native_succeeded = out.assembled.native_succeeded;
    out.complete = out.complete && out.assembled.complete && untransferred.empty() &&
                   untransferred_layouts.empty();
    out.report = {{"scope", "native_outer_builder_assembly"},
                  {"native_succeeded", out.native_succeeded},
                  {"complete", out.complete},
                  {"source_count", sources.size()},
                  {"untransferred_metadata_sources", std::move(untransferred)},
                  {"untransferred_raw_layout_sources", std::move(untransferred_layouts)},
                  {"matched", out.assembled.report}};
    return out;
}
} // namespace p3d::swept_detail
