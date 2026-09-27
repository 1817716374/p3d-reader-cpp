#include "native_tube_mesh_trim_output.hpp"
#include "native_bezier.hpp"
namespace p3d::swept_detail {
TubeMeshTrimOutput emit_tube_mesh_trimmed_strip(const TubeMeshTrimConnected &input,
                                                const TubeMeshEdgeOptions &options,
                                                const TubeMeshVisibilityLayout &layout,
                                                TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    require(layout.trimmed && layout.coordinate_count == input.points.size(),
            "native trimmed output visibility layout");
    require(input.facets.indices_in_range && input.facets.column_correspondence_valid,
            "native trimmed output facet correspondence");
    std::size_t stored = 0;
    auto extent = [&](std::size_t n) {
        require(n <= budget.max_control_points - stored, "native trimmed output storage budget");
        stored += n;
        work.charge(n);
    };
    extent(input.points.size());
    if (options.normals)
        extent(input.normals.size());
    if (options.parameters)
        extent(input.parameters.size());
    std::vector<NativePolyfaceVisitorFacet> facets;
    NativePolyfaceIndexState original;
    for (const auto &source : input.facets.facets) {
        require(source.indices_in_range && source.column_correspondence_valid &&
                    !source.indices.empty(),
                "native trimmed output source facet");
        extent(source.indices.size() + 1);
        NativePolyfaceVisitorFacet facet;
        for (auto i : source.indices) {
            require(i > 0 && std::size_t(i) <= input.points.size(),
                    "native trimmed output source index");
            facet.points.push_back(input.points[i - 1]);
            facet.client_indices[point_channel].push_back(i - 1);
            facet.visible.push_back(1);
            original.indices[point_channel].push_back(i);
        }
        original.indices[point_channel].push_back(0);
        facet.client_indices[point_channel].push_back(source.indices.front() - 1);
        facet.visible.push_back(1);
        facets.push_back(std::move(facet));
    }
    original.active[point_channel] =
        !input.points.empty() && !original.indices[point_channel].empty();
    auto triangles = triangulate_native_polyface_facets(facets, original, budget);
    TubeMeshTrimOutput out;
    out.mesh.points = input.points;
    out.mesh.point_indices = std::move(triangles.output.indices[point_channel]);
    out.mesh.point_index_active = triangles.output.active[point_channel];
    if (options.normals) {
        work.charge(out.mesh.point_indices.size());
        out.mesh.normals = input.normals;
        out.mesh.normal_indices = out.mesh.point_indices;
    }
    if (options.parameters) {
        work.charge(out.mesh.point_indices.size());
        out.mesh.parameters = input.parameters;
        out.mesh.parameter_indices = out.mesh.point_indices;
    }
    std::size_t invalid_normals = 0, invalid_parameters = 0;
    for (auto i : out.mesh.point_indices) {
        work.charge(1);
        if (!i)
            continue;
        const auto n = std::size_t(std::abs(i));
        invalid_normals += options.normals && n > out.mesh.normals.size();
        invalid_parameters += options.parameters && n > out.mesh.parameters.size();
    }
    auto visibility = apply_tube_mesh_edge_visibility(out.mesh.point_indices, layout, budget);
    out.mesh.point_indices = std::move(visibility.indices);
    out.complete = triangles.complete && invalid_normals == 0 && invalid_parameters == 0;
    out.report = {{"scope", "native_swept_trimmed_strip_output"},
                  {"complete", out.complete},
                  {"triangulation", std::move(triangles.report)},
                  {"edge_visibility", std::move(visibility.report)},
                  {"invalid_normal_indices", invalid_normals},
                  {"invalid_parameter_indices", invalid_parameters},
                  {"attribute_indices_copied_before_visibility", true},
                  {"caps_generated", false},
                  {"coordinate_combination_applied", false}};
    out.mesh.report = out.report;
    return out;
}
} // namespace p3d::swept_detail
