#include "native_tube_mesh_cap.hpp"
#include "native_projected_polygon.hpp"
#include "native_polyface_copy.hpp"
#include "native_polyface_mesh_face_data.hpp"
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
#include "mesh_registry.hpp"
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
bool disconnect(const Point3 &p) {
    const double marker = std::numeric_limits<double>::max();
    return p[0] == marker || p[1] == marker || p[2] == marker;
}
Point3 face_normal(const Point3 &a, const Point3 &b, const Point3 &c) {
    Point3 u, v, cross;
    for (unsigned k = 0; k < 3; ++k) {
        u[k] = finite(b[k] - a[k]);
        v[k] = finite(c[k] - a[k]);
    }
    for (unsigned k = 0; k < 3; ++k) {
        const auto i = (k + 1) % 3, j = (k + 2) % 3;
        cross[k] = finite(finite(u[i] * v[j]) - finite(u[j] * v[i]));
    }
    // addTriangulation normalizes here; FindOrAddNormal normalizes again.
    return mesh_detail::unit(cross);
}
} // namespace
NativeTubeMeshCap emit_native_tube_mesh_cap(const std::vector<std::vector<Point3>> &rings,
                                            bool reverse, const NativeTubeMeshCapOptions &options,
                                            TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    NativeTubeMeshCap out;
    out.mesh = make_native_polyface_mesh();
    auto &mesh = out.mesh;
    mesh.pool_active[native_normal_pool] = options.normals_required;
    mesh.pool_active[native_parameter_pool] = options.parameters_required;
    mesh.data.indices.active[normal_channel] = options.normals_required;
    mesh.data.indices.active[parameter_channel] = options.parameters_required;
    mesh.data.indices.active[face_channel] = options.parameters_required;
    // Native success tests the mesh handle, ignoring addRegion's return value.
    out.native_succeeded = !rings.empty();
    out.report = {{"scope", "native_polyline_mesh_cap"},
                  {"max_edges_per_face", 3},
                  {"max_edge_length", 0},
                  {"normals_required", options.normals_required},
                  {"parameters_required", options.parameters_required},
                  {"parameter_mode", options.parameter_mode},
                  {"solid_assembly_performed", false}};
    auto finish = [&](const char *reason) {
        out.report["native_succeeded"] = out.native_succeeded;
        out.report["triangulation_succeeded"] = out.triangulation_succeeded;
        out.report["complete"] = out.complete;
        out.report["reason"] = reason;
        return std::move(out);
    };
    out.input = prepare_native_tube_mesh_cap_input(rings, reverse, 0, budget);
    if (!out.input.preparation_succeeded)
        return finish("cap_input_preparation_failed");
    const bool simple = out.input.points.size() <= 3 &&
                        std::none_of(out.input.points.begin(), out.input.points.end(), disconnect);
    out.report["polygon_route"] = simple ? "simple_face" : "projected_triangulation";
    bool polygon_complete = true;
    if (simple) {
        out.polygon_points = out.input.points;
        for (std::size_t i = 0; i < out.polygon_points.size(); ++i)
            out.polygon_indices.push_back(static_cast<std::int32_t>(i + 1));
        out.polygon_indices.push_back(0);
    } else {
        auto polygon = triangulate_native_projected_polygon(
            out.input.points, out.input.projection.local_to_world,
            out.input.projection.world_to_local, 0, budget);
        polygon_complete = polygon.complete;
        out.report["polygon"] = std::move(polygon.report);
        if (!polygon.native_succeeded)
            return finish("projected_triangulation_failed");
        out.polygon_points = std::move(polygon.points);
        out.polygon_indices = std::move(polygon.indices);
    }
    require(out.polygon_indices.size() <= budget.max_control_points,
            "native cap polygon index storage budget");
    NativeBuilderCoordinateBatch batch;
    batch.points = out.polygon_points;
    auto record = native_null_face_data();
    bool parameters_complete = true;
    if (options.parameters_required) {
        auto parameters = prepare_native_polygon_parameters(out.polygon_points,
                                                            out.input.projection.world_to_local,
                                                            options.parameter_mode, budget);
        parameters_complete = parameters.complete;
        out.report["parameters"] = std::move(parameters.report);
        if (!parameters.face_distance_range)
            return finish("native_face_distance_range_unwritten");
        record.parameter_distance_range = *parameters.face_distance_range;
        batch.parameters = std::move(parameters.parameters);
    }
    struct Facet {
        std::size_t begin, end;
    };
    std::vector<Facet> facets;
    std::size_t begin = 0, counted_facets = 0, skipped = 0;
    for (std::size_t i = 0; i < out.polygon_indices.size(); ++i) {
        work.charge(1);
        if (out.polygon_indices[i])
            continue;
        const auto count = i - begin;
        if (count >= 3) {
            facets.push_back({begin, i});
            for (auto at = begin; at < i; ++at) {
                const auto index = static_cast<std::int64_t>(out.polygon_indices[at]);
                const auto source = static_cast<std::size_t>(index < 0 ? -index : index);
                require(source > 0 && source <= batch.points.size(), "native cap point reference");
            }
            if (count <= 4)
                ++counted_facets;
            if (options.normals_required) {
                work.charge(40);
                if (count <= 4) {
                    auto point = [&](std::size_t offset) -> const Point3 & {
                        const auto index =
                            static_cast<std::int64_t>(out.polygon_indices[begin + offset]);
                        return batch
                            .points[static_cast<std::size_t>(index < 0 ? -index : index) - 1];
                    };
                    batch.normals.push_back(face_normal(point(0), point(1), point(2)));
                } else {
                    Point3 normal;
                    for (unsigned k = 0; k < 3; ++k)
                        normal[k] = out.input.projection.local_to_world[k][2];
                    batch.normals.push_back(normal);
                }
            }
        } else {
            ++skipped;
        }
        begin = i + 1;
    }
    require(begin == out.polygon_indices.size(), "native cap unterminated polygon output");
    // The three native coordinate maps are independent. Preserve each map's
    // own insertion order; point/UV batches precede normals from facet order.
    auto coordinates = map_native_builder_coordinates({batch}, {}, budget);
    const auto &mapping = coordinates.batches.front();
    auto &indices = mesh.data.indices.indices;
    std::size_t output_storage = 0;
    auto append = [&](NativePolyfaceChannel channel, std::int32_t value) {
        require(output_storage < budget.max_control_points, "native cap index storage budget");
        ++output_storage;
        work.charge(1);
        mesh.data.indices.active[channel] = true;
        indices[channel].push_back(value);
    };
    auto one_based = [](std::size_t index) {
        require(index < INT32_MAX, "native cap builder index overflow");
        return static_cast<std::int32_t>(index + 1);
    };
    for (std::size_t f = 0; f < facets.size(); ++f) {
        const auto face = facets[f];
        for (auto i = face.begin; i < face.end; ++i) {
            const auto token = static_cast<std::int64_t>(out.polygon_indices[i]);
            const auto source = static_cast<std::size_t>(token < 0 ? -token : token) - 1;
            const auto point = one_based(mapping.points[source]);
            append(point_channel, token > 0 ? point : -point);
            if (options.normals_required)
                append(normal_channel, one_based(mapping.normals[f]));
            if (options.parameters_required)
                append(parameter_channel, one_based(mapping.parameters[source]));
        }
        append(point_channel, 0);
        if (options.normals_required)
            append(normal_channel, 0);
        if (options.parameters_required)
            append(parameter_channel, 0);
    }
    mesh.data.coordinates.points = std::move(coordinates.points);
    mesh.data.coordinates.normals = std::move(coordinates.normals);
    mesh.data.coordinates.parameters = std::move(coordinates.parameters);
    out.report["coordinate_insertion"] = std::move(coordinates.report);
    // SetNewFace recomputes XYZ/normal/UV ranges, retaining supplied distance
    // range; it does not activate the face-data pool itself.
    auto face_data = set_native_polyface_mesh_face_data(mesh, record, 0, budget);
    mesh = std::move(face_data.output);
    out.report["face_data"] = std::move(face_data.report);
    out.report["facet_count"] = facets.size();
    out.report["native_counted_facets"] = counted_facets;
    out.report["skipped_short_faces"] = skipped;
    out.triangulation_succeeded = counted_facets != 0;
    const bool face_data_complete = !options.parameters_required || face_data.complete;
    out.complete = out.triangulation_succeeded && polygon_complete && parameters_complete &&
                   face_data_complete;
    return finish(out.complete ? "cap_mesh_generated" : "cap_mesh_incomplete");
}
NativeTubeMeshCapPair emit_native_tube_mesh_cap_pair(const std::vector<std::vector<Point3>> &start,
                                                     const std::vector<std::vector<Point3>> &end,
                                                     bool eligible,
                                                     const NativeTubeMeshCapOptions &options,
                                                     TubeBudget &budget) {
    NativeTubeMeshCapPair out;
    out.report = {{"eligible", eligible}, {"ring_counts_equal", start.size() == end.size()}};
    if (eligible && start.size() == end.size()) {
        out.start = emit_native_tube_mesh_cap(start, true, options, budget);
        if (out.start->native_succeeded) {
            out.end = emit_native_tube_mesh_cap(end, false, options, budget);
            out.published = out.end->native_succeeded;
            out.complete = out.published && out.start->complete && out.end->complete;
        }
    }
    out.report["published"] = out.published;
    out.report["complete"] = out.complete;
    return out;
}
} // namespace p3d::swept_detail
