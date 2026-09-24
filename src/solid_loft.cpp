#include <p3d/solid.hpp>
#include "internal.hpp"

namespace p3d {
SolidMeshResult mesh_bgfb_section_loft(const Json &table, const PolyfaceMeshOptions &options,
                                       unsigned segments) {
    SolidMeshResult out;
    out.source = table;
    try {
        require(segments >= 3, "loft subdivision count must be at least three");
        LoftMeshOptions sampling;
        sampling.max_uv_edge = 1. / segments;
        sampling.max_vertices = static_cast<unsigned>(
            std::min<std::size_t>(options.max_points, std::numeric_limits<unsigned>::max()));
        sampling.max_triangles = static_cast<unsigned>(
            std::min<std::size_t>(std::min(options.max_triangles, options.max_corners / 3),
                                  std::numeric_limits<unsigned>::max()));
        const auto loft = SectionLoft::from_bgfb(table, sampling.max_cap_control_points);
        const auto mesh = loft.mesh(sampling);
        out.derived.report["loft"] = mesh.report;
        require(mesh.report.at("status") == "complete",
                mesh.report.value("reason", std::string("loft mesh incomplete")));
        require(mesh.face_parameters.size() == mesh.faces.size(), "loft parameter count mismatch");

        std::vector<std::array<std::int64_t, 3>> ids;
        for (const auto &part : mesh.parts) {
            require(part.native_face_indices.has_value(), "loft native face identity unavailable");
            require(part.first_face == ids.size() &&
                        part.face_count <= mesh.faces.size() - ids.size(),
                    "loft mesh part coverage mismatch");
            ids.insert(ids.end(), part.face_count, *part.native_face_indices);
        }
        require(ids.size() == mesh.faces.size(), "loft mesh face coverage mismatch");
        Json points = Json::array(), indices = Json::array();
        for (const auto &p : mesh.vertices)
            for (double x : p)
                points.push_back(x);
        for (const auto &t : mesh.faces) {
            for (auto i : t)
                indices.push_back(std::int64_t(i) + 1);
            indices.push_back(0);
        }
        auto derived = mesh_bgfb_polyface({{"_type", "Polyface"},
                                           {"meshStyle", 1},
                                           {"numPerFace", 0},
                                           {"point", std::move(points)},
                                           {"pointIndex", std::move(indices)}},
                                          options);
        require(derived.status == "meshed",
                derived.report.value("reason", std::string("loft triangle conversion failed")));
        std::vector<std::optional<std::array<Point2, 3>>> parameters;
        for (std::size_t f = 0; f < derived.geometry.faces.size(); ++f) {
            const auto source = derived.geometry.face_source_polygons.at(f);
            require(source && *source < ids.size(), "loft triangle source unavailable");
            const auto &uv = mesh.face_parameters[*source];
            std::optional<std::array<Point2, 3>> result;
            if (uv) {
                result.emplace();
                for (unsigned k = 0; k < 3; ++k) {
                    const auto corner = derived.face_source_corners.at(f)[k];
                    require(corner / 4 == *source && corner % 4 < 3,
                            "loft triangle source corner mismatch");
                    (*result)[k] = (*uv)[corner % 4];
                }
            }
            parameters.push_back(result);
        }
        derived.report["source_kind"] = "P3DSectionLoft";
        derived.report["scope"] = "derived_section_loft";
        derived.report["native_tessellation_equivalence"] = "not_established";
        derived.report["world_space_error_bound"] = nullptr;
        derived.report["surface_parameters"] = "side_bspline_uv_not_material_uv";
        derived.report["placement"] = "reconstruct_from_transformed_source_curves";
        derived.report["loft"] = mesh.report;
        out.derived = std::move(derived);
        out.face_indices = std::move(ids);
        out.surface_parameters = std::move(parameters);
    } catch (const std::exception &e) {
        // Preserve diagnostic evidence, but never expose a partial mesh or maps.
        const auto report = std::move(out.derived.report);
        out.derived = {};
        out.derived.report = report;
        out.derived.report["reason"] = e.what();
        out.face_indices.clear();
        out.surface_parameters.clear();
    }
    return out;
}
} // namespace p3d
