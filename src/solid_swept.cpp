#include <p3d/solid.hpp>
#include <p3d/swept_mesh.hpp>
#include "internal.hpp"
namespace p3d {
SolidMeshResult mesh_bgfb_native_sweep(const Json &table, const PolyfaceMeshOptions &options,
                                       unsigned circle_segments) {
    SolidMeshResult out;
    out.source = table;
    try {
        require(circle_segments >= 3, "native sweep angular subdivision must be at least three");
        SweptBodyMeshOptions native_options;
        native_options.max_control_points = options.max_points;
        native_options.max_work = options.max_curve_work;
        native_options.max_patches = options.max_triangles;
        native_options.max_sample_nodes = options.max_points;
        native_options.angle_tolerance = 6.283185307179586 / circle_segments;
        auto native = mesh_bgfb_swept_body(table, native_options);
        out.derived.report["native_sweep"] = native.report;
        require(native.status == "meshed",
                native.report.value("reason",
                                    std::string("native sweep mesh reconstruction incomplete: ") +
                                        native.status));
        require(native.point_indices.size() <= options.max_corners &&
                    native.point_indices.size() / 4 <= options.max_triangles,
                "native sweep output mesh budget");
        auto flatten = [](const auto &values) {
            Json out = Json::array();
            for (const auto &v : values)
                for (auto x : v)
                    out.push_back(x);
            return out;
        };
        Json generated{{"_type", "Polyface"},
                       {"meshStyle", 1},
                       {"numPerFace", 0},
                       {"twoSided", native.two_sided},
                       {"point", flatten(native.points)},
                       {"pointIndex", native.point_indices},
                       {"normal", flatten(native.normals)},
                       {"normalIndex", native.normal_indices},
                       {"param", flatten(native.parameters)},
                       {"paramIndex", native.parameter_indices}};
        out.derived = mesh_bgfb_polyface(generated, options);
        out.derived.report["source_kind"] = "P3DSweptBody";
        out.derived.report["scope"] = "native_source_sweep_mesh";
        out.derived.report["curve_work_steps"] = native.report.at("work_used");
        out.derived.report["native_sweep"] = std::move(native.report);
        // Runtime face records are not BGFB faceData serialization and are not
        // getFacets material/part IDs. Retain them separately from generated data.
        Json records = Json::array();
        for (const auto &f : native.face_data)
            records.push_back({{"parameter_distance_range", f.parameter_distance_range},
                               {"parameter_range", f.parameter_range},
                               {"point_range", f.point_range},
                               {"normal_range", f.normal_range},
                               {"source_index", f.source_index},
                               {"face_indices", f.face_indices}});
        out.derived.report["native_mesh_metadata"] = {
            {"layout", std::move(native.layout)},
            {"face_data", std::move(records)},
            {"face_data_indices", std::move(native.face_data_indices)}};
        out.derived.report["grouped_facet_id_mapping_status"] = "not_evaluated";
    } catch (const std::bad_alloc &) {
        throw;
    } catch (const std::exception &e) {
        out.derived.status = "not_evaluated";
        out.derived.report["reason"] = e.what();
        out.derived.geometry = {};
    }
    return out;
}
} // namespace p3d
