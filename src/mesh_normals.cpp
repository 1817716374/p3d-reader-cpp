#include "geometry.hpp"
#include "mesh_registry.hpp"

namespace p3d {
using mesh_detail::face_normal;
using mesh_detail::unit;

void evaluate_mesh_normals(Json &channels, const std::vector<Point3> &points,
                           const std::vector<Triangle> &faces) {
    const auto &source = channels.at("source");
    if (!source.contains("native_triangulation") || !source["native_triangulation"].is_object())
        return;
    const auto &routing = source["native_triangulation"];
    if (routing.value("status", "") != "mapped" ||
        !routing.value("source_polygon_layout_matches", false))
        return;
    Json report = {{"status", "invalid"},
                   {"triangle_input", "library_triangulation"},
                   {"native_glu_triangulation", "not_evaluated"},
                   {"zero_magnitude_normal", Point3{1, 0, 0}},
                   {"placement", "inverse_transpose_without_renormalization"}};
    try {
        require(channels["triangles"].size() == faces.size(), "mesh normal triangle mapping");
        const bool smoothing = routing["normal_mode"] == "smoothing_groups";
        mesh_detail::Registry<double, 3> positions(1e-7);
        std::vector<std::array<std::size_t, 3>> keys;
        std::vector<Point3> normals;
        std::vector<std::int32_t> groups;
        std::map<std::pair<std::int32_t, std::size_t>, Point3> sums;
        for (std::size_t i = 0; i < faces.size(); ++i) {
            std::array<Point3, 3> p;
            std::array<std::size_t, 3> ids{};
            for (unsigned j = 0; j < 3; ++j) {
                p[j] = points.at(faces[i][j]);
                for (double v : p[j])
                    require(std::isfinite(v), "nonfinite mesh normal position");
                if (smoothing) {
                    ids[j] = positions.insert(p[j]);
                    p[j] = positions.at(ids[j]);
                }
            }
            const auto &g =
                channels["triangles"].at(i).at("native_triangulation").at("normal_group");
            const auto group = smoothing && !g.is_null() ? g.get<std::int32_t>() : 0;
            keys.push_back(ids);
            groups.push_back(group);
            normals.push_back(face_normal(p[0], p[1], p[2]));
            if (group)
                for (auto id : ids)
                    for (unsigned k = 0; k < 3; ++k)
                        sums[{group, id}][k] += normals.back()[k];
        }
        for (auto &sum : sums)
            sum.second = unit(sum.second);
        Json values = Json::array();
        for (std::size_t i = 0; i < faces.size(); ++i) {
            std::array<Point3, 3> n;
            for (unsigned j = 0; j < 3; ++j)
                n[j] = groups[i] ? sums.at({groups[i], keys[i][j]}) : normals[i];
            values.push_back(n);
        }
        for (std::size_t i = 0; i < faces.size(); ++i) {
            auto &triangle = channels["triangles"][i]["native_triangulation"];
            triangle["source_corner_normals"] = values[i];
            triangle["corner_normals"] = values[i];
        }
        report["status"] = "computed";
        report["position_registry_count"] = smoothing ? Json(positions.size()) : Json();
    } catch (const std::exception &e) {
        report["error"] = e.what();
    }
    channels["normal_evaluation"] = std::move(report);
}
} // namespace p3d
