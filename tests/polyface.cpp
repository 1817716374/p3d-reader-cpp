#include <p3d/polyface.hpp>
#include <p3d/csg_mesh_tree.hpp>
#include "internal.hpp"

namespace {
using namespace p3d;
Json quad() {
    return {{"_type", "Polyface"},
            {"meshStyle", 1},
            {"numPerFace", 0},
            {"point", {0., 0., 0., 2., 0., 0., 2., 2., 0., 0., 2., 0.}},
            {"pointIndex", {1, -2, 3, 4, 0}},
            {"normal", {0., 0., 2.}},
            {"normalIndex", {1, 1, 1, 1, 0}},
            {"param", {0., 0., 1., 0., 1., 1., 0., 1.}},
            {"paramIndex", {4, 3, 2, 1, 0}},
            {"intColor", {0xff0000u, 0x00ff00u, 0x0000ffu, 0xffffffu}},
            {"doubleColor", {1., 0., 0.}},
            {"colorIndex", {1, 2, 3, 4, 0}},
            {"colorTable", nullptr},
            {"taggedNumericData", {{"custom", 42}}}};
}
double area(const Geometry &g) {
    double a = 0;
    for (const auto &f : g.faces) {
        const auto &p = g.vertices[f[0]], &q = g.vertices[f[1]], &r = g.vertices[f[2]];
        a += ((q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])) * .5;
    }
    return a;
}
} // namespace
unsigned polyface_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto table = quad();
    table["point"].push_back(999); // Native reader copies complete triples only.
    auto result = mesh_bgfb_polyface(table);
    check(result.status == "meshed", result.report.dump().c_str());
    check(result.source == table && result.geometry.vertices.size() == 4,
          "source tails and unknown attributes retained");
    check(result.report["channels"]["point"]["ignored_tail_scalars"] == 1,
          "native point scalar tail");
    check(result.geometry.faces.size() == 2 && area(result.geometry) == 4,
          "quad area and triangle count");
    for (std::size_t f = 0; f < result.geometry.faces.size(); ++f) {
        check(result.geometry.face_source_polygons[f] == 0, "triangles identify original polygon");
        check(result.geometry.face_normal_indices[f].has_value(), "explicit normal binding");
        check(result.face_int_color_indices[f].has_value(), "integer color binding");
        check(!result.face_double_color_indices[f].has_value(),
              "short float color pool is not hidden by longer integer pool");
        for (unsigned k = 0; k < 3; ++k) {
            const auto corner = result.face_source_corners[f][k];
            const auto signed_index = table["pointIndex"][corner].get<int>();
            check(result.geometry.faces[f][k] == unsigned(std::abs(signed_index) - 1),
                  "signed source point correspondence");
            check((*result.geometry.face_uv_indices[f])[k] == 3 - corner,
                  "UV uses independent original corner index");
        }
    }
    check(result.geometry.source_normals[0][2] == 2, "source normal magnitude not rewritten");
    check(result.report["attribute_bindings"]["doubleColor"]["status"] == "invalid_indices",
          "geometry success does not imply every attribute pool is mapped");
    auto fixed = quad();
    fixed["numPerFace"] = 5;
    auto f = mesh_bgfb_polyface(fixed);
    check(f.status == "meshed" && area(f.geometry) == 4, "fixed-width padded face");
    fixed["pointIndex"] = {1, 2, 0, 3, 0};
    check(mesh_bgfb_polyface(fixed).status != "meshed",
          "noncanonical fixed padding cannot silently drop later corners");
    auto concave = quad();
    concave["point"] = {0., 0., 0., 2., 0., 0., 2., 1., 0., 1., 1., 0., 1., 2., 0., 0., 2., 0.};
    concave["pointIndex"] = {1, 2, 3, 4, 5, 6, 0};
    f = mesh_bgfb_polyface(concave);
    check(f.status == "meshed" && f.geometry.faces.size() == 4 && area(f.geometry) == 3,
          "concave face triangulation");
    concave["pointIndex"] = {6, 5, 4, 3, 2, 1, 0};
    f = mesh_bgfb_polyface(concave);
    check(f.status == "meshed" && area(f.geometry) == -3, "source face winding retained");
    auto tiny = quad();
    for (auto &v : tiny["point"])
        v = v.get<double>() * 1e-12;
    f = mesh_bgfb_polyface(tiny);
    check(f.status == "meshed" && std::abs(area(f.geometry) - 4e-24) < 1e-35,
          "small valid geometry is not lost to absolute projection thresholds");
    auto crossed = quad();
    crossed["pointIndex"] = {1, 3, 2, 4, 0};
    check(mesh_bgfb_polyface(crossed).geometry.faces.empty(),
          "self-intersection cannot masquerade as a complete face");
    auto bad = quad();
    bad["pointIndex"] = {1, 2, 5, 0};
    f = mesh_bgfb_polyface(bad);
    check(f.status != "meshed" && f.geometry.vertices.empty() && f.source == bad,
          "bad geometry fails atomically but retains source");
    bad = quad();
    bad["pointIndex"] = {1, 2, 3};
    check(mesh_bgfb_polyface(bad).status != "meshed", "unterminated indexed face rejected");
    auto implicit = quad();
    implicit["meshStyle"] = 4;
    implicit["paramIndex"] = {99}; // Native sequential conversion replaces indices.
    f = mesh_bgfb_polyface(implicit);
    check(f.status == "meshed" && area(f.geometry) == 4, "implicit quad list");
    for (std::size_t i = 0; i < f.geometry.faces.size(); ++i)
        check(f.geometry.face_uv_indices[i] == std::optional<Triangle>(f.geometry.faces[i]),
              "implicit parameter binding follows point sequence, not stored index array");
    implicit["meshStyle"] = 3;
    f = mesh_bgfb_polyface(implicit);
    check(f.status == "meshed" && f.geometry.faces.size() == 1 &&
              f.report["unused_trailing_points"] == 1 && f.geometry.vertices.size() == 4,
          "implicit triangle list retains incomplete source tail points");
    implicit["meshStyle"] = 5;
    check(mesh_bgfb_polyface(implicit).status != "meshed", "grid layout not guessed");
    PolyfaceMeshOptions options;
    options.max_triangles = 1;
    check(mesh_bgfb_polyface(quad(), options).geometry.faces.empty(), "triangle output budget");
    options = {};
    options.max_polygon_edge_tests = 0;
    check(mesh_bgfb_polyface(quad(), options).status != "meshed", "edge validation budget");
    auto tetra = quad();
    tetra["point"] = {0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0., 1.};
    tetra["pointIndex"] = {1, 3, 2, 0, 1, 2, 4, 0, 1, 4, 3, 0, 2, 3, 4, 0};
    for (const char *key :
         {"normal", "normalIndex", "param", "paramIndex", "intColor", "doubleColor", "colorIndex"})
        tetra[key] = nullptr;
    Json entry = {{"status", "decoded"}, {"encoding", "bgfb"}, {"geometry", {{"geometry", tetra}}}};
    Json node = {{"status", "decoded"},
                 {"operation", 0},
                 {"geometry_indices", {0}},
                 {"cache_indices", Json::array()},
                 {"matrix_indices", Json::array()},
                 {"is_old_value", 0},
                 {"left_index", nullptr},
                 {"right_index", nullptr}};
    Json archive = {{"geometries", {entry}},
                    {"node_caches", Json::array()},
                    {"transforms", Json::array()},
                    {"angle_tolerance", 0},
                    {"tree", {{"nodes", {node}}, {"root_index", 0}}}};
    auto tree = evaluate_csg_polyface_archive(archive);
    check(tree.result.status == "evaluated" && tree.sources.size() == 1 &&
              tree.result.meshes[0].faces.size() == 4,
          "decoded BGFB Polyface enters CSG tree without caller triangulation");
    for (const auto &source : tree.result.meshes[0].face_sources)
        check(tree.sources.at(source.geometry_index)
                  .geometry.face_source_polygons.at(source.face_index)
                  .has_value(),
              "CSG output links through source triangle back to stored polygon");
    archive["geometries"].push_back(entry);
    options = {};
    options.max_points = 7;
    tree = evaluate_csg_polyface_archive(archive, {}, options);
    check(tree.result.status != "evaluated" && tree.result.meshes.empty(),
          "source point budget shared across archive geometries");
    return checks;
}
