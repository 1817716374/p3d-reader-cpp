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
    check(result.source_edges.size() == 4 && result.source_edges[0].visible &&
              !result.source_edges[1].visible && result.source_edges[3].end_corner == 0,
          "signed start index defines stored visibility including closing edge");
    std::size_t mapped_edges = 0, diagonals = 0;
    for (std::size_t i = 0; i < result.geometry.faces.size(); ++i)
        for (unsigned k = 0; k < 3; ++k) {
            const auto &link = result.face_source_edges[i][k];
            if (!link) {
                ++diagonals;
                continue;
            }
            ++mapped_edges;
            const auto &edge = result.source_edges.at(link->source_edge);
            check(edge.source_polygon == 0 &&
                      edge.points[link->reversed ? 1 : 0] == result.geometry.faces[i][k] &&
                      edge.points[link->reversed ? 0 : 1] == result.geometry.faces[i][(k + 1) % 3],
                  "triangle edge maps exact original adjacency and direction");
        }
    check(mapped_edges == 4 && diagonals == 2,
          "new diagonal has no invented source visibility flag");
    auto defaults = quad();
    defaults["pointIndex"] = {3, -4, 1, 2, 0};
    defaults["normal"] = {0., 0., 1., 0., 0., 2., 0., 0., 3., 0., 0., 4., 999.};
    defaults["normalIndex"] = nullptr;
    defaults["paramIndex"] = Json::array();
    defaults["colorIndex"] = nullptr;
    auto default_mesh = mesh_bgfb_polyface(defaults);
    check(default_mesh.status == "meshed" && default_mesh.source == defaults &&
              default_mesh.report["attribute_bindings"]["normal"]["binding_rule"] ==
                  "native_equal_point_count_default" &&
              default_mesh.report["attribute_bindings"]["param"]["index_source"] == "pointIndex",
          "native equal-size fallback uses copied pool count and retains raw absent indices");
    for (std::size_t i = 0; i < default_mesh.geometry.faces.size(); ++i) {
        check(default_mesh.geometry.face_normal_indices[i] ==
                      std::optional<Triangle>(default_mesh.geometry.faces[i]) &&
                  default_mesh.geometry.face_uv_indices[i] ==
                      std::optional<Triangle>(default_mesh.geometry.faces[i]),
              "default binding follows signed point values, not corner ordinal");
        check(!default_mesh.face_int_color_indices[i],
              "color does not inherit the normal and UV fallback rule");
    }
    defaults["paramIndex"] = {1, 2, 3, 4, 0};
    defaults["normal"] = {0., 0., 1.};
    default_mesh = mesh_bgfb_polyface(defaults);
    check(default_mesh.report["attribute_bindings"]["normal"]["status"] == "unbound_pool" &&
              default_mesh.report["attribute_bindings"]["param"]["binding_rule"] ==
                  "explicit_indices",
          "different-size pools stay unbound and explicit indices take precedence");
    defaults["paramIndex"] = {99};
    default_mesh = mesh_bgfb_polyface(defaults);
    check(default_mesh.report["attribute_bindings"]["param"]["status"] == "invalid_indices" &&
              default_mesh.report["attribute_bindings"]["param"]["index_source"] == "paramIndex",
          "invalid explicit indices never silently fall back to point indices");
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
    auto retraced = quad();
    retraced["point"] = {0., 0., 0., 4., 0., 0., 2., 0., 0., 2., 2., 0., 0., 2., 0.};
    retraced["pointIndex"] = {1, -2, 3, 4, 5, 0};
    retraced["param"] = {0., 0., 4., 0., 2., 0., 2., 2., 0., 2.};
    retraced["paramIndex"] = {5, 4, 3, 2, 1, 0};
    f = mesh_bgfb_polyface(retraced);
    check(f.status == "meshed" && f.geometry.vertices.size() == 5 && area(f.geometry) == 4,
          "collinear boundary overshoot cancels without welding or deleting source points");
    check(f.source == retraced && f.report["polygons"][0]["source_corners"].size() == 5 &&
              f.report["polygons"][0]["boundary_reductions"][0]["source_corner"] == 1,
          "exact retrace records removed derived corner and preserves full source");
    check(f.source_edges.size() == 5 && !f.source_edges[1].visible,
          "source edges removed from derived boundary remain inspectable");
    bool shortened_edge_unbound = false;
    for (std::size_t i = 0; i < f.geometry.faces.size(); ++i)
        for (unsigned k = 0; k < 3; ++k)
            if (f.geometry.faces[i][k] == 0 && f.geometry.faces[i][(k + 1) % 3] == 2)
                shortened_edge_unbound = !f.face_source_edges[i][k];
    check(shortened_edge_unbound, "shortened boundary does not borrow one original edge flag");
    for (std::size_t i = 0; i < f.geometry.faces.size(); ++i)
        for (unsigned k = 0; k < 3; ++k) {
            const auto corner = f.face_source_corners[i][k];
            check(corner != 1 && f.geometry.faces[i][k] == corner &&
                      (*f.geometry.face_uv_indices[i])[k] == 4 - corner,
                  "retained corner uses original point and independent attribute index");
        }
    retraced["pointIndex"] = {5, 4, 3, -2, 1, 0};
    f = mesh_bgfb_polyface(retraced);
    check(f.status == "meshed" && area(f.geometry) == -4,
          "exact edge cancellation preserves reverse winding");
    // A fully retraced spur produces an adjacent repeated point after its tip
    // is removed. This is local boundary cancellation, not global deduplication.
    retraced["pointIndex"] = {1, 2, 1, 3, 4, 5, 0};
    f = mesh_bgfb_polyface(retraced);
    check(f.status == "meshed" && area(f.geometry) == 4 && f.geometry.vertices.size() == 5,
          "full retraced spur and resulting zero-length edge");
    check(f.report["polygons"][0]["boundary_reduction_tests"].get<std::size_t>() <= 18,
          "local boundary worklist has linear candidate count");
    auto spatial = retraced;
    spatial["pointIndex"] = {1, 2, 3, 4, 5, 0};
    spatial["point"][5] = 0.001; // Only the XY projection is collinear.
    f = mesh_bgfb_polyface(spatial);
    check(f.report["polygons"][0]["boundary_reductions"].empty(),
          "projected collinearity does not erase a three-dimensional feature");
    check(f.status == "meshed" && f.geometry.faces.size() == 3 &&
              f.report["polygons"][0]["projected_boundary_splits"][0]["reason"] ==
                  "retained_spatial_triangle",
          "projected fold retains its nonzero three-dimensional triangle");
    check(f.geometry.faces[0] == Triangle{0, 1, 2} &&
              f.face_source_corners[0] == std::array<std::size_t, 3>{0, 1, 2} &&
              f.geometry.vertices[1][2] == 0.001,
          "spatial fold keeps original coordinates, winding and source corner links");
    PolyfaceMeshOptions fold_budget;
    fold_budget.max_triangles = 2;
    check(mesh_bgfb_polyface(spatial, fold_budget).geometry.faces.empty(),
          "retained spatial triangles count against the triangle budget");
    auto vertical = quad();
    vertical["point"] = {0., 0., 0., 0., 0., 1., 0., 0., 2., 2., 0., 0., 2., 2., 0., 0., 2., 0.};
    vertical["pointIndex"] = {2, 3, 4, 5, 6, 1, 0};
    f = mesh_bgfb_polyface(vertical);
    check(f.status == "meshed" &&
              f.report["polygons"][0]["projected_boundary_splits"][0]["reason"] ==
                  "exact_collinear_edge",
          "forward collinear edge along projection normal produces no zero-area triangle");
    for (const auto &t : f.geometry.faces) {
        const auto &a = f.geometry.vertices[t[0]], &b = f.geometry.vertices[t[1]],
                   &c = f.geometry.vertices[t[2]];
        Point3 cross{};
        for (unsigned k = 0; k < 3; ++k) {
            const auto x = (k + 1) % 3, y = (k + 2) % 3;
            cross[k] = (b[x] - a[x]) * (c[y] - a[y]) - (b[y] - a[y]) * (c[x] - a[x]);
        }
        check(cross != Point3{},
              "projected edge handling never emits an exactly collinear triangle");
    }
    auto diagonal = retraced;
    diagonal["pointIndex"] = {1, 2, 3, 4, 5, 0};
    diagonal["point"] = {0., 0., 0., 4., 4., 4., 2., 2., 2., 0., 4., 2., -2., 2., 0.};
    f = mesh_bgfb_polyface(diagonal);
    check(f.status == "meshed" && f.report["polygons"][0]["boundary_reductions"].size() == 1,
          "exact diagonal retrace in an oblique plane");
    auto near_line = quad();
    const double large = 134217728.;
    near_line["point"] = {0.,        0.,        0., large, large - 1, 0.,
                          large - 1, large - 2, 0., 0.,    large,     0.};
    near_line["pointIndex"] = {1, 2, 3, 4, 0};
    f = mesh_bgfb_polyface(near_line);
    check(f.report["polygons"][0]["boundary_reductions"].empty(),
          "nonzero exact determinant survives floating product cancellation");
    auto collapsed = quad();
    collapsed["pointIndex"] = {1, 2, 1, 0};
    f = mesh_bgfb_polyface(collapsed);
    check(f.status != "meshed" && f.geometry.vertices.empty() && f.source == collapsed,
          "zero-area loop cannot become a successful missing face");
    auto bad = quad();
    bad["pointIndex"] = {1, 2, 5, 0};
    f = mesh_bgfb_polyface(bad);
    check(f.status != "meshed" && f.geometry.vertices.empty() && f.source == bad,
          "bad geometry fails atomically but retains source");
    check(f.source_edges.empty() && f.face_source_edges.empty(),
          "failed geometry returns no partial source edge mappings");
    bad = quad();
    bad["pointIndex"] = {1, 2, 3};
    check(mesh_bgfb_polyface(bad).status != "meshed", "unterminated indexed face rejected");
    auto implicit = quad();
    implicit["meshStyle"] = 4;
    implicit["paramIndex"] = {99}; // Native sequential conversion replaces indices.
    f = mesh_bgfb_polyface(implicit);
    check(f.status == "meshed" && area(f.geometry) == 4, "implicit quad list");
    check(f.source_edges.size() == 4 &&
              std::all_of(f.source_edges.begin(), f.source_edges.end(),
                          [](const PolyfaceSourceEdge &e) { return e.visible; }),
          "implicit lists have the visible flags of generated positive indices");
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
