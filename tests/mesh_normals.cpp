#include "geometry.hpp"

unsigned mesh_normal_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto close = [](Point3 a, Point3 b) {
        for (unsigned k = 0; k < 3; ++k)
            if (std::abs(a[k] - b[k]) > 1e-12)
                return false;
        return true;
    };
    auto metadata = [](std::vector<int> groups, bool smoothing = true) {
        Json c = {{"source",
                   {{"native_triangulation",
                     {{"status", "mapped"},
                      {"source_polygon_layout_matches", true},
                      {"normal_mode", smoothing ? "smoothing_groups" : "flat_triangles"}}}}},
                  {"triangles", Json::array()}};
        for (int group : groups)
            c["triangles"].push_back(
                {{"source_corners", {0, 1, 2}},
                 {"color_indices", nullptr},
                 {"face_uv_point_indices", nullptr},
                 {"native_triangulation", {{"normal_group", group ? Json(group) : Json()}}}});
        return c;
    };
    auto normal = [](const Json &c, std::size_t f, unsigned k,
                     const char *name = "corner_normals") {
        return c["triangles"][f]["native_triangulation"][name][k].get<Point3>();
    };
    const double h = std::sqrt(.5);
    std::vector<Point3> points{{0, 0, 0}, {1, 0, 0}, {0, 100, 0}, {0, 0, 1}};
    std::vector<Triangle> faces{{0, 1, 2}, {0, 3, 1}};
    auto c = metadata({1, 1});
    evaluate_mesh_normals(c, points, faces);
    check(c["normal_evaluation"]["status"] == "computed" && close(normal(c, 0, 0), {0, h, h}) &&
              close(normal(c, 0, 1), {0, h, h}) && close(normal(c, 0, 2), {0, 0, 1}),
          "unequal face areas contribute equal unit normals at shared positions");
    check(close(normal(c, 1, 1), {0, 1, 0}) &&
              c["normal_evaluation"]["position_registry_count"] == 4,
          "unshared corners retain face normals");
    for (auto groups : {std::vector<int>{1, 3}, std::vector<int>{0, 1}}) {
        auto separate = metadata(groups);
        evaluate_mesh_normals(separate, points, faces);
        check(close(normal(separate, 0, 0), {0, 0, 1}) && close(normal(separate, 1, 0), {0, 1, 0}),
              "group equality is not bit intersection and zero group remains flat");
    }
    auto negative = metadata({-7, -7});
    evaluate_mesh_normals(negative, points, faces);
    check(close(normal(negative, 0, 0), {0, h, h}),
          "negative groups smooth once outer routing is activated");
    auto flat = metadata({-7, -7}, false);
    evaluate_mesh_normals(flat, points, faces);
    check(close(normal(flat, 0, 0), {0, 0, 1}) &&
              flat["normal_evaluation"]["position_registry_count"].is_null(),
          "flat routing does not activate smoothing from negative groups");
    auto zero = metadata({1, 1});
    evaluate_mesh_normals(zero, points, {{0, 1, 2}, {0, 2, 1}});
    check(close(normal(zero, 0, 0), {1, 0, 0}) && close(normal(zero, 1, 2), {1, 0, 0}),
          "opposite unit normals cancel to native positive-X fallback");
    auto degenerate = metadata({0}, false);
    evaluate_mesh_normals(degenerate, points, {{0, 0, 0}});
    check(close(normal(degenerate, 0, 0), {1, 0, 0}),
          "zero-area triangle uses native normalization fallback");
    auto near_points = points;
    near_points.push_back({.75e-7, 0, 0});
    auto near = metadata({1, 1});
    evaluate_mesh_normals(near, near_points, {{0, 1, 2}, {4, 3, 1}});
    check(close(normal(near, 0, 0), {0, h, h}) &&
              near["normal_evaluation"]["position_registry_count"] == 4,
          "position registry joins nearby source vertices without requiring equal indices");
    near_points[4] = {1.5e-7, 0, 0};
    near = metadata({1, 1});
    evaluate_mesh_normals(near, near_points, {{0, 1, 2}, {4, 3, 1}});
    check(close(normal(near, 0, 0), {0, 0, 1}) &&
              near["normal_evaluation"]["position_registry_count"] == 5,
          "outside-tolerance positions keep distinct normal accumulation");
    auto chain = metadata({1});
    std::vector<Point3> chain_points{{0, 0, 0}, {.75e-7, 0, 0}, {1.5e-7, 0, 0}};
    evaluate_mesh_normals(chain, chain_points, {{0, 1, 2}});
    check(chain["normal_evaluation"]["position_registry_count"] == 2,
          "tolerance matching is not a transitive connected-component merge");
    chain = metadata({1});
    evaluate_mesh_normals(chain, chain_points, {{1, 0, 2}});
    check(chain["normal_evaluation"]["position_registry_count"] == 1,
          "first representative follows corner insertion order");
    auto tiny = metadata({0}, false);
    evaluate_mesh_normals(tiny, {{0, 0, 0}, {1e-100, 0, 0}, {0, 1e-100, 0}}, {{0, 1, 2}});
    check(close(normal(tiny, 0, 0), {1, 0, 0}),
          "native squared-magnitude underflow retains the positive-X fallback");
    auto many_points = points;
    std::vector<Triangle> many_faces;
    std::vector<int> many_groups;
    for (unsigned i = 0; i < 80; ++i) {
        const double x = i % 2 ? double(80 - i) : -double(i + 1);
        auto n = std::uint32_t(many_points.size());
        many_points.insert(many_points.end(), {{x * 3, 0, 0}, {x * 3 + 1, 0, 0}, {x * 3, 1, 0}});
        many_faces.push_back({n, n + 1, n + 2});
        many_groups.push_back(1);
    }
    auto many = metadata(many_groups);
    evaluate_mesh_normals(many, many_points, many_faces);
    check(many["normal_evaluation"]["position_registry_count"] == 240,
          "registry retains distinct points through tree rotations");
    for (std::size_t i = 0; i < many_faces.size(); ++i)
        check(close(normal(many, i, 0), {0, 0, 1}),
              "tree insertion order does not change separated triangle normals");
    auto bad = metadata({1, 1});
    auto nonfinite = points;
    nonfinite[0][0] = std::numeric_limits<double>::infinity();
    evaluate_mesh_normals(bad, nonfinite, faces);
    check(bad["normal_evaluation"]["status"] == "invalid" &&
              !bad["triangles"][0]["native_triangulation"].contains("corner_normals"),
          "invalid arithmetic publishes a diagnostic instead of partial corner normals");
    auto unavailable = metadata({1, 1});
    unavailable["source"]["native_triangulation"]["status"] = "no_face_materials";
    evaluate_mesh_normals(unavailable, points, faces);
    check(!unavailable.contains("normal_evaluation"), "unmapped native consumer is not evaluated");
    unavailable["source"].erase("native_triangulation");
    evaluate_mesh_normals(unavailable, points, faces);
    check(!unavailable.contains("normal_evaluation"),
          "missing native routing leaves geometry usable");
    Geometry original;
    original.vertices = points;
    original.faces = faces;
    original.face_uvs.resize(2);
    original.face_source_polygons.resize(2);
    original.face_normal_indices.resize(2);
    original.face_uv_indices.resize(2);
    original.primitive_ranges =
        Json::array({{{"channel", "faces"}, {"start", 0}, {"count", 2}, {"mesh_channels", c}}});
    auto m = identity();
    m[0][0] = -2;
    m[1][1] = 3;
    m[2][2] = 4;
    Geometry merged;
    merge_geometry(merged, original, m);
    const auto &placed = merged.primitive_ranges[0]["mesh_channels"];
    check(merged.vertices.size() == points.size() && merged.normals.empty() &&
              merged.source_normals.empty(),
          "derived normals do not deduplicate source vertices or populate source normal pools");
    check(close(normal(placed, 0, 0), {0, h / 3, h / 4}) &&
              close(normal(placed, 0, 1), {0, 0, .25}) &&
              close(normal(placed, 0, 2), {0, h / 3, h / 4}),
          "derived corner normals use inverse transpose and follow reflected corner ordering");
    check(close(normal(placed, 0, 0, "source_corner_normals"), {0, h, h}),
          "source local derived normal survives instance placement");
    auto singular = identity();
    singular[2][2] = 0;
    Geometry collapsed;
    merge_geometry(collapsed, original, singular);
    const auto &invalid = collapsed.primitive_ranges[0]["mesh_channels"];
    check(
        invalid["normal_evaluation"]["placement_status"] == "singular_or_nonfinite" &&
            invalid["triangles"][0]["native_triangulation"]["corner_normals"][0].is_null() &&
            close(normal(invalid, 0, 0, "source_corner_normals"), {0, h, h}),
        "singular placement retains source local normals while marking placed values unavailable");
    return checks;
}
