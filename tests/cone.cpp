#include <p3d/solid.hpp>
#include <p3d/csg.hpp>
#include <p3d/csg_mesh_tree.hpp>
#include "geometry.hpp"
#include <cstring>

namespace {
using namespace p3d;
constexpr double pi = 3.141592653589793238462643383279502884;
Json cone(double a = 1, double b = 2) {
    return {{"_type", "DgnCone"},
            {"detail",
             {{"centerAX", 0.},
              {"centerAY", 0.},
              {"centerAZ", 0.},
              {"centerBX", 0.},
              {"centerBY", 0.},
              {"centerBZ", 3.},
              {"vector0X", 1.},
              {"vector0Y", 0.},
              {"vector0Z", 0.},
              {"vector90X", 0.},
              {"vector90Y", 1.},
              {"vector90Z", 0.},
              {"radiusA", a},
              {"radiusB", b},
              {"capped", true}}}};
}
template <class T> void put(Bytes &b, T v) {
    const auto *p = reinterpret_cast<const std::uint8_t *>(&v);
    b.insert(b.end(), p, p + sizeof(v));
}
template <class T> void at(Bytes &b, std::size_t i, T v) {
    std::memcpy(b.data() + i, &v, sizeof(v));
}
void block(Bytes &b, const Bytes &v) {
    put(b, int(v.size()));
    b.insert(b.end(), v.begin(), v.end());
}
Json decoded_cone() {
    Bytes geometry(176);
    std::memcpy(geometry.data(), "bg0001fb", 8);
    at(geometry, 8, 12);
    at(geometry, 12, std::uint16_t(8));
    at(geometry, 14, std::uint16_t(12));
    at(geometry, 16, std::uint16_t(4));
    at(geometry, 18, std::uint16_t(8));
    at(geometry, 20, 8);
    geometry[24] = 6;
    at(geometry, 28, 20);
    at(geometry, 32, std::uint16_t(6));
    at(geometry, 34, std::uint16_t(128));
    at(geometry, 36, std::uint16_t(8));
    at(geometry, 48, 16);
    // Native GeConeInfo layout stores radiusB before radiusA; no permutation
    // occurs in either the BGFB writer or the fixed-detail reader.
    const double values[] = {0, 0, 0, 0, 0, 3, 1, 0, 0, 0, 1, 0, 5, 2};
    for (unsigned i = 0; i < 14; ++i)
        at(geometry, 56 + 8 * i, values[i]);
    geometry[168] = 1;
    Bytes b;
    put(b, 22);
    put(b, 1);
    block(b, geometry);
    put(b, 0);
    put(b, 0);
    put(b, 8);
    block(b, {});
    block(b, {});
    block(b, {});
    return decode_csg_bytes(b).at("geometries")[0].at("geometry").at("geometry");
}
double volume(const std::vector<Point3> &p, const std::vector<Triangle> &f) {
    double v = 0;
    for (auto t : f) {
        auto a = p[t[0]], b = p[t[1]], c = p[t[2]];
        v += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
             a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    return v / 6;
}
Json archive(const Json &table, const Matrix4 &m, Json indices = {0}) {
    Json rows = Json::array();
    for (unsigned r = 0; r < 3; ++r)
        rows.push_back(m[r]);
    return {{"geometries",
             {{{"status", "decoded"}, {"encoding", "bgfb"}, {"geometry", {{"geometry", table}}}}}},
            {"node_caches", Json::array()},
            {"angle_tolerance", 0},
            {"transforms", {{{"matrix_3x4_rows", rows}}}},
            {"tree",
             {{"root_index", 0},
              {"nodes",
               {{{"status", "decoded"},
                 {"operation", 0},
                 {"geometry_indices", indices},
                 {"cache_indices", Json::array()},
                 {"matrix_indices", {0}},
                 {"is_old_value", 0},
                 {"left_index", nullptr},
                 {"right_index", nullptr}}}}}}};
}
} // namespace
unsigned cone_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto decoded = decoded_cone();
    check(decoded["detail"]["radiusA"] == 2 && decoded["detail"]["radiusB"] == 5,
          "P3D cone wire radii are B then A");
    auto decoded_mesh = mesh_bgfb_solid(decoded, {}, 8);
    check(decoded_mesh.derived.status == "meshed" &&
              decoded_mesh.derived.geometry.vertices[0][0] == 2 &&
              decoded_mesh.derived.geometry.vertices[8][0] == 5,
          "different end radii placed at their correct centers");
    for (unsigned n : {8u, 16u, 64u})
        for (auto radii : std::vector<Point2>{{1, 2}, {2, 2}, {0, 2}, {2, 0}, {-1, -2}})
            for (int mirror : {-1, 1}) {
                auto table = cone(radii[0], radii[1]);
                auto &d = table["detail"];
                d["vector0X"] = double(mirror);
                d["vector90X"] = .25;
                d["vector90Y"] = 1.5;
                d["centerBX"] = .5;
                auto result = mesh_bgfb_solid(table, {}, n);
                check(result.derived.status == "meshed", result.derived.report.dump().c_str());
                check(result.source == table, "cone source parameters remain intact");
                const auto &g = result.derived.geometry;
                const double expected =
                    4.5 * n * std::sin(2 * pi / n) *
                    (radii[0] * radii[0] + radii[0] * radii[1] + radii[1] * radii[1]) / 6;
                check(std::abs(volume(g.vertices, g.faces) - expected) < 1e-8,
                      "ruled polygon-frustum volume with apex/skew/mirror");
                const bool apex = radii[0] == 0 || radii[1] == 0;
                check(g.faces.size() == (apex ? 2 * n - 2 : 4 * n - 4) &&
                          g.vertices.size() == (apex ? n + 1 : 2 * n),
                      "apex shared as one derived corner; nondegenerate cap triangles");
                for (const auto &index : g.face_source_polygons) {
                    check(index && *index < result.face_indices.size(),
                          "cone native face correspondence");
                    const auto face = result.face_indices[*index];
                    check(face[0] == -1 || face == std::array<std::int64_t, 3>{0, 0, 0},
                          "all side facets identify one native side face");
                }
                check(result.derived.report["source_chord_error_bound"].get<double>() > 0,
                      "sampling reports source-space chord error bound");
            }
    auto open = cone(0, 1);
    open["detail"]["capped"] = false;
    auto result = mesh_bgfb_solid(open, {}, 8);
    check(result.derived.status == "meshed" && result.derived.geometry.faces.size() == 8 &&
              result.face_indices[0] == std::array<std::int64_t, 3>{0, 0, 0},
          "uncapped apex cone contains sides only");
    check(mesh_bgfb_solid(cone(0, 0)).derived.status != "meshed" &&
              mesh_bgfb_solid(cone(-1, 1)).derived.status != "meshed",
          "crossing and collapsed cones rejected");
    check(mesh_bgfb_solid(cone(), {}, 2).derived.status != "meshed", "invalid cone segmentation");
    const Json malformed = {{"_type", 6}};
    auto rejected = mesh_bgfb_solid(malformed);
    check(rejected.derived.status != "meshed" && rejected.source == malformed,
          "malformed solid type returns failure and retains original data");
    PolyfaceMeshOptions budget;
    budget.max_points = 15;
    check(mesh_bgfb_solid(cone(), budget, 8).derived.status != "meshed",
          "cone point budget before allocation");
    auto m = identity();
    m[0][0] = 2;
    m[1][1] = 3;
    m[2][2] = 4;
    auto placed = transform_bgfb_cone(cone(), m);
    check(placed.status == "transformed" && placed.transformed["detail"]["radiusA"] == 2 &&
              placed.transformed["detail"]["radiusB"] == 4,
          "native cone radii use first direction scale");
    check(placed.geometry_transform[0][0] == 2 && placed.geometry_transform[1][1] == 2 &&
              placed.geometry_transform[2][2] == 4,
          "native cone effective placement differs from anisotropic matrix");
    auto nonunit = cone();
    nonunit["detail"]["vector0X"] = 2.;
    nonunit["detail"]["vector90Y"] = 3.;
    placed = transform_bgfb_cone(nonunit, identity());
    check(placed.status == "transformed" && placed.transformed["detail"]["radiusA"] == 2 &&
              std::abs(placed.geometry_transform[1][1] - 2. / 3.) < 1e-15,
          "even identity placement normalizes nonunit native cone directions");
    auto singular = identity();
    singular[0][0] = 0;
    check(transform_bgfb_cone(cone(), singular).status != "transformed",
          "collapsed transformed cone direction");
    auto input = archive(cone(1, 1), m);
    const auto original = input;
    CsgMeshTreeOptions options;
    options.solid_circle_segments = 16;
    auto csg = evaluate_csg_polyface_archive(input, options);
    check(csg.result.status == "evaluated" && input == original,
          csg.result.diagnostics.dump().c_str());
    const double original_volume = 16 * std::sin(2 * pi / 16) * 3 / 2;
    check(std::abs(volume(csg.result.meshes[0].vertices, csg.result.meshes[0].faces) -
                   original_volume * 16) < 1e-8,
          "CSG respects cone normalization instead of ordinary affine determinant");
    for (const auto &f : csg.result.meshes[0].face_sources)
        check(f.source_kind == CsgSourceKind::solid && f.source_to_result[1][1] == 2 &&
                  f.corner_projection_distance[0] < 1e-8,
              "cone effective placement retained in original source provenance");
    input["tree"]["nodes"][0]["geometry_indices"] = {0, 0};
    csg = evaluate_csg_polyface_archive(input, options);
    check(csg.result.status == "evaluated", csg.result.diagnostics.dump().c_str());
    check(std::abs(volume(csg.result.meshes[0].vertices, csg.result.meshes[0].faces) -
                   original_volume * 256) < 1e-7,
          "repeated cone placement updates native parameter state before next normalization");
    return checks;
}
