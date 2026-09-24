#include <p3d/solid.hpp>
#include <p3d/csg.hpp>
#include <p3d/csg_mesh_tree.hpp>
#include "geometry.hpp"
#include <cstring>
#include <map>

namespace {
using namespace p3d;
constexpr double pi = 3.141592653589793238462643383279502884;
Json sphere(double start = -pi / 2, double sweep = pi, bool capped = false) {
    Json frame;
    for (unsigned r = 0; r < 3; ++r)
        for (unsigned c = 0; c < 4; ++c)
            frame[std::string("a") + "xyz"[r] + "xyzw"[c]] = r == c ? 1. : 0.;
    return {{"_type", "DgnSphere"},
            {"detail",
             {{"localToWorld", frame},
              {"startLatitudeRadians", start},
              {"latitudeSweepRadians", sweep},
              {"capped", capped}}}};
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
Json decoded_sphere() {
    Bytes g(176);
    std::memcpy(g.data(), "bg0001fb", 8);
    at(g, 8, 12);
    at(g, 12, std::uint16_t(8));
    at(g, 14, std::uint16_t(12));
    at(g, 16, std::uint16_t(4));
    at(g, 18, std::uint16_t(8));
    at(g, 20, 8);
    g[24] = 7;
    at(g, 28, 20);
    at(g, 32, std::uint16_t(6));
    at(g, 34, std::uint16_t(128));
    at(g, 36, std::uint16_t(8));
    at(g, 48, 16);
    const double v[] = {2, .25, .4, .5, 0, 3, 0, -.25, 0, 0, 4, 1, -.5, 1};
    for (unsigned i = 0; i < 14; ++i)
        at(g, 56 + 8 * i, v[i]);
    g[168] = 1;
    Bytes b;
    put(b, 22);
    put(b, 1);
    block(b, g);
    put(b, 0);
    put(b, 0);
    put(b, 8);
    block(b, {});
    block(b, {});
    block(b, {});
    return decode_csg_bytes(b).at("geometries")[0].at("geometry").at("geometry");
}
double volume(const std::vector<Point3> &points, const std::vector<Triangle> &faces) {
    double v = 0;
    for (auto f : faces) {
        const auto &a = points[f[0]], &b = points[f[1]], &c = points[f[2]];
        v += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
             a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    return v / 6;
}
double polygon_band_volume(double a, double b, unsigned n, std::size_t bands) {
    double v = 0;
    for (std::size_t i = 0; i < bands; ++i) {
        const auto x = a + (b - a) * double(i) / double(bands);
        const auto y = a + (b - a) * double(i + 1) / double(bands);
        const auto r = std::cos(x), s = std::cos(y);
        v += (std::sin(y) - std::sin(x)) * (r * r + r * s + s * s);
    }
    return std::abs(v * n * std::sin(2 * pi / n) / 6);
}
Json archive(const Json &table) {
    return {{"geometries",
             {{{"status", "decoded"}, {"encoding", "bgfb"}, {"geometry", {{"geometry", table}}}}}},
            {"node_caches", Json::array()},
            {"angle_tolerance", 8},
            {"transforms",
             {{{"matrix_3x4_rows", {{2., 0., 0., 0.}, {0., 3., 0., 0.}, {0., 0., 4., 0.}}}}}},
            {"tree",
             {{"root_index", 0},
              {"nodes",
               {{{"status", "decoded"},
                 {"operation", 0},
                 {"geometry_indices", {0}},
                 {"cache_indices", Json::array()},
                 {"matrix_indices", {0}},
                 {"is_old_value", 0},
                 {"left_index", nullptr},
                 {"right_index", nullptr}}}}}}};
}
} // namespace
unsigned sphere_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) {
        ++checks;
        require(value, message);
    };
    const auto decoded = decoded_sphere();
    check(decoded["detail"]["localToWorld"]["axy"] == .25 &&
              decoded["detail"]["localToWorld"]["axw"] == .5 &&
              decoded["detail"]["startLatitudeRadians"] == -.5 &&
              decoded["detail"]["latitudeSweepRadians"] == 1,
          "BGFB sphere wire matrix and angles");
    auto mesh = mesh_bgfb_solid(decoded, {}, 16);
    check(mesh.derived.status == "meshed" && mesh.source == decoded,
          "binary sphere detail reaches mesh without changing source");
    for (unsigned n : {3u, 8u, 16u})
        for (auto endpoints : std::vector<Point2>{{-pi / 2, pi / 2},
                                                  {pi / 2, -pi / 2},
                                                  {0, pi / 2},
                                                  {pi / 2, 0},
                                                  {-.5, .75},
                                                  {.75, -.5}})
            for (int hand : {-1, 1}) {
                auto table = sphere(endpoints[0], endpoints[1] - endpoints[0], true);
                auto &t = table["detail"]["localToWorld"];
                t["axx"] = 2. * hand;
                t["axy"] = .25;
                t["axz"] = .4;
                t["axw"] = .5;
                t["ayy"] = 3.;
                t["ayw"] = -.25;
                t["azz"] = 4.;
                t["azw"] = 1.;
                auto result = mesh_bgfb_solid(table, {}, n);
                check(result.derived.status == "meshed", result.derived.report.dump().c_str());
                check(result.source == table, "sphere analytic source unchanged");
                const auto &g = result.derived.geometry;
                const auto bands = result.derived.report["latitude_bands"].get<std::size_t>();
                const auto expected =
                    24 * polygon_band_volume(endpoints[0], endpoints[1], n, bands);
                check(std::abs(volume(g.vertices, g.faces) - expected) <
                          1e-9 * std::max(1., expected),
                      "sphere frustum-sum volume after shear/reflection/reversed latitude");
                std::map<std::pair<std::uint32_t, std::uint32_t>, std::pair<unsigned, int>> edges;
                for (const auto &f : g.faces)
                    for (unsigned k = 0; k < 3; ++k) {
                        const auto a = f[k], b = f[(k + 1) % 3];
                        auto &e = edges[std::minmax(a, b)];
                        ++e.first;
                        e.second += a < b ? 1 : -1;
                    }
                for (const auto &e : edges)
                    check(e.second.first == 2 && e.second.second == 0,
                          "shared poles and caps have opposite paired edges");
                for (const auto &p : g.vertices) {
                    const double z = (p[2] - 1) / 4, y = (p[1] + .25) / 3;
                    const double x = (p[0] - .5 - .25 * y - .4 * z) / (2 * hand);
                    check(std::abs(x * x + y * y + z * z - 1) < 1e-12,
                          "sphere points retain affine ellipsoid");
                }
                for (const auto &p : g.face_source_polygons) {
                    check(p && *p < result.face_indices.size(), "sphere face provenance");
                    const auto face = result.face_indices[*p];
                    check(face[0] == -1 || face == std::array<std::int64_t, 3>{0, 0, 0},
                          "one analytic sphere side face");
                }
            }
    auto info = native_bgfb_sphere_parameters(sphere(pi / 2, -pi));
    check(info["caps"] == Json({false, false}) && info["native_is_closed_solid"] == true &&
              info["sweep_info"]["native_return_value"] == false,
          "uncapped full reversed sphere native closure query");
    info = native_bgfb_sphere_parameters(sphere(2, -4), true);
    check(info["source_latitudes"] == Json({2., -2.}) &&
              info["sweep_info"]["latitudes"] == Json({-pi / 2, pi / 2}) &&
              info["native_is_closed_solid"] == true,
          "sweep query clamps and orders without replacing raw latitude domain");
    check(mesh_bgfb_solid(sphere(2, -4)).derived.status != "meshed",
          "clamped closed query is not a geometry validity guarantee");
    info = native_bgfb_sphere_parameters(sphere(0, pi / 2 - 5e-13, true));
    check(info["caps"] == Json({true, false}), "native cap polar tolerance");
    info = native_bgfb_sphere_parameters(sphere(0, pi / 2 - 2e-12, true));
    check(info["caps"] == Json({true, true}), "cap below strict polar threshold");
    auto open = mesh_bgfb_solid(sphere(-.5, 1, false), {}, 8);
    check(open.derived.status == "meshed", "uncapped spherical band meshes");
    for (auto face : open.face_indices)
        check(face == std::array<std::int64_t, 3>{0, 0, 0}, "open band has no cap identities");
    auto singular = sphere();
    singular["detail"]["localToWorld"]["axx"] = 0.;
    check(native_bgfb_sphere_parameters(singular)["status"] == "evaluated" &&
              mesh_bgfb_solid(singular).derived.status != "meshed",
          "parameter query does not imply invertible geometry");
    check(mesh_bgfb_solid(sphere(0, 0, true)).derived.status != "meshed", "zero latitude extent");
    check(mesh_bgfb_solid(sphere(), {}, 2).derived.status != "meshed",
          "invalid sphere segment count");
    check(mesh_bgfb_solid(sphere(), {}, std::numeric_limits<unsigned>::max()).derived.status !=
              "meshed",
          "sphere resource/count overflow checked before allocation");
    PolyfaceMeshOptions budget;
    budget.max_points = 5;
    check(mesh_bgfb_solid(sphere(), budget, 8).derived.status != "meshed", "sphere point budget");
    auto input = archive(sphere());
    const auto original = input;
    CsgMeshTreeOptions options;
    options.solid_circle_segments = 8;
    auto csg = evaluate_csg_polyface_archive(input, options);
    check(csg.result.status == "evaluated" && input == original,
          csg.result.diagnostics.dump().c_str());
    const auto unit_volume = polygon_band_volume(-pi / 2, pi / 2, 8, 4);
    check(std::abs(volume(csg.result.meshes[0].vertices, csg.result.meshes[0].faces) -
                   24 * unit_volume) < 1e-8,
          "sphere native placement preserves all affine axis scales");
    for (const auto &f : csg.result.meshes[0].face_sources)
        check(f.source_kind == CsgSourceKind::solid && f.source_to_result[1][1] == 3 &&
                  f.corner_projection_distance[0] < 1e-8,
              "sphere CSG affine provenance");
    input["tree"]["nodes"][0]["geometry_indices"] = {0, 0};
    csg = evaluate_csg_polyface_archive(input, options);
    check(csg.result.status == "evaluated", csg.result.diagnostics.dump().c_str());
    check(std::abs(volume(csg.result.meshes[0].vertices, csg.result.meshes[0].faces) -
                   576 * unit_volume) < 1e-7,
          "repeated sphere transformation and independent conversion snapshots");
    return checks;
}
