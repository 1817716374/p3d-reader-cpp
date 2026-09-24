#include <p3d/solid.hpp>
#include <p3d/csg.hpp>
#include <p3d/csg_mesh_tree.hpp>
#include "geometry.hpp"
#include <cstring>
#include <map>

namespace {
using namespace p3d;
constexpr double tau = 6.283185307179586476925286766559005768;
Json torus(double sweep = tau, double major = 4, double minor = 1, bool capped = true) {
    return {{"_type", "DgnTorusPipe"},
            {"detail",
             {{"centerX", 0.},
              {"centerY", 0.},
              {"centerZ", 0.},
              {"vectorXX", 1.},
              {"vectorXY", 0.},
              {"vectorXZ", 0.},
              {"vectorYX", 0.},
              {"vectorYY", 1.},
              {"vectorYZ", 0.},
              {"majorRadius", major},
              {"minorRadius", minor},
              {"sweepRadians", sweep},
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
Json decoded_torus() {
    Bytes g(160);
    std::memcpy(g.data(), "bg0001fb", 8);
    at(g, 8, 12);
    at(g, 12, std::uint16_t(8));
    at(g, 14, std::uint16_t(12));
    at(g, 16, std::uint16_t(4));
    at(g, 18, std::uint16_t(8));
    at(g, 20, 8);
    g[24] = 8;
    at(g, 28, 20);
    at(g, 32, std::uint16_t(6));
    at(g, 34, std::uint16_t(112));
    at(g, 36, std::uint16_t(8));
    at(g, 48, 16);
    double v[] = {0, 0, 0, 2, 0, 0, .4, 3, 0, 4, 1, tau / 4};
    for (unsigned i = 0; i < 12; ++i)
        at(g, 56 + 8 * i, v[i]);
    g[152] = 1;
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
double volume(const std::vector<Point3> &p, const std::vector<Triangle> &faces) {
    double sum = 0;
    for (auto f : faces) {
        auto a = p[f[0]], b = p[f[1]], c = p[f[2]];
        sum += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
               a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    return sum / 6;
}
double expected(double major, double minor, double scale, double sweep, unsigned n,
                std::size_t bands) {
    return std::abs(major) * minor * minor * scale * scale * (n * std::sin(tau / n) / 2) *
           (bands * std::sin(std::min(std::abs(sweep), tau) / bands));
}
Json archive(const Json &table, const Matrix4 &m) {
    Json rows = Json::array();
    for (unsigned i = 0; i < 3; ++i)
        rows.push_back(m[i]);
    return {{"geometries",
             {{{"status", "decoded"}, {"encoding", "bgfb"}, {"geometry", {{"geometry", table}}}}}},
            {"node_caches", Json::array()},
            {"angle_tolerance", 8},
            {"transforms", {{{"matrix_3x4_rows", rows}}}},
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
unsigned torus_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto decoded = decoded_torus();
    check(decoded["detail"]["vectorXX"] == 2 && decoded["detail"]["majorRadius"] == 4 &&
              decoded["detail"]["sweepRadians"] == tau / 4,
          "torus wire layout");
    auto mesh = mesh_bgfb_solid(decoded, {}, 8);
    check(mesh.derived.status == "meshed" && mesh.source == decoded,
          "binary torus source preserved");
    for (unsigned n : {4u, 8u})
        for (double sweep : {tau, -tau, tau / 4, -tau / 4, 2 * tau})
            for (auto radii : std::vector<Point2>{{4, 1}, {-4, 1}, {4, -1}})
                for (double scale : {1., 2.})
                    for (int hand : {-1, 1}) {
                        auto table = torus(sweep, radii[0], radii[1]);
                        auto &d = table["detail"];
                        d["vectorXX"] = scale * hand;
                        d["vectorYX"] = .4;
                        d["vectorYY"] = 3.;
                        auto r = mesh_bgfb_solid(table, {}, n);
                        check(r.derived.status == "meshed", r.derived.report.dump().c_str());
                        check(r.source == table, "original torus not rewritten");
                        const auto &g = r.derived.geometry;
                        auto bands = r.derived.report["sweep_bands"].get<std::size_t>();
                        double target = expected(radii[0], radii[1], scale, sweep, n, bands);
                        check(std::abs(volume(g.vertices, g.faces) - target) <
                                  1e-8 * std::max(1., target),
                              "polygon-section rotational volume with nonorthogonal axes and "
                              "signed radii");
                        std::map<std::pair<std::uint32_t, std::uint32_t>, std::pair<unsigned, int>>
                            edges;
                        for (auto f : g.faces)
                            for (unsigned k = 0; k < 3; ++k) {
                                auto a = f[k], b = f[(k + 1) % 3];
                                auto &e = edges[std::minmax(a, b)];
                                ++e.first;
                                e.second += a < b ? 1 : -1;
                            }
                        for (const auto &e : edges)
                            check(e.second.first == 2 && e.second.second == 0,
                                  "torus seam/cap edges paired with opposite directions");
                        for (const auto &p : g.vertices) {
                            double radial = std::hypot(p[0], p[1]) / scale;
                            check(std::abs(std::pow(radial - std::abs(radii[0]), 2) + p[2] * p[2] -
                                           radii[1] * radii[1]) < 1e-10,
                                  "native rotated section ignores raw second-vector scale/shear");
                        }
                        for (const auto &f : g.face_source_polygons)
                            check(f && *f < r.face_indices.size(), "torus native face mapping");
                    }
    auto info = native_bgfb_torus_parameters(torus());
    check(info["has_caps"] == false && info["native_full_circle"] == true &&
              info["enumerated_face_indices"].size() == 3,
          "full torus face enumeration differs from actual caps");
    info = native_bgfb_torus_parameters(torus(2 * tau));
    check(info["effective_sweep_radians"] == tau,
          "native rotational sweep limited to one revolution");
    const double threshold = 6.283185307178586;
    info = native_bgfb_torus_parameters(torus(threshold));
    check(info["has_caps"] == true, "full circle strict threshold boundary");
    info = native_bgfb_torus_parameters(torus(std::nextafter(threshold, tau)));
    check(info["has_caps"] == false, "full circle threshold next representable value");
    auto near = mesh_bgfb_solid(torus(tau - 5e-13), {}, 8);
    check(near.derived.status == "meshed" && near.derived.report["seam_joined"] == false &&
              near.face_indices.size() == 64,
          "near-full query does not force source endpoint welding");
    auto open = mesh_bgfb_solid(torus(tau / 4, 4, 1, false), {}, 8);
    check(open.derived.status == "meshed" && open.face_indices.size() == 16,
          "uncapped partial torus");
    check(mesh_bgfb_solid(torus(tau, 1, 1)).derived.status != "meshed" &&
              mesh_bgfb_solid(torus(tau, 1, 2)).derived.status != "meshed",
          "horn/spindle degeneracies explicitly unsupported");
    check(mesh_bgfb_solid(torus(0)).derived.status != "meshed", "zero torus sweep");
    check(mesh_bgfb_solid(torus(), {}, std::numeric_limits<unsigned>::max()).derived.status !=
              "meshed",
          "torus overflow/budget checked before allocation");
    auto m = identity();
    m[0][0] = 2;
    m[1][1] = 3;
    m[2][2] = 4;
    auto placed = transform_bgfb_torus(torus(), m);
    check(placed.status == "transformed" && placed.transformed["detail"]["majorRadius"] == 8 &&
              placed.transformed["detail"]["minorRadius"] == 2,
          "both torus radii use first direction scale");
    check(placed.geometry_transform[0][0] == 2 && placed.geometry_transform[1][1] == 2 &&
              placed.geometry_transform[2][2] == 2,
          "native torus transform reconstructs normal instead of affine height scale");
    auto reflected = identity();
    reflected[0][0] = -1;
    placed = transform_bgfb_torus(torus(), reflected);
    check(placed.status == "transformed" && placed.geometry_transform[2][2] == -1,
          "reflected axes reverse recomputed torus normal");
    auto flat = identity();
    flat[2][2] = 0;
    check(transform_bgfb_torus(torus(), flat).status == "transformed",
          "singular ambient transform can retain a valid native torus plane");
    flat[0][0] = 0;
    check(transform_bgfb_torus(torus(), flat).status != "transformed",
          "collapsed first torus direction rejected");
    auto nonunit = decoded;
    placed = transform_bgfb_torus(nonunit, identity());
    check(placed.status == "transformed" && placed.geometry_transform[0][0] == 1 &&
              placed.geometry_transform[1][1] == 1 && placed.geometry_transform[2][2] == 2,
          "identity transform of nonunit native torus changes section height");
    auto before = mesh_bgfb_solid(nonunit, {}, 8),
         after = mesh_bgfb_solid(placed.transformed, {}, 8);
    check(before.derived.status == "meshed" && after.derived.status == "meshed",
          "nonunit torus meshes around placement");
    for (std::size_t i = 0; i < before.derived.geometry.vertices.size(); ++i) {
        auto p = transform(placed.geometry_transform, before.derived.geometry.vertices[i]);
        for (unsigned k = 0; k < 3; ++k)
            check(std::abs(p[k] - after.derived.geometry.vertices[i][k]) < 1e-12,
                  "effective placement agrees with updated analytic section");
    }
    auto input = archive(torus(), m), original = input;
    CsgMeshTreeOptions options;
    options.solid_circle_segments = 8;
    auto csg = evaluate_csg_polyface_archive(input, options);
    check(csg.result.status == "evaluated" && input == original,
          csg.result.diagnostics.dump().c_str());
    const double v = expected(4, 1, 1, tau, 8, 8);
    check(std::abs(volume(csg.result.meshes[0].vertices, csg.result.meshes[0].faces) - 8 * v) <
              1e-8,
          "CSG uses native torus geometry mapping");
    for (const auto &f : csg.result.meshes[0].face_sources)
        check(f.source_kind == CsgSourceKind::solid && f.source_to_result[2][2] == 2 &&
                  f.corner_projection_distance[0] < 1e-8,
              "torus source provenance uses reconstructed normal scale");
    input["tree"]["nodes"][0]["geometry_indices"] = {0, 0};
    csg = evaluate_csg_polyface_archive(input, options);
    check(csg.result.status == "evaluated", csg.result.diagnostics.dump().c_str());
    check(std::abs(volume(csg.result.meshes[0].vertices, csg.result.meshes[0].faces) - 72 * v) <
              1e-7,
          "disjoint repeated torus conversions preserve separate radius states");
    return checks;
}
