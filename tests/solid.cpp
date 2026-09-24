#include <p3d/solid.hpp>
#include <p3d/csg_mesh_tree.hpp>
#include "geometry.hpp"

namespace {
using namespace p3d;
Json box() {
    return {{"_type", "DgnBox"},
            {"detail",
             {{"baseOriginX", 0.},
              {"baseOriginY", 0.},
              {"baseOriginZ", 0.},
              {"topOriginX", 0.},
              {"topOriginY", 0.},
              {"topOriginZ", 2.},
              {"vectorXX", 1.},
              {"vectorXY", 0.},
              {"vectorXZ", 0.},
              {"vectorYX", 0.},
              {"vectorYY", 1.},
              {"vectorYZ", 0.},
              {"baseX", 2.},
              {"baseY", 2.},
              {"topX", 2.},
              {"topY", 2.},
              {"capped", true}}}};
}
double volume(const std::vector<Point3> &p, const std::vector<Triangle> &faces) {
    double sum = 0;
    for (auto f : faces) {
        const auto a = p[f[0]], b = p[f[1]], c = p[f[2]];
        sum += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
               a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    return sum / 6;
}
Json entry(const Json &table) {
    return {{"status", "decoded"}, {"encoding", "bgfb"}, {"geometry", {{"geometry", table}}}};
}
Json tree(const Json &source, Json indices = {0}, double dx = 1.) {
    Json node = {{"status", "decoded"},         {"operation", 0},
                 {"geometry_indices", indices}, {"cache_indices", Json::array()},
                 {"matrix_indices", {0}},       {"is_old_value", 0},
                 {"left_index", nullptr},       {"right_index", nullptr}};
    return {{"geometries", {source}},
            {"node_caches", Json::array()},
            {"angle_tolerance", 0},
            {"transforms",
             {{{"matrix_3x4_rows", {{1., 0., 0., dx}, {0., 1., 0., 0.}, {0., 0., 1., 0.}}}}}},
            {"tree", {{"root_index", 0}, {"nodes", {node}}}}};
}
} // namespace
unsigned solid_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    const auto original = box();
    auto result = mesh_bgfb_solid(original);
    check(result.derived.status == "meshed" && result.source == original,
          "box source preserved separately from its derived Polyface");
    check(result.derived.geometry.vertices.size() == 8 &&
              result.derived.geometry.faces.size() == 12,
          "box corner pool and planar face triangulation");
    check(volume(result.derived.geometry.vertices, result.derived.geometry.faces) == 8,
          "positive box volume");
    check(result.face_indices ==
              std::vector<std::array<std::int64_t, 3>>{
                  {-1, 0, 0}, {-1, 1, 0}, {0, 0, 0}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}},
          "native cap and side face indices are not material part numbers");
    const auto &corners = result.derived.geometry.vertices;
    check(corners[0] == Point3{0, 0, 0} && corners[2] == Point3{0, 2, 0} &&
              corners[3] == Point3{2, 2, 0} && corners[7] == Point3{2, 2, 2},
          "native binary UV corner order");
    for (int mirror : {-1, 1})
        for (int widths : {-1, 1}) {
            auto skew = box();
            auto &d = skew["detail"];
            d["vectorXX"] = double(mirror);
            d["vectorYX"] = .25;
            d["topOriginX"] = .5;
            d["topOriginY"] = .75;
            d["topOriginZ"] = 4.;
            d["baseX"] = 2. * widths;
            d["topX"] = 4. * widths;
            d["baseY"] = 3.;
            d["topY"] = 1.;
            auto shaped = mesh_bgfb_solid(skew);
            check(shaped.derived.status == "meshed", shaped.derived.report.dump().c_str());
            check(std::abs(volume(shaped.derived.geometry.vertices, shaped.derived.geometry.faces) -
                           68. / 3.) < 1e-10,
                  "skew tapered box analytic integral and outward winding");
        }
    auto open = box();
    open["detail"]["capped"] = false;
    result = mesh_bgfb_solid(open);
    check(result.derived.status == "meshed" && result.derived.geometry.faces.size() == 8 &&
              result.face_indices.front() == std::array<std::int64_t, 3>{0, 0, 0},
          "uncapped box only has side faces");
    for (const char *key : {"baseX", "topY", "vectorXX", "topOriginZ"}) {
        auto invalid = box();
        invalid["detail"][key] = 0.;
        auto rejected = mesh_bgfb_solid(invalid);
        check(rejected.derived.status != "meshed" && rejected.derived.geometry.faces.empty() &&
                  rejected.source == invalid,
              "collapsed box is not invented as a complete solid");
    }
    auto crossing = box();
    crossing["detail"]["topX"] = -2.;
    check(mesh_bgfb_solid(crossing).derived.status != "meshed",
          "crossing box explicitly unsupported");
    PolyfaceMeshOptions budget;
    budget.max_points = 7;
    check(mesh_bgfb_solid(box(), budget).derived.status != "meshed", "derived box source budget");

    auto verify = [&](const CsgPolyfaceArchiveResult &out, double expected_volume) {
        check(out.result.status == "evaluated", out.result.diagnostics.dump().c_str());
        check(out.result.meshes.size() == 1, "CSG solid output group");
        const auto &mesh = out.result.meshes[0];
        check(std::abs(volume(mesh.vertices, mesh.faces) - expected_volume) < 1e-8,
              "CSG solid analytic volume");
        for (std::size_t f = 0; f < mesh.faces.size(); ++f) {
            const auto &s = mesh.face_sources[f];
            const auto &g = s.source_kind == CsgSourceKind::solid
                                ? out.solid_sources.at(s.geometry_index).derived.geometry
                                : out.sources.at(s.geometry_index).geometry;
            for (unsigned k = 0; k < 3; ++k) {
                Point3 p{};
                for (unsigned j = 0; j < 3; ++j)
                    for (unsigned axis = 0; axis < 3; ++axis)
                        p[axis] += s.corner_barycentric[k][j] *
                                   g.vertices.at(g.faces.at(s.face_index)[j])[axis];
                p = transform(s.source_to_result, p);
                for (unsigned axis = 0; axis < 3; ++axis)
                    check(std::abs(p[axis] - mesh.vertices[mesh.faces[f][k]][axis]) < 1e-8,
                          "solid/source-kind provenance reconstructs output corner");
            }
        }
    };
    auto input = tree(entry(box()), {0, 0});
    const auto saved = input;
    auto csg = evaluate_csg_polyface_archive(input);
    verify(csg, 12);
    bool first = false, second = false;
    for (const auto &s : csg.result.meshes[0].face_sources) {
        check(s.source_kind == CsgSourceKind::solid, "solid face source kind");
        first |= s.source_to_result[0][3] == 1;
        second |= s.source_to_result[0][3] == 2;
    }
    check(first && second && input == saved && csg.sources.empty() && csg.solid_sources.size() == 1,
          "each repeated solid conversion is distinct, not a shared Polyface cache");
    check(csg.result.diagnostics["solid_mesh_conversions"] == 2 &&
              csg.result.diagnostics["solid_triangles"] == 24,
          "solid conversion work accounted");
    auto nested =
        tree({{"status", "decoded"}, {"encoding", "csg_archive"}, {"geometry", tree(entry(box()))}},
             {0, 0}, 2.);
    csg = evaluate_csg_polyface_archive(nested);
    verify(csg, 12);
    check(csg.solid_source_paths[0].size() == 2, "nested analytic source path");
    // Mixed pools both have index zero: source_kind distinguishes them.
    input = tree(entry(box()), {0, 1}, 0.);
    auto shifted = box();
    shifted["detail"]["baseOriginX"] = 1.;
    shifted["detail"]["topOriginX"] = 1.;
    input["geometries"].push_back(entry(mesh_bgfb_solid(shifted).derived.source));
    csg = evaluate_csg_polyface_archive(input);
    verify(csg, 12);
    first = second = false;
    for (const auto &s : csg.result.meshes[0].face_sources) {
        first |= s.source_kind == CsgSourceKind::solid;
        second |= s.source_kind == CsgSourceKind::polyface;
    }
    check(first && second && csg.sources.size() == 1 && csg.solid_sources.size() == 1,
          "mixed analytic and Polyface sources stay distinct without content deduplication");
    check(evaluate_csg_polyface_archive(tree(entry(open))).result.status == "evaluated",
          "single open solid converts without requiring a closed Boolean operand");
    check(evaluate_csg_polyface_archive(tree(entry(open), {0, 0})).result.status != "evaluated",
          "actual open-solid Boolean remains unsupported");
    CsgMeshTreeOptions limits;
    limits.max_solid_triangles = 23;
    auto limited = evaluate_csg_polyface_archive(tree(entry(box()), {0, 0}), limits);
    check(limited.result.status == "work_limit" && limited.result.meshes.empty(),
          "repeated solid conversion shares cumulative triangle budget");
    return checks;
}
