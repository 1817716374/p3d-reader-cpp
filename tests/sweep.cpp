#include <p3d/solid.hpp>
#include <p3d/csg_mesh_tree.hpp>
#include "geometry.hpp"
#include <map>

namespace {
using namespace p3d;
Json line(std::vector<Point3> points) {
    Json a = Json::array();
    for (auto p : points)
        for (double v : p)
            a.push_back(v);
    return {{"_type", "LineString"}, {"points", a}};
}
Json loop(Json p, int kind = 2) {
    return {
        {"_type", "CurveVector"},
        {"type", kind},
        {"curves", {{{"geometry", p}, {"geometryType", p.at("_type") == "LineString" ? 4 : 2}}}}};
}
Json rectangle(double x, double y, double width, double height, double z = 0,
               bool reverse = false) {
    std::vector<Point3> p{
        {x, y, z}, {x + width, y, z}, {x + width, y + height, z}, {x, y + height, z}, {x, y, z}};
    if (reverse)
        std::reverse(p.begin(), p.end());
    return loop(line(p));
}
Json region(Json children, int type = 4) {
    Json members = Json::array();
    for (const auto &c : children)
        members.push_back({{"geometryType", 5}, {"geometry", c}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", members}};
}
Json extrude(Json p, Point3 d = {0, 0, 3}, bool capped = true) {
    return {{"_type", "DgnExtrusion"},
            {"baseCurve", p},
            {"capped", capped},
            {"extrusionVector", {{"x", d[0]}, {"y", d[1]}, {"z", d[2]}}}};
}
double volume(const Geometry &g) {
    long double v = 0;
    for (auto t : g.faces) {
        const auto a = g.vertices[t[0]], b = g.vertices[t[1]], c = g.vertices[t[2]];
        v += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
             a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    return double(v / 6);
}
bool closed(const Geometry &g) {
    std::map<std::array<std::uint32_t, 2>, std::pair<unsigned, int>> edges;
    for (auto t : g.faces)
        for (unsigned k = 0; k < 3; ++k) {
            const auto a = t[k], b = t[(k + 1) % 3];
            auto &e = edges[{std::min(a, b), std::max(a, b)}];
            ++e.first;
            e.second += a < b ? 1 : -1;
        }
    for (const auto &e : edges)
        if (e.second != std::make_pair(2u, 0))
            return false;
    return true;
}
Json archive(const Json &table, const Matrix4 &matrix) {
    Json rows = Json::array();
    for (unsigned i = 0; i < 3; ++i)
        rows.push_back(matrix[i]);
    return {{"geometries",
             {{{"status", "decoded"}, {"encoding", "bgfb"}, {"geometry", {{"geometry", table}}}}}},
            {"node_caches", Json::array()},
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
unsigned sweep_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    for (bool reverse : {false, true})
        for (double z : {-3., 3.})
            for (bool capped : {false, true}) {
                auto source = extrude(rectangle(0, 0, 4, 2, 0, reverse), {1, 2, z}, capped);
                auto r = mesh_bgfb_solid(source);
                check(r.derived.status == "meshed", "oblique extrusion meshes");
                check(r.source == source, "extrusion source remains intact");
                check(r.derived.geometry.faces.size() == (capped ? 12 : 8), "extrusion face count");
                if (capped) {
                    check(std::abs(volume(r.derived.geometry) - 24) < 1e-10,
                          "extrusion independent prism volume");
                    check(closed(r.derived.geometry), "extrusion cap and side edges pair");
                }
                std::set<std::array<std::int64_t, 3>> ids(r.face_indices.begin(),
                                                          r.face_indices.end());
                check(ids.count({0, 0, 0}) && ids.count({0, 0, 3}),
                      "polyline native component face IDs");
                check(ids.count({-1, 0, 0}) == unsigned(capped), "cap source ID only when emitted");
            }
    for (bool reverse : {false, true}) {
        auto p = region(
            {rectangle(0, 0, 8, 8), rectangle(2, 2, 4, 4, 0, reverse), rectangle(3, 3, 2, 2)});
        auto r = mesh_bgfb_solid(extrude(p));
        check(r.derived.status == "meshed", "nested parity contours mesh");
        check(std::abs(volume(r.derived.geometry) - 156) < 1e-9,
              "nested island/hole independent volume");
        check(closed(r.derived.geometry), "hole walls follow actual parity orientation");
    }
    auto r = mesh_bgfb_solid(extrude(region({rectangle(0, 0, 2, 2), rectangle(5, 0, 2, 2)}, 5)));
    check(r.derived.status == "meshed" && std::abs(volume(r.derived.geometry) - 24) < 1e-9,
          "union children remain independent region shells");
    check(closed(r.derived.geometry), "independent union shell boundaries");
    auto split = rectangle(0, 0, 2, 2);
    split["curves"] = Json::array();
    const std::vector<Point3> pts{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}, {0, 0, 0}};
    for (std::size_t i = 0; i + 1 < pts.size(); ++i)
        split["curves"].push_back({{"geometry", line({pts[i], pts[i + 1]})}});
    r = mesh_bgfb_solid(extrude(split));
    check(r.derived.status == "meshed" && closed(r.derived.geometry),
          "collinear cap samples preserve side joins");
    check(std::abs(volume(r.derived.geometry) - 12) < 1e-9, "separate primitive prism volume");
    Json arc = {{"_type", "EllipticArc"},
                {"arc",
                 {{"centerX", 0},
                  {"centerY", 0},
                  {"centerZ", 0},
                  {"vector0X", 2},
                  {"vector0Y", 0},
                  {"vector0Z", 0},
                  {"vector90X", 0},
                  {"vector90Y", 1},
                  {"vector90Z", 0},
                  {"startRadians", 0},
                  {"sweepRadians", 6.2831853071795864769}}}};
    for (unsigned segments : {8, 16, 32}) {
        r = mesh_bgfb_solid(extrude(loop(arc)), {}, segments);
        check(r.derived.status == "meshed" && closed(r.derived.geometry),
              "elliptic cylinder shared seam and caps");
        const auto expected = 3 * segments * std::sin(6.2831853071795864769 / segments);
        check(std::abs(volume(r.derived.geometry) - expected) < 1e-9,
              "elliptic cylinder independent polygon volume");
    }
    Json ruled = {
        {"_type", "DgnRuledSweep"},
        {"capped", true},
        {"curves", {rectangle(0, 0, 2, 2), rectangle(0, 0, 4, 4, 3), rectangle(0, 0, 6, 6, 6)}}};
    r = mesh_bgfb_solid(ruled);
    check(r.derived.status == "meshed" && closed(r.derived.geometry),
          "three-section ruled frustum");
    check(std::abs(volume(r.derived.geometry) - 104) < 1e-9,
          "sum of two independent frustum volumes");
    std::set<std::array<std::int64_t, 3>> ids(r.face_indices.begin(), r.face_indices.end());
    check(ids.count({0, 0, 3}) && ids.count({1, 0, 3}), "ruled native section-pair IDs");
    auto twisted = ruled;
    twisted["curves"] = {rectangle(0, 0, 2, 2),
                         loop(line({{1, -1, 3}, {3, 1, 3}, {1, 3, 3}, {-1, 1, 3}, {1, -1, 3}}))};
    r = mesh_bgfb_solid(twisted, {}, 8);
    check(r.derived.status == "meshed" && closed(r.derived.geometry),
          "nonplanar ruled patches have shared intermediate bands");
    check(r.derived.report["derived_ruled_bands"] == 8,
          "warped patch subdivision uses requested count");
    auto open = extrude(loop(line({{0, 0, 0}, {2, 0, 0}, {2, 1, 0}}), 1), {0, 0, 2}, false);
    r = mesh_bgfb_solid(open);
    check(r.derived.status == "meshed" && r.derived.geometry.faces.size() == 4,
          "open extrusion side sheet");
    auto bad = extrude(rectangle(0, 0, 2, 2));
    bad["baseCurve"]["curves"][0]["geometry"]["points"][12] = 1;
    check(mesh_bgfb_solid(bad).derived.status != "meshed",
          "closed flag cannot invent missing source segment");
    bad = ruled;
    bad["curves"][1]["curves"].push_back(bad["curves"][1]["curves"][0]);
    check(mesh_bgfb_solid(bad).derived.status != "meshed",
          "unequal source curves not resampled into guessed correspondence");
    auto budget = PolyfaceMeshOptions{};
    budget.max_triangles = 3;
    r = mesh_bgfb_solid(extrude(rectangle(0, 0, 2, 2)), budget);
    check(r.derived.status != "meshed" && r.derived.geometry.faces.empty(),
          "budget failure does not return partial solid");
    auto m = identity();
    m[0][0] = -2;
    m[0][1] = .5;
    m[1][1] = 3;
    m[2][2] = 4;
    m[0][3] = 10;
    const auto source = extrude(rectangle(0, 0, 2, 2));
    auto result = evaluate_csg_polyface_archive(archive(source, m));
    check(result.result.status == "evaluated", "CSG automatically accepts source extrusion");
    check(result.solid_sources.size() == 1 && result.solid_sources[0].source == source,
          "CSG sweep source provenance");
    check(result.result.meshes.size() == 1, "CSG sweep creates one result");
    Geometry placed;
    placed.vertices = result.result.meshes[0].vertices;
    placed.faces = result.result.meshes[0].faces;
    check(std::abs(volume(placed) - 288) < 1e-8 && closed(placed),
          "CSG extrusion preserves oblique mirrored affine volume");
    for (const auto &face : result.result.meshes[0].face_sources)
        check(face.source_kind == CsgSourceKind::solid && face.source_to_result == identity() &&
                  face.solid_snapshot.has_value() && result.solid_placements[0].matrix == m &&
                  face.corner_projection_distance[0] < 1e-8,
              "CSG source faces retain rebuilt snapshot mapping and source placement");
    auto segments = rectangle(0, 0, 2, 2);
    segments["curves"] = Json::array();
    const std::vector<Point3> corners{{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}, {0, 0, 0}};
    for (unsigned i = 0; i < 4; ++i) {
        Json s = Json::object();
        for (unsigned k = 0; k < 3; ++k) {
            s[std::string("point0") + "XYZ"[k]] = corners[i][k];
            s[std::string("point1") + "XYZ"[k]] = corners[i + 1][k];
        }
        segments["curves"].push_back({{"geometry", {{"_type", "LineSegment"}, {"segment", s}}}});
    }
    r = mesh_bgfb_solid(extrude(segments));
    check(r.derived.status == "meshed" && closed(r.derived.geometry),
          "line-segment primitives mesh");
    ids = {r.face_indices.begin(), r.face_indices.end()};
    check(ids.count({0, 3, 0}) && !ids.count({0, 0, 3}),
          "separate segment IDs differ from polyline components");
    auto nonplanar = rectangle(0, 0, 2, 2);
    nonplanar["curves"][0]["geometry"]["points"][8] = .5;
    r = mesh_bgfb_solid(extrude(nonplanar));
    check(r.derived.status == "meshed" && closed(r.derived.geometry) &&
              r.source == extrude(nonplanar),
          "nonplanar cap retains original height with closed side boundaries");
    check(std::abs(volume(r.derived.geometry) - 12) < 1e-9,
          "nonplanar prism independent projected-area times height volume");
    check(r.derived.report["cap_projections"][0]["bottom"]["max_sample_distance_from_plane"] > .1,
          "nonplanar cap is explicitly reported");
    check(std::find(r.derived.geometry.vertices.begin(), r.derived.geometry.vertices.end(),
                    Point3{2, 2, .5}) != r.derived.geometry.vertices.end(),
          "source cap height is not flattened");
    for (unsigned orientation = 0; orientation < 3; ++orientation)
        for (double sign : {-1., 1.}) {
            auto m = identity();
            m[0] = {0, sign * 2, 0, 13};
            m[1] = {0, 0, 3, -7};
            m[2] = {1, 0, .25, 5};
            for (unsigned i = 0; i < orientation; ++i) {
                const auto row = m[0];
                m[0] = m[1];
                m[1] = m[2];
                m[2] = row;
            }
            auto section = nonplanar;
            auto &values = section["curves"][0]["geometry"]["points"];
            for (std::size_t i = 0; i < values.size(); i += 3) {
                Point3 p{values[i].get<double>(), values[i + 1].get<double>(),
                         values[i + 2].get<double>()};
                p = transform(m, p);
                for (unsigned k = 0; k < 3; ++k)
                    values[i + k] = p[k];
            }
            auto source = extrude(section, vector_transform(m, {0, 0, 3}));
            r = mesh_bgfb_solid(source);
            check(r.derived.status == "meshed" && closed(r.derived.geometry),
                  "nonplanar caps survive axis permutation, shear and mirror");
            check(std::abs(volume(r.derived.geometry) - 72) < 1e-8,
                  "affine nonplanar prism has independently known volume");
            check(r.source == source, "nonplanar affine source is preserved");
        }
    auto high_sample =
        loop(line({{0, 0, 0}, {1, 0, .4}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}, {0, 0, 0}}));
    r = mesh_bgfb_solid(extrude(high_sample));
    check(r.derived.status == "meshed" && closed(r.derived.geometry),
          "projected collinear sample with distinct 3D height retains cap boundary");
    check(std::find(r.derived.geometry.vertices.begin(), r.derived.geometry.vertices.end(),
                    Point3{1, 0, .4}) != r.derived.geometry.vertices.end(),
          "height-bearing boundary sample survives projection");
    auto nonplanar_hole = region({rectangle(0, 0, 8, 8), rectangle(2, 2, 4, 4, .25)});
    r = mesh_bgfb_solid(extrude(nonplanar_hole));
    check(r.derived.status == "meshed" && closed(r.derived.geometry),
          "noncoplanar parity loops remain connected to caps");
    check(std::abs(volume(r.derived.geometry) - 144) < 1e-8,
          "noncoplanar annulus extrusion volume");
    auto rounding = rectangle(0, 1000, 2, 2);
    rounding["curves"] = {{{"geometry", line({{0, 1000, 0}, {2, 1000, 0}, {2, 1002, 0}})}},
                          {{"geometry", line({{2, 1002, 5e-13}, {0, 1002, 0}, {0, 1000, 0}})}}};
    const auto source_rounding = extrude(rounding);
    r = mesh_bgfb_solid(source_rounding);
    check(r.derived.status == "meshed" && closed(r.derived.geometry) && r.source == source_rounding,
          "component near zero can retain source rounding at whole-coordinate scale");
    check(r.derived.report["roundoff_join_count"] == 2 &&
              r.derived.report["max_join_distance"] >= 5e-13,
          "derived adjacent endpoint adjustment is explicitly measured");
    rounding["curves"][1]["geometry"]["points"][2] = 1e-6;
    r = mesh_bgfb_solid(extrude(rounding));
    check(r.derived.status != "meshed" && r.derived.geometry.faces.empty(),
          "actual gap is not merged by derived roundoff handling");
    return n;
}
