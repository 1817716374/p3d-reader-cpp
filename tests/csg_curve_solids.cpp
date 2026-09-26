#include <p3d/csg_mesh_tree.hpp>
#include "geometry.hpp"
using namespace p3d;
namespace {
Json cv(Json curves, int type) {
    Json members = Json::array();
    for (auto &c : curves)
        members.push_back({{"geometry", c}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", members}};
}
Json line(Point3 p, Point3 q) {
    Json s;
    for (unsigned i = 0; i < 3; ++i) {
        s[std::string("point0") + "XYZ"[i]] = p[i];
        s[std::string("point1") + "XYZ"[i]] = q[i];
    }
    return {{"_type", "LineSegment"}, {"segment", s}};
}
Json source(const std::string &kind, bool bent = false) {
    const std::vector<Point3> corners{{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}};
    Json lower = Json::array(), upper = Json::array(), guides = Json::array();
    for (unsigned i = 0; i < 4; ++i) {
        auto p = corners[i], q = corners[(i + 1) % 4], a = p, b = q;
        a[2] = b[2] = 3;
        lower.push_back(line(p, q));
        upper.push_back(line(a, b));
        guides.push_back(cv(
            Json::array({{{"_type", "LineString"},
                          {"points",
                           {p[0], p[1], 0., p[0] + (bent ? 1. : 0.), p[1], 1., p[0], p[1], 3.}}}}),
            1));
    }
    const auto bottom = cv(lower, 2), top = cv(upper, 2);
    if (kind == "DgnExtrusion")
        return {{"_type", kind},
                {"baseCurve", bottom},
                {"capped", true},
                {"extrusionVector", {{"x", 0}, {"y", 0}, {"z", 3}}}};
    if (kind == "DgnRuledSweep")
        return {{"_type", kind}, {"curves", {bottom, top}}, {"capped", true}};
    return {{"_type", "P3DSectionLoft"},
            {"section0", bottom},
            {"section1", top},
            {"guide_groups", Json::array({guides})},
            {"capped", true}};
}
Json archive(const Json &s, const Matrix4 &m) {
    Json rows = Json::array();
    for (unsigned i = 0; i < 3; ++i)
        rows.push_back(m[i]);
    return {{"geometries",
             {{{"status", "decoded"}, {"encoding", "bgfb"}, {"geometry", {{"geometry", s}}}}}},
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
double volume(const CsgTreeMesh &m) {
    long double sum = 0;
    for (auto f : m.faces) {
        auto a = m.vertices[f[0]], b = m.vertices[f[1]], c = m.vertices[f[2]];
        sum += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
               a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    return double(sum / 6);
}
bool projection(const CsgPolyfaceArchiveResult &r) {
    if (r.result.status != "evaluated")
        return false;
    for (const auto &m : r.result.meshes)
        for (std::size_t f = 0; f < m.faces.size(); ++f) {
            const auto &s = m.face_sources[f];
            if (!s.solid_snapshot)
                return false;
            const auto &snap = r.solid_snapshots.at(*s.solid_snapshot);
            if (snap.source_index != s.geometry_index)
                return false;
            const auto &g = snap.mesh.derived.geometry;
            for (unsigned k = 0; k < 3; ++k) {
                Point3 p{};
                for (unsigned j = 0; j < 3; ++j)
                    for (unsigned a = 0; a < 3; ++a)
                        p[a] +=
                            s.corner_barycentric[k][j] * g.vertices[g.faces.at(s.face_index)[j]][a];
                p = transform(s.source_to_result, p);
                const auto q = m.vertices[m.faces[f][k]];
                if (std::hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]) > 1e-7)
                    return false;
            }
        }
    return true;
}
} // namespace
unsigned csg_curve_solid_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *message) {
        ++n;
        require(ok, message);
    };
    CsgMeshTreeOptions options;
    options.solid_circle_segments = 3;
    for (const std::string kind : {"DgnExtrusion", "DgnRuledSweep", "P3DSectionLoft"}) {
        auto m = identity();
        m[0][0] = 2;
        m[0][1] = .25;
        m[1][1] = 3;
        m[2][2] = .5;
        m[0][3] = 7;
        const auto s = source(kind);
        const auto input = archive(s, m);
        const auto saved = input;
        const auto r = evaluate_csg_polyface_archive(input, options);
        check(r.result.status == "evaluated" && r.result.meshes.size() == 1,
              "curve solid reconstructs after native placement");
        check(std::abs(std::abs(volume(r.result.meshes[0])) - 36) < 1e-7,
              "reconstructed prism has independent determinant-scaled volume");
        check(projection(r), "snapshot triangles reconstruct every output corner");
        check(r.solid_sources[0].source == s && r.solid_sources[0].derived.status == "deferred" &&
                  input == saved,
              "source solid stays original and is not confused with rebuilt geometry");
        check(r.solid_snapshots.size() == 1 && r.solid_placements.size() == 1 &&
                  r.solid_placements[0].matrix == m,
              "one native source conversion creates one independent snapshot and placement event");
        const auto direct = transform_bgfb_curve_solid(s, m);
        check(direct.status == "transformed" &&
                  r.solid_snapshots[0].mesh.source == direct.transformed,
              "CSG snapshot retains the actual transformed source parameters");
        m = identity();
        m[0][3] = 1;
        auto repeated = archive(s, m);
        repeated["tree"]["nodes"][0]["geometry_indices"] = {0, 0};
        const auto twice = evaluate_csg_polyface_archive(repeated, options);
        check(twice.result.status == "evaluated" && twice.solid_snapshots.size() == 2 &&
                  projection(twice),
              "repeated original reference creates two immutable source snapshots");
        check(twice.solid_placements[1].previous == 0 && twice.solid_snapshots[0].placement == 0 &&
                  twice.solid_snapshots[1].placement == 1,
              "repeated source placement chains refer to previous state without merging matrices");
        check(std::abs(volume(twice.result.meshes[0]) - 18) < 1e-6,
              "union observes first and second placed solids rather than two copies of last state");
        auto cache = archive(s, m);
        const auto child = cache["tree"]["nodes"][0];
        cache["tree"]["nodes"].push_back(child);
        cache["tree"]["nodes"][0]["geometry_indices"] = Json::array();
        cache["tree"]["nodes"][0]["left_index"] = 1;
        const auto moved_cache = evaluate_csg_polyface_archive(cache, options);
        check(moved_cache.result.status == "evaluated" && moved_cache.solid_snapshots.size() == 1 &&
                  projection(moved_cache),
              "later cache placement preserves its snapshot instead of rebuilding original solid");
        check(moved_cache.result.meshes[0].face_sources[0].source_to_result[0][3] == 1,
              "snapshot-to-result matrix records only later multi-face cache placement");
        auto outer = archive(s, m);
        outer["geometries"][0] = {
            {"status", "decoded"}, {"encoding", "csg_archive"}, {"geometry", archive(s, m)}};
        const auto nested = evaluate_csg_polyface_archive(outer, options);
        check(nested.result.status == "evaluated" && projection(nested) &&
                  nested.solid_source_paths[0].size() == 2 && nested.solid_placements.size() == 2,
              "nested source transforms and original path survive independent reconstruction");
        for (unsigned i = 0; i < 4; ++i) {
            auto limit = options;
            if (i == 0)
                limit.max_solid_snapshots = 0;
            if (i == 1)
                limit.max_source_transform_points = 0;
            if (i == 2)
                limit.max_source_transform_nodes = 0;
            if (i == 3)
                limit.max_solid_triangles = 1;
            const auto failed = evaluate_csg_polyface_archive(input, limit);
            check(failed.result.status == "work_limit" && failed.result.meshes.empty() &&
                      failed.result.updated_nodes.empty(),
                  "reconstruction aggregate budget exhaustion clears complete result");
        }
    }
    auto m = identity();
    m[0][0] = 3;
    const auto bent = source("P3DSectionLoft", true);
    const auto r = evaluate_csg_polyface_archive(archive(bent, m), options);
    check(r.result.status == "evaluated" && projection(r),
          "length-reparameterized loft CSG has valid snapshot provenance");
    const auto before = SectionLoft::from_bgfb(bent).sides()[0].surface.point_at(.5, .4);
    const auto after = SectionLoft::from_bgfb(r.solid_snapshots[0].mesh.source)
                           .sides()[0]
                           .surface.point_at(.5, .4);
    check(std::abs(after[0] - 3 * before[0]) > .05 || std::abs(after[2] - before[2]) > .05,
          "CSG reconstructs length knots rather than applying a matrix to old loft samples");
    for (const std::string kind : {"DgnExtrusion", "DgnRuledSweep"}) {
        auto collapsed_source = source(kind);
        auto replace = [](Json &section, double z) {
            section["curves"][0]["geometry"] = {{"_type", "EllipticArc"},
                                                {"arc",
                                                 {{"centerX", 1.},
                                                  {"centerY", 0.},
                                                  {"centerZ", z},
                                                  {"vector0X", 1.},
                                                  {"vector0Y", 0.},
                                                  {"vector0Z", 0.},
                                                  {"vector90X", 0.},
                                                  {"vector90Y", 1e-7},
                                                  {"vector90Z", 0.},
                                                  {"startRadians", 0.},
                                                  {"sweepRadians", 3.141592653589793}}}};
        };
        if (kind == "DgnExtrusion")
            replace(collapsed_source["baseCurve"], 0);
        else {
            replace(collapsed_source["curves"][0], 0);
            replace(collapsed_source["curves"][1], 3);
        }
        const auto replacement =
            evaluate_csg_polyface_archive(archive(collapsed_source, identity()), options);
        check(replacement.result.status == "evaluated" && projection(replacement) &&
                  std::abs(volume(replacement.result.meshes[0]) - 12) < 1e-9 &&
                  replacement.solid_placements[0].report["arc_replacements"].size() ==
                      (kind == "DgnExtrusion" ? 1 : 2),
              "CSG transforms before meshing even when original sweep curve correspondence is not "
              "meshable");
        auto repeated = archive(source(kind), identity());
        repeated["tree"]["nodes"][0]["geometry_indices"] = {0, 0};
        auto limit = options;
        limit.max_solid_snapshots = 1;
        const auto partial = evaluate_csg_polyface_archive(repeated, limit);
        check(partial.result.status == "work_limit" && partial.result.meshes.empty() &&
                  partial.solid_snapshots.size() == 1,
              "second snapshot exhaustion keeps diagnostics but no partial evaluated result");
    }
    return n;
}
