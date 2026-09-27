#include <p3d/swept_body.hpp>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>
using namespace p3d;
namespace {
SweptBodySurface plane() {
    SweptBodySurface s;
    s.geometry = {{"_type", "BsplineSurface"},
                  {"orderU", 2},
                  {"orderV", 2},
                  {"numPolesU", 2},
                  {"numPolesV", 2},
                  {"closedU", false},
                  {"closedV", false},
                  {"poles", {0, 0, 0, 2, 0, 0, 0, 3, 0, 2, 3, 0}},
                  {"weights", nullptr},
                  {"knotsU", nullptr},
                  {"knotsV", nullptr},
                  {"boundaries", nullptr},
                  {"holeOrigin", 0},
                  {"numRulesU", 2},
                  {"numRulesV", 2}};
    return s;
}
Json first_geometry(const Json &root, std::size_t child = 0) {
    return root.at("curves").at(child).at("geometry").at("curves").at(0).at("geometry");
}
double area(const BsplineSurfaceMesh &mesh, bool uv = false) {
    double result = 0;
    for (const auto &f : mesh.faces) {
        Point2 p[3];
        for (unsigned j = 0; j < 3; ++j)
            p[j] =
                uv ? mesh.parameters[f[j]] : Point2{mesh.vertices[f[j]][0], mesh.vertices[f[j]][1]};
        result += ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) -
                   (p[2][0] - p[0][0]) * (p[1][1] - p[0][1])) /
                  2;
    }
    return result;
}
bool near(double a, double b) {
    return std::abs(a - b) < 1e-10;
}
} // namespace
unsigned swept_body_uv_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        if (!ok)
            throw std::runtime_error(why);
    };
    auto s = plane();
    const auto original = s.geometry;
    auto full = extract_swept_body_uv_boundaries(s);
    check(full.status == "extracted" && full.region["type"] == 4 &&
              full.region["curves"].size() == 1,
          "UV query keeps parity root and native unit rectangle child");
    check(first_geometry(full.region)["points"] ==
              Json({0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0}),
          "native rectangle remains one directed closed line string");
    s.geometry["holeOrigin"] = 1;
    auto empty = extract_swept_body_uv_boundaries(s);
    check(empty.status == "extracted" && empty.region["curves"].empty(),
          "no outer and no active records still creates an empty parity array");
    s.boundary_points = {{{.2, .2}, {.8, .2}, {.8, .8}, {.2, .8}}};
    const auto points = s.boundary_points;
    auto loop = extract_swept_body_uv_boundaries(s);
    check(first_geometry(loop.region)["points"].size() == 15 &&
              loop.report["records"][0]["closure"] == "first_appended",
          "UV polygon query closes an open saved point list");
    s.boundary_points[0].push_back({.2 + 1e-12, .2});
    auto snapped = extract_swept_body_uv_boundaries(s);
    check(first_geometry(snapped.region) == first_geometry(loop.region) &&
              snapped.report["records"][0]["closure"] == "last_replaced_by_first",
          "native near-equal endpoint is replaced without an extra vertex");
    s.boundary_points = points;
    std::reverse(s.boundary_points[0].begin(), s.boundary_points[0].end());
    auto clockwise = extract_swept_body_uv_boundaries(s);
    check(first_geometry(clockwise.region)["points"][1] == .8 &&
              clockwise.report["orientation_adjusted"] == false,
          "createLinestringArray false flag does not reverse clockwise loops");
    s.boundary_points = {{}, {{.4, .5}}};
    auto short_records = extract_swept_body_uv_boundaries(s);
    check(short_records.region["curves"].size() == 2 &&
              first_geometry(short_records.region)["points"].empty() &&
              first_geometry(short_records.region, 1)["points"] == Json({.4, .5, 0}),
          "empty and singleton records retain separate children and primitives");
    const auto pcurve = BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                                 {"order", 3},
                                                 {"closed", false},
                                                 {"poles", {.1, .1, 0, .6, .9, 0, .9, .1, 0}},
                                                 {"weights", {1., 2., 1.}},
                                                 {"knots", {2., 2., 2., 7., 7., 7.}}});
    s.boundary_points = points;
    s.boundary_curves = {{pcurve, pcurve}};
    auto exact = extract_swept_body_uv_boundaries(s);
    const auto &members = exact.region["curves"][0]["geometry"]["curves"];
    check(members.size() == 2 && members[0] == members[1],
          "equal independent parameter curves stay separate");
    const auto copied = BsplineCurve::from_bgfb(members[0]["geometry"]);
    check(copied.poles() == pcurve.poles() && copied.weights() == pcurve.weights() &&
              copied.knots() == pcurve.knots(),
          "parameter curve poles weights and raw knots are copied without conversion");
    check(exact.report["records"][0]["closure"] == "unchanged",
          "pcurves are not force-closed or fitted");
    SweptBodyUVBoundaryOptions polygon;
    polygon.prefer_parameter_curves = false;
    check(extract_swept_body_uv_boundaries(s, polygon).region == loop.region,
          "explicit point-cache branch ignores different parameter curves");
    s.geometry["holeOrigin"] = 0;
    auto outer = extract_swept_body_uv_boundaries(s);
    check(outer.region["curves"].size() == 2 &&
              outer.region["curves"][0] == full.region["curves"][0],
          "outer rectangle precedes stored loop records");
    polygon.include_outer = false;
    check(extract_swept_body_uv_boundaries(s, polygon).region == loop.region,
          "outer suppression is independent of source choice");
    for (auto limit : {SweptBodyUVBoundaryOptions{4, 10000, 100, true, true},
                       SweptBodyUVBoundaryOptions{100, 1, 100, true, true},
                       SweptBodyUVBoundaryOptions{100, 10000, 1, true, true}}) {
        auto failed = extract_swept_body_uv_boundaries(s, limit);
        check(failed.status == "not_extracted" && failed.region.is_null(),
              "UV extraction budgets clear partial tree");
    }
    auto mismatch = s;
    mismatch.boundary_curves.push_back({});
    check(extract_swept_body_uv_boundaries(mismatch).status == "not_extracted",
          "inconsistent active pcurve records rejected");
    check(extract_swept_body_uv_boundaries(mismatch, polygon).status == "extracted",
          "unused pcurve storage does not affect point branch");
    BsplineMeshOptions mo;
    mo.max_uv_edge = .4;
    auto hole = mesh_swept_body_surface(s, mo);
    check(hole.report["status"] == "complete" && near(area(hole), 6 * (1 - .36)),
          "runtime UV hole survives despite null BGFB tree and unrelated pcurves");
    check(hole.report["trim"]["representation"] == "runtime_uv_polygons" &&
              hole.report["trim"]["parameter_curve_error_bound"].is_null(),
          "cached polygons are not advertised as certified pcurve strokes");
    auto untrimmed = BsplineSurface::from_bgfb(s.geometry).mesh(mo);
    check(near(area(untrimmed), 6) && hole.faces.size() != untrimmed.faces.size(),
          "runtime mesh differs from accidentally untrimmed table mesh");
    s.geometry["holeOrigin"] = 1;
    auto island = mesh_swept_body_surface(s, mo);
    check(island.report["status"] == "complete" && near(area(island), 6 * .36),
          "inactive outer yields saved polygon island");
    s.geometry["knotsU"] = {2, 2, 5, 5};
    s.geometry["knotsV"] = {-4, -4, 8, 8};
    auto nonunit = mesh_swept_body_surface(s, mo);
    check(nonunit.report["status"] == "complete" && near(area(nonunit), area(island)),
          "runtime fractions are not normalized twice on non-unit surface knots");
    check(nonunit.parameters == island.parameters,
          "source knot range does not move fraction-space contours");
    const auto evaluator = BsplineSurface::from_bgfb(s.geometry);
    bool evaluated = true;
    for (std::size_t i = 0; i < nonunit.vertices.size(); ++i) {
        const auto p = evaluator.point_at(nonunit.parameters[i][0], nonunit.parameters[i][1]);
        for (unsigned j = 0; j < 3; ++j)
            evaluated &= near(p[j], nonunit.vertices[i][j]);
    }
    check(evaluated, "derived vertices evaluate the retained source surface");
    s.boundary_points.push_back(s.boundary_points.front());
    auto duplicate = mesh_swept_body_surface(s, mo);
    check(duplicate.report["status"] == "complete" && duplicate.faces.empty(),
          "equal independent runtime loops preserve parity cancellation");
    s.boundary_points = {{{-.5, 0}, {.5, 0}, {.5, 1}, {-.5, 1}}};
    auto clipped = mesh_swept_body_surface(s, mo);
    check(clipped.report["status"] == "complete" && near(area(clipped, true), .5),
          "runtime contour is clipped to unit domain without wrapping");
    s.boundary_points = {};
    check(near(area(mesh_swept_body_surface(s, mo)), 6),
          "absent effective boundaries leave full parameter domain");
    s.boundary_points = {{{.5, .5}}};
    check(mesh_swept_body_surface(s, mo).report["status"] == "incomplete",
          "singleton polygon cannot silently become an untrimmed mesh");
    s.boundary_points = points;
    auto low = mo;
    low.max_trim_segments = 3;
    auto failed = mesh_swept_body_surface(s, low);
    check(failed.report["status"] == "incomplete" && failed.vertices.empty() &&
              failed.faces.empty(),
          "runtime closure participates in segment budget");
    low = mo;
    low.max_vertices = 1;
    failed = mesh_swept_body_surface(s, low);
    check(failed.report["status"] == "incomplete" && failed.vertices.empty() &&
              failed.parameters.empty() && failed.faces.empty(),
          "failed tessellation does not publish partial output");
    auto mixed = s;
    mixed.geometry["boundaries"] = full.region;
    check(mesh_swept_body_surface(mixed, mo).report["status"] == "incomplete",
          "no implicit merge of runtime and serialized boundaries");
    auto bad = s;
    bad.boundary_points[0][0][0] = std::numeric_limits<double>::infinity();
    check(mesh_swept_body_surface(bad, mo).report["status"] == "incomplete",
          "nonfinite cached parameters rejected");
    auto job = std::async(std::launch::async, [s, mo] { return mesh_swept_body_surface(s, mo); });
    auto local = mesh_swept_body_surface(s, mo), parallel = job.get();
    check(local.vertices == parallel.vertices && local.faces == parallel.faces &&
              local.report == parallel.report,
          "concurrent runtime meshing has independent state");
    check(s.boundary_points == points, "queries and mesh keep source runtime points unchanged");
    check(plane().geometry == original, "source fixture remains independent");
    return n;
}
