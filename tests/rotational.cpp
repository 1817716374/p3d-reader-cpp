#include <p3d/solid.hpp>
#include <p3d/csg_mesh_tree.hpp>
#include "geometry.hpp"
using namespace p3d;
namespace {
constexpr double tau = 6.283185307179586476925286766559;
Json rectangle(double inner, double outer, double height) {
    return {{"_type", "CurveVector"},
            {"type", 2},
            {"curves",
             {{{"geometry",
                {{"_type", "LineString"},
                 {"points",
                  {inner, 0., 0., outer, 0., 0., outer, 0., height, inner, 0., height, inner, 0.,
                   0.}}}}}}}};
}
Json rotation(Json section, double angle, bool capped = true) {
    return {{"_type", "DgnRotationalSweep"},
            {"baseCurve", section},
            {"axis", {{"x", 0.}, {"y", 0.}, {"z", 0.}, {"ux", 0.}, {"uy", 0.}, {"uz", 4.}}},
            {"sweepRadians", angle},
            {"capped", capped},
            {"numVRules", 17}};
}
double volume(const Geometry &g) {
    long double v = 0;
    for (auto f : g.faces) {
        auto a = g.vertices[f[0]], b = g.vertices[f[1]], c = g.vertices[f[2]];
        v += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
             a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    return double(v / 6);
}
bool closed(const Geometry &g) {
    std::map<std::array<std::uint32_t, 2>, std::pair<unsigned, int>> edges;
    for (auto f : g.faces)
        for (unsigned i = 0; i < 3; ++i) {
            auto a = f[i], b = f[(i + 1) % 3];
            auto &e = edges[{std::min(a, b), std::max(a, b)}];
            ++e.first;
            e.second += a < b ? 1 : -1;
        }
    for (auto e : edges)
        if (e.second != std::make_pair(2u, 0))
            return false;
    return !edges.empty();
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
} // namespace
unsigned rotational_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    for (double inner : {0., 1.})
        for (double angle : {tau, -tau, tau / 4, -tau / 4}) {
            auto input = rotation(rectangle(inner, 3, 2), angle);
            const auto r = mesh_bgfb_solid(input, {}, 16);
            check(r.derived.status == "meshed" && r.source == input,
                  "rotational profile meshes without changing source");
            check(closed(r.derived.geometry),
                  "rotational seam, caps and axial poles form closed index edges");
            const double bands = std::ceil(std::abs(angle) / tau * 16);
            const double expected =
                .5 * (9 - inner * inner) * 2 * bands * std::sin(std::abs(angle) / bands);
            check(std::abs(volume(r.derived.geometry) - expected) < 1e-8,
                  "rotational sector volume matches independent polygon annulus formula");
            const std::set<std::array<std::int64_t, 3>> ids(r.face_indices.begin(),
                                                            r.face_indices.end());
            check(ids.count({-1, 0, 0}) == unsigned(std::abs(angle) < tau) && ids.count({0, 0, 1}),
                  "rotational native caps and single-profile component indices");
            check(r.derived.report["rotation_seam_joined"] == (std::abs(angle) == tau),
                  "only exact revolution closes rotation seam");
        }
    auto input = rotation(rectangle(1, 3, 2), tau / 4, false);
    auto r = mesh_bgfb_solid(input, {}, 16);
    check(r.derived.status == "meshed" && !closed(r.derived.geometry),
          "uncapped partial revolution retains open ends");
    input = rotation(rectangle(1, 3, 2), std::nextafter(tau, 0.));
    r = mesh_bgfb_solid(input, {}, 16);
    check(r.derived.status == "meshed" && r.derived.report["native_full_circle"] == true &&
              r.derived.report["native_has_caps"] == false &&
              r.derived.report["rotation_seam_joined"] == false && !closed(r.derived.geometry),
          "native full-circle cap tolerance does not invent exact closure");
    Json section = {{"_type", "CurveVector"},
                    {"type", 1},
                    {"curves",
                     {{{"geometry",
                        {{"_type", "LineSegment"},
                         {"segment",
                          {{"point0X", 2.},
                           {"point0Y", 0.},
                           {"point0Z", 0.},
                           {"point1X", 2.},
                           {"point1Y", 0.},
                           {"point1Z", 3.}}}}}}}}};
    r = mesh_bgfb_solid(rotation(section, tau, false), {}, 12);
    check(r.derived.status == "meshed" && r.derived.geometry.faces.size() == 24 &&
              !closed(r.derived.geometry),
          "open line profile forms an uncapped cylinder wall");
    auto spline_section = rectangle(0, 3, 2);
    auto &curve = spline_section["curves"][0]["geometry"];
    curve = {{"_type", "BsplineCurve"},  {"order", 2},       {"closed", false},
             {"poles", curve["points"]}, {"knots", nullptr}, {"weights", nullptr}};
    r = mesh_bgfb_solid(rotation(spline_section, tau), {}, 8);
    check(r.derived.status == "meshed" && closed(r.derived.geometry),
          "B-spline knot breaks and axial points survive rotational reconstruction");
    check(std::abs(volume(r.derived.geometry) - 9 * 8 * std::sin(tau / 8)) < 1e-8,
          "B-spline cylinder has analytic sampled volume");
    Json ellipse = {{"_type", "EllipticArc"},
                    {"arc",
                     {{"centerX", 3.},
                      {"centerY", 0.},
                      {"centerZ", 0.},
                      {"vector0X", 1.},
                      {"vector0Y", 0.},
                      {"vector0Z", 0.},
                      {"vector90X", 0.},
                      {"vector90Y", 0.},
                      {"vector90Z", 1.},
                      {"startRadians", 0.},
                      {"sweepRadians", tau}}}};
    auto torus_section = rectangle(1, 2, 1);
    torus_section["curves"][0]["geometry"] = ellipse;
    r = mesh_bgfb_solid(rotation(torus_section, tau), {}, 12);
    check(r.derived.status == "meshed" && closed(r.derived.geometry),
          "ellipse profile produces closed rotational torus");
    check(std::abs(volume(r.derived.geometry) -
                   .5 * 12 * std::sin(tau / 12) * 3 * 12 * std::sin(tau / 12)) < 1e-8,
          "polygon torus volume agrees with section area and polygon centroid path");
    auto m = identity();
    m[0][0] = -2;
    m[1][1] = 1;
    m[2][2] = 3;
    m[0][3] = 7;
    input = rotation(rectangle(1, 3, 2), tau / 4);
    const auto transformed = transform_bgfb_curve_solid(input, m);
    check(transformed.status == "transformed" &&
              transformed.transformed["sweepRadians"] == -tau / 4 &&
              transformed.report["rotational_sweep_reversed"] == true &&
              transformed.transformed["axis"]["x"] == 7 &&
              transformed.transformed["axis"]["uz"] == 12,
          "native mirrored source placement reverses angle and transforms axis ray");
    CsgMeshTreeOptions options;
    options.solid_circle_segments = 16;
    const auto csg = evaluate_csg_polyface_archive(archive(input, m), options);
    check(csg.result.status == "evaluated" && csg.solid_snapshots.size() == 1,
          "rotational solid participates in source CSG reconstruction");
    // X-radius doubled, axial height tripled: actual reconstructed volume scales
    // by 12, unlike the input matrix determinant magnitude of 6.
    const double expected = 12 * .5 * (9 - 1) * 2 * 4 * std::sin(tau / 16);
    check(std::abs(volume(csg.solid_snapshots[0].mesh.derived.geometry) - expected) < 1e-7,
          "anisotropic rotational placement rebuilds source instead of affine old triangles");
    bool provenance = true;
    for (const auto &mesh : csg.result.meshes)
        for (std::size_t f = 0; f < mesh.faces.size(); ++f) {
            const auto &s = mesh.face_sources[f];
            provenance &= s.solid_snapshot == 0 && s.geometry_index == 0;
            for (double d : s.corner_projection_distance)
                provenance &= d < 1e-8;
        }
    check(provenance, "CSG result points trace to rotational snapshot");
    for (unsigned mode = 0; mode < 4; ++mode) {
        auto bad = input;
        PolyfaceMeshOptions budget;
        if (mode == 0)
            bad["axis"]["uz"] = 0.;
        if (mode == 1)
            bad["sweepRadians"] = 0.;
        if (mode == 2)
            bad["sweepRadians"] = 2 * tau;
        if (mode == 3)
            budget.max_points = 10;
        auto failed = mesh_bgfb_solid(bad, budget, 16);
        check(failed.derived.status != "meshed" && failed.derived.geometry.faces.empty(),
              "invalid rotational source or exhausted budget never returns partial mesh");
    }
    Json parity = {{"_type", "CurveVector"}, {"type", 4}, {"curves", Json::array()}};
    auto outer = rectangle(1, 3, 2), hole = rectangle(1.5, 2.5, 1);
    auto &hole_points = hole["curves"][0]["geometry"]["points"];
    for (std::size_t i = 2; i < hole_points.size(); i += 3)
        hole_points[i] = hole_points[i].get<double>() + .5;
    hole["type"] = 3;
    parity["curves"] = {{{"geometry", outer}}, {{"geometry", hole}}};
    r = mesh_bgfb_solid(rotation(parity, tau), {}, 12);
    check(r.derived.status == "meshed" && closed(r.derived.geometry) &&
              std::abs(volume(r.derived.geometry) - 6 * 12 * std::sin(tau / 12)) < 1e-8,
          "rotated parity hole preserves shell orientation and independent volume difference");
    const auto original = rotation(rectangle(1, 3, 2), tau / 4);
    const auto original_mesh = mesh_bgfb_solid(original, {}, 16);
    auto rigid = identity();
    rigid[0] = {0, 0, 1, 13};
    rigid[1] = {1, 0, 0, -5};
    rigid[2] = {0, 1, 0, 7};
    const auto rotated_source = transform_bgfb_curve_solid(original, rigid);
    r = mesh_bgfb_solid(rotated_source.transformed, {}, 16);
    check(r.derived.status == "meshed" && closed(r.derived.geometry),
          "translated non-Z rotation axis uses the transformed ray origin and direction");
    bool rigid_points =
        r.derived.geometry.vertices.size() == original_mesh.derived.geometry.vertices.size();
    if (rigid_points)
        for (std::size_t i = 0; i < r.derived.geometry.vertices.size(); ++i) {
            const auto expected_point =
                transform(rigid, original_mesh.derived.geometry.vertices[i]);
            const auto actual = r.derived.geometry.vertices[i];
            rigid_points &= std::hypot(actual[0] - expected_point[0], actual[1] - expected_point[1],
                                       actual[2] - expected_point[2]) < 1e-12;
        }
    check(rigid_points, "arbitrary-axis reconstruction commutes with independent rigid placement");
    auto stationary = section;
    stationary["curves"][0]["geometry"]["segment"]["point0X"] = 0.;
    stationary["curves"][0]["geometry"]["segment"]["point1X"] = 0.;
    PolyfaceMeshOptions index_budget;
    index_budget.max_corners = 20;
    const auto excessive_rows = mesh_bgfb_solid(rotation(stationary, tau, false), index_budget, 16);
    check(excessive_rows.derived.status != "meshed" &&
              excessive_rows.derived.geometry.faces.empty() &&
              excessive_rows.derived.report["reason"] == "rotational sweep row index budget",
          "stationary axis samples cannot bypass the cumulative row allocation budget");
    return n;
}
