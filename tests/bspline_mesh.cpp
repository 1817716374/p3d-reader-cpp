#include "internal.hpp"
using namespace p3d;
namespace {
Json loop(const std::vector<Point2> &points) {
    Json coordinates = Json::array();
    for (const auto &p : points) {
        coordinates.push_back(p[0]);
        coordinates.push_back(p[1]);
        coordinates.push_back(0);
    }
    return {{"_type", "CurveVector"},
            {"type", 2},
            {"curves",
             Json::array({{{"_type", "VariantGeometry"},
                           {"geometry", {{"_type", "LineString"}, {"points", coordinates}}}}})}};
}
Json rectangle(double a, double b, double c, double d) {
    return loop({{a, b}, {c, b}, {c, d}, {a, d}, {a, b}});
}
Json group(const std::vector<Json> &loops) {
    Json children = Json::array();
    for (const auto &item : loops)
        children.push_back({{"_type", "VariantGeometry"}, {"geometry", item}});
    return {{"_type", "CurveVector"}, {"type", 4}, {"curves", children}};
}
Json plane(Json boundary = nullptr, int hole = 0) {
    return {{"_type", "BsplineSurface"},
            {"numPolesU", 2},
            {"numPolesV", 2},
            {"orderU", 2},
            {"orderV", 2},
            {"closedU", false},
            {"closedV", false},
            {"poles", {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0}},
            {"numRulesU", 0},
            {"numRulesV", 0},
            {"weights", nullptr},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"boundaries", boundary},
            {"holeOrigin", hole}};
}
double area(Point2 a, Point2 b, Point2 c) {
    return ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2;
}
} // namespace
unsigned bspline_mesh_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const std::string &message) {
        ++checks;
        require(value, message);
    };
    BsplineMeshOptions options;
    options.max_uv_edge = .18;
    auto verify = [&](const BsplineSurface &surface, double expected_area,
                      double area_tolerance = 2e-12) {
        const auto mesh = surface.mesh(options);
        check(mesh.report.at("status") == "complete",
              "surface mesh complete: " + mesh.report.dump());
        check(mesh.vertices.size() == mesh.parameters.size(),
              "surface mesh paired position/UV arrays");
        double total_area = 0;
        std::map<std::array<unsigned, 2>, std::pair<unsigned, int>> edges;
        auto trim = surface.trim(options.uv_tolerance);
        for (const auto &face : mesh.faces) {
            for (auto index : face)
                require(index < mesh.parameters.size(), "mesh index range");
            const auto a = mesh.parameters[face[0]], b = mesh.parameters[face[1]],
                       c = mesh.parameters[face[2]];
            const auto triangle_area = area(a, b, c);
            require(triangle_area > 0, "mesh positive UV winding");
            total_area += triangle_area;
            const Point2 center{(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3};
            require(trim.classify(center) != TrimLocation::Outside,
                    "triangle centroid outside source region");
            for (unsigned i = 0; i < 3; ++i) {
                const auto x = face[i], y = face[(i + 1) % 3];
                const auto p = mesh.parameters[x], q = mesh.parameters[y];
                require(std::hypot(p[0] - q[0], p[1] - q[1]) <= options.max_uv_edge * (1 + 1e-14),
                        "mesh UV edge limit");
                auto &edge = edges[{std::min(x, y), std::max(x, y)}];
                ++edge.first;
                edge.second += x < y ? 1 : -1;
            }
        }
        for (const auto &entry : edges) {
            require(entry.second.first <= 2, "nonmanifold derived edge");
            if (entry.second.first == 2) {
                require(entry.second.second == 0, "shared edge orientation");
                continue;
            }
            auto a = mesh.parameters[entry.first[0]], b = mesh.parameters[entry.first[1]];
            const Point2 middle{(a[0] + b[0]) / 2, (a[1] + b[1]) / 2};
            const bool square_edge =
                middle[0] == 0 || middle[0] == 1 || middle[1] == 0 || middle[1] == 1;
            const auto location = trim.classify(middle);
            require(square_edge || location == TrimLocation::BoundaryBand ||
                        location == TrimLocation::Indeterminate,
                    "unexpected interior boundary or T junction");
        }
        check(std::abs(total_area - expected_area) < area_tolerance,
              "analytical region area conserved: expected=" + Json(expected_area).dump() +
                  " actual=" + Json(total_area).dump() + " source=" + surface.boundaries().dump());
        return mesh;
    };
    verify(BsplineSurface::from_bgfb(plane()), 1);
    verify(BsplineSurface::from_bgfb(plane(nullptr, 1)), 1);
    auto scaled_domain = plane();
    scaled_domain["knotsU"] = {2, 2, 5, 5};
    scaled_domain["knotsV"] = {-3, -3, 7, 7};
    verify(BsplineSurface::from_bgfb(scaled_domain), 1);
    scaled_domain["boundaries"] = rectangle(.2, .2, .8, .8);
    check(BsplineSurface::from_bgfb(scaled_domain).mesh(options).report["status"] == "incomplete",
          "non-normalized source trim UV is not silently treated as surface fractions");
    verify(BsplineSurface::from_bgfb(plane(rectangle(.2, .2, .8, .8), 1)), .36);
    verify(BsplineSurface::from_bgfb(plane(rectangle(.2, .2, .8, .8), 0)), .64);
    verify(
        BsplineSurface::from_bgfb(plane(group({rectangle(.1, .1, .9, .9), rectangle(.3, .3, .7, .7),
                                               rectangle(.4, .4, .6, .6)}),
                                        1)),
        .52);
    verify(BsplineSurface::from_bgfb(
               plane(group({rectangle(.1, .1, .6, .6), rectangle(.4, .4, .9, .9)}), 1)),
           .42);
    const auto duplicates = group({rectangle(.2, .2, .8, .8), rectangle(.2, .2, .8, .8)});
    verify(BsplineSurface::from_bgfb(plane(duplicates, 1)), 0);
    verify(BsplineSurface::from_bgfb(plane(duplicates, 0)), 1);
    verify(BsplineSurface::from_bgfb(plane(rectangle(-.2, .2, .4, .8), 1)), .24);
    verify(BsplineSurface::from_bgfb(plane(rectangle(-2, -2, -1, -1), 1)), 0);
    verify(BsplineSurface::from_bgfb(plane(rectangle(-2, -2, -1, -1), 0)), 1);
    verify(BsplineSurface::from_bgfb(
               plane(loop({{.1, .1}, {.9, .9}, {.1, .9}, {.9, .1}, {.1, .1}}), 1)),
           .32);
    verify(BsplineSurface::from_bgfb(plane(rectangle(0, .2, .5, .8), 0)), .7);
    Json ellipse = {{"_type", "EllipticArc"},
                    {"arc",
                     {{"centerX", .5},
                      {"centerY", .5},
                      {"centerZ", 0},
                      {"vector0X", .3},
                      {"vector0Y", 0},
                      {"vector0Z", 0},
                      {"vector90X", 0},
                      {"vector90Y", .2},
                      {"vector90Z", 0},
                      {"startRadians", 0},
                      {"sweepRadians", 6.2831853071795864769}}}};
    for (const auto hole : {0, 1}) {
        Json boundary = {
            {"_type", "CurveVector"},
            {"type", 2},
            {"curves", Json::array({{{"_type", "VariantGeometry"}, {"geometry", ellipse}}})}};
        verify(BsplineSurface::from_bgfb(plane(boundary, hole)),
               hole ? .06 * 3.141592653589793 : 1 - .06 * 3.141592653589793, 2e-6);
        ellipse["arc"]["sweepRadians"] = -6.2831853071795864769;
    }

    auto curved = plane();
    curved["numPolesU"] = curved["numPolesV"] = 3;
    curved["orderU"] = curved["orderV"] = 3;
    curved["poles"] = Json::array();
    for (unsigned j = 0; j < 3; ++j)
        for (unsigned i = 0; i < 3; ++i) {
            curved["poles"].push_back(i / 2.0);
            curved["poles"].push_back(j / 2.0);
            curved["poles"].push_back(i == 1 && j == 1 ? 1 : 0);
        }
    const auto curved_mesh = verify(BsplineSurface::from_bgfb(curved), 1);
    bool elevated = false;
    for (std::size_t i = 0; i < curved_mesh.vertices.size(); ++i) {
        const auto uv = curved_mesh.parameters[i];
        const auto p = curved_mesh.vertices[i];
        require(std::abs(p[0] - uv[0]) < 1e-14 && std::abs(p[1] - uv[1]) < 1e-14 &&
                    std::abs(p[2] - 4 * uv[0] * (1 - uv[0]) * uv[1] * (1 - uv[1])) < 1e-14,
                "quadratic surface analytic reconstruction");
        elevated |= p[2] > .2;
    }
    check(elevated && curved_mesh.report["world_space_error_bound"].is_null(),
          "interior surface sampling without invented world-space error bound");
    auto rational = plane();
    rational["poles"] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 2, 2, 0};
    rational["weights"] = {1, 1, 1, 2};
    for (unsigned negative = 0; negative < 2; ++negative) {
        const auto mesh = verify(BsplineSurface::from_bgfb(rational), 1);
        for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
            const auto uv = mesh.parameters[i],
                       p = Point2{mesh.vertices[i][0], mesh.vertices[i][1]};
            const double uv_product = uv[0] * uv[1], denominator = 1 + uv_product;
            require(std::abs(p[0] - (uv[0] + uv_product) / denominator) < 1e-14 &&
                        std::abs(p[1] - (uv[1] + uv_product) / denominator) < 1e-14,
                    "rational surface analytic reconstruction");
        }
        for (auto &p : rational["poles"])
            p = -p.get<double>();
        for (auto &w : rational["weights"])
            w = -w.get<double>();
    }
    auto periodic = plane();
    periodic["numPolesU"] = 4;
    periodic["closedU"] = true;
    periodic["poles"] = {1, 0, 0, 0, 1, 0, -1, 0, 0, 0, -1, 0,
                         1, 0, 1, 0, 1, 1, -1, 0, 1, 0, -1, 1};
    verify(BsplineSurface::from_bgfb(periodic), 1);
    periodic["boundaries"] = rectangle(.8, .2, 1.2, .8);
    check(BsplineSurface::from_bgfb(periodic).mesh(options).report["status"] == "incomplete",
          "periodic trimmed loops are not silently clipped without seam rules");
    rational["weights"] = {1, -1, 1, 2};
    check(BsplineSurface::from_bgfb(rational).mesh(options).faces.empty(),
          "unverified denominator cannot yield a mesh through an undiscovered singularity");
    auto discontinuous = plane();
    discontinuous["numPolesU"] = 4;
    discontinuous["knotsU"] = {0, 0, .5, .5, 1, 1};
    discontinuous["poles"] = {0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0,
                              0, 1, 0, 1, 1, 0, 2, 1, 0, 3, 1, 0};
    check(BsplineSurface::from_bgfb(discontinuous).mesh(options).report["status"] == "incomplete",
          "discontinuous patches cannot be bridged by shared knot-side evaluation");
    const auto simple = BsplineSurface::from_bgfb(plane());
    auto limited = options;
    limited.max_triangles = 1;
    auto failed = simple.mesh(limited);
    check(failed.report["status"] == "incomplete" && failed.vertices.empty() &&
              failed.faces.empty() && failed.parameters.empty(),
          "triangle budget failure does not expose partial geometry as a complete mesh");
    limited = options;
    limited.max_vertices = 3;
    check(simple.mesh(limited).report["status"] == "incomplete", "input vertex budget enforced");
    limited = options;
    limited.max_uv_edge = 0;
    bool rejected = false;
    try {
        (void)simple.mesh(limited);
    } catch (const std::exception &) {
        rejected = true;
    }
    check(rejected, "invalid surface mesh tolerance rejected");
    return checks;
}
