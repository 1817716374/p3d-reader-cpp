#include "internal.hpp"
#include <functional>

namespace {
using namespace p3d;
Json variant(Json g) {
    return {{"_type", "VariantGeometry"}, {"geometry", std::move(g)}};
}
Json rectangle(double a, double b, double c, double d, bool reverse = false) {
    Json p = reverse ? Json{a, b, 0, a, d, 0, c, d, 0, c, b, 0, a, b, 0}
                     : Json{a, b, 0, c, b, 0, c, d, 0, a, d, 0, a, b, 0};
    return {{"_type", "CurveVector"},
            {"type", 2},
            {"curves", Json::array({variant({{"_type", "LineString"}, {"points", p}})})}};
}
Json group(const std::vector<Json> &loops) {
    Json children = Json::array();
    for (const auto &loop : loops)
        children.push_back(variant(loop));
    return {{"_type", "CurveVector"}, {"type", 4}, {"curves", children}};
}
Point2 diamond(double f) {
    const std::array<Point2, 5> p{{{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 0}}};
    const unsigned i = std::min(3u, unsigned(4 * f));
    const double t = 4 * f - i;
    return {(1 - t) * p[i][0] + t * p[i + 1][0], (1 - t) * p[i][1] + t * p[i + 1][1]};
}
Point2 circle(double f) {
    const unsigned i = std::min(2u, unsigned(3 * f));
    const double t = 3 * f - i, a = (1 - t) * (1 - t), b = 2 * t * (1 - t), c = t * t;
    const double w = a + .5 * b + c, x = (a + .5 * b - .5 * c) / w,
                 y = (b + c) * std::sqrt(3.) / 2 / w;
    const double angle = i * 2 * std::acos(-1.) / 3;
    return {std::cos(angle) * x - std::sin(angle) * y, std::sin(angle) * x + std::cos(angle) * y};
}
Point3 expected(Point2 uv, bool closed_u, bool closed_v, bool rational) {
    const auto a = rational ? circle(uv[0]) : diamond(uv[0]);
    const auto b = rational ? circle(uv[1]) : diamond(uv[1]);
    if (!closed_v)
        return {a[0], a[1], uv[1]};
    if (!closed_u)
        return {b[0], b[1], uv[0]};
    return {(3 + b[0]) * a[0], (3 + b[0]) * a[1], b[1]};
}
Json periodic(bool closed_u, bool closed_v, bool rational = false) {
    const double s = std::sqrt(3.) / 2;
    // Homogeneous circle controls for three 120-degree rational quadratic arcs.
    const std::vector<Point3> ring =
        rational ? std::vector<Point3>{{1, 0, 1},    {.5, s, .5},  {-.5, s, 1}, {-1, 0, .5},
                                       {-.5, -s, 1}, {.5, -s, .5}, {1, 0, 1}}
                 : std::vector<Point3>{{1, 0, 1}, {0, 1, 1}, {-1, 0, 1}, {0, -1, 1}};
    const auto nu = closed_u ? ring.size() : 2, nv = closed_v ? ring.size() : 2;
    Json p = Json::array(), w = Json::array();
    for (std::size_t j = 0; j < nv; ++j)
        for (std::size_t i = 0; i < nu; ++i) {
            Point3 h{};
            double weight = 1;
            if (!closed_v) {
                h = {ring[i][0], ring[i][1], double(j) * ring[i][2]};
                weight = ring[i][2];
            } else if (!closed_u) {
                h = {ring[j][0], ring[j][1], double(i) * ring[j][2]};
                weight = ring[j][2];
            } else {
                h = {(3 * ring[j][2] + ring[j][0]) * ring[i][0],
                     (3 * ring[j][2] + ring[j][0]) * ring[i][1], ring[j][1] * ring[i][2]};
                weight = ring[j][2] * ring[i][2];
            }
            for (double v : h)
                p.push_back(v);
            w.push_back(weight);
        }
    const Json knots = {-1. / 3, 0, 0, 0, 1. / 3, 1. / 3, 2. / 3, 2. / 3, 1, 1, 1, 4. / 3};
    return {{"_type", "BsplineSurface"},
            {"numPolesU", nu},
            {"numPolesV", nv},
            {"orderU", closed_u && rational ? 3 : 2},
            {"orderV", closed_v && rational ? 3 : 2},
            {"closedU", closed_u},
            {"closedV", closed_v},
            {"poles", p},
            {"weights", rational ? w : Json(nullptr)},
            {"knotsU", closed_u && rational ? knots : Json(nullptr)},
            {"knotsV", closed_v && rational ? knots : Json(nullptr)},
            {"numRulesU", 0},
            {"numRulesV", 0},
            {"holeOrigin", 1},
            {"boundaries", nullptr}};
}
} // namespace

unsigned bspline_periodic_trim_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const std::string &message) {
        ++checks;
        require(ok, message);
    };
    BsplineMeshOptions options;
    options.max_uv_edge = 1. / 12;
    auto verify = [&](const Json &table, double area, bool cu, bool cv, bool rational) {
        const auto surface = BsplineSurface::from_bgfb(table);
        const auto mesh = surface.mesh(options);
        check(mesh.report.at("status") == "complete", "periodic trim mesh: " + mesh.report.dump());
        check(mesh.report.at("trim_domain_policy") == "source_uv_clipped_to_active_domain",
              "mesh identifies saved-UV clipping without period translation");
        const auto region = surface.trim_normalized(options.uv_tolerance);
        double actual = 0;
        std::map<std::array<unsigned, 2>, std::pair<unsigned, int>> edges;
        for (const auto &f : mesh.faces) {
            const auto a = mesh.parameters[f[0]], b = mesh.parameters[f[1]],
                       c = mesh.parameters[f[2]];
            const double signed_area =
                ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2;
            require(signed_area > 0, "periodic trimmed triangle UV orientation");
            actual += signed_area;
            const Point2 center{(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3};
            require(region.classify(center) != TrimLocation::Outside,
                    "periodic triangle outside saved trim region");
            for (unsigned j = 0; j < 3; ++j) {
                auto &edge =
                    edges[{std::min(f[j], f[(j + 1) % 3]), std::max(f[j], f[(j + 1) % 3])}];
                ++edge.first;
                edge.second += f[j] < f[(j + 1) % 3] ? 1 : -1;
            }
        }
        check(std::abs(actual - area) < 2e-12, "periodic trim conserves analytical parameter area");
        for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
            const auto uv = mesh.parameters[i];
            require(uv[0] >= 0 && uv[0] <= 1 && uv[1] >= 0 && uv[1] <= 1,
                    "periodic output stays in its active rectangle");
            const auto p = expected(uv, cu, cv, rational);
            for (unsigned a = 0; a < 3; ++a)
                require(std::abs(p[a] - mesh.vertices[i][a]) < 2e-12,
                        "independent periodic surface evaluation");
        }
        for (const auto &entry : edges) {
            const auto a = mesh.parameters[entry.first[0]], b = mesh.parameters[entry.first[1]];
            require(std::hypot(a[0] - b[0], a[1] - b[1]) <= options.max_uv_edge * (1 + 1e-14),
                    "periodic mesh edge refinement");
            require(entry.second.first <= 2, "periodic parameter mesh edge manifold");
            if (entry.second.first == 2)
                require(entry.second.second == 0, "periodic shared-edge orientation");
            else if (!((a[0] == 0 && b[0] == 0) || (a[0] == 1 && b[0] == 1) ||
                       (a[1] == 0 && b[1] == 0) || (a[1] == 1 && b[1] == 1))) {
                const auto loc = region.classify({(a[0] + b[0]) / 2, (a[1] + b[1]) / 2});
                require(loc == TrimLocation::BoundaryBand || loc == TrimLocation::Indeterminate,
                        "periodic mesh has no unexplained interior crack");
            }
        }
        check(surface.boundaries() == table.at("boundaries"),
              "periodic mesh preserves source trim tree");
        return mesh;
    };
    for (const auto flags : {std::array<bool, 2>{true, false}, {false, true}, {true, true}})
        for (bool rational : {false, true}) {
            const bool cu = flags[0], cv = flags[1];
            auto table = periodic(cu, cv, rational);
            table["boundaries"] = rectangle(.1, .2, .9, .8);
            verify(table, .48, cu, cv,
                   rational); // An edge wider than half a period is not shortened.
            table["boundaries"] = rectangle(.1, .2, .9, .8, true);
            verify(table, .48, cu, cv, rational);
            table["holeOrigin"] = 0;
            verify(table, .52, cu, cv, rational);
            table["holeOrigin"] = 1;
            table["boundaries"] = rectangle(.8, .2, 1.2, .8);
            verify(table, .12, cu, cv, rational); // No duplicate at U=[0,.2].
            table["holeOrigin"] = 0;
            verify(table, .88, cu, cv, rational);
            table["boundaries"] = rectangle(.2, .8, .8, 1.2);
            verify(table, .88, cu, cv, rational);
            table["holeOrigin"] = 1;
            table["boundaries"] = rectangle(-.1, -.1, .2, .2);
            verify(table, .04, cu, cv, rational); // Neither direction is silently reduced modulo 1.
            table["boundaries"] = rectangle(2, .2, 3, .8);
            verify(table, 0, cu, cv, rational);
            table["boundaries"] = group({rectangle(0, .2, .15, .8), rectangle(.85, .2, 1, .8)});
            verify(table, .18, cu, cv, rational);
            const auto duplicate = rectangle(.2, .2, .8, .8);
            table["boundaries"] = group({duplicate, duplicate});
            verify(table, 0, cu, cv, rational);
            table["holeOrigin"] = 0;
            verify(table, 1, cu, cv, rational);
            table["holeOrigin"] = 1;
            table["boundaries"] = rectangle(0, 0, 1, 1);
            const auto full = verify(table, 1, cu, cv, rational);
            for (unsigned axis = 0; axis < 2; ++axis)
                if (flags[axis]) {
                    bool paired = false;
                    for (std::size_t i = 0; i < full.parameters.size(); ++i)
                        if (full.parameters[i][axis] == 0)
                            for (std::size_t j = 0; j < full.parameters.size(); ++j)
                                if (full.parameters[j][axis] == 1 &&
                                    full.parameters[j][1 - axis] == full.parameters[i][1 - axis]) {
                                    require(i != j, "periodic seam UV vertices remain distinct");
                                    for (unsigned k = 0; k < 3; ++k)
                                        require(std::abs(full.vertices[i][k] -
                                                         full.vertices[j][k]) < 2e-12,
                                                "paired seam points agree in world space");
                                    paired = true;
                                }
                    check(paired,
                          "trimmed periodic surface retains both representations of the seam");
                }
        }
    auto scaled = periodic(true, true);
    const auto base = BsplineSurface::from_bgfb(scaled);
    auto ku = base.u().knots(), kv = base.v().knots();
    for (auto &k : ku)
        k = 2 + 3 * k;
    for (auto &k : kv)
        k = -3 + 10 * k;
    scaled["knotsU"] = ku;
    scaled["knotsV"] = kv;
    scaled["boundaries"] = rectangle(4.4, -1, 5.6, 5);
    verify(scaled, .12, true, true, false);
    // A line from one seam to the other is not a closed UV contour, even
    // when its endpoints map to the same spatial point on the periodic surface.
    scaled["boundaries"] = rectangle(2, -1, 5, 5);
    scaled["boundaries"]["curves"][0]["geometry"]["points"] = {2, -1, 0, 5, -1, 0};
    const auto open = BsplineSurface::from_bgfb(scaled).mesh(options);
    check(open.report.at("status") == "incomplete" && open.vertices.empty() && open.faces.empty(),
          "UV-open boundary is not repaired by joining spatially coincident seam endpoints");
    scaled["boundaries"] = rectangle(2, -3, 5, 7);
    options.max_vertices = 4;
    const auto limited = BsplineSurface::from_bgfb(scaled).mesh(options);
    check(limited.report.at("status") == "incomplete" && limited.vertices.empty() &&
              limited.faces.empty(),
          "periodic trim budget failure publishes no partial surface");
    return checks;
}
