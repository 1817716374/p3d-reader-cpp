#include "internal.hpp"
#include <future>
using namespace p3d;
namespace {
Json array(Json geometries, unsigned type) {
    Json curves = Json::array();
    for (auto &g : geometries)
        curves.push_back({{"geometry", g}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", curves}};
}
Json line(Point3 a, Point3 b) {
    return {{"_type", "BsplineCurve"}, {"order", 2},
            {"closed", false},         {"weights", nullptr},
            {"knots", nullptr},        {"poles", {a[0], a[1], a[2], b[0], b[1], b[2]}}};
}
Json prism(unsigned loops = 1, unsigned type = 2, bool capped = true) {
    Json bottom = Json::array(), top = Json::array(), groups = Json::array();
    for (unsigned ring = 0; ring < loops; ++ring) {
        const double a = ring ? .5 : 0, b = ring ? 1.5 : 2;
        std::vector<Point3> p = {{a, a, 0}, {b, a, 0}, {b, b, 0}, {a, b, 0}, {a, a, 0}};
        if (ring)
            std::reverse(p.begin(), p.end());
        Json lower = Json::array(), upper = Json::array(), guides = Json::array();
        for (unsigned i = 0; i < 4; ++i) {
            auto x = p[i], y = p[i + 1], z = x, w = y;
            z[2] = w[2] = 3;
            lower.push_back(line(x, y));
            upper.push_back(line(z, w));
            guides.push_back(array(Json::array({line(x, z)}), 1));
        }
        if (type == 1)
            guides.push_back(array(Json::array({line(p.back(), {a, a, 3})}), 1));
        bottom.push_back(array(lower, ring ? 3 : type));
        top.push_back(array(upper, ring ? 3 : type));
        groups.push_back(guides);
    }
    return {{"_type", "P3DSectionLoft"},
            {"capped", capped},
            {"section0", loops == 1 ? bottom[0] : array(bottom, 4)},
            {"section1", loops == 1 ? top[0] : array(top, 4)},
            {"guide_groups", groups}};
}
double volume(const LoftMesh &mesh) {
    double total = 0;
    auto local = [&](Point3 p) {
        for (unsigned k = 0; k < 3; ++k)
            p[k] -= mesh.vertices.front()[k];
        return p;
    };
    for (const auto &f : mesh.faces) {
        const auto a = local(mesh.vertices[f[0]]), b = local(mesh.vertices[f[1]]),
                   c = local(mesh.vertices[f[2]]);
        total += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
                 a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    return total / 6;
}
void transform_curves(Json &j, const std::function<Point3(Point3)> &fn) {
    if (j.is_object()) {
        if (j.value("_type", std::string()) == "BsplineCurve") {
            auto &p = j.at("poles");
            for (std::size_t i = 0; i < p.size(); i += 3) {
                const auto q =
                    fn({p[i].get<double>(), p[i + 1].get<double>(), p[i + 2].get<double>()});
                for (unsigned k = 0; k < 3; ++k)
                    p[i + k] = q[k];
            }
        } else
            for (auto &v : j)
                if (v.is_structured())
                    transform_curves(v, fn);
    } else if (j.is_array())
        for (auto &v : j)
            transform_curves(v, fn);
}
} // namespace
unsigned loft_mesh_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const std::string &message) {
        ++checks;
        require(value, message);
    };
    LoftMeshOptions options;
    options.max_uv_edge = .2;
    auto verify = [&](const Json &source, bool edge_closed) {
        const auto loft = SectionLoft::from_bgfb(source);
        const auto mesh = loft.mesh(options);
        check(mesh.report.at("status") == "complete", "loft mesh complete: " + mesh.report.dump());
        check(mesh.report.at("indexed_edge_topology").at("closed_oriented_edges") == edge_closed,
              "loft mesh declared edge topology");
        check(mesh.face_parameters.size() == mesh.faces.size(), "loft mesh face parameter pairing");
        std::map<std::array<unsigned, 2>, std::pair<unsigned, int>> edges;
        std::size_t covered = 0;
        for (const auto &part : mesh.parts) {
            require(part.first_face == covered && part.face_count > 0,
                    "contiguous nonempty mesh part ranges");
            covered += part.face_count;
            for (std::size_t i = part.first_face; i < covered; ++i) {
                const auto &face = mesh.faces.at(i);
                for (unsigned k = 0; k < 3; ++k) {
                    require(face[k] < mesh.vertices.size(), "loft mesh index range");
                    const auto a = face[k], b = face[(k + 1) % 3];
                    auto &e = edges[{std::min(a, b), std::max(a, b)}];
                    ++e.first;
                    e.second += a < b ? 1 : -1;
                }
                const auto &uv = mesh.face_parameters[i];
                if (part.role == "side") {
                    require(part.side_index.has_value() && uv.has_value(), "side mesh provenance");
                    const auto &surface = loft.sides().at(*part.side_index).surface;
                    for (unsigned k = 0; k < 3; ++k) {
                        const auto p = surface.point_at((*uv)[k][0], (*uv)[k][1]);
                        const auto q = mesh.vertices[face[k]];
                        require(std::hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]) <=
                                    options.join_tolerance + 1e-13,
                                "side sample follows surface");
                        const auto a = (*uv)[k], b = (*uv)[(k + 1) % 3];
                        require(std::hypot(a[0] - b[0], a[1] - b[1]) <=
                                    options.max_uv_edge * (1 + 1e-14),
                                "side UV edge bound");
                    }
                } else
                    require(!uv && !part.side_index,
                            "cap does not invent side UV or native material ID");
            }
        }
        check(covered == mesh.faces.size(), "mesh part coverage");
        std::size_t boundary = 0;
        for (const auto &e : edges) {
            require(e.second.first <= 2, "derived mesh nonmanifold edge");
            if (e.second.first == 2)
                require(e.second.second == 0, "derived mesh orientation conflict");
            else
                ++boundary;
        }
        check((boundary == 0) == edge_closed, "independent edge incidence verification");
        check(loft.source() == source && mesh.report["world_space_error_bound"].is_null(),
              "source preservation and explicit accuracy scope");
        return mesh;
    };
    const auto simple = verify(prism(), true);
    check(std::abs(volume(simple) - 12) < 1e-10, "closed prism analytical volume");
    auto rotated = prism();
    transform_curves(rotated, [](Point3 p) {
        return Point3{1234 + p[2], 2345 + .6 * p[0] - .8 * p[1], 3456 + .8 * p[0] + .6 * p[1]};
    });
    const auto rotated_mesh = verify(rotated, true);
    check(std::abs(volume(rotated_mesh) - 12) < 1e-8 &&
              rotated_mesh.report.at("cap_collinearity_distance").get<double>() <=
                  options.join_tolerance,
          "translated rotated cap retains all boundary samples under bounded roundoff handling");
    auto reflected = prism();
    transform_curves(reflected, [](Point3 p) {
        p[0] = -p[0];
        return p;
    });
    check(std::abs(volume(verify(reflected, true)) + 12) < 1e-10,
          "derived mesh preserves source orientation instead of forcing positive volume");
    const auto holes = verify(prism(2), true);
    check(std::abs(volume(holes) - 9) < 1e-10, "hole retained in cap and whole mesh volume");
    auto wrong_orientation = prism(2);
    const auto swap_xy = [](Point3 p) {
        std::swap(p[0], p[1]);
        return p;
    };
    transform_curves(wrong_orientation["section0"]["curves"][1], swap_xy);
    transform_curves(wrong_orientation["section1"]["curves"][1], swap_xy);
    transform_curves(wrong_orientation["guide_groups"][1], swap_xy);
    const auto conflicted = SectionLoft::from_bgfb(wrong_orientation).mesh(options);
    check(conflicted.report["status"] == "complete" &&
              conflicted.report["indexed_edge_topology"]["orientation_conflicts"] > 0 &&
              conflicted.report["indexed_edge_topology"]["closed_oriented_edges"] == false,
          "wrong source inner winding is reported instead of silently reorienting source sides");
    auto touching = prism(2);
    const auto shift = [](Point3 p) {
        p[0] += .5;
        p[1] += .5;
        return p;
    };
    transform_curves(touching["section0"]["curves"][1], shift);
    transform_curves(touching["section1"]["curves"][1], shift);
    transform_curves(touching["guide_groups"][1], shift);
    const auto contact = SectionLoft::from_bgfb(touching).mesh(options);
    check(contact.report["status"] == "incomplete" && contact.vertices.empty() &&
              contact.faces.empty(),
          "contacting cap rings cannot silently leave unmatched side boundaries");
    for (const auto &part : holes.parts)
        if (part.role != "side")
            for (std::size_t i = part.first_face; i < part.first_face + part.face_count; ++i) {
                Point3 p{};
                for (auto index : holes.faces[i])
                    for (unsigned k = 0; k < 3; ++k)
                        p[k] += holes.vertices[index][k] / 3;
                require(!(p[0] > .5 && p[0] < 1.5 && p[1] > .5 && p[1] < 1.5),
                        "cap triangle inside hole");
            }
    verify(prism(1, 2, false), false);
    const auto open = verify(prism(1, 1, true), false);
    check(open.report["indexed_edge_topology"]["boundary_edges"] > 0,
          "open source does not silently identify independent first/last guide interiors");
    auto circle = prism();
    constexpr double pi = 3.1415926535897932384626433832795;
    for (unsigned i = 0; i < 4; ++i) {
        const double t = i * pi / 2;
        for (unsigned end = 0; end < 2; ++end) {
            Json arc = {{"_type", "EllipticArc"},
                        {"arc",
                         {{"centerX", 0},
                          {"centerY", 0},
                          {"centerZ", end ? 3 : 0},
                          {"vector0X", 1},
                          {"vector0Y", 0},
                          {"vector0Z", 0},
                          {"vector90X", 0},
                          {"vector90Y", 1},
                          {"vector90Z", 0},
                          {"startRadians", t},
                          {"sweepRadians", pi / 2}}}};
            circle[end ? "section1" : "section0"]["curves"][i]["geometry"] = arc;
        }
        circle["guide_groups"][0][i] = array(
            Json::array({line({std::cos(t), std::sin(t), 0}, {std::cos(t), std::sin(t), 3})}), 1);
    }
    const auto cylinder = verify(circle, true);
    auto quarter = [](double t) {
        const double a = (1 - t) * (1 - t), b = std::sqrt(2.0) * t * (1 - t), c = t * t;
        return Point2{(a + b) / (a + b + c), (b + c) / (a + b + c)};
    };
    double polygon_volume = 0;
    for (unsigned i = 0; i < 8; ++i) {
        const auto a = quarter(i / 8.0), b = quarter((i + 1) / 8.0);
        polygon_volume += 6 * (a[0] * b[1] - a[1] * b[0]);
    }
    check(std::abs(volume(cylinder) - polygon_volume) < 1e-10,
          "rational circular loft matches independently sampled analytical polygon volume");
    options.max_uv_edge = .1;
    const auto fine_cylinder = verify(circle, true);
    options.max_uv_edge = .2;
    check(std::abs(volume(fine_cylinder) - 3 * pi) < .02 &&
              std::abs(volume(fine_cylinder) - 3 * pi) < .4 * std::abs(volume(cylinder) - 3 * pi),
          "finer side UV sampling converges toward analytical cylinder volume");
    auto knots = prism();
    for (const auto name : {"section0", "section1"}) {
        auto &c = knots[name]["curves"][0]["geometry"];
        const double z = std::string(name) == "section0" ? 0 : 3;
        c["knots"] = {0, 0, .17, .41, 1, 1};
        c["poles"] = {0, 0, z, .34, 0, z, .82, 0, z, 2, 0, z};
    }
    verify(knots, true);
    auto nonplanar = prism();
    auto &curve = nonplanar["section0"]["curves"][0]["geometry"];
    curve["order"] = 3;
    curve["poles"] = {0, 0, 0, 1, 0, 1, 2, 0, 0};
    const auto nonplanar_loft = SectionLoft::from_bgfb(nonplanar);
    check(nonplanar_loft.cap_regions().report["status"] == "complete" &&
              nonplanar_loft.mesh(options).report["status"] == "incomplete",
          "nonplanar cap cannot be flattened because native boundary closure succeeded");
    nonplanar["capped"] = false;
    verify(nonplanar, false);
    const auto loft = SectionLoft::from_bgfb(prism());
    for (unsigned which = 0; which < 3; ++which) {
        auto limited = options;
        if (which == 0)
            limited.max_vertices = 3;
        if (which == 1)
            limited.max_triangles = 1;
        if (which == 2)
            limited.max_cap_control_points = 1;
        const auto result = loft.mesh(limited);
        check(result.report["status"] == "incomplete" && result.vertices.empty() &&
                  result.faces.empty() && result.parts.empty() && result.face_parameters.empty(),
              "failed mesh has no partial result");
    }
    auto invalid = options;
    invalid.max_uv_edge = 0;
    bool threw = false;
    try {
        loft.mesh(invalid);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, "invalid mesh options throw");
    auto task = std::async(std::launch::async, [&] { return loft.mesh(options); });
    const auto repeat = task.get();
    check(repeat.vertices == simple.vertices && repeat.faces == simple.faces &&
              repeat.report == simple.report,
          "loft mesh repeatable and concurrent");
    return checks;
}
