#include "internal.hpp"
#include "bspline_denominator.hpp"
#include <p3d/solid.hpp>
#include <p3d/csg_mesh_tree.hpp>
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
        const auto native = loft.face_indices(options.max_cap_control_points);
        check(native.report["status"] == "complete" && native.indices.size() == mesh.parts.size(),
              "mesh parts cover the native loft face enumeration");
        check(mesh.report.at("status") == "complete", "loft mesh complete: " + mesh.report.dump());
        check(mesh.report.at("indexed_edge_topology").at("closed_oriented_edges") == edge_closed,
              "loft mesh declared edge topology");
        check(mesh.face_parameters.size() == mesh.faces.size(), "loft mesh face parameter pairing");
        std::map<std::array<unsigned, 2>, std::pair<unsigned, int>> edges;
        std::size_t covered = 0;
        for (const auto &part : mesh.parts) {
            const auto ordinal = part.role == "bottom" ? 0
                                 : part.role == "top"
                                     ? 1
                                     : *part.side_index + (source.at("capped").get<bool>() ? 2 : 0);
            require(part.native_face_indices == native.indices.at(ordinal),
                    "mesh range identifies its native solid face independently of output order");
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
    // A mixed-sign control polygon is not a Cartesian convex hull. The
    // rational quadratic at t=.5 has Z=-99*height, although its Cartesian
    // middle control has Z=height and both endpoints have Z=0.
    auto rational_quadratic = [](double height, double sign = 1.) {
        return BsplineCurve::from_bgfb(
            {{"_type", "BsplineCurve"},
             {"order", 3},
             {"closed", false},
             {"poles", {0., 0., 0., -.99 * sign, 0., -.99 * height * sign, 2 * sign, 0., 0.}},
             {"weights", {sign, -.99 * sign, sign}},
             {"knots", {2, 2, 2, 7, 7, 7}}});
    };
    constexpr double plane_tolerance = 1e-5;
    for (double sign : {1., -1.}) {
        const auto outside = rational_quadratic(.5 * plane_tolerance, sign);
        check(std::abs(outside.point_at(.5)[2] + 49.5 * plane_tolerance) < 1e-15,
              "independent rational quadratic exposes control-hull planarity error");
        const auto rejected =
            certify_curve_plane(outside, {0, 0, 0}, {0, 0, 1}, plane_tolerance, 100000);
        check(rejected["status"] == "unverified" && rejected["distance_bound"].is_null(),
              "all controls within tolerance do not certify a mixed-sign curve within tolerance");
        const auto inside = rational_quadratic(.005 * plane_tolerance, sign);
        const auto proof =
            certify_curve_plane(inside, {0, 0, 0}, {0, 0, 7}, plane_tolerance, 100000);
        check(proof["status"] == "verified" && proof["visited_cells"] > 1 &&
                  proof["distance_bound"].get<double>() >= .495 * plane_tolerance &&
                  proof["distance_bound"].get<double>() <= plane_tolerance,
              "adaptive rational plane bound accepts a finite mixed-sign curve and normalizes the "
              "plane normal");
        for (unsigned i = 0; i <= 100; ++i)
            check(std::abs(inside.point_at(i / 100.)[2]) <= proof["distance_bound"].get<double>(),
                  "certified plane bound encloses independent curve evaluations");
        check(certify_curve_plane(inside, {0, 0, 0}, {0, 0, 1}, plane_tolerance, 1)["status"] ==
                  "unverified",
              "plane proof cannot ignore a depleted work budget");
        const auto exact_plane = rational_quadratic(0, sign);
        const auto exact_proof = certify_curve_plane(exact_plane, {0, 0, 0}, {0, 0, 1}, 0, 100000);
        check(exact_proof["status"] == "verified" && exact_proof["distance_bound"] == 0,
              "exact zero residual still proves the mixed-sign denominator before accepting zero "
              "tolerance");
    }
    {
        auto table = line({0, 0, 0}, {4, 0, 0});
        table["order"] = 3;
        table["knots"] = {2, 2, 2, 3, 3, 7, 7, 7};
        table["poles"] = {0, 0, 0, -.99, 0, 0, 2, 0, 0, -2.7, 0, 0, 4, 0, 0};
        table["weights"] = {1, -.99, 1, -.9, 1};
        const auto proof =
            certify_curve_plane(BsplineCurve::from_bgfb(table), {0, 0, 0}, {0, 0, 1}, 0, 100000);
        check(proof["status"] == "verified" && proof["verified_knot_spans"] == 2,
              "plane proof covers every nonuniform repeated-knot span in the original domain");
        table["knots"] = nullptr;
        table["poles"] = {0, 0, 0, 0, 0, 0, 2, 0, 0};
        table["weights"] = {1, 0, 1};
        check(certify_curve_plane(BsplineCurve::from_bgfb(table), {0, 0, 0}, {0, 0, 1}, 0,
                                  100000)["status"] == "verified",
              "plane proof uses homogeneous controls without dividing a zero control weight");
        table["weights"] = {1, -1, 1};
        check(certify_curve_plane(BsplineCurve::from_bgfb(table), {0, 0, 0}, {0, 0, 1}, 0,
                                  100000)["status"] == "unverified",
              "zero residual cannot conceal a rational singularity in the boundary");
    }
    auto signed_prism = prism();
    for (const auto name : {"section0", "section1"}) {
        const double z = std::string(name) == "section0" ? 0 : 3;
        auto &c = signed_prism[name]["curves"][0]["geometry"];
        c["order"] = 4;
        c["poles"] = {0, 0, z, 0, 0, -.1 * z, 4. / 3, 0, z, 2, 0, z};
        c["weights"] = {1, -.1, 1, 1};
    }
    const auto signed_loft = SectionLoft::from_bgfb(signed_prism);
    const auto signed_mesh = verify(signed_prism, true);
    check(std::abs(volume(signed_mesh) - 12) < 1e-9,
          "mixed-sign cap mesh retains independently known prism volume and closed topology");
    unsigned proof_steps = 0, adaptive_curves = 0;
    for (const auto &cap : signed_mesh.report["cap_planarity"])
        for (const auto &proof : cap) {
            check(proof["status"] == "verified", "every completed cap boundary is plane-verified");
            if (proof["method"] == "rational_bernstein_plane_bound") {
                ++adaptive_curves;
                proof_steps += proof["work_steps"].get<unsigned>();
                check(proof["distance_bound"].get<double>() <= options.planarity_tolerance,
                      "cap reports the full-curve distance bound");
            }
        }
    check(adaptive_curves == 2 && proof_steps > 1,
          "both caps use their mixed-sign source boundary");
    std::function<void(Json &)> flip_weights = [&](Json &j) {
        if (j.is_object() && j.value("_type", std::string()) == "BsplineCurve") {
            if (j["weights"].is_null())
                j["weights"] = std::vector<double>(j["poles"].size() / 3, 1.);
            for (auto &x : j["poles"])
                x = -x.get<double>();
            for (auto &w : j["weights"])
                w = -w.get<double>();
        } else if (j.is_structured())
            for (auto &child : j)
                if (child.is_structured())
                    flip_weights(child);
    };
    auto inverse_source = signed_prism;
    flip_weights(inverse_source["section0"]);
    flip_weights(inverse_source["section1"]);
    check(std::abs(volume(verify(inverse_source, true)) - 12) < 1e-9,
          "negating source homogeneous sections preserves mixed-sign cap geometry and orientation");
    std::function<void(Json &)> rotate_weighted = [&](Json &j) {
        if (j.is_object() && j.value("_type", std::string()) == "BsplineCurve") {
            for (std::size_t i = 0; i < j["poles"].size() / 3; ++i) {
                const double w = j["weights"].is_null() ? 1 : j["weights"][i].get<double>();
                const double x = j["poles"][3 * i].get<double>() / w,
                             y = j["poles"][3 * i + 1].get<double>() / w,
                             z = j["poles"][3 * i + 2].get<double>() / w;
                j["poles"][3 * i] = (1234 + .6 * x - .8 * z) * w;
                j["poles"][3 * i + 1] = (-2345 + .64 * x + .6 * y + .48 * z) * w;
                j["poles"][3 * i + 2] = (3456 + .48 * x - .8 * y + .36 * z) * w;
            }
        } else if (j.is_structured())
            for (auto &child : j)
                if (child.is_structured())
                    rotate_weighted(child);
    };
    auto oblique_source = signed_prism;
    rotate_weighted(oblique_source);
    check(
        std::abs(volume(verify(oblique_source, true)) - 12) < 1e-8,
        "translated oblique mixed-sign caps verify homogeneous plane residuals in all coordinates");
    auto plane_limited = options;
    plane_limited.max_planarity_steps = proof_steps - 1;
    const auto depleted = signed_loft.mesh(plane_limited);
    check(depleted.report["status"] == "incomplete" && depleted.vertices.empty() &&
              depleted.faces.empty() && depleted.parts.empty() && depleted.face_parameters.empty(),
          "plane work budget is shared by both caps and failure clears all partial geometry");
    plane_limited.max_planarity_steps = proof_steps;
    const auto exact_budget = signed_loft.mesh(plane_limited);
    check(exact_budget.report["status"] == "complete" &&
              exact_budget.vertices == signed_mesh.vertices &&
              exact_budget.faces == signed_mesh.faces,
          "exact combined plane budget gives identical complete geometry");
    auto sparse_source = prism();
    for (const auto name : {"section0", "section1"}) {
        const double z = std::string(name) == "section0" ? 0 : 3;
        auto &c = sparse_source[name]["curves"][0]["geometry"];
        c["order"] = 3;
        c["poles"] = {0, 0, z, -.99, 0, -.99 * (z + .5 * plane_tolerance), 2, 0, z};
        c["weights"] = {1, -.99, 1};
    }
    auto sparse_options = options;
    sparse_options.max_uv_edge = 4; // Boundary samples alone see only planar endpoints.
    sparse_options.planarity_tolerance = plane_tolerance;
    const auto sparse = SectionLoft::from_bgfb(sparse_source).mesh(sparse_options);
    check(sparse.report["status"] == "incomplete" && sparse.vertices.empty() &&
              sparse.faces.empty() &&
              sparse.report["reason"] == "loft cap plane bound not established",
          "cap proof rejects nonplanarity between otherwise planar mesh samples");
    plane_limited.max_planarity_steps = 0;
    threw = false;
    try {
        signed_loft.mesh(plane_limited);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, "zero plane work budget is an invalid mesh option");
    auto signed_task = std::async(std::launch::async, [&] { return signed_loft.mesh(options); });
    check(signed_task.get().report == signed_mesh.report && signed_loft.source() == signed_prism,
          "mixed-sign plane proofs are deterministic and do not mutate source data");
    for (unsigned loops : {1u, 2u}) {
        const auto source = prism(loops);
        const auto parsed = SectionLoft::from_bgfb(source);
        const auto solid = mesh_bgfb_solid(source, {}, 4);
        const auto &geometry = solid.derived.geometry;
        check(solid.derived.status == "meshed" && solid.source == source,
              "unified loft mesh retains original source");
        check(solid.surface_parameters.size() == geometry.faces.size(),
              "loft output triangle surface parameter alignment");
        LoftMesh shape;
        shape.vertices = geometry.vertices;
        shape.faces = geometry.faces;
        check(std::abs(volume(shape) - (loops == 1 ? 12. : 9.)) < 1e-9,
              "unified loft prism and annular prism independent volumes");
        bool correct = true, side_found = false, cap_found = false;
        for (std::size_t f = 0; f < geometry.faces.size(); ++f) {
            const auto face = solid.face_indices.at(*geometry.face_source_polygons.at(f));
            const auto &uv = solid.surface_parameters[f];
            if (face[0] == -1) {
                cap_found = true;
                correct &= !uv;
            } else {
                side_found = true;
                correct &= uv.has_value();
                if (uv)
                    for (unsigned k = 0; k < 3; ++k) {
                        const auto p = parsed.sides()
                                           .at(std::size_t(face[1]))
                                           .surface.point_at((*uv)[k][0], (*uv)[k][1]);
                        const auto q = geometry.vertices.at(geometry.faces[f][k]);
                        for (unsigned axis = 0; axis < 3; ++axis)
                            correct &= std::abs(p[axis] - q[axis]) < 1e-9;
                    }
            }
        }
        check(correct && side_found && cap_found,
              "native loft face ID and each corner UV evaluate back to its 3D output");
        check(geometry.uvs.empty() &&
                  std::none_of(geometry.face_uvs.begin(), geometry.face_uvs.end(),
                               [](const auto &uv) { return uv.has_value(); }),
              "surface coordinates do not invent material texture coordinates");
    }
    const auto source_loft = prism();
    PolyfaceMeshOptions low;
    for (unsigned budget = 0; budget < 3; ++budget) {
        low = {};
        if (budget == 0)
            low.max_points = 3;
        if (budget == 1)
            low.max_triangles = 1;
        if (budget == 2)
            low.max_corners = 3;
        const auto failure = mesh_bgfb_solid(source_loft, low, 4);
        check(failure.derived.status != "meshed" && failure.derived.geometry.vertices.empty() &&
                  failure.derived.geometry.faces.empty() && failure.face_indices.empty() &&
                  failure.surface_parameters.empty() && failure.source == source_loft,
              "unified loft budgets fail without partial output or lost source");
    }
    check(mesh_bgfb_solid(source_loft, {}, 0).derived.status != "meshed",
          "invalid loft sampling count rejected");
    check(mesh_bgfb_solid(prism(1, 2, false), {}, 4).derived.status == "meshed",
          "unified loft supports uncapped sides");
    Json rows = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}};
    Json archive = {{"geometries",
                     {{{"status", "decoded"},
                       {"encoding", "bgfb"},
                       {"geometry", {{"geometry", source_loft}}}}}},
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
    CsgMeshTreeOptions tree_options;
    tree_options.solid_circle_segments = 4;
    const auto csg = evaluate_csg_polyface_archive(archive, tree_options);
    check(csg.result.status == "evaluated" && csg.result.meshes.size() == 1 &&
              csg.solid_sources.size() == 1 && csg.solid_snapshots.size() == 1 &&
              !csg.solid_snapshots[0].mesh.surface_parameters.empty(),
          "identity CSG loft retains source analytic parameters");
    for (double scale : {2., -1., .5}) {
        auto changed = archive;
        changed["transforms"][0]["matrix_3x4_rows"][0][0] = scale;
        const auto placed_loft = evaluate_csg_polyface_archive(changed, tree_options);
        check(placed_loft.result.status == "evaluated" && placed_loft.solid_snapshots.size() == 1 &&
                  placed_loft.solid_placements[0].matrix[0][0] == scale &&
                  placed_loft.result.meshes[0].face_sources[0].solid_snapshot == 0,
              "nonidentity CSG loft reconstructs source curves and records a new mesh source");
    }
    // A polyline's length knots change under anisotropic placement, even when
    // every corresponding guide has the same number of primitives.
    auto bent = source_loft;
    for (auto &guide : bent["guide_groups"][0]) {
        const auto poles = guide["curves"][0]["geometry"]["poles"];
        const double x = poles[0], y = poles[1];
        guide["curves"][0]["geometry"] = {{"_type", "LineString"},
                                          {"points", {x, y, 0., x + 1, y, 1., x, y, 3.}}};
    }
    auto scaled = bent;
    transform_curves(scaled, [](Point3 p) {
        p[0] *= 3;
        return p;
    });
    for (auto &guide : scaled["guide_groups"][0]) {
        auto &points = guide["curves"][0]["geometry"]["points"];
        for (std::size_t i = 0; i < points.size(); i += 3)
            points[i] = 3 * points[i].get<double>();
    }
    const auto before = SectionLoft::from_bgfb(bent).sides()[0].surface.point_at(.5, .4);
    const auto after = SectionLoft::from_bgfb(scaled).sides()[0].surface.point_at(.5, .4);
    check(std::abs(after[0] - 3 * before[0]) > .05 || std::abs(after[2] - before[2]) > .05,
          "source loft reconstruction demonstrably differs from transforming cached surface");
    Matrix4 placement{};
    for (unsigned i = 0; i < 4; ++i)
        placement[i][i] = 1;
    const auto identity_matrix = placement;
    placement[0][0] = 3;
    const auto source_placed = transform_bgfb_section_loft(bent, placement);
    check(source_placed.status == "transformed" && source_placed.transformed == scaled,
          "loft source placement equals independently transformed sections and guides");
    check(mesh_bgfb_solid(source_placed.transformed, {}, 4).derived.status == "meshed",
          "placed bent guides reconstruct and mesh through the public solid entry");
    auto weighted_source = source_loft;
    auto &weighted = weighted_source["section0"]["curves"][0]["geometry"];
    weighted["weights"] = {2., 0.};
    weighted["poles"] = {2., 4., 6., 8., 10., 12.};
    weighted["unknown_tag"] = {7, 8, 9};
    placement[0] = {2, .5, 0, 7};
    placement[1] = {0, -3, 1, -4};
    placement[2] = {1, 0, 2, 5};
    const auto rational = transform_bgfb_section_loft(weighted_source, placement);
    check(rational.status == "transformed", "rational loft source transform succeeds");
    const auto &rp = rational.transformed["section0"]["curves"][0]["geometry"];
    check(rp["poles"] == Json({20., -14., 24., 21., -18., 32.}) &&
              rp["weights"] == weighted["weights"] && rp["knots"] == weighted["knots"] &&
              rp["unknown_tag"] == weighted["unknown_tag"],
          "homogeneous translation handles unequal and zero weights without altering metadata");
    auto repeated = transform_bgfb_section_loft(rational.transformed, placement);
    check(repeated.status == "transformed" &&
              repeated.transformed["section0"]["curves"][0]["geometry"]["poles"] ==
                  Json({47., 58., 78., 33., 86., 85.}),
          "repeated placement updates current homogeneous coordinates in native order");
    auto tiny = identity_matrix;
    tiny[0][3] = 5e-11;
    const auto small = transform_bgfb_section_loft(bent, tiny);
    check(small.status == "transformed" && small.report["bspline_skipped"] == 8 &&
              small.transformed["section0"] == bent["section0"] &&
              small.transformed["guide_groups"][0][0]["curves"][0]["geometry"]["points"][0] ==
                  5e-11,
          "native near-identity skips B-splines but still transforms line-string guides");
    tiny[0][3] = 1e-10;
    check(transform_bgfb_section_loft(bent, tiny).report["bspline_skipped"] == 0,
          "native translation identity boundary is strict");
    tiny = identity_matrix;
    tiny[0][1] = 1e-12;
    check(transform_bgfb_section_loft(bent, tiny).report["bspline_skipped"] == 8,
          "native linear identity boundary is inclusive");
    tiny[0][1] = std::nextafter(1e-12, 1.);
    check(transform_bgfb_section_loft(bent, tiny).report["bspline_skipped"] == 0,
          "one representable step above identity boundary applies B-spline placement");
    auto arc_source = source_loft;
    arc_source["section0"]["curves"][0]["geometry"] = {{"_type", "EllipticArc"},
                                                       {"arc",
                                                        {{"centerX", 1},
                                                         {"centerY", 2},
                                                         {"centerZ", 3},
                                                         {"vector0X", 2},
                                                         {"vector0Y", 0},
                                                         {"vector0Z", 0},
                                                         {"vector90X", 0},
                                                         {"vector90Y", 3},
                                                         {"vector90Z", 0},
                                                         {"startRadians", .3},
                                                         {"sweepRadians", -1.2}}}};
    const auto arc_result = transform_bgfb_section_loft(arc_source, placement);
    const auto &arc = arc_result.transformed["section0"]["curves"][0]["geometry"]["arc"];
    check(arc_result.status == "transformed" && arc["centerX"] == 10. && arc["centerY"] == -7. &&
              arc["centerZ"] == 12. && arc["vector0X"] == 4. && arc["vector0Z"] == 2. &&
              arc["vector90X"] == 1.5 && arc["vector90Y"] == -9. && arc["startRadians"] == .3 &&
              arc["sweepRadians"] == -1.2,
          "ellipse center is a point, axes are vectors, angular parameters are retained");
    auto singular = identity_matrix;
    singular[0][0] = 0;
    check(transform_bgfb_section_loft(source_loft, singular).status == "transformed",
          "source placement can succeed even when later mesh reconstruction degenerates");
    const auto collapsed = transform_bgfb_section_loft(arc_source, singular);
    check(collapsed.status == "transformed" &&
              collapsed.transformed["section0"]["curves"][0]["geometryType"] == 1 &&
              collapsed.report["arc_replacements"][0]["source_path"] ==
                  "/section0/curves/0/geometry",
          "collapsed ellipse replacement preserves source path and updates union discriminator");
    for (unsigned mode = 0; mode < 3; ++mode) {
        LoftSourceTransformOptions limit;
        if (mode == 0)
            limit.max_points = 1;
        if (mode == 1)
            limit.max_curve_nodes = 1;
        if (mode == 2)
            limit.max_depth = 0;
        const auto limited = transform_bgfb_section_loft(bent, placement, limit);
        check(limited.status == "not_evaluated" && limited.transformed.is_null(),
              "aggregate source transformation budgets clear partial work");
    }
    auto bad_matrix = placement;
    bad_matrix[3][0] = .01;
    check(transform_bgfb_section_loft(bent, bad_matrix).transformed.is_null(),
          "projective matrix cannot be treated as native affine placement");
    bad_matrix = placement;
    bad_matrix[0][0] = std::numeric_limits<double>::infinity();
    check(transform_bgfb_section_loft(bent, bad_matrix).transformed.is_null(),
          "nonfinite placement is rejected");
    auto parallel = std::async(std::launch::async, [&] {
        return transform_bgfb_section_loft(weighted_source, placement);
    });
    check(parallel.get().transformed == rational.transformed &&
              weighted_source["section0"]["curves"][0]["geometry"]["poles"] ==
                  Json({2., 4., 6., 8., 10., 12.}),
          "parallel source placement is deterministic and leaves caller input unchanged");
    const auto nested = transform_bgfb_section_loft(prism(2), placement);
    const auto nested_mesh = mesh_bgfb_solid(nested.transformed, {}, 4);
    LoftMesh nested_shape;
    nested_shape.vertices = nested_mesh.derived.geometry.vertices;
    nested_shape.faces = nested_mesh.derived.geometry.faces;
    check(nested.status == "transformed" && nested_mesh.derived.status == "meshed" &&
              std::abs(std::abs(volume(nested_shape)) - 103.5) < 1e-8,
          "nested parity sections and guide groups preserve independent transformed volume");
    constexpr double tau = 2 * pi;
    // An independent bounded-period critical-angle oracle, rather than the
    // native fmod normalization implemented in the library.
    for (bool sine : {false, true})
        for (double start : {-5 * tau - .3, -tau, -.2, 0., pi / 2, 3 * tau + .4})
            for (double sweep : {-3 * tau, -4., -.2, 0., .2, 4., 3 * tau}) {
                auto input = arc_source;
                auto &a = input["section0"]["curves"][0]["geometry"]["arc"];
                a["startRadians"] = start;
                a["sweepRadians"] = sweep;
                a["vector0X"] = sine ? 1e-7 : 2.;
                a["vector90Y"] = sine ? 3. : 1e-7;
                input["section0"]["curves"][0]["geometry"]["unknown_tag"] = {17, 19};
                input["section0"]["curves"][0]["geometryType"] = 2;
                const auto copy = input;
                const auto r = transform_bgfb_section_loft(input, identity_matrix);
                double lo = std::min(sine ? std::sin(start) : std::cos(start),
                                     sine ? std::sin(start + sweep) : std::cos(start + sweep));
                double hi = std::max(sine ? std::sin(start) : std::cos(start),
                                     sine ? std::sin(start + sweep) : std::cos(start + sweep));
                for (int period = -10; period <= 10; ++period)
                    for (unsigned extremum = 0; extremum < 2; ++extremum) {
                        const double angle = (sine ? pi / 2 : 0.) + extremum * pi + period * tau;
                        if (angle >= std::min(start, start + sweep) &&
                            angle <= std::max(start, start + sweep)) {
                            if (extremum)
                                lo = -1.;
                            else
                                hi = 1.;
                        }
                    }
                const auto &range = r.report["arc_replacements"][0]["scalar_interval"];
                const auto &s = r.transformed["section0"]["curves"][0]["geometry"]["segment"];
                const auto p = s.at(sine ? "point0Y" : "point0X").get<double>();
                const auto q = s.at(sine ? "point1Y" : "point1X").get<double>();
                check(r.status == "transformed" && std::abs(range[0].get<double>() - lo) < 1e-14 &&
                          std::abs(range[1].get<double>() - hi) < 1e-14 &&
                          std::abs(p - ((sine ? 2. : 1.) + (sine ? 3. : 2.) * lo)) < 1e-13 &&
                          std::abs(q - ((sine ? 2. : 1.) + (sine ? 3. : 2.) * hi)) < 1e-13 &&
                          input == copy &&
                          r.report["arc_replacements"][0]["transformed_arc"]["unknown_tag"] ==
                              Json({17, 19}),
                      "collapsed arc interval and ordered endpoints match independent periodic "
                      "extrema");
            }
    auto both = arc_source;
    auto &both_arc = both["section0"]["curves"][0]["geometry"]["arc"];
    both_arc["vector0X"] = 1e-7;
    both_arc["vector90Y"] = 2e-7;
    both_arc["startRadians"] = 0.;
    both_arc["sweepRadians"] = tau;
    const auto priority = transform_bgfb_section_loft(both, identity_matrix);
    check(priority.status == "transformed" &&
              priority.report["arc_replacements"][0]["retained_axis"] == "vector90" &&
              priority.transformed["section0"]["curves"][0]["geometry"]["segment"]["point0X"] == 1.,
          "both short axes use first-axis branch and discard its nonzero contribution");
    both_arc["vector0X"] = 0.;
    both_arc["vector90Y"] = 0.;
    const auto point_segment = transform_bgfb_section_loft(both, identity_matrix);
    const auto &ps = point_segment.transformed["section0"]["curves"][0]["geometry"]["segment"];
    check(point_segment.status == "transformed" && ps["point0X"] == ps["point1X"] &&
              ps["point0Y"] == ps["point1Y"] && ps["point0Z"] == ps["point1Z"],
          "fully collapsed ellipse retains a degenerate source segment rather than vanishing");
    both_arc["vector0X"] = 1e-5;
    both_arc["vector90Y"] = 3.;
    check(transform_bgfb_section_loft(both, identity_matrix).report["arc_replacements"].empty(),
          "native collapse length threshold is strict");
    both_arc["vector0X"] = std::nextafter(1e-5, 0.);
    check(transform_bgfb_section_loft(both, identity_matrix).report["arc_replacements"].size() == 1,
          "one floating-point step below collapse threshold changes representation");
    auto tiny_prism = source_loft;
    for (const auto *name : {"section0", "section1"}) {
        auto a = arc_source["section0"]["curves"][0]["geometry"];
        a["arc"]["centerX"] = 1.;
        a["arc"]["centerY"] = 0.;
        a["arc"]["centerZ"] = std::string(name) == "section0" ? 0. : 3.;
        a["arc"]["vector0X"] = 1.;
        a["arc"]["vector90Y"] = 1e-7;
        a["arc"]["startRadians"] = 0.;
        a["arc"]["sweepRadians"] = pi;
        tiny_prism[name]["curves"][0]["geometry"] = a;
    }
    const auto replaced = transform_bgfb_section_loft(tiny_prism, identity_matrix);
    const auto rebuilt_mesh = mesh_bgfb_solid(replaced.transformed, {}, 4);
    LoftMesh rebuilt_shape;
    rebuilt_shape.vertices = rebuilt_mesh.derived.geometry.vertices;
    rebuilt_shape.faces = rebuilt_mesh.derived.geometry.faces;
    check(replaced.status == "transformed" && rebuilt_mesh.derived.status == "meshed" &&
              std::abs(volume(rebuilt_shape) - 12.) < 1e-9,
          "native minimum-to-maximum segment order rebuilds a closed loft with known volume");
    const auto twice_replaced = transform_bgfb_section_loft(replaced.transformed, identity_matrix);
    check(twice_replaced.status == "transformed" &&
              twice_replaced.report["arc_replacements"].empty(),
          "a replaced source remains a line segment on later visits");
    auto csg_replacement = archive;
    csg_replacement["geometries"][0]["geometry"]["geometry"] = tiny_prism;
    const auto rebuilt_csg = evaluate_csg_polyface_archive(csg_replacement, tree_options);
    check(rebuilt_csg.result.status == "evaluated" && rebuilt_csg.solid_snapshots.size() == 1 &&
              rebuilt_csg.solid_placements[0].report["arc_replacements"].size() == 2 &&
              rebuilt_csg.solid_sources[0].source == tiny_prism,
          "CSG reconstructs identity-triggered replacements and retains the original source");
    return checks;
}
