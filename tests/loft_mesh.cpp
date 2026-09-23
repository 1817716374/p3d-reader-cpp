#include "internal.hpp"
#include "bspline_denominator.hpp"
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
    return checks;
}
