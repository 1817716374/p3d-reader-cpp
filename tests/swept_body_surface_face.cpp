#include <p3d/swept_body.hpp>
#include <cmath>
#include <future>
#include <stdexcept>
using namespace p3d;
namespace {
SweptBodySurface net(double h = 0) {
    SweptBodySurface s;
    s.geometry = {{"_type", "BsplineSurface"},
                  {"orderU", 2},
                  {"orderV", 2},
                  {"numPolesU", 2},
                  {"numPolesV", 2},
                  {"closedU", false},
                  {"closedV", false},
                  {"poles", {-2, -3, h, 2, -3, -h, -2, 3, -h, 2, 3, h}},
                  {"weights", nullptr},
                  {"knotsU", nullptr},
                  {"knotsV", nullptr},
                  {"boundaries", nullptr},
                  {"holeOrigin", 1},
                  {"numRulesU", 2},
                  {"numRulesV", 2}};
    return s;
}
bool near(double a, double b, double tolerance = 1e-9) {
    return std::abs(a - b) <= tolerance;
}
} // namespace
unsigned swept_body_surface_face_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        if (!ok)
            throw std::runtime_error(why);
    };
    auto s = net();
    const auto original = s.geometry;
    auto f = extract_swept_body_surface_face(s);
    check(f.status == "extracted" && f.region["type"] == 2 && !f.surface,
          "planar side becomes native outer-loop region");
    check(f.region["curves"].size() == 4 && f.report["native_geometry_kind"] == 2,
          "planar side contains native four directed boundaries");
    const auto &p = f.report["plane_query"];
    check(p["principal_moments"] == Json({52., 36., 16.}), "rectangle analytic inertia moments");
    check(p["centroid"] == Json({0., 0., 0.}) && p["jacobi_converged"] == true,
          "rectangle centroid and diagonal eigensystem");
    check(near(p["extent_lengths"][2], 0), "rectangle principal thickness zero");
    check(p["extent_transform"] ==
              Json({{0., 4., 0., -2.}, {-6., 0., 0., 3.}, {0., 0., 0., 0.}, {0., 0., 0., 1.}}),
          "native cyclic axes, original range swap and corner shift");
    check(s.geometry == original, "face extraction leaves source table unchanged");
    // A rigid rotation with three nonzero normal coordinates, then translation.
    const Matrix3 rotation{{{.36, -.8, .48}, {.48, .6, .64}, {-.8, 0, .6}}};
    const Point3 translation{12, -25, 70};
    for (unsigned k = 0; k < 4; ++k) {
        Point3 xyz{};
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j)
                xyz[i] += rotation[i][j] * original["poles"][3 * k + j].get<double>();
        for (unsigned i = 0; i < 3; ++i)
            s.geometry["poles"][3 * k + i] = xyz[i] + translation[i];
    }
    const auto tilted = extract_swept_body_surface_face(s);
    check(tilted.status == "extracted" && !tilted.region.is_null(),
          "tilted plane classified by full control net");
    const auto &t = tilted.report["plane_query"];
    const auto axes = t["principal_axes"].get<Matrix3>();
    const auto moments = t["principal_moments"].get<Point3>();
    for (unsigned i = 0; i < 3; ++i) {
        check(near(t["centroid"][i], translation[i]), "rigid translation centroid");
        check(near(moments[i], Point3{52, 36, 16}[i]), "rotation preserves analytic moments");
    }
    // Check eigenvectors against independently transformed diagonal inertia.
    const Point3 diagonal{36, 16, 52};
    for (unsigned col = 0; col < 3; ++col)
        for (unsigned row = 0; row < 3; ++row) {
            double lhs = 0;
            for (unsigned j = 0; j < 3; ++j)
                for (unsigned k = 0; k < 3; ++k)
                    lhs += rotation[row][k] * diagonal[k] * rotation[j][k] * axes[j][col];
            check(near(lhs, moments[col] * axes[row][col]), "principal eigenvector residual");
        }
    check(t["jacobi_rotations"].get<unsigned>() > 0, "inclined net exercises Jacobi rotations");
    // Variable rational weights preserve the same Euclidean control points.
    s.geometry["weights"] = {2., .5, 4., 8.};
    for (unsigned k = 0; k < 4; ++k)
        for (unsigned j = 0; j < 3; ++j)
            s.geometry["poles"][3 * k + j] = s.geometry["poles"][3 * k + j].get<double>() *
                                             s.geometry["weights"][k].get<double>();
    auto rational = extract_swept_body_surface_face(s);
    check(rational.status == "extracted" && rational.report["plane_query"] == t,
          "principal extents use deweighted rational poles");
    auto bent = net(.2);
    bent.boundary_points = {{{.1, .1}, {.9, .1}, {.1, .1}}};
    const auto curved = extract_swept_body_surface_face(bent);
    check(curved.status == "extracted" && curved.region.is_null() && curved.surface.has_value(),
          "bilinear saddle remains a surface");
    check(curved.surface->geometry == bent.geometry &&
              curved.surface->boundary_points == bent.boundary_points,
          "curved output keeps native runtime trim data");
    check(near(curved.report["plane_query"]["extent_lengths"][2], .4), "saddle full net thickness");
    check(!extract_swept_body_surface_face(net(1e-6)).report["plane_query"]["planar"].get<bool>(),
          "thin resolvable saddle remains nonplanar");
    check(extract_swept_body_surface_face(net(1e-12)).report["plane_query"]["planar"] == true,
          "native tolerance accepts negligible thickness");
    auto point = net();
    point.geometry["poles"] = std::vector<double>(12, 7);
    auto collapsed = extract_swept_body_surface_face(point);
    check(collapsed.status == "extracted" && collapsed.report["plane_query"]["planar"] == true,
          "coincident net preserves native degenerate planar result");
    auto line = net();
    for (unsigned k = 0; k < 4; ++k) {
        line.geometry["poles"][3 * k + 1] = 0;
        line.geometry["poles"][3 * k + 2] = 0;
    }
    check(extract_swept_body_surface_face(line).report["plane_query"]["planar"] == true,
          "collinear net is not spuriously rejected by three-point fitting");
    auto square = net();
    for (unsigned k = 0; k < 4; ++k)
        square.geometry["poles"][3 * k + 1] = original["poles"][3 * k + 1].get<double>() * 2 / 3;
    auto square_face = extract_swept_body_surface_face(square);
    check(square_face.report["plane_query"]["principal_axes"] ==
              Json({{0., 0., -1.}, {0., 1., 0.}, {1., 0., 0.}}),
          "equal principal moments retain native strict swap ordering and handedness");
    auto huge = net();
    huge.geometry["poles"][0] = 1e200;
    check(extract_swept_body_surface_face(huge).status == "not_extracted",
          "overflow is an unsupported numerical result, not proof of nonplanarity");
    auto empty = net();
    empty.boundary_points = {{}};
    check(extract_swept_body_surface_face(empty).status == "native_empty",
          "planar side with no resulting boundary is native null");
    auto zero = s;
    zero.geometry["weights"][0] = 0;
    auto rejected = extract_swept_body_surface_face(zero);
    check(rejected.status == "not_extracted" && rejected.region.is_null() && !rejected.surface,
          "unrepresentable zero-weight principal extents clear derived output");
    for (auto opt : {SweptBodyBoundaryOptions{3, 100000, 100, true},
                     SweptBodyBoundaryOptions{100, 1, 100, true},
                     SweptBodyBoundaryOptions{100, 100000, 2, true}}) {
        auto limited = extract_swept_body_surface_face(net(), opt);
        check(limited.status == "not_extracted" && limited.region.is_null() && !limited.surface,
              "control work and curve budgets never publish partial faces");
    }
    SweptBodyBoundaryOptions no_outer;
    no_outer.include_outer = false;
    check(extract_swept_body_surface_face(net(), no_outer).region == f.region,
          "native face entry always requests outer edges");
    auto concurrent =
        std::async(std::launch::async, [s] { return extract_swept_body_surface_face(s); });
    check(concurrent.get().report == rational.report,
          "independent concurrent principal-axis state");
    return n;
}
