#include "native_tube_mesh_patch.hpp"
#include "native_curve_conversion.hpp"
#include <p3d/swept_patches.hpp>
#include <future>
#include <limits>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
TubeMeshBoundaryCurves graphs(std::vector<std::vector<Point2>> points) {
    TubeBudget b;
    return tube_mesh_boundary_curves(points, b);
}
TubeMeshPatchPreparation prepare(const BsplineSurface &s) {
    TubeBudget b;
    return prepare_tube_mesh_patch(s, {}, b);
}
Json surface(unsigned order, std::size_t nu, Json knots, bool rational = false) {
    Json xyz = Json::array(), weights = Json::array();
    for (std::size_t v = 0; v < 3; ++v)
        for (std::size_t u = 0; u < nu; ++u) {
            const double w = 1 + double(u + v) / 7;
            xyz.push_back(double(u) * (rational ? w : 1));
            xyz.push_back(double(v) * (rational ? w : 1));
            xyz.push_back(double((u * u + 3 * v) % 7) * (rational ? w : 1));
            weights.push_back(w);
        }
    return {{"_type", "BsplineSurface"},
            {"orderU", order},
            {"orderV", 3},
            {"closedU", false},
            {"closedV", false},
            {"numPolesU", nu},
            {"numPolesV", 3},
            {"knotsU", knots},
            {"knotsV", {-2, -2, -2, 5, 5, 5}},
            {"poles", xyz},
            {"weights", rational ? weights : Json()},
            {"numRulesU", 29},
            {"numRulesV", 71},
            {"holeOrigin", 0},
            {"boundaries", nullptr}};
}
bool near(const Point3 &a, const Point3 &b) {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < 1e-9;
}
Json group(Json values, unsigned type = 1) {
    Json members = Json::array();
    for (const auto &v : values)
        members.push_back({{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", members}};
}
Json line(Point3 a, Point3 b) {
    Json p;
    for (unsigned i = 0; i < 3; ++i) {
        p[std::string("point0") + "XYZ"[i]] = a[i];
        p[std::string("point1") + "XYZ"[i]] = b[i];
    }
    return {{"_type", "LineSegment"}, {"segment", p}};
}
} // namespace
unsigned native_tube_mesh_patch_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "native mesh preparation rejects malformed storage or resource exhaustion");
    };
    auto empty = graphs({});
    check(empty.success && !empty.lower && !empty.upper,
          "no runtime boundary keeps native defaults");
    check(graphs({{}, {{.5, .5}}}).success, "empty first record bypasses later records");
    auto rectangle = graphs({{{0, .1}, {1, .2}, {1, .9}, {0, .8}, {99, 99}}});
    check(rectangle.success && !rectangle.lower && !rectangle.upper,
          "four-knot graph is omitted even for sloped endpoints; final closure point is ignored");
    const std::vector<Point2> boundary{{0, .1},  {.25, .2}, {.7, .3}, {1, .4}, {1, .9},
                                       {.6, .8}, {.2, .7},  {0, .6},  {0, .1}};
    auto g = graphs({boundary, {{.5, .5}}});
    check(g.success && g.lower && g.upper && g.report["consumed_records"] == 1,
          "only the first saved boundary supplies both scalar graphs");
    check(g.lower->knots() == std::vector<double>{0, 0, .25, .7, 1, 1} &&
              g.upper->knots() == std::vector<double>{0, 0, .2, .6, 1, 1},
          "native upper graph reverses knots and controls together");
    check(near(g.lower->point_at(.125), Point3{.15, 0, 0}) &&
              near(g.upper->point_at(.4), Point3{.75, 0, 0}),
          "scalar graph interpolation is V as function of U");
    auto bad = boundary;
    bad[0][0] = .2;
    check(!graphs({bad}).success, "first U must be near zero");
    bad = boundary;
    bad[2][0] = .25;
    check(!graphs({bad}).success, "lower interior U values are strictly increasing");
    bad = boundary;
    bad[4][0] = .9;
    check(!graphs({bad}).success, "upper turn must also lie at U one");
    bad = boundary;
    bad[6][0] = .6;
    check(!graphs({bad}).success, "upper interior U values are strictly decreasing");
    bad = boundary;
    bad[7][0] = .1;
    check(!graphs({bad}).success, "penultimate record must end at U zero");
    bad = boundary;
    bad[0][0] = 1e-12;
    bad[3][0] = 1 + 1e-12;
    bad[4][0] = 1 - 1e-12;
    bad[7][0] = -1e-12;
    check(graphs({bad}).success, "boundary side recognition follows native relative tolerance");
    rejects([&] { graphs({{{0, 0}, {1, 0}}}); });
    bad = boundary;
    bad[1][1] = std::numeric_limits<double>::infinity();
    rejects([&] { graphs({bad}); });
    for (bool rational : {false, true})
        for (unsigned order : {2u, 3u, 4u}) {
            const std::size_t nu = order + 2;
            Json knots = Json::array();
            for (unsigned i = 0; i < order; ++i)
                knots.push_back(2.);
            knots.push_back(3.2);
            knots.push_back(5.5);
            for (unsigned i = 0; i < order; ++i)
                knots.push_back(7.);
            auto table = surface(order, nu, knots, rational);
            const auto s = BsplineSurface::from_bgfb(table);
            auto r = prepare(s);
            check(r.success && r.strips.size() == 3,
                  "native U strip count follows active compressed intervals");
            const double ends[]{2, 3.2, 5.5, 7};
            for (std::size_t j = 0; j < r.strips.size(); ++j) {
                const auto &strip = r.strips[j];
                check(strip.u().pole_count() == order &&
                          strip.u().knot_domain() == std::array<double, 2>{0, 1} &&
                          strip.v().knots() == s.v().knots() && strip.num_rules_u() == 0 &&
                          strip.num_rules_v() == 71 && strip.rational() == rational &&
                          strip.boundaries().is_null(),
                      "strip output preserves V and rational storage while resetting U and "
                      "boundaries");
                for (double u : {0., .21, .63, 1.})
                    for (double v : {0., .4, 1.})
                        check(near(strip.point_at(u, v),
                                   s.point_at(((1 - u) * ends[j] + u * ends[j + 1] - 2) / 5, v)),
                              "strip geometry agrees with independent source basis evaluation");
            }
            check(s.poles() == BsplineSurface::from_bgfb(table).poles(),
                  "strip preparation leaves source unchanged");
        }
    auto q = surface(3, 4, {0, 0, 0, .5, 1, 1, 1});
    q["poles"] = {0, 0, 0, 1, 2, 0, 3, 2, 0, 4, 0, 0, 0, 0, 1, 1, 2, 1,
                  3, 2, 1, 4, 0, 1, 0, 0, 2, 1, 2, 2, 3, 2, 2, 4, 0, 2};
    const auto qs = BsplineSurface::from_bgfb(q);
    auto qr = prepare(qs);
    check(qr.strips[0].poles()[2] == Point3{2, 2, 0} && qr.strips[1].poles()[0] == Point3{2, 2, 0},
          "native knot insertion propagates exact shared join into next strip");
    auto multi = surface(3, 5, {0, 0, 0, .4, .4, 1, 1, 1}, true);
    auto mr = prepare(BsplineSurface::from_bgfb(multi));
    check(mr.strips.size() == 2 && mr.strips[0].poles()[2] == mr.strips[1].poles()[0],
          "degree-multiplicity join copies source boundary without interpolation");
    auto gap = surface(3, 6, {0, 0, 0, .4, .4, .4, 1, 1, 1});
    rejects([&] { prepare(BsplineSurface::from_bgfb(gap)); });
    auto unclamped = q;
    unclamped["knotsU"] = {-1, -.5, 0, .5, 1, 1.5, 2};
    auto ur = prepare(BsplineSurface::from_bgfb(unclamped));
    check(
        ur.success && ur.strips[0].poles()[0] == Point3{0, 0, 0} &&
            !near(ur.strips[0].point_at(0, 0), BsplineSurface::from_bgfb(unclamped).point_at(0, 0)),
        "open nonclamped U keeps native first controls instead of inventing endpoint clamping");
    auto zero_weights = q;
    zero_weights["weights"] = std::vector<double>(12, 0.);
    auto zr = prepare(BsplineSurface::from_bgfb(zero_weights));
    check(zr.success && zr.strips[0].rational() && zr.strips[0].weights()[2] == 0 &&
              zr.strips[0].poles()[2] == Point3{2, 2, 0},
          "strip construction preserves zero homogeneous weights without deweighting controls");
    auto unit_weights = q;
    unit_weights["weights"] = std::vector<double>(12, 1. + 5e-9);
    auto wr = prepare(BsplineSurface::from_bgfb(unit_weights));
    check(wr.success && wr.strips[0].rational() && wr.strips[0].weights()[0] == 1. + 5e-9,
          "surface strips retain rational storage rather than applying curve near-unit weight "
          "omission");
    const Json arc{{"_type", "EllipticArc"},
                   {"arc",
                    {{"centerX", 0},
                     {"centerY", 0},
                     {"centerZ", 0},
                     {"vector0X", 2},
                     {"vector0Y", 0},
                     {"vector0Z", 0},
                     {"vector90X", 0},
                     {"vector90Y", 2},
                     {"vector90Z", 0},
                     {"startRadians", 0},
                     {"sweepRadians", 2 * std::acos(-1.)}}}};
    const auto circle = curve_detail::ellipse_to_bspline(arc);
    auto periodic = surface(3, circle.poles().size(), circle.knots(), true);
    Json cp = Json::array(), cw = Json::array();
    for (unsigned v = 0; v < 3; ++v)
        for (std::size_t u = 0; u < circle.poles().size(); ++u) {
            cp.push_back(circle.poles()[u][0]);
            cp.push_back(circle.poles()[u][1]);
            cp.push_back(v * circle.weights()[u]);
            cw.push_back(circle.weights()[u]);
        }
    periodic["poles"] = cp;
    periodic["weights"] = cw;
    periodic["closedU"] = true;
    const auto ps = BsplineSurface::from_bgfb(periodic);
    auto pr = prepare(ps);
    check(pr.success && pr.strips.size() == 3 && pr.report["opening_rows"].size() == 3,
          "periodic U opens each actual row and uses first row knot plan");
    for (std::size_t i = 0; i < 3; ++i)
        for (double u : {0., .3, 1.})
            for (double v : {0., .4, 1.})
                check(near(pr.strips[i].point_at(u, v), ps.point_at((i + u) / 3., v)),
                      "periodic rational strip geometry agrees with original cylinder");
    // U opening must leave the independent periodic V representation intact.
    auto pv = periodic;
    pv["closedV"] = true;
    pv["knotsV"] = nullptr;
    const auto pvs = BsplineSurface::from_bgfb(pv);
    auto pvr = prepare(pvs);
    check(pvr.success && pvr.strips[0].v().closed() && pvr.strips[0].v().knots() == pvs.v().knots(),
          "U strip extraction does not open or normalize periodic V");
    const Json profile = group({{{"_type", "LineString"},
                                 {"points", {-1, -1, 0, 1, -1, 0, 1, 1, 0, -1, 1, 0, -1, -1, 0}}}},
                               2);
    auto patharc = arc;
    patharc["arc"]["centerX"] = -10;
    patharc["arc"]["vector0X"] = 10;
    patharc["arc"]["vector90Y"] = 0;
    patharc["arc"]["vector90Z"] = 10;
    patharc["arc"]["sweepRadians"] = std::acos(-1.);
    auto patches = reconstruct_bgfb_swept_body_patches(
        {{"_type", "P3DSweptBody"},
         {"profile", profile},
         {"path", group({patharc, line({-20, 0, 0}, {-25, 0, 0})})}});
    check(patches.status == "reconstructed",
          "actual curved-path fixture has native whole-section patches");
    for (const auto &g : patches.groups)
        for (const auto &p : g.patches) {
            TubeBudget b;
            auto actual = prepare_tube_mesh_patch(BsplineSurface::from_bgfb(p.geometry),
                                                  p.boundary_points, b);
            check(actual.success && actual.strips.size() == 4,
                  "actual trimmed patch passes boundary graphs and strip preparation");
        }
    rejects([&] {
        TubeBudget b;
        b.max_work = 0;
        prepare_tube_mesh_patch(qs, {}, b);
    });
    rejects([&] {
        TubeBudget b;
        b.max_control_points = 15;
        prepare_tube_mesh_patch(qs, {}, b);
    });
    auto future = std::async(std::launch::async, [&] { return prepare(qs).strips[1].poles(); });
    check(future.get() == qr.strips[1].poles(),
          "independent concurrent preparation has no shared mutable plan");
    return n;
}
