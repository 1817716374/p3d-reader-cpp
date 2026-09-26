#include "native_tube.hpp"
#include "native_curve_affine.hpp"
#include "native_pcurve_points.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json line(Point3 a, Point3 b) {
    return {{"_type", "LineSegment"},
            {"segment",
             {{"point0X", a[0]},
              {"point0Y", a[1]},
              {"point0Z", a[2]},
              {"point1X", b[0]},
              {"point1Y", b[1]},
              {"point1Z", b[2]}}}};
}
Json group(unsigned type, std::initializer_list<Json> geometries) {
    Json entries = Json::array();
    for (const auto &g : geometries)
        entries.push_back({{"geometry", g}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", entries}};
}
Json spline(unsigned order, Json poles, Json weights = nullptr, bool closed = false,
            Json knots = nullptr) {
    return {{"_type", "BsplineCurve"}, {"order", order},     {"closed", closed},
            {"poles", poles},          {"weights", weights}, {"knots", knots}};
}
bool near(const Point3 &a, const Point3 &b) {
    for (unsigned k = 0; k < 3; ++k)
        if (std::abs(a[k] - b[k]) > 2e-11 * std::max({1., std::abs(a[k]), std::abs(b[k])}))
            return false;
    return true;
}
} // namespace
unsigned native_tube_placement_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn, const char *why) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    const auto profile = group(1, {line({11, 22, 33}, {14, 25, 36})});
    for (Point3 delta : {Point3{0, 0, 8}, Point3{8, 0, 0}, Point3{2, 3, 4}, Point3{0, 0, -8}}) {
        const Point3 end{10 + delta[0], 20 + delta[1], 30 + delta[2]};
        const auto path = group(1, {line({10, 20, 30}, end)});
        TubeBudget b;
        const auto output = prepare_swept_tube_surfaces(profile, path, b);
        check(output.surfaces.size() == 1 && output.report["orientation_applied"] == false &&
                  output.report["native_face_indices"].is_null() &&
                  output.report["placement"]["inverse_succeeded"] == true,
              "source profile and path produce one surface without claiming face orientation");
        const auto s = BsplineSurface::from_bgfb(output.surfaces.front());
        for (double u : {0., .2, .65, 1.})
            for (double v : {0., .3, .8, 1.})
                check(
                    near(s.point_at(u, v), {11 + 3 * u + delta[0] * v, 22 + 3 * u + delta[1] * v,
                                            33 + 3 * u + delta[2] * v}),
                    "world profile placement followed by straight sweep preserves oblique local Z");
    }
    const auto unclamped =
        BsplineCurve::from_bgfb(spline(3, {10, 20, 30, 11, 20, 31, 13, 22, 32, 14, 23, 34}, nullptr,
                                       false, {-2, -1, 0, 1, 2, 3, 4}));
    TubeBudget ub;
    const auto placed = place_tube_profile(profile, unclamped, ub);
    check(placed.report["origin_rule"] == "first_stored_control" &&
              placed.report["frame"][0][3] == 10 && placed.report["frame"][1][3] == 20 &&
              placed.report["frame"][2][3] == 30 && !near(unclamped.point_at(0), {10, 20, 30}),
          "open nonclamped placement uses stored first control instead of evaluated curve start");
    check(placed.trace.source_knots() == unclamped.source_knots(),
          "placement keeps source knot domain and exterior knots");
    const auto periodic =
        BsplineCurve::from_bgfb(spline(3, {2, 0, 0, 0, 2, 0, -2, 0, 0, 0, -2, 0}, nullptr, true));
    TubeBudget pb;
    const auto closed = place_tube_profile(profile, periodic, pb);
    const auto origin = detail::pcurve_point(periodic, 0).point;
    check(closed.report["origin_rule"] == "closed_curve_point" &&
              closed.report["frame"][0][3] == origin[0] &&
              closed.report["frame"][1][3] == origin[1] && !near(origin, periodic.poles().front()),
          "closed source origin uses native curve point instead of stored first control");
    const auto vertical = BsplineCurve::from_bgfb(spline(2, {10, 20, 30, 10, 20, 38}));
    TubeBudget wb;
    const auto weighted =
        place_tube_profile(group(1, {spline(2, {3, 4, 5, 6, 7, 8}, {0, -2})}), vertical, wb);
    check(
        weighted.sections[0].poles()[0] == Point3{3, 4, 5} &&
            weighted.sections[0].poles()[1] == Point3{26, 47, 68} &&
            weighted.sections[0].weights() == std::vector<double>{0, -2},
        "profile placement transforms weighted XYZ without division, preserving zero and signed W");
    for (double shift : {0., std::nextafter(1e-10, 0.), 1e-10, -1e-10}) {
        TubeBudget b;
        const auto p = place_tube_profile(
            profile, BsplineCurve::from_bgfb(spline(2, {shift, 0, 0, shift, 0, 8})), b);
        check(p.report["identity_transform_skipped"] == (std::abs(shift) < 1e-10),
              "native near-identity translation predicate has strict endpoints");
        check(p.sections[0].poles()[0][0] == (std::abs(shift) < 1e-10 ? 11. : 11. - shift),
              "identity skip retains exact source weighted control bits");
    }
    TubeBudget degenerate_budget;
    const auto singular = place_tube_profile(
        profile, BsplineCurve::from_bgfb(spline(2, {2, 3, 4, 2, 3, 4})), degenerate_budget);
    check(singular.report["inverse_succeeded"] == false &&
              singular.report["identity_transform_skipped"] == true &&
              singular.sections[0].poles().front() == Point3{11, 22, 33},
          "singular source frame uses full identity fallback including zero translation");
    const auto polygon = group(
        2,
        {Json{{"_type", "LineString"}, {"points", {0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0, 0}}}});
    const auto parity = group(4, {polygon, polygon});
    const auto weighted_path =
        group(1, {spline(2, {8.777397980257993, 0, 0, 8.777397980257993, 0, 15.78662822965214},
                         {15.78662822965214, 15.78662822965214})});
    const auto saved = weighted_path.dump();
    TubeBudget mb;
    const auto multi = prepare_swept_tube_surfaces(parity, weighted_path, mb);
    check(multi.surfaces.size() == 2 && multi.report["rings"][0]["source_ring_index"] == 0 &&
              multi.report["rings"][1]["source_ring_index"] == 1,
          "parity rings retain original order and duplicates without geometric deduplication");
    check(multi.trace.poles()[0][0] == 8.777397980258003 && weighted_path.dump() == saved,
          "placement plus two rings carry six native weight round trips without editing input");
    check(multi.report["rings"][0]["tube"]["source_frame"]["frame"] !=
              multi.report["rings"][1]["tube"]["source_frame"]["frame"],
          "later ring frame sees cumulative native trace state");
    TubeBudget cb;
    const auto circle_path =
        group(1, {spline(3, {2, 0, 0, 0, 2, 0, -2, 0, 0, 0, -2, 0}, nullptr, true)});
    const auto closed_surface = prepare_swept_tube_surfaces(profile, circle_path, cb);
    check(!closed_surface.report["rings"][0]["v_closure"].is_null() &&
              closed_surface.trace.closed(),
          "closed trace requests post-generation V closure separately from patch stitching");
    auto future = std::async(std::launch::async, [&] {
        TubeBudget b;
        return prepare_swept_tube_surfaces(parity, weighted_path, b);
    });
    const auto concurrent = future.get();
    check(concurrent.surfaces == multi.surfaces && concurrent.trace.poles() == multi.trace.poles(),
          "independent multi-ring preparations are deterministic and share no working state");
    rejects(
        [&] {
            TubeBudget b;
            b.max_work = mb.work - 1;
            prepare_swept_tube_surfaces(parity, weighted_path, b);
        },
        "conversion placement per-ring surface and closure share the work budget");
    rejects(
        [&] {
            TubeBudget b;
            b.max_control_points = multi.report["total_control_points"].get<std::size_t>() - 1;
            prepare_swept_tube_surfaces(parity, weighted_path, b);
        },
        "total generated controls are bounded across duplicate rings");
    rejects(
        [&] {
            TubeBudget b;
            place_tube_profile(nullptr, vertical, b);
        },
        "null profile is not replaced with an empty successful surface list");
    rejects(
        [&] {
            TubeBudget b;
            prepare_swept_tube_surfaces(profile, group(4, {polygon}), b);
        },
        "path conversion does not accept a parity profile as a trace");
    return n;
}
