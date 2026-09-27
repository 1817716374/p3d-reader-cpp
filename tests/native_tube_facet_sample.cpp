#include "native_tube_facet_sample.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineSurface strip(const std::vector<double> &y, double z = 0, bool weighted = false) {
    Json poles = Json::array(), weights = Json::array();
    for (double v : y)
        for (unsigned i = 0; i < 2; ++i) {
            const double w = weighted ? 2 + i : 1;
            poles.push_back(i * w);
            poles.push_back(v * w);
            poles.push_back(z * w);
            if (weighted)
                weights.push_back(w);
        }
    return BsplineSurface::from_bgfb({{"_type", "BsplineSurface"},
                                      {"numPolesU", 2},
                                      {"numPolesV", y.size()},
                                      {"orderU", 2},
                                      {"orderV", 2},
                                      {"closedU", false},
                                      {"closedV", false},
                                      {"knotsU", nullptr},
                                      {"knotsV", nullptr},
                                      {"poles", poles},
                                      {"weights", weighted ? weights : Json()},
                                      {"boundaries", nullptr},
                                      {"numRulesU", 0},
                                      {"numRulesV", 0},
                                      {"holeOrigin", 0}});
}
bool near(double a, double b) {
    return std::abs(a - b) < 2e-13;
}
BsplineCurve polyline(const std::vector<Point3> &points) {
    Json poles = Json::array();
    for (const auto &p : points)
        for (double x : p)
            poles.push_back(x);
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", 2},
                                    {"closed", false},
                                    {"poles", poles},
                                    {"knots", nullptr},
                                    {"weights", nullptr}});
}
BsplineSurface profile_strip(const std::vector<Point3> &profile) {
    Json poles = Json::array();
    for (const auto &p : profile)
        for (unsigned u = 0; u < 2; ++u) {
            poles.push_back(p[0] + u);
            poles.push_back(p[1]);
            poles.push_back(p[2]);
        }
    return BsplineSurface::from_bgfb({{"_type", "BsplineSurface"},
                                      {"numPolesU", 2},
                                      {"numPolesV", profile.size()},
                                      {"orderU", 2},
                                      {"orderV", 2},
                                      {"closedU", false},
                                      {"closedV", false},
                                      {"knotsU", nullptr},
                                      {"knotsV", nullptr},
                                      {"poles", poles},
                                      {"weights", nullptr},
                                      {"boundaries", nullptr},
                                      {"numRulesU", 0},
                                      {"numRulesV", 0},
                                      {"holeOrigin", 0}});
}
} // namespace
unsigned native_tube_facet_sample_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool failed = false;
        try {
            fn();
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, "facet sample rejects invalid arithmetic or exhausted resources");
    };
    const std::array<Point3, 2> plane{{{0, 0, 0}, {0, 1, 0}}};
    auto sample = [&](const BsplineSurface &a, const BsplineSurface &b, bool same = true) {
        TubeBudget budget;
        return sample_tube_facet_plane(a, b, .25, plane, same, budget);
    };
    const auto zigzag = strip({-1, 1, -1, 1});
    const auto hit = sample(zigzag, zigzag);
    check(hit.parameters && near((*hit.parameters)[0], 5. / 6) &&
              near((*hit.parameters)[1], 1. / 6),
          "three-crossing isocurves select last first, not a nearest or matched parameter pair");
    check(hit.report["first_intersections"] == 3 && hit.report["second_intersections"] == 3 &&
              hit.report["selected_points_equal"] == true && hit.report["status"] == "resolved",
          "plane-sample status retains actual query counts and selected-point comparison");
    check(hit.first.curve.poles().size() == 4 && hit.first.curve.poles()[0] == Point3{.25, -1, 0},
          "query uses constant U curves with V controls, not constant V curves");
    const auto different = strip({-1, 1, -1, 1}, 3);
    const auto pending = sample(zigzag, different);
    check(!pending.parameters && pending.report["status"] == "fallback_required" &&
              pending.report["reason"] == "plane_intersection_points_differ",
          "unequal selected 3D points require native pair fallback, not a definitive failure");
    check(pending.second.curve.poles()[0] == Point3{.25, -1, 3},
          "fallback retains the exact already-constructed second curve");
    const auto independent = sample(zigzag, different, false);
    check(independent.parameters && near((*independent.parameters)[0], 5. / 6) &&
              near((*independent.parameters)[1], 1. / 6) &&
              independent.report["selected_points_equal"].is_null(),
          "disabled same-point test retains distinct intersections without checking coincidence");
    const auto missing = sample(strip({1, 2}), zigzag);
    check(!missing.parameters && missing.report["first_intersections"] == 0 &&
              missing.report["second_intersections"] == 3,
          "both queries execute even when first has no hits");
    const auto endpoints = sample(strip({-1, 0}), strip({0, 1}));
    check(endpoints.parameters == std::optional<Point2>{{1, 0}},
          "native plane intersections include the joined endpoints");
    const auto repeated = sample(strip({-1, 0, 1, 0, -1}), strip({-1, 0, 1, 0, -1}));
    check(repeated.parameters && near((*repeated.parameters)[0], .75) &&
              near((*repeated.parameters)[1], .25) && repeated.report["first_intersections"] == 4,
          "shared Bezier span endpoints remain duplicated in native query order");
    const auto rational = strip({-1, 1, -1, 1}, 0, true);
    const auto rh = sample(rational, rational);
    check(rh.parameters && near((*rh.parameters)[0], 5. / 6) && near((*rh.parameters)[1], 1. / 6) &&
              rh.first.curve.rational() && rh.first.curve.weights() == std::vector<double>(4, 2.25),
          "plane query consumes homogeneous constant U output with the evaluated row weights");
    TubeBudget no_points;
    no_points.max_control_points = 7;
    rejects([&] { sample_tube_facet_plane(zigzag, zigzag, .25, plane, true, no_points); });
    TubeBudget no_work;
    no_work.max_work = 0;
    rejects([&] { sample_tube_facet_plane(zigzag, zigzag, .25, plane, true, no_work); });
    TubeBudget invalid;
    rejects([&] {
        sample_tube_facet_plane(zigzag, zigzag, std::numeric_limits<double>::quiet_NaN(), plane,
                                true, invalid);
    });
    auto select = [](const std::vector<double> &a, const std::vector<double> &b) {
        TubeBudget budget;
        return select_tube_facet_fallback_parameters(a, b, budget);
    };
    check(select({.1, .9, .4}, {.3, .8, .2}) == std::optional<Point2>{{.9, .2}},
          "native fallback chooses independent extrema, not the same pair index");
    check(select({.1, .9}, {.3}) == std::optional<Point2>{{.9, .3}},
          "native fallback selection does not require equal array sizes");
    check(select({-2, -3}, {3, 4}) == std::optional<Point2>{{-1, 2}},
          "nonempty fallback arrays preserve native sentinel clipping");
    check(select({1 + 5e-11}, {-5e-11}) == std::optional<Point2>{{1, 0}},
          "native fallback snaps only first-to-one and second-to-zero");
    const auto exact = select({1 - 2e-10}, {1e-10});
    check(exact && (*exact)[0] == 1 - 2e-10 && (*exact)[1] == 1e-10,
          "native snapping uses strict tolerance and leaves other values unchanged");
    check(select({5e-11}, {1 + 5e-11}) == std::optional<Point2>{{5e-11, 1 + 5e-11}},
          "the opposite endpoints are never snapped");
    check(!select({}, {.1}) && !select({.2}, {}), "empty fallback arrays have no selected result");
    TubeBudget fallback_limit;
    fallback_limit.max_work = 3;
    rejects([&] { select_tube_facet_fallback_parameters({.1, .2}, {.3, .4}, fallback_limit); });
    rejects([&] { select({std::numeric_limits<double>::infinity()}, {.1}); });
    const auto original = zigzag.poles();
    auto future = std::async(std::launch::async, [&] { return sample(zigzag, zigzag); });
    const auto now = sample(zigzag, zigzag), concurrent = future.get();
    check(now.parameters == concurrent.parameters && zigzag.poles() == original,
          "sampling shares immutable surfaces across concurrent independent requests");
    auto pair = [](const BsplineCurve &a, const BsplineCurve &b) {
        TubeBudget budget;
        return sample_tube_facet_pair(a, b, budget);
    };
    const auto yz_a = polyline({{0, -1, 0}, {0, 1, 0}});
    const auto yz_b = polyline({{0, 0, -1}, {0, 0, 1}});
    const auto yz = pair(yz_a, yz_b);
    check(yz.parameters && near((*yz.parameters)[0], .5) && near((*yz.parameters)[1], .5),
          "pair intersection transforms the joint YZ frame before the XY query");
    check(yz.report["planarity"]["planar"] == true &&
              yz.report["transform"]["clone_each_occurrence"] == true &&
              yz.report["intersections"]["count"] == 1,
          "pair diagnostics retain joint planarity, independent clones and actual hit count");
    const auto rational_a = BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                                     {"order", 2},
                                                     {"closed", false},
                                                     {"poles", {0, -1, 0, 0, 2, 0}},
                                                     {"knots", nullptr},
                                                     {"weights", {1, 2}}});
    const auto rational_b = BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                                     {"order", 2},
                                                     {"closed", false},
                                                     {"poles", {0, 0, -2, 0, 0, 1}},
                                                     {"knots", nullptr},
                                                     {"weights", {2, 1}}});
    const auto rational_pair = pair(rational_a, rational_b);
    check(rational_pair.parameters && near((*rational_pair.parameters)[0], 1. / 3) &&
              near((*rational_pair.parameters)[1], 2. / 3),
          "frame and pair wrappers preserve unequal rational weights and native curve parameters");
    const auto translated =
        pair(polyline({{5, 2, 7}, {5, 4, 7}}), polyline({{5, 3, 5}, {5, 3, 9}}));
    check(translated.parameters && near((*translated.parameters)[0], .5) &&
              near((*translated.parameters)[1], .5),
          "joint frame translation is applied to both original curve copies");
    const auto pair_endpoints =
        pair(polyline({{0, -1, 0}, {0, 0, 0}}), polyline({{0, 0, 0}, {0, 0, 1}}));
    check(pair_endpoints.parameters == std::optional<Point2>{{1, 0}},
          "full pair fallback retains joined endpoints and native endpoint snapping");
    const auto skew = pair(yz_a, polyline({{-1, 0, 1}, {1, 0, 1}}));
    check(!skew.parameters && skew.report["status"] == "native_failure" &&
              skew.report["reason"] == "pair_not_planar_or_frame_failed" &&
              skew.report["transform"].is_null() && skew.report["intersections"].is_null(),
          "skew lines fail joint planarity before any projected intersection");
    const auto separated = pair(yz_a, polyline({{0, 2, -1}, {0, 2, 1}}));
    check(!separated.parameters && separated.report["planarity"]["planar"] == true &&
              separated.report["reason"] == "no_pair_intersections",
          "coplanar finite curves without intersection remain an explicit native failure");
    const auto multiple = pair(polyline({{0, -1, 0}, {1, 1, 0}, {2, -1, 0}, {3, 1, 0}}),
                               polyline({{0, 0, 0}, {3, 0, 0}}));
    check(multiple.parameters && near((*multiple.parameters)[0], 5. / 6) &&
              near((*multiple.parameters)[1], 1. / 6) &&
              multiple.report["intersections"]["count"] == 3,
          "pair fallback independently selects extrema even at different spatial intersections");
    const auto self_curve = polyline({{0, 0, 0}, {1, 1, 0}, {0, 1, 0}, {1, 0, 0}});
    const auto self_poles = self_curve.poles();
    const auto self = pair(self_curve, self_curve);
    check(self.parameters && self_curve.poles() == self_poles &&
              self.report["transform"]["clone_each_occurrence"] == true,
          "shared input identity keeps both independent native copies and leaves source unchanged");
    const auto first_surface = profile_strip({{0, -1, 0}, {0, 1, 0}});
    const auto second_surface = profile_strip({{-1, -.5, 0}, {1, .5, 0}});
    const std::array<Point3, 2> offset_plane{{{0, .4, 0}, {0, 1, 0}}};
    auto seam = [&](bool same, const std::array<Point3, 2> &p) {
        TubeBudget budget;
        return sample_tube_facet_seam(first_surface, second_surface, .25, p, same, budget);
    };
    const auto full = seam(true, offset_plane);
    check(full.parameters && near((*full.parameters)[0], .5) && near((*full.parameters)[1], .5),
          "full seam replaces mismatched plane hits with transformed pair intersection");
    check(full.report["method"] == "curve_pair_fallback" &&
              full.report["plane_query"]["reason"] == "plane_intersection_points_differ" &&
              full.report["pair_query"]["status"] == "resolved" &&
              full.report["native_result"] == true,
          "full seam exposes primary rejection and resolved fallback separately");
    const auto unchecked = seam(false, offset_plane);
    check(unchecked.parameters && near((*unchecked.parameters)[0], .7) &&
              near((*unchecked.parameters)[1], .9) && unchecked.report["method"] == "plane" &&
              unchecked.report["pair_query"].is_null(),
          "disabled coincidence check retains distinct plane hits without a pair query");
    const auto absent_plane = seam(true, {{{0, 3, 0}, {0, 1, 0}}});
    check(absent_plane.parameters && near((*absent_plane.parameters)[0], .5) &&
              near((*absent_plane.parameters)[1], .5) &&
              absent_plane.report["plane_query"]["first_intersections"] == 0 &&
              absent_plane.report["plane_query"]["second_intersections"] == 0,
          "missing both plane hits still allows the original isocurves to intersect");
    TubeBudget full_failure_budget;
    const auto full_failure =
        sample_tube_facet_seam(first_surface, profile_strip({{-1, 0, 1}, {1, 0, 1}}), .25,
                               offset_plane, true, full_failure_budget);
    check(!full_failure.parameters && full_failure.report["native_result"] == false &&
              full_failure.report["status"] == "native_failure",
          "both failed routes yield no fabricated seam parameters");
    TubeBudget pair_points;
    pair_points.max_control_points = 3;
    rejects([&] { sample_tube_facet_pair(yz_a, yz_b, pair_points); });
    TubeBudget pair_work;
    pair_work.max_work = 0;
    rejects([&] { sample_tube_facet_pair(yz_a, yz_b, pair_work); });
    // Give the full query exactly the primary route's work allowance. The
    // required fallback must report resource exhaustion, not native failure.
    TubeBudget primary_budget;
    sample_tube_facet_plane(first_surface, second_surface, .25, offset_plane, true, primary_budget);
    TubeBudget seam_work;
    seam_work.max_work = primary_budget.work;
    rejects([&] {
        sample_tube_facet_seam(first_surface, second_surface, .25, offset_plane, true, seam_work);
    });
    const auto curve_before = yz_a.poles();
    auto pair_future = std::async(std::launch::async, [&] { return pair(yz_a, yz_b); });
    const auto pair_now = pair(yz_a, yz_b), pair_concurrent = pair_future.get();
    check(pair_now.report == pair_concurrent.report && yz_a.poles() == curve_before,
          "joint frame and transformed pair queries are deterministic on shared immutable curves");
    return n;
}
