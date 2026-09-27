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
    return n;
}
