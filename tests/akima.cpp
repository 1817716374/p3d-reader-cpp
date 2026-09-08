#include "internal.hpp"
using namespace p3d;
namespace {
Json table(const std::vector<Point3> &points) {
    Json values = Json::array();
    for (const auto &p : points)
        for (double x : p)
            values.push_back(x);
    return {{"_type", "AkimaCurve"}, {"points", values}};
}
bool near(Point3 a, Point3 b, double tolerance = 1e-12) {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) <= tolerance;
}
} // namespace
unsigned akima_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto rejects = [&](const Json &v, const char *message) {
        bool failed = false;
        try {
            AkimaCurve::from_bgfb(v);
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, message);
    };
    const std::vector<Point3> bend{{-1, 0, 0}, {0, 0, 0}, {1, 0, 0},
                                   {1, 2, 0},  {1, 3, 0}, {1, 3, 1}};
    const auto input = table(bend);
    const auto curve = AkimaCurve::from_bgfb(input);
    const auto &b = curve.bspline();
    check(curve.source_points() == bend &&
              curve.retained_point_indices() == std::vector<std::size_t>({0, 1, 2, 3, 4, 5}),
          "Akima retains the original support and interpolation points");
    check(b.order() == 4 && !b.closed() && !b.rational() && b.poles().size() == 4 &&
              b.knots() == std::vector<double>({0, 0, 0, 0, 1, 1, 1, 1}),
          "six Akima points produce one open nonrational cubic span");
    check(near(b.poles()[1], {4. / 3, 1. / 3, 0}) && near(b.poles()[2], {1, 4. / 3, 0}),
          "native per-coordinate direction weights are not renormalized to a unit tangent");
    for (unsigned i = 0; i <= 100; ++i) {
        const double t = i / 100.;
        check(near(b.point_at(t), {1 + t * (1 - t) * (1 - t), t + 2 * t * t - t * t * t, 0}),
              "Akima bend agrees with independently expanded analytic cubic");
    }
    auto reverse = bend;
    std::reverse(reverse.begin(), reverse.end());
    const auto reversed = AkimaCurve::from_bgfb(table(reverse));
    for (unsigned i = 0; i <= 20; ++i)
        check(near(reversed.bspline().point_at(i / 20.), b.point_at(1 - i / 20.)),
              "reversing source points reverses the same Akima curve");
    const std::vector<Point3> straight{{-4, 0, 0}, {-1, 0, 0}, {0, 0, 0}, {2, 0, 0},
                                       {5, 0, 0},  {9, 0, 0},  {10, 0, 0}};
    const auto multi = AkimaCurve::from_bgfb(table(straight));
    check(multi.bspline().poles().size() == 7 &&
              multi.bspline().knots() == std::vector<double>({0, 0, 0, 0, .4, .4, .4, 1, 1, 1, 1}),
          "Akima knots use central chord lengths with triple interior multiplicity");
    for (unsigned i = 0; i <= 100; ++i)
        check(near(multi.bspline().point_at(i / 100.), {i / 20., 0, 0}),
              "nonuniform straight Akima data preserves chord parameterization");
    for (double x : {1e-6, std::numeric_limits<double>::max()}) {
        auto filtered = straight;
        filtered.insert(filtered.begin() + 3, {x, 0, 0});
        const auto c = AkimaCurve::from_bgfb(table(filtered));
        check(c.source_points() == filtered && c.bspline().poles() == multi.bspline().poles() &&
                  c.retained_point_indices() == std::vector<std::size_t>({0, 1, 2, 4, 5, 6, 7}) &&
                  c.report()["ignored_points"][0]["source_index"] == 3,
              "native disconnect/proximity filtering has source provenance and leaves source data "
              "intact");
        check(c.report()["first_interpolated_source_index"] == 2 &&
                  c.report()["last_interpolated_source_index"] == 5,
              "interpolation endpoint indices refer to the original unfiltered point array");
    }
    auto distant_supports = straight;
    for (double offset : {std::nextafter(1e-5, 0.), 1e-5}) {
        std::vector<Point3> boundary_points{{-2, 0, 0}, {-1, 0, 0}, {0, 0, 0}, {offset, 0, 0},
                                            {1, 0, 0},  {2, 0, 0},  {3, 0, 0}};
        const auto c = AkimaCurve::from_bgfb(table(boundary_points));
        check(c.retained_point_indices().size() == (offset == 1e-5 ? 7u : 6u),
              "native proximity predicate excludes strictly smaller distance and retains equality");
    }
    distant_supports[0][0] = -1e9;
    distant_supports[6][0] = 1e9;
    check(AkimaCurve::from_bgfb(table(distant_supports)).report()["central_range_size"] == 5,
          "outer support points do not enlarge native proximity threshold");
    auto small = straight;
    for (auto &p : small)
        for (auto &x : p)
            x *= 1e-10;
    check(AkimaCurve::from_bgfb(table(small)).report()["proximity_threshold_squared"] == 1e-28,
          "native proximity filter retains its absolute squared-distance floor");
    auto too_few = bend;
    too_few.pop_back();
    rejects(table(too_few), "Akima requires the full six-point support window");
    auto collapsed = bend;
    collapsed[3] = collapsed[2];
    rejects(table(collapsed), "native filtering must leave six points");
    rejects(table(std::vector<Point3>(6, Point3{0, 0, 0})),
            "all repeated Akima points cannot form a curve");
    auto malformed = input;
    malformed["points"].push_back(1);
    rejects(malformed, "partial Akima XYZ triplet is rejected without dropping a coordinate");
    malformed = input;
    malformed["points"][0] = std::numeric_limits<double>::infinity();
    rejects(malformed, "nonfinite Akima coordinate is rejected");
    malformed["points"][0] = std::numeric_limits<double>::quiet_NaN();
    rejects(malformed, "NaN Akima coordinate is rejected");
    malformed["points"][0] = 1e300;
    rejects(malformed, "finite source coordinate causing Akima chord overflow is rejected");
    rejects(table(std::vector<Point3>(5001, Point3{0, 0, 0})),
            "native Akima maximum input count is enforced");
    std::vector<Point3> large;
    for (unsigned i = 0; i < 5000; ++i)
        large.push_back({double(i), 0, 0});
    const auto maximum = AkimaCurve::from_bgfb(table(large));
    check(maximum.bspline().poles().size() == 3 * (5000 - 5) + 1 &&
              near(maximum.bspline().point_at(0), {2, 0, 0}) &&
              near(maximum.bspline().point_at(1), {4997, 0, 0}),
          "maximum supported Akima input preserves the native interpolation endpoints");
    const auto loop = table({{.8, .8, 0},
                             {.2, .8, 0},
                             {.2, .2, 0},
                             {.8, .2, 0},
                             {.8, .8, 0},
                             {.2, .8, 0},
                             {.2, .2, 0},
                             {.8, .2, 0},
                             {.8, .8, 0}});
    Json surface = {
        {"_type", "BsplineSurface"},
        {"numPolesU", 2},
        {"numPolesV", 2},
        {"orderU", 2},
        {"orderV", 2},
        {"closedU", false},
        {"closedV", false},
        {"numRulesU", 0},
        {"numRulesV", 0},
        {"holeOrigin", 1},
        {"weights", nullptr},
        {"knotsU", nullptr},
        {"knotsV", nullptr},
        {"poles", {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0}},
        {"boundaries",
         {{"_type", "CurveVector"}, {"type", 1}, {"curves", Json::array({{{"geometry", loop}}})}}}};
    const auto trim = BsplineSurface::from_bgfb(surface).trim(1e-5);
    check(AkimaCurve::from_bgfb(loop).retained_point_indices().size() == 9,
          "nonadjacent repeated Akima points and support reuse are not globally deduplicated");
    check(trim.report()["status"] == "complete" &&
              trim.report()["loops"][0]["effective_boundary_type"] == 2 &&
              trim.classify({.5, .5}) == TrimLocation::Inside &&
              trim.classify({.01, .01}) == TrimLocation::Outside,
          "Akima source endpoints and derived spans support physically closed Open trim paths");
    return checks;
}
