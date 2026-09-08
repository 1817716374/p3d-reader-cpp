#include "internal.hpp"
using namespace p3d;
namespace {
Json variant(Json geometry) {
    return {{"_type", "VariantGeometry"}, {"geometry", geometry}};
}
Json array(int type, Json curves) {
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", curves}};
}
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
Json rectangle(double x0, double y0, double x1, double y1, int type = 2, bool reverse = false) {
    Json points = reverse ? Json{x0, y0, 0, x0, y1, 0, x1, y1, 0, x1, y0, 0, x0, y0, 0}
                          : Json{x0, y0, 0, x1, y0, 0, x1, y1, 0, x0, y1, 0, x0, y0, 0};
    return array(type, Json::array({variant({{"_type", "LineString"}, {"points", points}})}));
}
BsplineSurface surface(Json boundary, int hole = 1) {
    return BsplineSurface::from_bgfb({{"_type", "BsplineSurface"},
                                      {"numPolesU", 2},
                                      {"numPolesV", 2},
                                      {"orderU", 2},
                                      {"orderV", 2},
                                      {"closedU", false},
                                      {"closedV", false},
                                      {"numRulesU", 0},
                                      {"numRulesV", 0},
                                      {"poles", {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0}},
                                      {"weights", nullptr},
                                      {"knotsU", nullptr},
                                      {"knotsV", nullptr},
                                      {"boundaries", boundary},
                                      {"holeOrigin", hole}});
}
Json spline(unsigned order, bool closed, Json poles, Json weights = nullptr, Json knots = nullptr) {
    return {{"_type", "BsplineCurve"}, {"order", order},     {"closed", closed},
            {"poles", poles},          {"weights", weights}, {"knots", knots}};
}
double distance(Point2 p, Point2 a, Point2 b) {
    const double x = b[0] - a[0], y = b[1] - a[1], l = x * x + y * y;
    const double t =
        l == 0 ? 0 : std::max(0., std::min(1., ((p[0] - a[0]) * x + (p[1] - a[1]) * y) / l));
    return std::hypot(p[0] - a[0] - t * x, p[1] - a[1] - t * y);
}
} // namespace
unsigned bspline_trim_tests() {
    unsigned n = 0;
    auto check = [&](bool v, const char *message) {
        ++n;
        require(v, message);
    };
    auto rejects = [&](auto f, const char *message) {
        bool failed = false;
        try {
            f();
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, message);
    };
    const auto inside = TrimLocation::Inside, outside = TrimLocation::Outside,
               on = TrimLocation::BoundaryBand, unknown = TrimLocation::Indeterminate;
    auto region = surface(rectangle(.2, .2, .8, .8)).trim(1e-6);
    check(region.report()["status"] == "complete" && region.loops().size() == 1 &&
              region.classify({.5, .5}) == inside && region.classify({.1, .1}) == outside,
          "single trim outer loop classifies inside and outside with default outer boundary "
          "disabled");
    check(region.classify({.2, .5}) == on && region.classify({.2 + 5e-7, .5}) == on &&
              region.classify({.2 + 2e-6, .5}) == inside,
          "trim boundary band has the caller's explicit UV tolerance");
    auto hole = surface(rectangle(.2, .2, .8, .8), 0).trim(1e-6);
    check(hole.classify({.5, .5}) == outside && hole.classify({.1, .1}) == inside,
          "active default outer boundary complements trim parity");
    for (int type : {4, 5}) {
        auto root = array(type, Json::array({variant(rectangle(.1, .1, .9, .9)),
                                             variant(rectangle(.4, .4, .6, .6, 3, true))}));
        auto r = surface(root).trim(1e-6);
        check(r.report()["status"] == "complete" && r.classify({.2, .2}) == inside &&
                  r.classify({.5, .5}) == outside && r.classify({.05, .05}) == outside,
              "native parity and union trim trees both flatten to boundary parity independent of "
              "orientation");
        check(r.report()["loops"][1]["source_path"] == "/curves/1/geometry" &&
                  r.report()["loops"][1]["source_boundary_type"] == 3,
              "trim loops retain native tree provenance and inner type");
    }
    auto overlapping = array(
        5, Json::array({variant(rectangle(.1, .1, .6, .6)), variant(rectangle(.4, .4, .9, .9))}));
    auto overlap = surface(overlapping).trim(1e-6);
    check(overlap.classify({.5, .5}) == outside && overlap.classify({.2, .2}) == inside,
          "overlapping union members use the actual surface trim parity instead of inferred set "
          "union");
    auto duplicated = array(
        4, Json::array({variant(rectangle(.2, .2, .8, .8)), variant(rectangle(.2, .2, .8, .8))}));
    check(surface(duplicated).trim(1e-6).classify({.5, .5}) == outside,
          "duplicate native loops are preserved and cancel by parity rather than deduplicated");
    auto closed_open = surface(rectangle(.2, .2, .8, .8, 1)).trim(1e-6);
    check(closed_open.report()["loops"][0]["source_boundary_type"] == 1 &&
              closed_open.report()["loops"][0]["effective_boundary_type"] == 2,
          "physically closed open path is promoted to outer boundary without changing source type");
    auto open = array(1, Json::array({variant(line({.2, .2, 0}, {.8, .2, 0}))}));
    auto ignored = surface(open).trim(1e-6);
    check(ignored.loops().empty() &&
              ignored.report()["ignored"][0]["reason"] == "open_boundary_not_closed" &&
              ignored.classify({.5, .5}) == inside,
          "native open boundary is ignored and zero effective loops use native untrimmed result");
    auto none = surface(rectangle(.2, .2, .8, .8, 0)).trim(1e-6);
    check(none.loops().empty() && none.report()["status"] == "complete",
          "None boundary type is not inferred from geometric closure");
    auto region_line = array(4, Json::array({variant(line({0, 0, 0}, {1, 1, 0}))}));
    auto nonarray = surface(region_line).trim(1e-6);
    check(nonarray.report()["ignored"][0]["reason"] == "region_member_is_not_curve_array",
          "region members lacking a child curve array follow native ignore rule");
    for (const auto &member :
         std::vector<Json>{rectangle(.3, .3, .7, .7),
                           {{"_type", "PointString"}, {"points", {.3, .3, 0, .7, .7, 0}}}}) {
        auto root = rectangle(.2, .2, .8, .8);
        root["curves"].push_back(variant(member));
        const auto r = surface(root).trim(1e-6);
        check(r.report()["status"] == "complete" && r.loops().size() == 1 &&
                  r.classify({.5, .5}) == inside &&
                  r.report()["ignored"][0]["source_path"] == "/curves/1/geometry" &&
                  r.report()["ignored"][0]["reason"] == "native_trim_conversion_unavailable",
              "direct array and point-string members follow native failed conversion without "
              "becoming a hole or an inferred line");
        const auto ignored_only = surface(array(2, Json::array({variant(member)}))).trim(1e-6);
        check(ignored_only.report()["status"] == "complete" && ignored_only.loops().empty() &&
                  ignored_only.classify({.1, .1}) == inside,
              "a boundary containing only natively unconvertible members has no effective loop");
    }
    auto nested_open = rectangle(.2, .2, .8, .8, 1);
    nested_open["curves"].push_back(
        variant(array(1, Json::array({variant(line({.2, .2, 0}, {.1, .1, 0}))}))));
    const auto source_open = surface(nested_open).trim(1e-6);
    check(source_open.report()["status"] == "complete" && source_open.loops().empty() &&
              source_open.report()["ignored"][0]["reason"] == "open_boundary_not_closed",
          "source child-array endpoints are checked before ignoring its failed conversion");
    nested_open["curves"][1]["geometry"]["curves"][0]["geometry"] = line({.1, .1, 0}, {.2, .2, 0});
    const auto source_closed = surface(nested_open).trim(1e-6);
    check(source_closed.report()["status"] == "complete" && source_closed.loops().size() == 1 &&
              source_closed.report()["loops"][0]["effective_boundary_type"] == 2,
          "a child array can close the source Open path while its own geometry is not stroked");
    for (unsigned count : {0u, 1u, 2u}) {
        auto with_points = rectangle(.2, .2, .8, .8, 1);
        Json points = count == 0   ? Json::array()
                      : count == 1 ? Json{.9, .9, 0}
                                   : Json{.2, .2, 0, .9, .9, 0};
        with_points["curves"].push_back(variant({{"_type", "PointString"}, {"points", points}}));
        const auto r = surface(with_points).trim(1e-6);
        check(r.report()["status"] == "complete" && r.loops().size() == (count == 0 ? 1 : 0),
              "even a singleton PointString contributes endpoints to native Open closure");
    }
    open["type"] = 2;
    auto gap = surface(open).trim(1e-6);
    check(gap.report()["status"] == "incomplete" && gap.classify({.5, .5}) == unknown,
          "declared closed boundary with geometric gap is not silently closed or treated as "
          "untrimmed");
    auto unsupported = array(2, Json::array({variant({{"_type", "InterpolationCurve"}})}));
    check(surface(unsupported).trim(1e-6).classify({.5, .5}) == unknown,
          "unsupported trim curves cannot produce a false inside result");
    check(surface(rectangle(.2, .2, .8, .8)).trim(1e-6, 2).classify({.5, .5}) == unknown,
          "trim segment budget failure stays explicit");
    check(surface(rectangle(.2, .2, .8, .8)).trim(1e-20).classify({.5, .5}) == unknown,
          "unresolvable coordinate precision stays explicit");
    rejects([&] { surface(nullptr).trim(0); }, "zero trim tolerance rejected");
    rejects([&] { surface(nullptr).trim(1e-6, 0); }, "zero trim budget rejected");
    rejects([&] { region.classify({-1, .5}); }, "trim query outside normalized UV square rejected");
    rejects([&] { region.classify({.5, std::numeric_limits<double>::quiet_NaN()}); },
            "NaN trim query rejected");
    auto arc = [](double sweep) {
        return Json{{"_type", "EllipticArc"},
                    {"arc",
                     {{"centerX", .5},
                      {"centerY", .5},
                      {"centerZ", 0},
                      {"vector0X", .3},
                      {"vector0Y", 0},
                      {"vector0Z", 0},
                      {"vector90X", 0},
                      {"vector90Y", .2},
                      {"vector90Z", 0},
                      {"startRadians", 0},
                      {"sweepRadians", sweep}}}};
    };
    for (double sign : {1., -1.}) {
        auto ellipse =
            surface(array(2, Json::array({variant(arc(sign * 6.2831853071795864769))}))).trim(1e-5);
        check(ellipse.report()["status"] == "complete" && ellipse.classify({.5, .5}) == inside &&
                  ellipse.classify({.9, .5}) == outside && ellipse.classify({.8, .5}) == on,
              "rational ellipse trimming retains clockwise/counterclockwise source geometry");
        for (unsigned j = 0; j <= 10; ++j)
            for (unsigned i = 0; i <= 10; ++i) {
                const double x = i / 10., y = j / 10.,
                             r = (x - .5) * (x - .5) / .09 + (y - .5) * (y - .5) / .04;
                if (std::abs(r - 1) < 1e-3)
                    continue;
                check(ellipse.classify({x, y}) == (r < 1 ? inside : outside),
                      "elliptical trim parity agrees with analytic region away from boundary band");
            }
    }
    const double w = std::sqrt(.5), s = std::sqrt(3.) / 2;
    auto quarter = spline(3, false, {.8, .5, 0, .8 * w, .8 * w, 0, .5, .8, 0}, {1, w, 1});
    auto sector = array(2, Json::array({variant(quarter), variant(line({.5, .8, 0}, {.5, .5, 0})),
                                        variant(line({.5, .5, 0}, {.8, .5, 0}))}));
    auto q = surface(sector).trim(1e-5);
    check(q.report()["status"] == "complete" && q.classify({.6, .6}) == inside &&
              q.classify({.75, .75}) == outside,
          "rational B-spline and line primitives form a source-ordered trim loop");
    auto negative_sector = sector;
    auto &negative = negative_sector["curves"][0]["geometry"];
    for (auto &v : negative["poles"])
        v = -v.get<double>();
    for (auto &v : negative["weights"])
        v = -v.get<double>();
    auto negative_trim = surface(negative_sector).trim(1e-5);
    check(negative_trim.report()["status"] == "complete" &&
              negative_trim.classify({.6, .6}) == inside &&
              negative_trim.classify({.75, .75}) == outside,
          "uniformly negative homogeneous scaling preserves rational trim geometry");
    for (double middle_weight : {0., -.2}) {
        auto finite_sector = sector;
        finite_sector["curves"][0]["geometry"]["weights"][1] = middle_weight;
        const auto finite_trim = surface(finite_sector).trim(1e-5);
        check(finite_trim.report()["status"] == "complete",
              "zero or mixed-sign interior controls can have a strictly positive denominator");
        const auto evaluator = BsplineCurve::from_bgfb(finite_sector["curves"][0]["geometry"]);
        const auto &loop = finite_trim.loops().at(0);
        const double bound = finite_trim.report()["loops"][0]["deviation_bound"].get<double>();
        for (unsigned i = 0; i <= 100; ++i) {
            const auto p = evaluator.point_at(i / 100.0);
            double d = std::numeric_limits<double>::infinity();
            for (std::size_t j = 1; j < loop.size(); ++j)
                d = std::min(d, distance({p[0], p[1]}, loop[j - 1], loop[j]));
            check(d <= bound + 1e-12, "mixed homogeneous weights preserve the evaluated curve "
                                      "within the reported derived-polyline error");
        }
    }
    for (double middle_weight : {-1., -2.}) {
        auto singular_sector = sector;
        singular_sector["curves"][0]["geometry"]["weights"][1] = middle_weight;
        const auto singular_trim = surface(singular_sector).trim(1e-5, 2000);
        check(singular_trim.report()["status"] == "incomplete" &&
                  !singular_trim.report()["errors"].empty() &&
                  singular_trim.classify({.6, .6}) == unknown,
              "tangent or crossing interior denominator singularity stays indeterminate");
    }
    Json poles = Json::array(), weights = Json::array();
    for (const auto &h : std::vector<Point3>{{1, 0, 1},
                                             {.5, s, .5},
                                             {-.5, s, 1},
                                             {-1, 0, .5},
                                             {-.5, -s, 1},
                                             {.5, -s, .5},
                                             {1, 0, 1}}) {
        poles.push_back(.5 * h[2] + .3 * h[0]);
        poles.push_back(.5 * h[2] + .3 * h[1]);
        poles.push_back(0);
        weights.push_back(h[2]);
    }
    std::vector<Json> curves{
        spline(3, true, poles, weights,
               {-1. / 3, 0, 0, 0, 1. / 3, 1. / 3, 2. / 3, 2. / 3, 1, 1, 1, 4. / 3}),
        spline(3, true, {.2, .2, 0, .8, .2, 0, .8, .8, 0, .2, .8, 0}),
        spline(3, true, {.2, .2, 0, .8, .2, 0, .8, .8, 0, .2, .8, 0}, nullptr,
               {-.7, -.4, 0, .1, .3, .6, 1, 1.1, 1.3}),
        spline(3, false, {.2, .2, 0, .8, .2, 0, .8, .8, 0, .2, .8, 0, .2, .2, 0}),
        spline(5, false,
               {.2, .2, 0, .6, .1, 0, .9, .4, 0, .8, .9, 0, .3, .9, 0, .1, .6, 0, .2, .2, 0})};
    for (const auto &curve : curves) {
        auto compiled = surface(array(2, Json::array({variant(curve)}))).trim(1e-4);
        check(compiled.report()["status"] == "complete",
              "general-order and periodic trim B-spline spans form a continuous closed loop");
        const auto evaluator = BsplineCurve::from_bgfb(curve);
        const auto &loop = compiled.loops().at(0);
        const double bound = compiled.report()["loops"][0]["deviation_bound"].get<double>();
        for (unsigned i = 0; i <= 100; ++i) {
            const auto p = evaluator.point_at(i / 100.0);
            double d = std::numeric_limits<double>::infinity();
            for (std::size_t j = 1; j < loop.size(); ++j)
                d = std::min(d, distance({p[0], p[1]}, loop[j - 1], loop[j]));
            check(d <= bound + 1e-12, "local Bezier extraction and stroke stay within control-hull "
                                      "bound of independent B-spline evaluator");
        }
        check(compiled.classify({.5, .5}) == inside && compiled.classify({.05, .05}) == outside,
              "periodic and nonuniform B-spline trim regions preserve source loop orientation and "
              "closure");
    }
    return n;
}
