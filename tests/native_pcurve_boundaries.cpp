#include "internal.hpp"
#include "loft_curve.hpp"
#include "p3d/pcurve.hpp"
using namespace p3d;
namespace {
Json plane(Json boundary = nullptr) {
    return {{"_type", "BsplineSurface"},
            {"numPolesU", 2},
            {"numPolesV", 2},
            {"orderU", 2},
            {"orderV", 2},
            {"closedU", false},
            {"closedV", false},
            {"poles", {0, 0, 0, 3, 0, 0, 0, 4, 0, 3, 4, 0}},
            {"weights", nullptr},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"numRulesU", 0},
            {"numRulesV", 0},
            {"boundaries", boundary},
            {"holeOrigin", 0}};
}
Json array(unsigned type, std::vector<Json> curves) {
    Json entries = Json::array();
    for (auto &c : curves)
        entries.push_back({{"geometry", c}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", entries}};
}
Json line(std::vector<double> p) {
    return {{"_type", "LineString"}, {"points", p}};
}
Json spline(unsigned order, std::vector<double> p, Json weights = nullptr, bool closed = false,
            Json knots = nullptr) {
    return {{"_type", "BsplineCurve"}, {"order", order}, {"closed", closed}, {"poles", p},
            {"weights", weights},      {"knots", knots}};
}
Json arc(double start, double sweep, double rx = .2, double ry = .1) {
    return {{"_type", "EllipticArc"},
            {"arc",
             {{"centerX", .5},
              {"centerY", .5},
              {"centerZ", 0},
              {"vector0X", rx},
              {"vector0Y", 0},
              {"vector0Z", 0},
              {"vector90X", 0},
              {"vector90Y", ry},
              {"vector90Z", 0},
              {"startRadians", start},
              {"sweepRadians", sweep}}}};
}
PCurveLoopStrokes sample(Json boundary, const PCurveBoundaryStrokeOptions &options = {}) {
    return sample_native_surface_boundaries(BsplineSurface::from_bgfb(plane(boundary)), options);
}
} // namespace
unsigned native_pcurve_boundary_tests() {
    unsigned checks = 0;
    auto check = [&](bool condition, const char *message) {
        ++checks;
        require(condition, message);
    };
    const auto square = line({0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0});
    const auto boundary = array(2, {square});
    auto r = sample(boundary);
    check(r.report["status"] == "complete" && r.loops.size() == 1 && r.loops[0].size() == 17,
          "source boundary line string is sampled as one native member");
    check(r.report["boundary_sources"][0]["members"].size() == 1 &&
              r.report["converted_controls"] == 5,
          "line-string control count and member grouping are retained");
    for (unsigned i = 0; i < 5; ++i)
        check(r.loops[0][i].parameter == Point3{double(i) / 4, 0, 0} &&
                  r.loops[0][i].position == Point3{.75 * i, 0, 0},
              "source square first edge independent quarter samples");
    check(r.loops[0].front().parameter == r.loops[0].back().parameter,
          "explicit source closure is retained");
    r = sample(array(2, {line({0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0})}));
    check(r.report["status"] == "complete" && r.loops[0].size() == 13 &&
              r.report["converted_controls"] == 4,
          "native line-string conversion retains repeated controls and zero-length segments");
    check(r.loops[0].back().parameter == Point3{1, 1, 0},
          "source restroke does not synthesize a closing edge");
    const auto inner = array(3, {line({.2, .2, 0, .4, .2, 0, .4, .4, 0, .2, .2, 0})});
    r = sample(array(5, {boundary, array(4, {inner}), square, array(0, {square})}));
    check(r.report["status"] == "complete" && r.loops.size() == 2 &&
              r.report["ignored"].size() == 2,
          "region recurses through child arrays only and ignores type-none arrays");
    check(r.report["boundary_sources"][1]["source_path"] ==
                  "/curves/1/geometry/curves/0/geometry" &&
              r.report["boundary_sources"][1]["source_boundary_type"] == 3,
          "nested source loop provenance is preserved");
    r = sample(array(5, {boundary, boundary}));
    check(r.report["status"] == "complete" && r.loops.size() == 2 &&
              r.report["converted_controls"] == 10,
          "distinct identical source records are not deduplicated");
    r = sample(array(1, {square}));
    check(r.report["status"] == "complete" && r.loops.size() == 1 &&
              r.report["boundary_sources"][0]["effective_boundary_type"] == 2,
          "closed source Open array is admitted as Outer");
    r = sample(array(1, {line({0, 0, 0, 1, 0, 0})}));
    check(r.report["status"] == "complete" && r.loops.empty() &&
              r.report["ignored"][0]["reason"] == "open_boundary_not_closed",
          "nonclosed Open boundary follows native exclusion");
    const Json single = {{"_type", "PointString"}, {"points", {0, 0, 0}}};
    r = sample(array(1, {line({0, 0, 0, 1, 0, 0}), array(1, {single})}));
    check(r.report["status"] == "complete" && r.loops.size() == 1 && r.loops[0].size() == 5 &&
              r.loops[0].back().parameter == Point3{1, 0, 0},
          "Open closure reads ignored child-array endpoints before direct-member conversion");
    r = sample(array(2, {single, array(2, {square}), line({.5, .5, 0})}));
    check(r.report["status"] == "complete" && r.loops.empty() && r.report["ignored"].size() == 4,
          "unconvertible direct members are reported and an empty converted loop is omitted");

    const auto c = spline(3, {.1, .2, 0, .5, .8, 0, .9, .2, 0}, nullptr, false,
                          Json::array({2, 2, 2, 7, 7, 7}));
    r = sample(array(2, {c}));
    const auto source_curve = BsplineCurve::from_bgfb(c);
    PCurveStrokeOptions options;
    options.uv_tolerance = .01;
    options.spatial_tolerance = .0005;
    const auto direct =
        sample_native_pcurve(BsplineSurface::from_bgfb(plane()), source_curve, options);
    check(r.report["status"] == "complete" && r.loops[0].size() == direct.samples.size() &&
              r.report["boundary_sources"][0]["members"][0]["source_curve_knot_domain"] ==
                  Json::array({2, 7}),
          "open source B-spline is copied without normalizing its knot domain");
    for (std::size_t i = 0; i < direct.samples.size(); ++i)
        check(r.loops[0][i].parameter == direct.samples[i].parameter &&
                  r.loops[0][i].position == direct.samples[i].position,
              "whole source B-spline follows the same native stream as explicit sampling");
    auto p = plane(array(2, {line({2, -3, 0, 6, -3, 0, 6, 1, 0, 2, 1, 0, 2, -3, 0})}));
    p["knotsU"] = {2, 2, 6, 6};
    p["knotsV"] = {-3, -3, 1, 1};
    const auto original = p;
    const auto s = BsplineSurface::from_bgfb(p);
    r = sample_native_surface_boundaries(s);
    const auto unit = sample(boundary);
    check(r.report["status"] == "complete" && r.loops[0].size() == unit.loops[0].size(),
          "source knot-domain boundary is mapped to fractions before stroking");
    for (std::size_t i = 0; i < r.loops[0].size(); ++i)
        check(r.loops[0][i].parameter == unit.loops[0][i].parameter &&
                  r.loops[0][i].position == unit.loops[0][i].position,
              "source-domain square matches independent fraction square");
    check(p == original && s.boundaries() == original["boundaries"] &&
              s.u().knot_domain() == Point2{2, 6},
          "source geometry and knot domains remain immutable");
    auto mixed = spline(3, {.1, .2, 0, -.1, -.14, 0, .9, .2, 0}, Json::array({1, -.2, 1}));
    r = sample(array(2, {mixed}));
    check(r.report["status"] == "complete" && r.loops[0].front().parameter == Point3{.1, .2, 0} &&
              r.loops[0].back().parameter == Point3{.9, .2, 0},
          "open mixed-weight curves bypass positive-weight loft restrictions");
    const auto cyclic = spline(2, {.1, .1, 0, .9, .1, 0, .5, .9, 0}, nullptr, true);
    r = sample(array(2, {cyclic}));
    check(r.report["status"] == "complete" &&
              std::abs(r.loops[0].front().parameter[0] - r.loops[0].back().parameter[0]) < 1e-14 &&
              std::abs(r.loops[0].front().parameter[1] - r.loops[0].back().parameter[1]) < 1e-14 &&
              r.report["boundary_sources"][0]["members"][0]["source_curve_closed"] == true,
          "closed B-spline is opened at the native seam before sampling");
    auto signed_cycle = cyclic;
    signed_cycle["weights"] = {1, -1, 1};
    r = sample(array(5, {boundary, array(2, {signed_cycle})}));
    check(r.report["status"] == "complete" && r.loops.size() == 2,
          "mixed-weight periodic opening keeps both source boundaries");
    const auto polynomial_cycle = sample(array(2, {cyclic}));
    check(r.loops[1].size() == polynomial_cycle.loops[0].size(),
          "opened linear native sampler still ignores signed weights");
    for (std::size_t i = 0; i < r.loops[1].size(); ++i)
        check(r.loops[1][i].parameter == polynomial_cycle.loops[0][i].parameter,
              "signed linear cycle keeps native stored-XYZ sampling");
    auto zero_cycle = signed_cycle;
    zero_cycle["weights"][1] = 0;
    r = sample(array(5, {boundary, array(2, {zero_cycle})}));
    check(r.report["status"] == "incomplete" && r.loops.empty(),
          "generic periodic opening requiring zero-weight division fails atomically");

    // A clamped-like closed quadratic opens by stripping one exterior knot at
    // each end. This branch neither deweights controls nor normalizes [0,2].
    auto special =
        spline(3, {0, 0, 0, .5, .8, 0, .8, .5, 0, 0, 0, 0}, Json::array({-1, .5, -.5, -1}), true,
               Json::array({-1, 0, 0, 0, 1, 2, 2, 2, 3}));
    Json opening;
    auto source = BsplineCurve::from_bgfb(special);
    auto opened = loft_detail::open_periodic_boundary(source, 100, &opening);
    check(opening["method"] == "strip_exterior_knots" &&
              opened.knots == std::vector<double>{0, 0, 0, 1, 2, 2, 2},
          "special periodic opening retains nonunit knot domain");
    check(opened.table()["poles"] == special["poles"] &&
              opened.table()["weights"] == special["weights"] &&
              source.knots() == special["knots"].get<std::vector<double>>(),
          "special signed opening copies controls and leaves source intact");
    special["weights"][1] = 0;
    opened = loft_detail::open_periodic_boundary(BsplineCurve::from_bgfb(special), 100, &opening);
    check(opening["method"] == "strip_exterior_knots" && opened.poles[1][3] == 0 &&
              opened.poles[1][0] == .5,
          "special seam may retain zero-weight homogeneous controls without division");
    for (auto &w : special["weights"])
        w = 0;
    special["poles"][9] = .5;
    opened = loft_detail::open_periodic_boundary(BsplineCurve::from_bgfb(special), 100, &opening);
    check(opening["method"] == "strip_exterior_knots" && opened.poles.back()[0] == .5,
          "empty weighted range yields native capped seam tolerance one");
    special["weights"] = {-1, -1, -1, -1};
    special["poles"] = {0, 0, 0, .1, .2, 0, .2, .1, 0, .0001, 0, 0};
    opened = loft_detail::open_periodic_boundary(BsplineCurve::from_bgfb(special), 100, &opening);
    check(opening["method"] == "cyclic_seam_fallback",
          "negative weights contribute to seam range instead of being skipped");
    special["poles"] = {-.1, -.1, 0, -.9, -.1, 0, -.9, -.9, 0, -.1, -.1, 0};
    r = sample(array(2, {special}));
    check(r.report["status"] == "complete" &&
              r.report["boundary_sources"][0]["members"][0]["prepared_curve_knot_domain"] ==
                  Json::array({0, 2}),
          "source boundary API reports the retained special-seam domain");
    special["weights"][1] = 0;
    for (unsigned k = 3; k < 6; ++k)
        special["poles"][k] = 0;
    r = sample(array(2, {special}));
    check(r.report["status"] == "complete" && r.loops.size() == 1,
          "finite special-seam curve with an internal zero weight can be sampled");

    const auto mixed_quadratic =
        spline(3, {.1, .2, 0, .3, .7, 0, -.05, -.08, 0, .7, .3, 0, .9, .1, 0},
               Json::array({1, 1, -.1, 1, 1}), true);
    source = BsplineCurve::from_bgfb(mixed_quadratic);
    opened = loft_detail::open_periodic_boundary(source, 100, &opening);
    const auto mixed_opened = BsplineCurve::from_bgfb(opened.table());
    for (unsigned i = 0; i <= 100; ++i) {
        const auto a = source.point_at(i / 100.), b = mixed_opened.point_at(i / 100.);
        for (unsigned k = 0; k < 3; ++k)
            check(std::abs(a[k] - b[k]) < 1e-12,
                  "cyclic homogeneous insertion preserves mixed-weight quadratic geometry");
    }
    r = sample(array(2, {mixed_quadratic}));
    check(r.report["status"] == "complete" && r.loops.size() == 1,
          "mixed-weight quadratic source boundary reaches the public sampling API");

    const auto discontinuous =
        spline(3, {.1, .2, 0, .2, .4, 0, .3, .6, 0, .7, .8, 0, .8, .6, 0, .9, .4, 0}, nullptr, true,
               Json::array({-.25, -.1, 0, .5, .5, .5, .75, .9, 1, 1.5, 1.5}));
    source = BsplineCurve::from_bgfb(discontinuous);
    opened = loft_detail::open_periodic_boundary(source, 100, &opening);
    const auto independent = BsplineCurve::from_bgfb(opened.table());
    check(std::count(opened.knots.begin(), opened.knots.end(), .5) == 3,
          "periodic opening retains full internal knot multiplicity");
    for (double f : {0., .125, .49, .49999999, .5, .50000001, .625, .9, 1.}) {
        const auto a = source.point_at(f), b = independent.point_at(f);
        for (unsigned k = 0; k < 3; ++k)
            check(std::abs(a[k] - b[k]) < 1e-13,
                  "opened discontinuous periodic curve agrees on both sides of the break");
    }
    r = sample(array(2, {discontinuous}));
    check(r.report["status"] == "complete" && r.loops.size() == 1,
          "full internal knot multiplicity no longer triggers a loft-only rejection");
    auto rounded = cyclic;
    rounded["weights"] = {7, -3, 11};
    rounded["poles"][0] = .1;
    source = BsplineCurve::from_bgfb(rounded);
    opened = loft_detail::open_periodic_boundary(source, 100, &opening);
    const double roundtrip = (.1 * (1. / 7)) * 7;
    check(roundtrip != .1 && opened.poles.front()[0] == roundtrip &&
              source.poles().front()[0] == .1,
          "native reciprocal-weight roundtrip is observable only in the working copy");
    for (double w : {0., std::numeric_limits<double>::denorm_min()}) {
        rounded["weights"][0] = w;
        bool failed = false;
        try {
            loft_detail::open_periodic_boundary(BsplineCurve::from_bgfb(rounded), 100);
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, "generic periodic opening rejects nonfinite reciprocal preprocessing");
    }

    constexpr double pi = 3.141592653589793;
    r = sample(array(2, {arc(pi / 4, pi / 2)}));
    const Point3 expected{.5 + .2 / std::sqrt(5.), .5 + .2 / std::sqrt(5.), 0};
    check(r.report["status"] == "complete" && r.report["converted_controls"] == 3,
          "native elliptic quarter arc converts as one rational quadratic");
    for (unsigned k = 0; k < 3; ++k)
        check(std::abs(r.loops[0].front().parameter[k] - expected[k]) < 1e-14,
              "native ellipse axis-length angle adjustment is not source-angle substitution");
    for (double sweep : {2 * pi / 3, pi, 1.8 * pi, 2 * pi, -2 * pi}) {
        r = sample(array(2, {arc(0, sweep, .2, .2)}));
        check(r.report["status"] == "complete" && r.loops.size() == 1,
              "one/two/three-span and both-direction circular boundaries sample");
        for (const auto &sample : r.loops[0])
            check(std::abs(std::hypot(sample.parameter[0] - .5, sample.parameter[1] - .5) - .2) <
                      1e-12,
                  "native converted circular boundary stays on independent circle equation");
        check(r.report["boundary_sources"][0]["members"][0]["source_curve_closed"] ==
                  (std::abs(sweep) == 2 * pi),
              "native full-circle flag is carried into opening");
    }
    r = sample(array(1, {arc(0, 2 * pi, .2, .1)}));
    check(r.report["status"] == "complete" && r.loops.size() == 1 &&
              r.report["opened_controls"] == 7,
          "full ellipse satisfies source Open closure then uses native clamped seam opening");
    for (const auto &sample : r.loops[0]) {
        const double u = (sample.parameter[0] - .5) / .2, v = (sample.parameter[1] - .5) / .1;
        check(std::abs(u * u + v * v - 1) < 1e-12,
              "full converted ellipse independent conic equation");
    }
    r = sample(array(2, {arc(0, 0)}));
    check(r.report["status"] == "complete" && r.loops.size() == 1,
          "zero-sweep native ellipse is retained as a degenerate rational member");
    r = sample(array(2, {Json{{"_type", "MysteryCurve"}}}));
    check(r.report["status"] == "incomplete" && r.loops.empty() &&
              r.report["failed_source_path"] == "/curves/0/geometry",
          "unknown source primitive reports exact path rather than native ignored status");
    PCurveBoundaryStrokeOptions limited;
    limited.max_controls = 4;
    check(sample(boundary, limited).report["status"] == "incomplete",
          "source control budget applies before conversion");
    limited = {};
    limited.max_tree_visits = 2;
    check(sample(array(5, {boundary, boundary}), limited).report["status"] == "incomplete",
          "tree traversal budget applies to region and member visits");
    limited = {};
    limited.max_tree_depth = 1;
    check(sample(array(5, {array(4, {boundary})}), limited).report["status"] == "incomplete",
          "source tree nesting is bounded");
    limited = {};
    limited.sampling.max_points = 3;
    r = sample(boundary, limited);
    check(r.report["status"] == "incomplete" && r.loops.empty() &&
              r.report["boundary_sources"].size() == 1,
          "sampling failure retains preparation provenance but returns no partial loops");
    r = sample(nullptr);
    check(r.report["status"] == "complete" && r.loops.empty(),
          "untrimmed source surface has no invented boundary");
    const Json akima = {
        {"_type", "AkimaCurve"},
        {"points", {-1, .25, 0, -.5, .25, 0, 0, .25, 0, .5, .25, 0, 1, .25, 0, 1.5, .25, 0}}};
    r = sample(array(2, {akima}));
    check(r.report["status"] == "complete" && r.loops[0].size() == 3 &&
              r.report["converted_controls"] == 4,
          "Akima source is converted once to one complete native cubic");
    for (unsigned i = 0; i < 3; ++i)
        check(std::abs(r.loops[0][i].parameter[0] - i * .25) < 1e-14 &&
                  std::abs(r.loops[0][i].parameter[1] - .25) < 1e-14,
              "straight Akima fixture has independently known endpoints and midpoint");
    const Json interpolation = {{"_type", "InterpolationCurve"},
                                {"order", 4},
                                {"closed", false},
                                {"isChordLenKnots", 0},
                                {"isColinearTangents", 0},
                                {"isChordLenTangents", 0},
                                {"isNaturalTangents", 0},
                                {"startTangent", nullptr},
                                {"endTangent", nullptr},
                                {"knots", nullptr},
                                {"fitPoints", {0, .25, 0, 1, .75, 0}}};
    r = sample(array(2, {interpolation}));
    check(r.report["status"] == "complete" && r.loops[0].size() == 3,
          "interpolation source retains whole native cubic scheduling");
    for (unsigned i = 0; i < 3; ++i)
        check(std::abs(r.loops[0][i].parameter[0] - i * .5) < 1e-14 &&
                  std::abs(r.loops[0][i].parameter[1] - (.25 + i * .25)) < 1e-14,
              "two-point interpolation samples agree with independent line equation");
    Json transform;
    const std::array<const char *, 12> names{"axx", "axy", "axz", "axw", "ayx", "ayy",
                                             "ayz", "ayw", "azx", "azy", "azz", "azw"};
    for (unsigned i = 0; i < 12; ++i)
        transform[names[i]] = i == 0 || i == 5 || i == 10 ? 1. : 0.;
    const Json spiral = {{"_type", "TransitionSpiral"},
                         {"detail",
                          {{"transform", transform},
                           {"fractionA", 0.},
                           {"fractionB", 1.},
                           {"bearing0Radians", 0.},
                           {"bearing1Radians", .5},
                           {"curvature0", .5},
                           {"curvature1", .5},
                           {"spiralType", 10},
                           {"constructionHint", 0}}},
                         {"extraData", nullptr},
                         {"directDetail", nullptr}};
    const auto fitted = TransitionSpiral::from_bgfb(spiral).native_fit().curve;
    const auto first = fitted.point_at(0), last = fitted.point_at(1);
    r = sample(array(1, {spiral, line({last[0], last[1], last[2], first[0], first[1], first[2]})}));
    check(r.report["status"] == "complete" && r.loops.size() == 1 &&
              r.report["converted_controls"] == fitted.poles().size() + 2,
          "Open closure and conversion reuse the same source spiral fit without extra native "
          "geometry");
    check(r.report["boundary_sources"][0]["members"][0]["source_type"] == "TransitionSpiral",
          "fitted primitive provenance remains the original source type");
    auto weighted_domain = plane(array(2, {mixed}));
    weighted_domain["knotsU"] = {2, 2, 6, 6};
    weighted_domain["knotsV"] = {-3, -3, 1, 1};
    auto &values = weighted_domain["boundaries"]["curves"][0]["geometry"]["poles"];
    for (unsigned i = 0; i < 3; ++i) {
        const double w = mixed["weights"][i].get<double>();
        values[3 * i] = 2 * w + 4 * mixed["poles"][3 * i].get<double>();
        values[3 * i + 1] = -3 * w + 4 * mixed["poles"][3 * i + 1].get<double>();
    }
    r = sample_native_surface_boundaries(BsplineSurface::from_bgfb(weighted_domain));
    const auto unit_weighted = sample(array(2, {mixed}));
    check(r.report["status"] == "complete" && r.loops[0].size() == unit_weighted.loops[0].size(),
          "homogeneous source-UV affine transform applies translation times signed weight");
    for (std::size_t i = 0; i < r.loops[0].size(); ++i)
        for (unsigned k = 0; k < 3; ++k)
            check(std::abs(r.loops[0][i].parameter[k] - unit_weighted.loops[0][i].parameter[k]) <
                      1e-12,
                  "signed-weight normalized source curve agrees with independent fraction curve");
    return checks;
}
