#include "internal.hpp"
#include "p3d/pcurve.hpp"
#include "native_pcurve_points.hpp"
using namespace p3d;
namespace {
Json plane() {
    return {{"_type", "BsplineSurface"},
            {"numPolesU", 2},
            {"numPolesV", 2},
            {"orderU", 2},
            {"orderV", 2},
            {"closedU", false},
            {"closedV", false},
            {"poles", {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0}},
            {"numRulesU", 0},
            {"numRulesV", 0},
            {"weights", nullptr},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"boundaries", nullptr},
            {"holeOrigin", 0}};
}
BsplineCurve curve(unsigned order, const std::vector<double> &poles, bool closed = false,
                   Json weights = nullptr) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"closed", closed},
                                    {"poles", poles},
                                    {"weights", weights},
                                    {"knots", nullptr}});
}
} // namespace
unsigned native_pcurve_tests() {
    unsigned checks = 0;
    auto check = [&](bool v, const char *s) {
        ++checks;
        require(v, s);
    };
    const auto surface = BsplineSurface::from_bgfb(plane());
    const auto line = curve(2, {0, 0, 0, 1, 0, 0});
    PCurveStrokeOptions options;
    options.uv_tolerance = options.spatial_tolerance = .01;
    const auto plain = sample_native_pcurve(surface, line, options);
    check(plain.report["status"] == "complete" && plain.samples.size() == 5,
          "native linear sampler has two half intervals with two substeps each");
    for (unsigned i = 0; i < plain.samples.size(); ++i)
        check(plain.samples[i].parameter == Point3{double(i) / 4, 0, 0} &&
                  plain.samples[i].position == Point3{double(i) / 4, 0, 0},
              "plane linear samples use exact expected quarters");
    auto ignored = options;
    ignored.minimum_points = 100;
    ignored.start_fraction = .3;
    ignored.end_fraction = .4;
    auto same = sample_native_pcurve(surface, line, ignored);
    check(same.samples.size() == 5 && same.samples.front().parameter[0] == 0 &&
              same.samples.back().parameter[0] == 1 && !same.report["fraction_interval_applied"],
          "order two ignores minimum points and fraction interval as native dispatch does");
    const auto weighted_line = sample_native_pcurve(
        surface, curve(2, {0, 0, 0, 1, 0, 0}, false, Json::array({1, 2})), options);
    check(weighted_line.samples.back().parameter[0] == 1 &&
              !weighted_line.report["linear_pole_weights_applied"],
          "native order two dispatch consumes stored pole coordinates without deweighting");
    auto outside = sample_native_pcurve(surface, curve(2, {-1, -1, 2, 2, 2, 2}), options);
    check(outside.report["status"] == "complete" &&
              outside.samples.front().parameter == Point3{0, 0, 2} &&
              outside.samples.back().parameter == Point3{1, 1, 2},
          "linear endpoints clamp XY while retaining parameter Z");
    check(outside.report["intervals"].size() == 6,
          "linear crossing splits at the active surface endpoints before clamping");
    auto cycle =
        sample_native_pcurve(surface, curve(2, {.2, .2, 0, .8, .2, 0, .5, .8, 0}, true), options);
    check(cycle.report["status"] == "complete" && cycle.samples.size() == 13 &&
              std::hypot(cycle.samples.front().parameter[0] - cycle.samples.back().parameter[0],
                         cycle.samples.front().parameter[1] - cycle.samples.back().parameter[1]) <
                  1e-15,
          "closed order two includes last-to-first pole segment");
    const PCurveSample previous{{0, 0, 0}, {0, 0, 0}};
    auto joined = sample_native_pcurve(surface, line, options, previous);
    check(joined.samples.size() == 4 && joined.report["omitted_starts"] == 1 &&
              joined.samples.front().parameter[0] == .25,
          "same native stream omits previous sample without returning it again");
    auto world_only =
        sample_native_pcurve(surface, line, options, PCurveSample{{.9, .9, 0}, {0, 0, 0}});
    check(world_only.samples.size() == 4 && world_only.report["omitted_starts"] == 1,
          "matching world position can omit a start with different UV coordinates");
    auto large_tolerance = options;
    large_tolerance.spatial_tolerance = 1e5;
    auto uv_only = sample_native_pcurve(surface, line, large_tolerance,
                                        PCurveSample{{1e-8, 0, 0}, {10, 10, 10}});
    check(uv_only.samples.size() == 4 && uv_only.report["omitted_starts"] == 1,
          "UV start threshold is inclusive and independent of world position");
    auto separated =
        sample_native_pcurve(surface, line, options, PCurveSample{{.5, .5, 0}, {.5, .5, 0}});
    check(separated.samples.size() == 5 && separated.report["omitted_starts"] == 0,
          "distinct native start remains present and is not deduplicated");

    // Independent polynomial oracle S(u,v)=(u,v,u^2).
    auto parabola = plane();
    parabola["orderU"] = 3;
    parabola["numPolesU"] = 3;
    parabola["poles"] = {0, 0, 0, .5, 0, 0, 1, 0, 1, 0, 1, 0, .5, 1, 0, 1, 1, 1};
    auto bent = sample_native_pcurve(BsplineSurface::from_bgfb(parabola), line, options);
    check(bent.report["status"] == "complete" && bent.samples.size() == 10 &&
              bent.report["intervals"][0]["subdivisions"] == 5 &&
              bent.report["intervals"][1]["subdivisions"] == 4,
          "spatial parabola requires five and four substeps from analytic chord distances");
    for (const auto &p : bent.samples)
        check(std::abs(p.position[0] - p.parameter[0]) < 1e-14 &&
                  std::abs(p.position[2] - p.parameter[0] * p.parameter[0]) < 1e-14,
              "native line samples follow the independently known curved surface");

    // Independent parameter curve C(t)=(t,t^2,0), on a flat surface.
    const auto quadratic = curve(3, {0, 0, 0, .5, 0, 0, 1, 1, 0});
    auto uv_options = options;
    uv_options.spatial_tolerance = 100;
    auto q = sample_native_pcurve(surface, quadratic, uv_options);
    check(q.report["status"] == "complete" && q.samples.size() == 11 &&
              q.report["intervals"][0]["subdivisions"] == 10,
          "nonlinear PCurve refines from its UV error even on a flat surface");
    for (const auto &p : q.samples)
        check(std::abs(p.parameter[1] - p.parameter[0] * p.parameter[0]) < 1e-14,
              "high order PCurve samples agree with independent polynomial oracle");
    uv_options.start_fraction = .2;
    uv_options.end_fraction = .8;
    uv_options.minimum_points = 3;
    q = sample_native_pcurve(surface, quadratic, uv_options);
    check(q.report["status"] == "complete" && q.report["intervals"].size() == 2 &&
              std::abs(q.samples.front().parameter[0] - .2) < 1e-14 &&
              std::abs(q.samples.back().parameter[0] - .8) < 1e-14,
          "higher order sampling respects selected fraction interval and initial partition");
    const double w = std::sqrt(.5);
    const auto rational =
        curve(3, {.8, .5, 0, .8 * w, .8 * w, 0, .5, .8, 0}, false, Json::array({1, w, 1}));
    auto arc = sample_native_pcurve(surface, rational, options);
    check(arc.report["status"] == "complete", "rational higher order PCurve sampling succeeds");
    for (const auto &p : arc.samples)
        check(std::abs(std::hypot(p.parameter[0] - .5, p.parameter[1] - .5) - .3) < 1e-14,
              "rational PCurve follows analytic circle rather than its control polygon");

    auto knotted = plane();
    knotted["numPolesU"] = 3;
    knotted["knotsU"] = {0, 0, .3, 1, 1};
    knotted["poles"] = {0, 0, 0, .3, 0, 0, 1, 0, 0, 0, 1, 0, .3, 1, 0, 1, 1, 0};
    auto split = sample_native_pcurve(BsplineSurface::from_bgfb(knotted), line, options);
    check(split.report["status"] == "complete" && split.report["intervals"].size() == 4 &&
              split.samples.size() == 9 && std::abs(split.samples[4].parameter[0] - .3) < 1e-14,
          "linear surface direction splits at each active knot before native stroking");
    knotted["knotsU"] = {2, 2, 5, 12, 12};
    knotted["knotsV"] = {-3, -3, 7, 7};
    auto normalized = sample_native_pcurve(BsplineSurface::from_bgfb(knotted), line, options);
    check(
        normalized.samples.size() == split.samples.size() &&
            normalized.report["surface_split_knots"] == split.report["surface_split_knots"],
        "surface knot ranges normalize independently without changing PCurve fraction coordinates");
    auto cubic_surface = plane();
    cubic_surface["orderU"] = 3;
    cubic_surface["numPolesU"] = 4;
    cubic_surface["knotsU"] = {0, 0, 0, .3, 1, 1, 1};
    cubic_surface["poles"] = {0, 0, 0, .15, 0, 0, .65, 0, 0, 1, 0, 0,
                              0, 1, 0, .15, 1, 0, .65, 1, 0, 1, 1, 0};
    auto simple_knot =
        sample_native_pcurve(BsplineSurface::from_bgfb(cubic_surface), line, options);
    check(simple_knot.report["surface_split_knots"]["u"] == Json::array({0, 1}),
          "higher order surface simple knots are not high-multiplicity stroke splitters");
    cubic_surface["numPolesU"] = 5;
    cubic_surface["knotsU"] = {0, 0, 0, .3, .3, 1, 1, 1};
    cubic_surface["poles"] = {0, 0, 0, .15, 0, 0, .3, 0, 0, .65, 0, 0, 1, 0, 0,
                              0, 1, 0, .15, 1, 0, .3, 1, 0, .65, 1, 0, 1, 1, 0};
    auto repeated = sample_native_pcurve(BsplineSurface::from_bgfb(cubic_surface), line, options);
    check(repeated.report["status"] == "complete" &&
              repeated.report["surface_split_knots"]["u"] == Json::array({0, .3, 1}) &&
              repeated.report["intervals"].size() == 4,
          "higher order surface double knots split native linear stroking");
    cubic_surface["knotsU"][4] = .3 + 1e-15;
    auto near_repeat =
        sample_native_pcurve(BsplineSurface::from_bgfb(cubic_surface), line, options);
    check(near_repeat.report["surface_split_knots"]["u"] == Json::array({0, .3, 1}),
          "native near-knot compression retains the first representative");
    auto periodic = plane();
    periodic["closedU"] = true;
    periodic["numPolesU"] = 4;
    periodic["poles"] = {1, 0, 0, 0, 1, 0, -1, 0, 0, 0, -1, 0,
                         1, 0, 1, 0, 1, 1, -1, 0, 1, 0, -1, 1};
    auto ring = sample_native_pcurve(BsplineSurface::from_bgfb(periodic), line, options);
    check(ring.report["status"] == "complete" && ring.samples.size() == 17 &&
              ring.samples.front().parameter[0] == 0 && ring.samples.back().parameter[0] == 1 &&
              ring.samples.front().position == ring.samples.back().position,
          "periodic surface seam keeps distinct start/end UV samples without spatial welding");
    periodic["knotsU"] = {-.25, 0, 0, .5, .75, 1, 1.25};
    const auto normalized_periodic =
        sample_native_pcurve(BsplineSurface::from_bgfb(periodic), line, options);
    for (auto &k : periodic["knotsU"])
        k = 2 + 4 * k.get<double>();
    const auto original_periodic = BsplineSurface::from_bgfb(periodic);
    const auto prepared_periodic = sample_native_pcurve(original_periodic, line, options);
    check(
        prepared_periodic.report["status"] == "complete" &&
            prepared_periodic.report["surface_knot_preparation"] == "normalized_copy" &&
            prepared_periodic.samples.front().position ==
                normalized_periodic.samples.front().position &&
            original_periodic.u().knot_domain() == Point2{2, 6} &&
            original_periodic.u().periodic_pole_shift() == 0,
        "normalized native surface copy recomputes closed pole convention without mutating source");
    // This cubic passes the native three-point test while deviating between samples.
    auto hidden_curve = curve(4, {0, .5, 0, 1. / 3, 5. / 6, 0, 2. / 3, 1. / 6, 0, 1, .5, 0});
    auto hidden = sample_native_pcurve(surface, hidden_curve, options);
    check(hidden.report["status"] == "complete" && hidden.samples.size() == 3 &&
              hidden.report["sampled_tolerances_met"] &&
              hidden.report["continuous_error_bound"].is_null() &&
              hidden_curve.point_at(.25)[1] > .59,
          "native sampled error acceptance must not be described as a continuous error guarantee");
    auto failed_options = options;
    failed_options.max_points = 4;
    auto failed = sample_native_pcurve(surface, line, failed_options);
    check(failed.report["status"] == "incomplete" && failed.samples.empty(),
          "point budget failure publishes no partial native stroke");
    failed_options = options;
    failed_options.max_evaluations = 3;
    failed = sample_native_pcurve(surface, line, failed_options);
    check(failed.report["status"] == "incomplete" && failed.samples.empty(),
          "evaluation budget failure publishes no partial native stroke");
    auto capped = options;
    capped.spatial_tolerance = 1e-30;
    auto cap = sample_native_pcurve(BsplineSurface::from_bgfb(parabola), line, capped);
    check(cap.report["status"] == "complete" && !cap.report["sampled_tolerances_met"] &&
              cap.report["intervals"][0]["stop"] == "native_subdivision_limit" &&
              cap.report["continuous_error_bound"].is_null(),
          "native stopping at 2048 substeps does not falsely assert tolerance satisfaction");
    auto defaults = options;
    defaults.uv_tolerance = -1;
    defaults.spatial_tolerance = 0;
    auto fallback = sample_native_pcurve(surface, line, defaults);
    check(fallback.report["uv_tolerance"] == .001 && fallback.report["spatial_tolerance"] == 1e-7,
          "nonpositive tolerances select native branch defaults");
    auto backwards = uv_options;
    backwards.start_fraction = .8;
    backwards.end_fraction = .2;
    backwards.uv_tolerance = 1e-15;
    auto reverse = sample_native_pcurve(surface, quadratic, backwards);
    check(reverse.report["status"] == "complete" && !reverse.report["sampled_tolerances_met"] &&
              reverse.report["intervals"][0]["stop"] == "native_parameter_step_limit",
          "signed native parameter step stops reversed intervals without inventing refinement");
    auto nonlinear_outside =
        sample_native_pcurve(surface, curve(3, {-.1, 0, 0, .5, .5, 0, 1, 1, 0}), options);
    check(nonlinear_outside.report["status"] == "complete" &&
              nonlinear_outside.samples.front().parameter[0] == -.1 &&
              nonlinear_outside.samples.front().position[0] == 0 &&
              nonlinear_outside.report["clamped_surface_evaluations"] > 0,
          "nonlinear out-of-domain UV stays stored while surface evaluation clamps its parameters");
    auto extended_options = options;
    extended_options.start_fraction = -.25;
    extended_options.end_fraction = 1.25;
    auto extended = sample_native_pcurve(surface, quadratic, extended_options);
    check(extended.report["status"] == "complete" &&
              extended.samples.front().parameter == Point3{0, 0, 0} &&
              extended.samples.back().parameter == Point3{1, 1, 0},
          "finite curve fraction intervals can extend beyond native clamped endpoints");
    const auto zero_curve = curve(3, {0, 0, 0, .5, 0, 0, 1, 1, 0}, false, Json::array({0, 0, 0}));
    const auto zero_point = detail::pcurve_point(zero_curve, .25);
    check(zero_point.zero_weight_fallback && zero_point.point == Point3{.25, .0625, 0},
          "native curve zero evaluated weight returns the homogeneous numerator");
    const auto zero_samples = sample_native_pcurve(surface, zero_curve, options);
    check(zero_samples.report["status"] == "complete" &&
              zero_samples.report["curve_zero_weight_fallbacks"] ==
                  zero_samples.report["evaluations"] &&
              zero_samples.report["curve_zero_weight_fallback_fractions"].size() ==
                  zero_samples.report["curve_zero_weight_fallbacks"].get<unsigned>() &&
              zero_curve.weights() == std::vector<double>({0, 0, 0}),
          "zero-weight fallback is reported for every attempted sample without changing source "
          "weights");
    const auto isolated_zero =
        curve(3, {.2, .2, 0, .4, .4, 0, .6, .6, 0}, false, Json::array({1, -1, 1}));
    check(detail::pcurve_point(isolated_zero, .5).zero_weight_fallback &&
              !detail::pcurve_point(isolated_zero, .25).zero_weight_fallback,
          "native curve weight fallback tests the evaluated denominator rather than control signs");
    auto zero_surface = plane();
    zero_surface["weights"] = {0, 0, 0, 0};
    const auto zero_mesh =
        sample_native_pcurve(BsplineSurface::from_bgfb(zero_surface), line, options);
    check(zero_mesh.report["status"] == "incomplete" && zero_mesh.samples.empty(),
          "surface zero-weight failure must not inherit the distinct curve caller fallback");
    const auto closed_curve = curve(3, {.2, .2, 0, .8, .2, 0, .8, .8, 0, .2, .8, 0}, true);
    check(detail::pcurve_point(closed_curve, -.25).point ==
                  detail::pcurve_point(closed_curve, 0).point &&
              detail::pcurve_point(closed_curve, 1.25).point ==
                  detail::pcurve_point(closed_curve, 1).point,
          "native point caller clamps closed curve parameters instead of applying derivative "
          "wrapping");
    auto cancellation = plane();
    cancellation["poles"] = {1e16, 0, 0, -1e16, 0, 0, 1, 0, 0, 1, 0, 0};
    const auto cancellation_surface = BsplineSurface::from_bgfb(cancellation);
    check(detail::pcurve_surface_point(cancellation_surface, .5, .5)[0] == .25 &&
              cancellation_surface.point_at(.5, .5)[0] == .5,
          "native surface sums U outside and V inside without changing general mathematical "
          "evaluator");
    const auto constant_curve = curve(3, {1, 1, 1, 1, 1, 1, 1, 1, 1});
    const double expected_constant = (.7 * .7 + (.3 * .7 + .7 * .3)) + .3 * .3;
    check(detail::pcurve_point(constant_curve, .3).point[0] == expected_constant,
          "native polynomial curve does not renormalize the sum of blending coefficients");
    Json tiny = {{"_type", "BsplineCurve"}, {"order", 3},
                 {"closed", false},         {"poles", {0, 0, 0, .5, 0, 0, 1, 1, 0}},
                 {"weights", nullptr},      {"knots", {0, 0, 0, 1e-310, 1e-310, 1e-310}}};
    bool native_overflow = false;
    try {
        (void)detail::pcurve_point(BsplineCurve::from_bgfb(tiny), .5);
    } catch (const std::exception &) {
        native_overflow = true;
    }
    check(native_overflow,
          "native divide-before-multiply overflow is not hidden by ratio-first evaluation");
    std::vector<double> many_poles;
    for (unsigned i = 0; i < 27; ++i) {
        many_poles.push_back(double(i) / 26);
        many_poles.push_back(0);
        many_poles.push_back(0);
    }
    const auto excessive = sample_native_pcurve(surface, curve(27, many_poles), options);
    check(excessive.report["status"] == "incomplete" && excessive.samples.empty(),
          "fixed-size native point callers explicitly reject curve orders above 26");
    // Independent de Casteljau reference: no knot-span or blending recurrence.
    using H = std::array<double, 4>;
    auto casteljau = [](std::vector<H> p, double t) {
        t = std::clamp(t, 0., 1.);
        for (std::size_t n = p.size(); n > 1; --n)
            for (std::size_t i = 0; i + 1 < n; ++i)
                for (unsigned k = 0; k < 4; ++k)
                    p[i][k] = (1 - t) * p[i][k] + t * p[i + 1][k];
        return p.front();
    };
    auto matches = [](Point3 p, H h) {
        for (unsigned k = 0; k < 3; ++k)
            if (std::abs(p[k] - h[k] / h[3]) > 1e-11)
                return false;
        return true;
    };
    for (unsigned order : {2u, 3u, 4u, 8u, 26u})
        for (bool rational : {false, true}) {
            std::vector<H> controls;
            std::vector<double> coordinates, weights;
            for (unsigned i = 0; i < order; ++i) {
                const double w = rational ? 1 + double(i % 3) / 4 : 1;
                H h{double(i) / (order - 1) * w, std::sin(double(i)) * w, .2 * i * w, w};
                controls.push_back(h);
                coordinates.insert(coordinates.end(), h.begin(), h.begin() + 3);
                weights.push_back(w);
            }
            const auto bezier =
                curve(order, coordinates, false, rational ? Json(weights) : Json(nullptr));
            for (double t : {-.2, 0., .1, .3, .5, .9, 1., 1.2})
                check(matches(detail::pcurve_point(bezier, t).point, casteljau(controls, t)),
                      "native curve basis agrees with independent clamped homogeneous de Casteljau "
                      "oracle");
            auto patch = plane();
            patch["orderU"] = order;
            patch["numPolesU"] = order;
            patch["orderV"] = 4;
            patch["numPolesV"] = 4;
            coordinates.clear();
            weights.clear();
            std::vector<std::vector<H>> rows;
            for (unsigned j = 0; j < 4; ++j) {
                std::vector<H> row;
                for (unsigned i = 0; i < order; ++i) {
                    const double w = rational ? 1 + double((i + 2 * j) % 3) / 4 : 1;
                    H h{(i + .2 * j) / (order - 1) * w, (j + .1 * i) / 3 * w,
                        (.07 * i * j - .2 * i) * w, w};
                    row.push_back(h);
                    coordinates.insert(coordinates.end(), h.begin(), h.begin() + 3);
                    weights.push_back(w);
                }
                rows.push_back(row);
            }
            patch["poles"] = coordinates;
            patch["weights"] = rational ? Json(weights) : Json(nullptr);
            const auto tensor = BsplineSurface::from_bgfb(patch);
            for (double u : {-.2, .1, .4, 1.2})
                for (double v : {-.1, .2, .7, 1.1}) {
                    std::vector<H> column;
                    for (const auto &row : rows)
                        column.push_back(casteljau(row, u));
                    check(matches(detail::pcurve_surface_point(tensor, u, v), casteljau(column, v)),
                          "native tensor basis agrees with independent two-direction homogeneous "
                          "de Casteljau oracle");
                }
        }
    Json discontinuous = {{"_type", "BsplineCurve"},
                          {"order", 3},
                          {"closed", false},
                          {"poles", {0, 0, 0, .1, 0, 0, .2, 0, 0, .7, 0, 0, .8, 0, 0, 1, 0, 0}},
                          {"weights", nullptr},
                          {"knots", {0, 0, 0, .5, .5, .5, 1, 1, 1}}};
    const auto jumped = BsplineCurve::from_bgfb(discontinuous);
    check(detail::pcurve_point(jumped, .5).point[0] == .7 &&
              std::abs(detail::pcurve_point(jumped, std::nextafter(.5, 0.)).point[0] - .2) < 1e-14,
          "native basis chooses the right side of an exact full-multiplicity interior knot");
    bool threw = false;
    try {
        auto invalid = options;
        invalid.uv_tolerance = std::numeric_limits<double>::infinity();
        (void)sample_native_pcurve(surface, line, invalid);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, "non-finite sampling options are rejected");
    return checks;
}
