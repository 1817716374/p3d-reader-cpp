#include "internal.hpp"
#include "p3d/pcurve.hpp"
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
    check(nonlinear_outside.report["status"] == "incomplete" && nonlinear_outside.samples.empty(),
          "unsupported out-of-domain nonlinear evaluation does not use linear endpoint clamping");
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
