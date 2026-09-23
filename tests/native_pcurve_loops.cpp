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
            {"poles", {0, 0, 0, 3, 0, 0, 0, 4, 0, 3, 4, 0}},
            {"numRulesU", 0},
            {"numRulesV", 0},
            {"weights", nullptr},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"boundaries", nullptr},
            {"holeOrigin", 0}};
}
BsplineCurve line(Point3 a, Point3 b) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", 2},
                                    {"closed", false},
                                    {"poles", {a[0], a[1], a[2], b[0], b[1], b[2]}},
                                    {"knots", nullptr},
                                    {"weights", nullptr}});
}
} // namespace
unsigned native_pcurve_loop_tests() {
    unsigned checks = 0;
    auto check = [&](bool v, const char *s) {
        ++checks;
        require(v, s);
    };
    const auto source = plane();
    const auto surface = BsplineSurface::from_bgfb(source);
    const auto a = line({0, 0, 0}, {1, 0, 0});
    const auto b = line({1, 0, 0}, {1, 1, 0});
    const std::vector<std::vector<BsplineCurve>> loops{{a, b}, {a}, {}};
    auto r = sample_native_pcurve_loops(surface, loops);
    check(r.report["status"] == "complete" && r.loops.size() == 3 && r.loops[0].size() == 9 &&
              r.loops[1].size() == 5 && r.loops[2].empty(),
          "native loop streams retain member order and reset previous state between loops");
    check(r.report["sample_count"] == 14 && r.report["evaluations"] == 15 &&
              r.report["omitted_starts"] == 1,
          "loop totals include omitted start evaluations");
    check(r.report["control_range"]["low"] == Json::array({0, 0, 0}) &&
              r.report["control_range"]["high"] == Json::array({3, 4, 0}) &&
              r.report["control_range"]["diagonal"] == 5 && r.report["uv_tolerance"] == .01 &&
              r.report["spatial_tolerance"] == .0005,
          "both-negative tolerances select full Cartesian control range");
    check(r.loops[0].front().parameter == Point3{0, 0, 0} &&
              r.loops[0].back().parameter == Point3{1, 1, 0},
          "restroke does not append an implicit closing edge");
    const auto &second = r.report["loops"][0]["members"][1];
    check(second["sample_offset"] == 5 && second["sample_count"] == 4,
          "member provenance refers only to newly appended points");
    for (unsigned i = 0; i < 5; ++i)
        check(r.loops[0][i].position == Point3{.75 * i, 0, 0} &&
                  r.loops[1][i].position == r.loops[0][i].position,
              "identical curves in separate loops retain independent ordered output");
    for (unsigned i = 1; i <= 4; ++i)
        check(r.loops[0][4 + i].position == Point3{3, double(i), 0},
              "adjacent linear members share exactly their start sample");
    check(source == plane(), "native loop sampling does not modify source JSON");

    PCurveLoopStrokeOptions opt;
    for (const auto &v : std::vector<std::array<double, 4>>{{-1, .2, .001, .2},
                                                            {.3, -1, .3, 1e-7},
                                                            {0, 0, .001, 1e-7},
                                                            {0, -1, .001, 1e-7},
                                                            {-1, 0, .001, 1e-7},
                                                            {.4, .5, .4, .5}}) {
        opt.uv_tolerance = v[0];
        opt.spatial_tolerance = v[1];
        const auto t = sample_native_pcurve_loops(surface, {{a}}, opt);
        check(t.report["status"] == "complete" &&
                  t.report["tolerance_selection"] == "individual_tolerances" &&
                  t.report["uv_tolerance"] == v[2] && t.report["spatial_tolerance"] == v[3] &&
                  !t.report.contains("control_range"),
              "range defaults require two strictly negative tolerances");
    }
    opt = {};
    for (const auto &weights : std::vector<std::vector<double>>{
             {2, 2, 2, 2}, {-2, -2, -2, -2}, {1, -2, 3, 4}, {1e-20, 1e-20, 1e-20, 1e-20}}) {
        auto weighted = source;
        weighted["weights"] = weights;
        for (unsigned i = 0; i < 4; ++i)
            for (unsigned k = 0; k < 3; ++k)
                weighted["poles"][3 * i + k] =
                    source["poles"][3 * i + k].get<double>() * weights[i];
        const auto t = sample_native_pcurve_loops(BsplineSurface::from_bgfb(weighted),
                                                  {{line({0, 0, 0}, {0, 0, 0})}});
        check(t.report["status"] == "complete" && t.report["control_range"]["diagonal"] == 5 &&
                  t.report["spatial_tolerance"] == .0005,
              "range deweights negative and tiny weights without filtering poles");
    }
    auto zero = source;
    zero["weights"] = {1, 1, 1, 0};
    for (unsigned k = 9; k < 12; ++k)
        zero["poles"][k] = 0;
    auto t =
        sample_native_pcurve_loops(BsplineSurface::from_bgfb(zero), {{line({0, 0, 0}, {0, 0, 0})}});
    check(t.report["status"] == "incomplete" && t.loops.empty() &&
              t.report["reason"].get<std::string>().find("zero-weight") != std::string::npos,
          "automatic range reports division by zero rather than removing a pole");
    opt.uv_tolerance = opt.spatial_tolerance = .01;
    t = sample_native_pcurve_loops(BsplineSurface::from_bgfb(zero), {{line({0, 0, 0}, {0, 0, 0})}},
                                   opt);
    check(t.report["status"] == "complete", "explicit tolerances bypass automatic range division");
    auto collapsed = source;
    for (auto &p : collapsed["poles"])
        p = 0;
    t = sample_native_pcurve_loops(BsplineSurface::from_bgfb(collapsed), {{a, b}});
    check(t.report["status"] == "complete" && t.report["control_range"]["diagonal"] == 0 &&
              t.report["spatial_tolerance"] == 1e-7,
          "zero-size range uses single-curve spatial fallback after range selection");

    auto nonunit = source;
    nonunit["knotsU"] = {2, 2, 5, 5};
    nonunit["knotsV"] = {-3, -3, 9, 9};
    t = sample_native_pcurve_loops(BsplineSurface::from_bgfb(nonunit), loops);
    check(t.report["status"] == "complete" &&
              t.report["surface_knot_preparation"] == "normalized_copy" &&
              t.loops[0].size() == r.loops[0].size(),
          "surface is prepared once for all loop members");
    for (unsigned i = 0; i < t.loops[0].size(); ++i)
        check(t.loops[0][i].position == r.loops[0][i].position &&
                  t.loops[0][i].parameter == r.loops[0][i].parameter,
              "prepared loops already use fractions independently of source surface knot domain");
    t = sample_native_pcurve_loops(surface,
                                   {{line({0, 0, 0}, {.25, 0, 0}), line({.75, 0, 0}, {1, 0, 0})}});
    check(t.report["status"] == "complete" && t.loops[0].size() == 10 &&
              t.loops[0][4].parameter[0] == .25 && t.loops[0][5].parameter[0] == .75,
          "native restroke retains a discontinuous stream without filling its gap");

    opt = {};
    opt.max_points = 14;
    opt.max_evaluations = 15;
    t = sample_native_pcurve_loops(surface, loops, opt);
    check(t.report["status"] == "complete",
          "exact global budgets do not charge previous sample twice");
    opt.max_points = 13;
    t = sample_native_pcurve_loops(surface, loops, opt);
    check(t.report["status"] == "incomplete" && t.loops.empty() && t.report["failed_loop"] == 1 &&
              t.report["failed_curve"] == 0 && t.report["sample_count"] == 0,
          "point budget failure in a later loop clears earlier output");
    opt.max_points = 14;
    opt.max_evaluations = 14;
    t = sample_native_pcurve_loops(surface, loops, opt);
    check(t.report["status"] == "incomplete" && t.loops.empty() && t.report["evaluations"] == 14,
          "evaluation budget spans all loops and failed attempts");
    opt = {};
    opt.max_loops = 2;
    check(sample_native_pcurve_loops(surface, loops, opt).report["status"] == "incomplete",
          "empty loops still count against loop budget");
    opt = {};
    opt.max_curves = 2;
    check(sample_native_pcurve_loops(surface, loops, opt).report["status"] == "incomplete",
          "curve count budget is shared across loops");
    check(sample_native_pcurve_loops(surface, {}).report["status"] == "complete",
          "empty prepared boundary list completes without inventing a loop");
    auto unprepared = BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                               {"order", 2},
                                               {"closed", false},
                                               {"poles", {0, 0, 0, 1, 0, 0}},
                                               {"knots", {2, 2, 4, 4}},
                                               {"weights", nullptr}});
    t = sample_native_pcurve_loops(surface, {{a, unprepared}});
    check(t.report["status"] == "incomplete" && t.loops.empty() && t.report["failed_curve"] == 1,
          "loop sampler refuses to silently normalize unprepared curve knots");
    auto closed = BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                           {"order", 2},
                                           {"closed", true},
                                           {"poles", {0, 0, 0, 1, 0, 0, 1, 1, 0}},
                                           {"knots", nullptr},
                                           {"weights", nullptr}});
    check(sample_native_pcurve_loops(surface, {{closed}}).report["status"] == "incomplete",
          "closed source curves require native opening before loop sampling");
    opt = {};
    opt.max_evaluations = 0;
    bool threw = false;
    try {
        sample_native_pcurve_loops(surface, loops, opt);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, "zero budget is an invalid option");
    opt = {};
    opt.uv_tolerance = std::numeric_limits<double>::infinity();
    threw = false;
    try {
        sample_native_pcurve_loops(surface, loops, opt);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, "nonfinite tolerance is an invalid option");
    auto disconnect = source;
    disconnect["poles"][0] = std::numeric_limits<double>::max();
    t = sample_native_pcurve_loops(BsplineSurface::from_bgfb(disconnect), {});
    check(t.report["status"] == "complete" && t.report["control_range"]["diagonal"] == 5 &&
              t.report["control_range"]["skipped_disconnect_poles"] == 1,
          "native Cartesian range skips the entire disconnect point");
    for (auto &p : disconnect["poles"])
        p = std::numeric_limits<double>::max();
    t = sample_native_pcurve_loops(BsplineSurface::from_bgfb(disconnect), {});
    check(t.report["status"] == "incomplete" && t.loops.empty(),
          "all-disconnect range cannot provide a finite automatic tolerance");
    const auto arch = BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                               {"order", 3},
                                               {"closed", false},
                                               {"poles", {0, 0, 0, .5, 1, 0, 1, 0, 0}},
                                               {"knots", nullptr},
                                               {"weights", nullptr}});
    opt = {};
    opt.uv_tolerance = opt.spatial_tolerance = 1e-20;
    t = sample_native_pcurve_loops(surface, {{a, arch}}, opt);
    check(t.report["status"] == "complete" && !t.report["sampled_tolerances_met"].get<bool>() &&
              t.report["continuous_error_bound"].is_null(),
          "loop completion preserves native cap failure to meet sampled tolerances");
    const auto used = t.report["evaluations"].get<unsigned>();
    check(used > t.report["sample_count"].get<unsigned>(),
          "loop evaluation accounting includes discarded adaptive retries");
    opt.max_evaluations = used - 1;
    t = sample_native_pcurve_loops(surface, {{a, arch}}, opt);
    check(t.report["status"] == "incomplete" && t.loops.empty() &&
              t.report["evaluations"] == used - 1 && t.report["failed_curve"] == 1,
          "adaptive retries consume the same global budget as earlier members");
    return checks;
}
