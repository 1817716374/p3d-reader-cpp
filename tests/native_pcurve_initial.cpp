#include "internal.hpp"
#include "p3d/pcurve.hpp"
using namespace p3d;
namespace {
Json ring(Json primitive) {
    return {{"_type", "CurveVector"},
            {"type", 2},
            {"curves", Json::array({Json{{"geometry", std::move(primitive)}}})}};
}
Json line(double a, double b) {
    return {{"_type", "LineString"}, {"points", {a, .5, 0, b, .5, 0}}};
}
Json plane(Json boundary, double a = 0, double b = 1) {
    return {{"_type", "BsplineSurface"},
            {"numPolesU", 2},
            {"numPolesV", 2},
            {"orderU", 2},
            {"orderV", 2},
            {"closedU", false},
            {"closedV", false},
            {"poles", {0, 0, 0, 3, 0, 0, 0, 4, 0, 3, 4, 0}},
            {"weights", nullptr},
            {"knotsU", {a, a, b, b}},
            {"knotsV", {0, 0, 1, 1}},
            {"numRulesU", 0},
            {"numRulesV", 0},
            {"holeOrigin", -2},
            {"boundaries", std::move(boundary)}};
}
} // namespace
unsigned native_pcurve_initial_tests() {
    unsigned checks = 0;
    auto check = [&](bool valid, const char *message) {
        ++checks;
        require(valid, message);
    };
    for (auto primitive : {line(.25, .75), Json{{"_type", "BsplineCurve"},
                                                {"order", 3},
                                                {"closed", false},
                                                {"poles", {.1, .2, 0, .5, .8, 0, .9, .2, 0}},
                                                {"knots", nullptr},
                                                {"weights", nullptr}}}) {
        const auto source = BsplineSurface::from_bgfb(plane(ring(primitive)));
        const auto initial = sample_native_initial_surface_boundaries(source);
        const auto normalized = sample_native_surface_boundaries(source);
        check(initial.report["status"] == "complete" &&
                  initial.loops[0].size() == normalized.loops[0].size(),
              "unit-domain initial and normalized boundary paths have matching sample counts");
        for (std::size_t i = 0; i < initial.loops[0].size(); ++i)
            check(initial.loops[0][i].parameter == normalized.loops[0][i].parameter &&
                      initial.loops[0][i].position == normalized.loops[0][i].position,
                  "unit-domain initial and normalized samples agree exactly");
        check(initial.report["algorithm"] == "native_initial_surface_boundary_cache" &&
                  initial.report["parameter_coordinates"] == "initial_cache_uv" &&
                  initial.report["surface_knot_preparation"] == "source_knots" &&
                  initial.report["outer_boundary_active"] == source.outer_boundary_active(),
              "initial cache reports its coordinate/preparation contract and source outer flag");
    }
    auto table = plane(ring(line(.25, 1.5)), 0, 2);
    const auto original = table;
    auto source = BsplineSurface::from_bgfb(table);
    auto r = sample_native_initial_surface_boundaries(source);
    auto normalized = sample_native_surface_boundaries(source);
    check(r.report["status"] == "complete" && r.loops.size() == 1 && r.loops[0].size() == 5,
          "initial cache retains source domain and applies native linear-UV clamping");
    for (unsigned i = 0; i < r.loops[0].size(); ++i) {
        const double u = .25 + .75 * i / 4;
        check(std::abs(r.loops[0][i].parameter[0] - u) < 1e-14 &&
                  std::abs(r.loops[0][i].position[0] - 1.5 * u) < 1e-14 &&
                  r.loops[0][i].position[1] == 2,
              "nonunit initial linear cache matches an independent surface polynomial");
    }
    check(normalized.loops[0].back().parameter[0] == .75 &&
              normalized.loops[0].back().position[0] == 2.25 &&
              r.loops[0].back().position[0] == 1.5,
          "initial source cache is not silently replaced by normalized restroking");
    check(table == original && source.u().knot_domain() == Point2{0, 2} &&
              source.boundaries() == original["boundaries"],
          "initial cache leaves source coordinates and knots unchanged");
    check(r.report["surface_split_knots"]["u"] == Json::array({0, 2}) &&
              r.report["boundary_coordinate_preparation"] == "source_uv_unchanged",
          "initial knot split plan uses raw source knots");

    table = plane(ring(line(.25, .75)), -1, 1);
    r = sample_native_initial_surface_boundaries(BsplineSurface::from_bgfb(table));
    check(r.report["status"] == "complete", "nonzero lower knot can use the initial native kernel");
    for (const auto &sample : r.loops[0])
        check(std::abs(sample.position[0] - 3 * sample.parameter[0]) < 1e-14,
              "initial surface kernel uses (1-u)*firstKnot+u, not source-knot point evaluation");
    table = plane(ring(line(.25, .75)), 2, 6);
    r = sample_native_initial_surface_boundaries(BsplineSurface::from_bgfb(table));
    check(r.report["status"] == "incomplete" && r.loops.empty(),
          "invalid native initial knot windows fail instead of normalizing to a different surface");

    table = plane(ring(line(.2, 1.8)), 0, 2);
    table["numPolesU"] = 3;
    table["knotsU"] = {0, 0, .375, 2, 2};
    table["poles"] = {0, 0, 0, .5625, 0, 0, 3, 0, 0, 0, 4, 0, .5625, 4, 0, 3, 4, 0};
    r = sample_native_initial_surface_boundaries(BsplineSurface::from_bgfb(table));
    check(r.report["status"] == "complete" &&
              r.report["surface_split_knots"]["u"] == Json::array({0, .375, 2}),
          "source internal knot is not rescaled during initial boundary splitting");
    bool has_split = false;
    for (const auto &sample : r.loops[0]) {
        has_split = has_split || sample.parameter[0] == .375;
        check(std::abs(sample.position[0] - 1.5 * sample.parameter[0]) < 1e-14,
              "raw-knot split samples follow independent piecewise-linear surface");
    }
    check(has_split, "raw source knot creates an explicit boundary segment endpoint");

    Json high = {{"_type", "BsplineCurve"}, {"order", 3},
                 {"closed", false},         {"poles", {.25, .5, 0, 1.5, .5, 0, 1.75, .5, 0}},
                 {"weights", nullptr},      {"knots", nullptr}};
    r = sample_native_initial_surface_boundaries(
        BsplineSurface::from_bgfb(plane(ring(high), 0, 2)));
    check(r.report["status"] == "complete" && r.loops[0].back().parameter[0] == 1.75 &&
              r.loops[0].back().position[0] == 1.5,
          "high-order initial UV remains unmodified while native surface point is clamped");
    for (Json empty :
         {Json(nullptr), Json{{"_type", "CurveVector"}, {"type", 2}, {"curves", Json::array()}}}) {
        table = plane(empty);
        table["weights"] = {0, 0, 0, 0};
        r = sample_native_initial_surface_boundaries(BsplineSurface::from_bgfb(table));
        check(r.report["status"] == "complete" && r.loops.empty() &&
                  r.report["sampling_performed"] == false && r.report["evaluations"] == 0,
              "reader with absent or empty boundaries does not evaluate unusable surface weights");
    }
    PCurveBoundaryStrokeOptions limited;
    limited.sampling.max_points = 3;
    r = sample_native_initial_surface_boundaries(BsplineSurface::from_bgfb(plane(ring(line(0, 1)))),
                                                 limited);
    check(r.report["status"] == "incomplete" && r.loops.empty() &&
              r.report["boundary_sources"].size() == 1,
          "initial cache shares global sampling budgets and source failure provenance");
    return checks;
}
