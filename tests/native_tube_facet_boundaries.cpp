#include "native_tube_facet_boundaries.hpp"
#include "native_tube_facet_seams.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineCurve curve(Json poles, unsigned order = 2, Json knots = nullptr, Json weights = nullptr,
                   bool closed = false) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"closed", closed},
                                    {"poles", poles},
                                    {"knots", knots},
                                    {"weights", weights}});
}
TubeFacetComposition prepared(const BsplineCurve *prefix, const BsplineCurve *suffix) {
    TubeFacetComposition c;
    c.prepared = true;
    c.report = {{"seams_applied", true}};
    if (prefix)
        c.working_prefix_path = *prefix;
    if (suffix)
        c.working_suffix_path = *suffix;
    return c;
}
} // namespace
unsigned native_tube_facet_boundaries_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto f, const char *why) {
        bool failed = false;
        try {
            f();
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, why);
    };
    auto compose = [](std::vector<Point3> &out, const TubeFacetBoundarySegments &a,
                      TubeFacetBoundarySegments &b) {
        TubeBudget budget;
        return compose_tube_facet_boundary(out, a, b, budget);
    };
    TubeFacetBoundarySegments empty;
    std::vector<Point3> polygon{{7, 8, 9}};
    auto noop = compose(polygon, empty, empty);
    check(polygon == std::vector<Point3>{{7, 8, 9}} && noop["output_unchanged"] == true,
          "both missing sides keep prior polygon instead of inventing a unit rectangle");
    TubeFacetBoundarySegments lower{{{0, .2, 7}, {.4, .3, 8}}, {{.4, .3, 9}, {1, .2, 10}}};
    const auto lower_original = lower;
    auto lower_only = compose(polygon, lower, empty);
    check(
        polygon ==
            std::vector<Point3>{
                {0, .2, 7}, {.4, .3, 9}, {1, .2, 10}, {1, 1, 0}, {0, 1, 0}, {0, .2, 7}},
        "first side replaces each intermediate endpoint with the next segment start positionally");
    check(lower == lower_original && lower_only["second_segments_reversed"] == false,
          "forward source segments remain unchanged");
    TubeFacetBoundarySegments upper{{{0, .7, 1}, {.3, .8, 2}}, {{.4, .9, 3}, {1, .7, 4}}};
    const auto upper_original = upper;
    auto both = compose(polygon, lower, upper);
    const std::vector<Point3> golden{{0, .2, 7},  {.4, .3, 9}, {1, .2, 10}, {1, .7, 4},
                                     {.3, .8, 2}, {0, .7, 1},  {0, .2, 7}};
    check(polygon == golden,
          "second side traverses reversed segments backwards without gap repair");
    check(upper ==
                  TubeFacetBoundarySegments{{{.3, .8, 2}, {0, .7, 1}}, {{1, .7, 4}, {.4, .9, 3}}} &&
              both["second_segments_reversed"] == true,
          "second input point order mutates but its segment order is unchanged");
    compose(polygon, lower, upper);
    check(upper == upper_original && polygon != golden,
          "repeated native composition reverses the carried second side again");
    upper = upper_original;
    compose(polygon, empty, upper);
    check(polygon ==
              std::vector<Point3>{
                  {0, 0, 0}, {1, 0, 0}, {1, .7, 4}, {.3, .8, 2}, {0, .7, 1}, {0, 0, 0}},
          "missing first side contributes raw zero-one bottom corners");
    auto shared = lower_original;
    auto shared_result = compose(polygon, shared, shared);
    check(polygon == std::vector<Point3>{{0, .2, 7},
                                         {.4, .3, 9},
                                         {1, .2, 10},
                                         {1, .2, 10},
                                         {.4, .3, 8},
                                         {0, .2, 7},
                                         {0, .2, 7}} &&
              shared_result["shared_segment_list"] == true,
          "same segment list is read forwards before in-place reversal and repeated closure is "
          "retained");
    TubeFacetBoundarySegments singletons{{{5, 6, 7}}, {{8, 9, 10}}};
    compose(polygon, singletons, empty);
    check(polygon == std::vector<Point3>{{8, 9, 10}, {1, 1, 0}, {0, 1, 0}, {8, 9, 10}},
          "singleton segments follow native endpoint omission rather than deduplication");
    auto malformed = TubeFacetBoundarySegments{{}};
    const auto before_polygon = polygon;
    rejects([&] { compose(polygon, malformed, empty); },
            "empty inner segment native underflow is rejected");
    check(polygon == before_polygon, "invalid boundary segment does not discard the prior polygon");
    rejects([&] { compose(lower[0], lower, empty); },
            "output-input segment alias invalidates native iteration");
    auto upper_keep = upper_original;
    TubeBudget limit;
    limit.max_control_points = 6;
    rejects([&] { compose_tube_facet_boundary(polygon, lower, upper_keep, limit); },
            "combined boundary output size includes closing point");
    check(upper_keep == upper_original && polygon == before_polygon,
          "budget rejection occurs before segment reversal or output replacement");
    std::vector<std::vector<Point2>> uv{{{9, 9}, {8, 8}}};
    TubeBudget b;
    check(!append_tube_facet_uv_boundary(uv, {}, b) && uv.size() == 1,
          "empty native polygon append returns false and preserves prior boundaries");
    check(append_tube_facet_uv_boundary(uv, {{2, 3, 7}, {4, 5, -9}, {2, 3, 8}}, b) &&
              uv == std::vector<std::vector<Point2>>{{{9, 9}, {8, 8}}, {{2, 3}, {4, 5}, {2, 3}}},
          "native boundary appends raw XY without normalization or Z retention");
    check(append_tube_facet_uv_boundary(uv, {{7, 8, 99}}, b) &&
              uv.back() == std::vector<Point2>{{7, 8}},
          "native boundary storage accepts a single point without inventing closure");
    const auto uv_before = uv;
    TubeBudget uv_limit;
    uv_limit.max_control_points = 6;
    rejects([&] { append_tube_facet_uv_boundary(uv, {{1, 1, 0}}, uv_limit); },
            "UV boundary budget counts previously stored polygons");
    check(uv == uv_before, "UV append failure leaves existing boundaries untouched");
    const auto prefix = curve({0, 0, 0, -2, 0, 0});
    const auto suffix = curve({0, 0, 0, 0, 3, 0});
    const auto section = curve({0, 0, 0, .2, 0, 0});
    TubeBudget integration;
    auto chain =
        prepare_tube_facet_composition(&prefix, &section, &suffix, &section, false, integration);
    rejects([&] { prepare_tube_facet_trim_path(chain, integration); },
            "trim path cannot precede actual seam processing");
    check(process_tube_facet_seams(chain, integration).status == TubeFacetSeamStatus::complete,
          "generated branch composition completes seam pass before path assembly");
    const auto report_before = chain.report;
    const auto path = prepare_tube_facet_trim_path(chain, integration);
    check(path.curve.poles() == std::vector<Point3>{{-2, 0, 0}, {0, 0, 0}, {0, 3, 0}} &&
              path.curve.knots() == std::vector<double>{0, 0, .5, 1, 1},
          "assembled path reverses prefix and gives unequal length branches equal knot intervals");
    check(chain.report == report_before && chain.working_prefix_path->poles() == prefix.poles() &&
              path.report["chain_finalized"] == false,
          "trim path preparation leaves branch working curves and finalization state unchanged");
    const auto gap = curve({7, 0, 0, 10, 0, 0});
    TubeBudget gap_budget;
    const auto gap_path = prepare_tube_facet_trim_path(prepared(&prefix, &gap), gap_budget);
    check(gap_path.curve.poles().size() == 4 &&
              gap_path.curve.knots() == std::vector<double>{0, 0, .5, .5, 1, 1},
          "path assembly preserves discontinuous branches without forced joining");
    const auto cubic = curve({0, 0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0}, 4);
    TubeBudget order_budget;
    const auto raised = prepare_tube_facet_trim_path(prepared(&prefix, &cubic), order_budget);
    check(raised.curve.order() == 4 && raised.curve.poles().size() == 7,
          "mixed path orders use native opening and degree elevation wrapper");
    const auto nonunit = curve({0, 0, 0, -2, 0, 0}, 2, {2, 2, 7, 7});
    TubeBudget prefix_budget;
    auto sole_prefix = prepare_tube_facet_trim_path(prepared(&nonunit, nullptr), prefix_budget);
    check(sole_prefix.curve.knots() == std::vector<double>{0, 0, 1, 1} &&
              sole_prefix.curve.poles().front() == Point3{-2, 0, 0},
          "sole prefix applies native reversal and knot normalization");
    const auto zero_weight = curve({0, 0, 0, 1, 0, 0}, 2, {2, 2, 7, 7}, {0, 1});
    TubeBudget copy_budget;
    auto copied = prepare_tube_facet_trim_path(prepared(nullptr, &zero_weight), copy_budget);
    check(copied.curve.knots() == zero_weight.knots() &&
              copied.curve.weights() == zero_weight.weights(),
          "sole suffix copies raw domain and zero weights without geometric queries");
    TubeBudget no_path;
    rejects([&] { prepare_tube_facet_trim_path(prepared(nullptr, nullptr), no_path); },
            "absent source paths are not fabricated from facet geometry");
    TubeBudget no_work;
    no_work.max_work = 0;
    rejects([&] { prepare_tube_facet_trim_path(prepared(&prefix, &suffix), no_work); },
            "path preparation obeys the shared work budget");
    auto worker = [&] {
        auto second = upper_original;
        std::vector<Point3> out;
        compose(out, lower_original, second);
        return out;
    };
    auto f = std::async(std::launch::async, worker), g = std::async(std::launch::async, worker);
    check(f.get() == golden && g.get() == golden,
          "independent concurrent boundary work has no shared scratch state");
    return n;
}
