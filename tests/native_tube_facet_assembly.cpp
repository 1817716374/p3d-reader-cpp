#include "native_tube_facet_assembly.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
using Polygon = std::vector<Point2>;
const Polygon square{{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}};
TubeFacetSurface surface(unsigned order, double start, double end, bool weighted = false) {
    Json poles = Json::array(), weights = Json::array();
    for (unsigned v = 0; v < order; ++v)
        for (unsigned u = 0; u < 2; ++u) {
            const double w = weighted ? 2. : 1.;
            poles.push_back(double(u) * w);
            poles.push_back((start + (end - start) * double(v) / (order - 1)) * w);
            poles.push_back(0);
            if (weighted)
                weights.push_back(w);
        }
    return {{{"_type", "BsplineSurface"},
             {"orderU", 2},
             {"orderV", order},
             {"numPolesU", 2},
             {"numPolesV", order},
             {"closedU", false},
             {"closedV", false},
             {"knotsU", nullptr},
             {"knotsV", nullptr},
             {"weights", weighted ? weights : Json()},
             {"poles", poles},
             {"boundaries", nullptr},
             {"holeOrigin", 0},
             {"numRulesU", 9},
             {"numRulesV", 11}},
            {}};
}
TubeFacetGroupGeneration generation(std::vector<TubeFacetSurface> surfaces) {
    TubeFacetGroupGeneration out;
    out.success = true;
    out.groups = {{{0, std::move(surfaces)}}};
    return out;
}
TubeFacetPathClassification classes(std::vector<std::vector<std::int32_t>> indices) {
    TubeFacetPathClassification out;
    out.success = true;
    out.groups = std::move(indices);
    return out;
}
TubeFacetGroupAssembly assemble(TubeFacetGroupGeneration source,
                                std::vector<std::vector<std::int32_t>> indices) {
    TubeBudget b;
    return assemble_tube_facet_groups(std::move(source), classes(std::move(indices)), b);
}
Json line(Point3 a, Point3 z) {
    Json s;
    for (unsigned i = 0; i < 3; ++i) {
        s[std::string("point0") + "XYZ"[i]] = a[i];
        s[std::string("point1") + "XYZ"[i]] = z[i];
    }
    return {{"_type", "LineSegment"}, {"segment", s}};
}
Json group(const std::vector<Json> &curves, unsigned type = 1) {
    Json values = Json::array();
    for (const auto &c : curves)
        values.push_back({{"geometry", c}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", values}};
}
} // namespace
unsigned native_tube_facet_assembly_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *message) {
        require(ok, message);
        ++n;
    };
    auto splice = [&](const Polygon &a, const Polygon &z) {
        TubeBudget b;
        return splice_tube_facet_boundaries(a, z, b);
    };
    check(splice(square, square) == square, "two square boundaries preserve the outer rectangle");
    const Polygon a{{2, .2}, {3, .4}, {4, .6}, {100, 100}, {101, 101}, {999, 999}};
    const Polygon z{{-100, -100}, {-101, -101}, {5, .2}, {6, .4}, {7, .6}, {-999, -999}};
    const Polygon expected{{2, .1}, {3, .2}, {4, .3}, {5, .6}, {6, .7}, {7, .8}, {2, .1}};
    check(splice(a, z) == expected, "asymmetric positional splice drops specific source ends");
    check(splice({}, z) == Polygon{{0, 0}, {1, 0}, {5, .6}, {6, .7}, {7, .8}, {0, 0}},
          "missing first trim inserts only lower unit corners");
    check(splice(a, {}) == Polygon{{2, .1}, {3, .2}, {4, .3}, {1, 1}, {0, 1}, {2, .1}},
          "missing second trim inserts upper corners and closes to transformed first point");
    check(splice({}, {}).empty(), "two absent trims yield no synthesized polygon");
    for (unsigned count = 1; count < 5; ++count) {
        const Polygon short_side(count, Point2{0, 0});
        check(splice(short_side, square).empty() && splice(square, short_side).empty() &&
                  splice({}, short_side).empty() && splice(short_side, {}).empty(),
              "each nonempty side needs five entries even when the other side is absent");
    }
    auto outside = square;
    outside[0] = {-2, -8};
    check(splice(outside, square).front() == Point2{-2, -4},
          "splice does not clamp source coordinates to the unit domain");
    auto untrimmed = generation({surface(2, 0, 1), surface(2, 1, 2), surface(2, 2, 3)});
    const auto saved = untrimmed.groups[0][0].surfaces[0].geometry;
    auto joined = assemble(untrimmed, {{0, 1, 2}});
    check(joined.success && joined.groups[0][0] == std::vector<std::size_t>{0},
          "same path group and V-order class combine in the first native object");
    const auto &result = joined.generation.groups[0][0].surfaces[0];
    check(result.geometry["numPolesV"] == 4 && result.geometry["numRulesU"] == 4 &&
              result.geometry["numRulesV"] == 2 && result.boundaries.empty(),
          "sequential combination updates control/rule counts without inventing boundaries");
    check(result.geometry["knotsV"] == Json{0, 0, .25, .5, 1, 1},
          "three-piece native sequential combination does not assign equal final thirds");
    check(untrimmed.groups[0][0].surfaces[0].geometry == saved,
          "assembly mutates only its private source working copies");
    auto distinct = assemble(untrimmed, {{0}, {1, 2}});
    check(distinct.success && distinct.groups[0][0] == std::vector<std::size_t>({0, 1}) &&
              distinct.generation.groups[0][0].surfaces[0].geometry == saved,
          "classification boundaries prevent merging geometrically compatible neighbors");
    auto mixed = assemble(generation({surface(2, 0, 1), surface(3, 1, 2), surface(3, 2, 3),
                                      surface(2, 3, 4), surface(2, 4, 5)}),
                          {{0, 1, 2, 3, 4}});
    check(mixed.success && mixed.groups[0][0] == std::vector<std::size_t>({0, 1, 3}) &&
              mixed.generation.groups[0][0].surfaces[1].geometry["numPolesV"] == 5 &&
              mixed.generation.groups[0][0].surfaces[3].geometry["numPolesV"] == 3,
          "both transitions of the V-order-two predicate create separate native runs");
    auto first = surface(2, 0, 1), second = surface(2, 1, 2);
    first.boundaries = {a};
    second.boundaries = {z};
    auto trimmed = assemble(generation({first, second}), {{0, 1}});
    check(trimmed.success &&
              trimmed.generation.groups[0][0].surfaces[0].boundaries ==
                  std::vector<Polygon>{expected} &&
              trimmed.generation.groups[0][0].surfaces[0].geometry["holeOrigin"] == 1,
          "combination uses original first UV caches then writes one active rebuilt boundary");
    check(trimmed.generation.groups[0][0].surfaces[1].boundaries == second.boundaries &&
              trimmed.generation.groups[0][0].surfaces[0].pcurves.empty(),
          "existing incoming trim remains intact while rebuilt output has no pcurves");
    second.boundaries.clear();
    auto only_first = assemble(generation({first, second}), {{0, 1}});
    const auto &incoming = only_first.generation.groups[0][0].surfaces[1];
    check(
        only_first.success && incoming.geometry["holeOrigin"] == 1 &&
            incoming.pcurves.size() == 1 && incoming.pcurves[0].size() == 4 &&
            incoming.boundaries[0] == Polygon{{0, 0},
                                              {.25, 0},
                                              {.5, 0},
                                              {.75, 0},
                                              {1, 0},
                                              {1, .25},
                                              {1, .5},
                                              {1, .75},
                                              {1, 1},
                                              {.75, 1},
                                              {.5, 1},
                                              {.25, 1},
                                              {0, 1},
                                              {0, .75},
                                              {0, .5},
                                              {0, .25},
                                              {0, 0}},
        "native helper initializes the untrimmed incoming source with four unit pcurves and cache");
    check(only_first.generation.groups[0][0].surfaces[0].boundaries[0] == splice(a, {}),
          "new incoming unit trim does not replace the pre-combination empty splice input");
    first.boundaries.clear();
    second.boundaries = {z};
    auto only_second = assemble(generation({first, second}), {{0, 1}});
    check(
        only_second.success &&
            only_second.generation.groups[0][0].surfaces[0].boundaries[0] == splice({}, z),
        "untrimmed current gets initialized but the splice still uses the saved empty first cache");
    auto weighted =
        assemble(generation({surface(3, 0, 1, true), surface(3, 1, 2, true)}), {{0, 1}});
    const auto &w = weighted.generation.groups[0][0].surfaces[0].geometry;
    check(weighted.success && w["weights"] == Json(std::vector<double>(10, 2)) &&
              w["numPolesV"] == 5 && w["poles"][25] == 4,
          "higher-order rational combination preserves weighted source controls");
    auto change = surface(3, 1, 2);
    change.boundaries = {z};
    first.boundaries = {a};
    auto split_trim = assemble(generation({first, change}), {{0, 1}});
    check(split_trim.success && split_trim.groups[0][0] == std::vector<std::size_t>({0, 1}) &&
              split_trim.generation.groups[0][0].surfaces[0].boundaries[0] == a &&
              split_trim.generation.groups[0][0].surfaces[1].boundaries[0] == expected &&
              split_trim.generation.groups[0][0].surfaces[1].geometry["numPolesV"] == 3,
          "order transition writes the splice to incoming object even though geometry was not "
          "combined");
    change.boundaries.clear();
    auto split_no_trim = assemble(generation({first, change}), {{0, 1}});
    check(split_no_trim.success &&
              split_no_trim.generation.groups[0][0].surfaces[1].boundaries.empty(),
          "transition to untrimmed input skips boundary reconstruction and trim initialization");
    auto too_short = first;
    too_short.boundaries = {{{0, 0}, {1, 0}, {1, 1}, {0, 1}}};
    auto fail = assemble(generation({too_short, second}), {{0, 1}});
    check(!fail.success && fail.groups.empty() &&
              fail.generation.groups[0][0].surfaces[0].boundaries.empty() &&
              fail.generation.groups[0][0].surfaces[0].geometry["numPolesV"] == 3,
          "splice failure happens after geometry combination and clearing the target trims");
    auto empty_record = first;
    empty_record.boundaries = {Polygon{}};
    auto empty_fail = assemble(generation({empty_record, empty_record}), {{0, 1}});
    check(!empty_fail.success &&
              empty_fail.generation.groups[0][0].surfaces[0].boundaries.size() == 2,
          "both empty first caches fail before clearing temporary combined trim records");
    auto multiple = generation({surface(2, 0, 1), surface(2, 1, 2)});
    multiple.groups.push_back({{0, {surface(2, 0, 1)}}});
    auto mismatch = assemble(multiple, {{0, 1}});
    check(!mismatch.success && mismatch.groups.size() == 1 &&
              mismatch.groups[0][0] == std::vector<std::size_t>{0},
          "later count mismatch retains previously published profile groups");
    auto failed_member = generation({surface(2, 0, 1), surface(2, 1, 2)});
    failed_member.groups[0].push_back({1, {surface(2, 0, 1)}});
    check(assemble(failed_member, {{0, 1}}).groups.empty(),
          "failure in a later member discards the entire unfinished profile group");
    auto empty_group = generation({surface(2, 0, 1)});
    empty_group.groups.push_back({});
    auto kept_empty = assemble(empty_group, {{0}});
    check(kept_empty.success && kept_empty.groups.size() == 2 && kept_empty.groups[1].empty(),
          "empty profile groups remain explicit outputs");
    bool caught = false;
    try {
        TubeBudget b;
        b.max_work = 0;
        assemble_tube_facet_groups(untrimmed, classes({{0, 1, 2}}), b);
    } catch (const std::exception &) {
        caught = true;
    }
    check(caught, "assembly budget exhaustion is an exception rather than native failure");
    auto future = std::async(std::launch::async, [&] { return assemble(untrimmed, {{0, 1, 2}}); });
    auto local = assemble(untrimmed, {{0, 1, 2}}), concurrent = future.get();
    check(local.groups == concurrent.groups && local.report == concurrent.report &&
              local.generation.groups[0][0].surfaces[0].geometry ==
                  concurrent.generation.groups[0][0].surfaces[0].geometry,
          "assembly does not share mutable state across parser threads");
    const auto profile = group({line({-1, -1, 0}, {1, -1, 0}), line({1, -1, 0}, {1, 1, 0}),
                                line({1, 1, 0}, {-1, 1, 0}), line({-1, 1, 0}, {-1, -1, 0})},
                               2);
    for (const auto &path : {group({line({0, 0, 0}, {0, 0, 4})}),
                             group({line({0, 0, 0}, {0, 0, 2}), line({0, 0, 2}, {0, 0, 4})}),
                             group({line({0, 0, 0}, {0, 0, 4}), line({0, 0, 4}, {2, 0, 4})})}) {
        TubeBudget b;
        auto actual = assemble_tube_facet_groups(generate_tube_facet_groups(profile, path, b), b);
        check(actual.success && actual.groups.size() == 1 && actual.groups[0].size() == 4,
              "real source preparation/generation/classification reaches surface assembly");
        for (std::size_t m = 0; m < 4; ++m)
            check(actual.groups[0][m].size() == actual.classification.groups.size(),
                  "actual member output preserves native path grouping");
    }
    const Json arc{{"_type", "EllipticArc"},
                   {"arc",
                    {{"centerX", -10},
                     {"centerY", 0},
                     {"centerZ", 0},
                     {"vector0X", 10},
                     {"vector0Y", 0},
                     {"vector0Z", 0},
                     {"vector90X", 0},
                     {"vector90Y", 0},
                     {"vector90Z", 10},
                     {"startRadians", 0},
                     {"sweepRadians", 3.141592653589793}}}};
    TubeBudget curved_budget;
    auto curved = assemble_tube_facet_groups(
        generate_tube_facet_groups(profile, group({arc}), curved_budget), curved_budget);
    check(curved.success &&
              curved.classification.groups == std::vector<std::vector<std::int32_t>>{{0, 1}},
          "actual semicircle converts to two native path patches in one assembly class");
    for (std::size_t m = 0; m < 4; ++m) {
        const auto &s = curved.generation.groups[0][m].surfaces[curved.groups[0][m][0]];
        check(curved.groups[0][m].size() == 1 && s.geometry["orderV"] == 3 &&
                  s.geometry["numPolesV"] == 5 && !s.geometry["weights"].is_null(),
              "actual curved path assembly retains a rational quadratic two-span surface");
    }
    return n;
}
