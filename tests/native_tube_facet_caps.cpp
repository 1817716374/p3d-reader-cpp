#include "native_tube_facet_caps.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
TubeFacetSurface wall(Point3 a, Point3 z, bool rational = false) {
    Json poles = Json::array(), weights = Json::array();
    for (double h : {0., 4.})
        for (const auto &p : {a, z}) {
            const double w = rational ? 2. : 1.;
            poles.push_back(p[0] * w);
            poles.push_back(p[1] * w);
            poles.push_back((p[2] + h) * w);
            if (rational)
                weights.push_back(w);
        }
    return {{{"_type", "BsplineSurface"},
             {"numPolesU", 2},
             {"numPolesV", 2},
             {"orderU", 2},
             {"orderV", 2},
             {"closedU", false},
             {"closedV", false},
             {"knotsU", {2, 2, 5, 5}},
             {"knotsV", {7, 7, 11, 11}},
             {"poles", poles},
             {"weights", rational ? weights : Json()},
             {"holeOrigin", 0},
             {"numRulesU", 2},
             {"numRulesV", 2},
             {"boundaries", nullptr}},
            {}};
}
std::vector<TubeFacetSurface> walls(double lo = -1, double hi = 1, bool rational = false) {
    const Point3 p[]{{lo, lo, 0}, {hi, lo, 0}, {hi, hi, 0}, {lo, hi, 0}, {lo, lo, 0}};
    std::vector<TubeFacetSurface> out;
    for (unsigned i = 0; i < 4; ++i)
        out.push_back(wall(p[i], p[i + 1], rational));
    return out;
}
TubeFacetCapPair pair(const std::vector<TubeFacetSurface> &a,
                      const std::vector<TubeFacetSurface> &z) {
    TubeFacetCapPair p;
    for (const auto &s : a)
        p.first.push_back(&s);
    for (const auto &s : z)
        p.last.push_back(&s);
    return p;
}
TubeCaps caps(const std::vector<TubeFacetCapPair> &pairs) {
    TubeBudget b;
    return tube_facet_cap_regions(pairs, b);
}
Json line(Point3 a, Point3 z) {
    Json s;
    for (unsigned i = 0; i < 3; ++i) {
        s[std::string("point0") + "XYZ"[i]] = a[i];
        s[std::string("point1") + "XYZ"[i]] = z[i];
    }
    return {{"_type", "LineSegment"}, {"segment", s}};
}
Json group(std::vector<Json> values, unsigned type = 1) {
    Json entries = Json::array();
    for (auto &value : values)
        entries.push_back({{"geometry", std::move(value)}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", entries}};
}
Json rectangle(double lo = -1, double hi = 1) {
    return group({line({lo, lo, 0}, {hi, lo, 0}), line({hi, lo, 0}, {hi, hi, 0}),
                  line({hi, hi, 0}, {lo, hi, 0}), line({lo, hi, 0}, {lo, lo, 0})},
                 2);
}
} // namespace
unsigned native_tube_facet_caps_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        require(ok, why);
        ++n;
    };
    auto sides = walls();
    auto single = caps({pair(sides, sides)});
    check(
        single.report["native_result"] && single.start["type"] == 2 && single.end["type"] == 2 &&
            single.start["curves"].size() == 4 && single.end["curves"].size() == 4,
        "single cap contains every source member rather than requiring each isocurve to be closed");
    check(single.start["curves"][0]["geometry"]["poles"] == Json{-1, -1, 0, -1, 1, 0} &&
              single.end["curves"][0]["geometry"]["poles"] == Json{-1, -1, 4, 1, -1, 4},
          "start reverses member order and each direction while end retains source order");
    check(single.start["curves"][0]["geometry"]["knots"] == Json{0, 0, 1, 1} &&
              single.end["curves"][0]["geometry"]["knots"] == Json{2, 2, 5, 5},
          "native reversal normalizes start knots but untouched end keeps the source U domain");
    auto displaced = sides;
    for (auto &s : displaced)
        for (std::size_t i = 2; i < s.geometry["poles"].size(); i += 3)
            s.geometry["poles"][i] = s.geometry["poles"][i].get<double>() + 10;
    auto different_ends = caps({pair(sides, displaced)});
    check(different_ends.report["native_result"] &&
              different_ends.end["curves"][0]["geometry"]["poles"][2] == 14,
          "end cap uses the independently selected last surface of each member");
    auto inner = walls(-.25, .25);
    auto multi = caps({pair(sides, sides), pair(inner, inner)});
    check(multi.report["native_result"] && multi.start["type"] == 4 && multi.end["type"] == 4 &&
              multi.start["curves"][0]["geometry"]["type"] == 2 &&
              multi.start["curves"][1]["geometry"]["type"] == 3,
          "multiple profile groups create parity caps with positional outer and inner flags");
    check(multi.start["curves"][0]["geometry"] == single.start &&
              multi.start["curves"][1]["geometry"]["curves"][0]["geometry"]["poles"][0] == -.25,
          "parity reversal preserves ring order and reverses members inside each ring");
    auto reordered = caps({pair(inner, inner), pair(sides, sides)});
    check(reordered.report["native_result"] &&
              reordered.start["curves"][0]["geometry"]["type"] == 2 &&
              reordered.start["curves"][0]["geometry"]["curves"][0]["geometry"]["poles"][0] == -.25,
          "native cap labels are not corrected by inferred geometric containment");
    auto broken = sides;
    broken[3].geometry["poles"][3] = -.5;
    auto failed = caps({pair(broken, sides)});
    check(!failed.report["native_result"].get<bool>() && failed.start["curves"].size() == 4 &&
              failed.start["curves"][0]["geometry"]["poles"] == Json{-1, -1, 0, 1, -1, 0} &&
              !failed.report["start_reversed"].get<bool>(),
          "single ring closure failure retains both already built unreversed regions");
    auto partial = caps({pair(sides, sides), pair(broken, sides)});
    check(!partial.report["native_result"].get<bool>() && partial.start["type"] == 4 &&
              partial.start["curves"].size() == 1 && partial.report["completed_rings"] == 1 &&
              partial.start["curves"][0]["geometry"]["curves"][0]["geometry"]["poles"] ==
                  Json{-1, -1, 0, 1, -1, 0},
          "multi-ring failure retains previous child rings but not the failing child or reversal");
    auto gap = sides;
    gap[1] = wall({8, 8, 0}, {9, 8, 0});
    check(caps({pair(gap, gap)}).report["native_result"],
          "native closure compares outer endpoints without checking interior member connectivity");
    const std::vector<TubeFacetSurface> empty;
    auto mismatch = caps({pair(sides, empty)}), no_groups = caps({}),
         empty_ring = caps({pair(empty, empty)});
    check(!mismatch.report["native_result"].get<bool>() && mismatch.start.is_null() &&
              mismatch.end.is_null(),
          "single unequal counts fail before creating region objects");
    check(!no_groups.report["native_result"].get<bool>() && no_groups.start.is_null(),
          "empty pair list returns native false");
    check(!empty_ring.report["native_result"].get<bool>() && empty_ring.start["type"] == 2 &&
              empty_ring.start["curves"].empty(),
          "empty single ring fails after allocating empty regions");
    auto weighted = walls(-1, 1, true);
    auto rational = caps({pair(weighted, weighted)});
    check(rational.report["native_result"] &&
              rational.start["curves"][0]["geometry"]["weights"] == Json{2, 2} &&
              rational.end["curves"][0]["geometry"]["poles"][2] == 8,
          "rational endpoint isocurves preserve weighted controls and weights through reversal");
    auto trimmed = sides;
    trimmed[0].boundaries = {{{.2, .3}, {.4, .5}, {.6, .7}}};
    auto ignored = caps({pair(trimmed, trimmed)});
    check(ignored.start == single.start && ignored.end == single.end,
          "cap isocurves ignore existing UV clipping instead of trimming the source edge");
    auto periodic = wall({0, 0, 0}, {1, 0, 0});
    periodic.geometry["numPolesU"] = 4;
    periodic.geometry["closedU"] = true;
    periodic.geometry["knotsU"] = nullptr;
    periodic.geometry["poles"] = {0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0,
                                  0, 0, 4, 1, 0, 4, 1, 1, 4, 0, 1, 4};
    std::vector<TubeFacetSurface> periodic_list{periodic};
    auto periodic_caps = caps({pair(periodic_list, periodic_list)});
    check(
        periodic_caps.report["native_result"] && periodic_caps.start["curves"].size() == 1 &&
            periodic_caps.start["curves"][0]["geometry"]["closed"].get<bool>(),
        "periodic U cap curve follows opening-reversal-reclosure without flattening to a polygon");
    bool caught = false;
    try {
        TubeBudget b;
        b.max_work = 0;
        tube_facet_cap_regions({pair(sides, sides)}, b);
    } catch (const std::exception &) {
        caught = true;
    }
    check(caught, "cap resource limit remains an exception");
    auto future = std::async(std::launch::async, [&] { return caps({pair(sides, sides)}); });
    auto again = future.get();
    check(again.start == single.start && again.end == single.end && again.report == single.report,
          "cap construction is deterministic with independent concurrent state");
    const auto path = group({line({0, 0, 0}, {0, 0, 4})});
    for (bool cap : {false, true}) {
        TubeBudget b;
        auto actual = prepare_swept_tube_facets_with_caps(rectangle(), path, cap, b);
        check(actual.success && actual.sides.success && actual.caps.size() == (cap ? 2 : 0),
              "source-to-side-assembly-to-caps wrapper publishes native cap choice");
        const Json side_indices{{0, 0, 0}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}};
        Json expected = cap ? Json{{-1, 0, 0}, {-1, 1, 0}} : Json::array();
        for (const auto &index : side_indices)
            expected.push_back(index);
        check(actual.face_indices["face_indices"] == expected &&
                  actual.face_indices["side_locations"].size() == 4,
              "actual cap and side output drives native face numbering without geometry copies");
        if (cap)
            check(actual.caps[0]["curves"].size() == 4 &&
                      actual.report["selected_surfaces"].size() == 1,
                  "actual member selection yields all cap edges with source surface provenance");
    }
    TubeBudget b;
    auto actual_multi = prepare_swept_tube_facets_with_caps(
        group({rectangle(), rectangle(-.25, .25)}, 4), path, true, b);
    check(actual_multi.success && actual_multi.caps.size() == 2 &&
              actual_multi.caps[0]["type"] == 4 && actual_multi.caps[0]["curves"].size() == 2,
          "actual multiring source passes placement, generation, assembly and cap grouping");
    TubeBudget prepared_budget;
    auto prepared = assemble_tube_facet_groups(
        generate_tube_facet_groups(rectangle(), path, prepared_budget), prepared_budget);
    auto closed_source = prepared;
    auto &source = closed_source.generation.preparation.placement->branches.path.sources.path;
    source.endpoints_found = true;
    source.source_endpoints = {Point3{0, 0, 0}, Point3{0, 0, 0}};
    auto skipped = cap_tube_facet_assembly(std::move(closed_source), true, prepared_budget);
    check(skipped.success && skipped.caps.empty() &&
              skipped.report["source_endpoint_closed"].get<bool>() &&
              !skipped.report["caps_requested"].get<bool>(),
          "saved original source endpoint closure suppresses caps independently of working path "
          "closure");
    auto &bad = prepared.generation.groups[0][0].surfaces[prepared.groups[0][0].front()].geometry;
    bad["poles"][0] = bad["poles"][0].get<double>() + 100;
    auto failed_outer = cap_tube_facet_assembly(std::move(prepared), true, prepared_budget);
    check(!failed_outer.success && failed_outer.caps.empty() && !failed_outer.sides.groups.empty(),
          "failed cap construction publishes neither cap and retains assembled side groups");
    check(failed_outer.face_indices["face_indices"].empty(),
          "native face query publishes no indices after cap failure even with retained sides");
    TubeBudget failed_budget;
    TubeFacetGroupGeneration failed_generation;
    auto propagated = cap_tube_facet_assembly(
        assemble_tube_facet_groups(std::move(failed_generation), failed_budget), true,
        failed_budget);
    check(!propagated.success && propagated.caps.empty() &&
              !propagated.report["caps_requested"].get<bool>(),
          "failed generation skips classification and cap source access");
    return n;
}
