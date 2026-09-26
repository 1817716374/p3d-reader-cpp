#include "native_tube_facets.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json group(int type, Json input) {
    Json curves = Json::array();
    for (const auto &g : input)
        curves.push_back({{"geometry", g}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", curves}};
}
Json line() {
    return {{"_type", "LineSegment"},
            {"source_marker", 19},
            {"segment",
             {{"point0X", 1.},
              {"point0Y", 2.},
              {"point0Z", 3.},
              {"point1X", 1.},
              {"point1Y", 2.},
              {"point1Z", 3.}}}};
}
Json arc(bool closed) {
    return {{"_type", "EllipticArc"},
            {"arc",
             {{"centerX", 0.},
              {"centerY", 0.},
              {"centerZ", 0.},
              {"vector0X", 2.},
              {"vector0Y", 0.},
              {"vector0Z", 0.},
              {"vector90X", 0.},
              {"vector90Y", 1.},
              {"vector90Z", 0.},
              {"startRadians", 0.},
              {"sweepRadians", closed ? 6.283185307179586 : 1.}}}};
}
TubeFacetProfile partition(const Json &j) {
    TubeBudget b;
    return partition_tube_facet_profile(std::make_shared<const Json>(j), b);
}
} // namespace
unsigned native_tube_facets_tests() {
    unsigned count = 0;
    auto check = [&](bool ok, const char *why) {
        ++count;
        require(ok, why);
    };
    auto rejects = [&](auto fn, const char *why) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    const Json poly{{"_type", "LineString"},
                    {"source_marker", 42},
                    {"points", {0., 0., 0., 1., 2., 3., 1., 2., 3., 0., 0., 0.}}};
    const Json spline{{"_type", "BsplineCurve"}, {"order", 2},
                      {"closed", false},         {"poles", {0., 0., 0., 1., 2., 0., 0., 0., 0.}},
                      {"weights", nullptr},      {"knots", nullptr}};
    const auto input = group(2, {line(), poly, arc(false), arc(true), spline, line()});
    auto owned = std::make_shared<const Json>(input);
    const auto *first = &owned->at("curves")[0].at("geometry");
    TubeBudget b;
    auto result = partition_tube_facet_profile(owned, b);
    check(result.report["native_result"] == true && result.groups.size() == 1 &&
              result.groups[0].size() == 8,
          "native profile partition retains primitive order and expands line segments");
    const auto &members = result.groups[0];
    check(&members[0].geometry() == first && members[0].source_geometry == first &&
              !members[0].derived_line && members[0].boundary_type == 1,
          "native line wrapper reuses original primitive and never classifies zero line as outer "
          "loop");
    for (unsigned i = 0; i < 3; ++i) {
        const auto &m = members[i + 1];
        check(m.source_member == 1 && m.source_segment == i && m.boundary_type == 1 &&
                  m.derived_line.has_value() && m.geometry().at("_type") == "LineSegment" &&
                  !m.geometry().contains("source_marker"),
              "polyline creates new lines with source segment locations and no copied wrapper "
              "metadata");
        for (unsigned end = 0; end < 2; ++end)
            for (unsigned axis = 0; axis < 3; ++axis)
                check(m.geometry().at("segment").at(std::string("point") + char('0' + end) +
                                                    "XYZ"[axis]) ==
                          poly["points"][3 * (i + end) + axis],
                      "derived segment stores exact adjacent source coordinates");
    }
    check(members[4].boundary_type == 1 && members[5].boundary_type == 2 &&
              members[6].boundary_type == 2 && members[6].geometry()["closed"] == false,
          "ellipse and spline group types use source endpoints rather than saved closed state");
    check(&members[0].geometry() != &members[7].geometry() &&
              members[0].geometry() == members[7].geometry(),
          "equal native source primitives remain separate without invented deduplication");
    owned.reset();
    auto moved = std::move(result);
    auto copied = moved;
    check(&copied.groups[0][0].geometry() == first &&
              copied.groups[0][2].geometry() == moved.groups[0][2].geometry(),
          "partition ownership keeps immutable source references valid across copies and moves");
    check(*copied.source == input, "partition does not modify source curves or metadata");
    for (int type : {0, 1, 2, 3, 5, -1, 200}) {
        const auto r = partition(group(type, {line()}));
        check(r.report["native_result"] == true && r.groups.size() == 1 && r.groups[0].size() == 1,
              "facet partition only treats boundary four as parity");
    }
    const Json ignored{{"_type", "PointString"}, {"points", {1., 2., 3.}}};
    const auto skipped = partition(group(5, {ignored, group(2, {line()}), line()}));
    check(skipped.report["native_result"] == true && skipped.groups[0].size() == 1 &&
              skipped.report["skipped_primitives"].size() == 2 &&
              skipped.groups[0][0].source_member == 2,
          "nonparticipating primitives and nested groups are skipped rather than flattened");
    const auto empty = partition(group(1, Json::array()));
    const auto empty_parity = partition(group(4, Json::array()));
    check(empty.report["native_result"] == true && empty.groups.size() == 1 &&
              empty.groups[0].empty() && empty_parity.report["native_result"] == true &&
              empty_parity.groups.empty(),
          "raw partition distinguishes empty simple group from empty parity success");
    const auto parity = group(4, {group(1, {line()}), group(0, {arc(false)}), group(3, {line()})});
    const auto rings = partition(parity);
    check(rings.report["native_result"] == true && rings.groups.size() == 3 &&
              rings.groups[1][0].boundary_type == 1,
          "facet parity partition accepts open child groups without imposing whole-surface closure "
          "rules");
    const Json short_poly{{"_type", "LineString"}, {"points", {1., 2., 3.}}};
    const auto failed_simple = partition(group(1, {line(), short_poly}));
    check(failed_simple.report["native_result"] == false && failed_simple.groups.empty() &&
              failed_simple.report["discarded_temporary_members"] == 1,
          "short polyline failure discards the simple temporary group");
    const auto failed_parity = partition(
        group(4, {group(2, {line()}), group(1, {line(), short_poly}), group(2, {line()})}));
    check(failed_parity.report["native_result"] == false && failed_parity.groups.size() == 1 &&
              failed_parity.report["failure_path"] ==
                  "/profile/curves/1/geometry/curves/1/geometry",
          "parity failure preserves prior groups and skips later siblings");
    for (const auto &bad : {Json(nullptr), group(4, {line()}), group(4, {group(4, {})})})
        check(partition(bad).report["native_result"] == false,
              "null profile missing child array and nested parity return native failure");
    auto job = std::async(std::launch::async, [&] { return partition(parity); });
    const auto concurrent = job.get();
    check(concurrent.report == rings.report &&
              concurrent.groups[1][0].geometry() == rings.groups[1][0].geometry(),
          "facet partition has no shared mutable working state");
    TubeFacetGroups facets{{{}, {Json{{"id", 1}}, nullptr}},
                           {},
                           {{Json{{"id", 2}}}, {}, {Json{{"id", 3}}, Json{{"id", 3}}}}};
    TubeBudget indices_budget;
    const auto indices = enumerate_tube_facet_indices(facets, 3, true, indices_budget);
    check(indices["face_indices"] ==
              Json::array(
                  {{-1, 0, 0}, {-1, 1, 0}, {0, 0, 0}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {0, 4, 0}}),
          "native face enumeration caps at two cap indices and numbers every nested slot globally");
    const auto &links = indices["side_locations"];
    check(links[0]["group"] == 0 && links[0]["member"] == 1 && links[0]["patch"] == 0 &&
              links[2]["group"] == 2 && links[4]["member"] == 2 && links[4]["patch"] == 1,
          "native flattened face numbers retain their exact three-level source location");
    for (std::size_t caps : {0u, 1u, 2u}) {
        TubeBudget local;
        check(enumerate_tube_facet_indices({}, caps, true, local)["face_indices"].size() == caps,
              "caps enumerate independently of empty side groups");
    }
    TubeBudget failed_budget;
    check(enumerate_tube_facet_indices(facets, 2, false, failed_budget)["face_indices"].empty() &&
              failed_budget.work == 0,
          "failed native generation appends no indices even with partial surface data");
    rejects(
        [&] {
            TubeBudget local;
            local.max_work = b.work - 1;
            partition_tube_facet_profile(std::make_shared<const Json>(input), local);
        },
        "partition charges source visits derived segments and endpoint queries");
    rejects(
        [&] {
            TubeBudget local;
            local.max_work = indices_budget.work - 1;
            enumerate_tube_facet_indices(facets, 3, true, local);
        },
        "face enumeration has cumulative work bounds");
    rejects(
        [&] {
            TubeBudget local;
            local.max_control_points = 2;
            partition_tube_facet_profile(std::make_shared<const Json>(group(1, {poly})), local);
        },
        "polyline source allocation is bounded");
    return count;
}
