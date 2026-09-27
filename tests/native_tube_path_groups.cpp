#include "native_tube_path_groups.hpp"
#include "native_tube_facet_groups.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json line(Point3 a, Point3 z) {
    Json p;
    for (unsigned k = 0; k < 3; ++k) {
        p[std::string("point0") + "XYZ"[k]] = a[k];
        p[std::string("point1") + "XYZ"[k]] = z[k];
    }
    return {{"_type", "LineSegment"}, {"segment", p}};
}
Json group(Json values, unsigned type = 1) {
    Json curves = Json::array();
    for (const auto &v : values)
        curves.push_back({{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", curves}};
}
Json poly(Json points) {
    return {{"_type", "LineString"}, {"points", points}};
}
Json spline(Json knots = nullptr, bool closed = false) {
    return {{"_type", "BsplineCurve"}, {"order", 2},
            {"closed", closed},        {"poles", {0, 0, 0, 1, 0, 0, 2, 0, 0}},
            {"knots", knots},          {"weights", nullptr}};
}
Json arc(double sweep) {
    return {{"_type", "EllipticArc"},
            {"arc",
             {{"centerX", 0},
              {"centerY", 0},
              {"centerZ", 0},
              {"vector0X", 1},
              {"vector0Y", 0},
              {"vector0Z", 0},
              {"vector90X", 0},
              {"vector90Y", 1},
              {"vector90Z", 0},
              {"startRadians", 0},
              {"sweepRadians", sweep}}}};
}
} // namespace
unsigned native_tube_path_groups_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto classify = [](const Json &j) {
        TubeBudget b;
        return classify_tube_facet_path(j, b);
    };
    auto expect = [&](const Json &source, const Json &groups, const char *why) {
        const auto r = classify(source);
        check(r.success && Json(r.groups) == groups, why);
    };
    const auto x = line({0, 0, 0}, {1, 0, 0});
    const auto y = line({0, 0, 0}, {0, 1, 0});
    const auto reversed_x = line({8, 9, 10}, {7, 9, 10});
    const auto zero = line({4, 5, 6}, {4, 5, 6});
    const auto ignored = Json{{"_type", "PointString"}, {"points", {0, 0, 0}}};
    expect(group({x, x, y, y, x}), {{0, 1}, {2, 3}, {4}},
           "adjacent straight directions determine grouping with global source-order indices");
    expect(group({x, reversed_x}), {{0, 1}},
           "parallel opposite and disconnected lines remain grouped without connectivity repair");
    expect(group({x, zero, y}), {{0, 1, 2}},
           "zero direction updates previous state and can bridge otherwise nonparallel segments");
    expect(group({poly({0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0})}), {{0, 1, 2}},
           "polyline native segments retain duplicate vertices and zero directions");
    expect(group({x, poly({7, 0, 0, 8, 0, 0, 8, 1, 0}), y}), {{0, 1}, {2, 3}},
           "straight groups continue across line and polyline primitive boundaries");
    expect(group({x, ignored, group({y}), x}), {{0, 1}},
           "ignored primitive kinds do not recurse, emit indices or reset linear state");
    for (double d : {0., 1e-12, std::nextafter(1e-12, 2e-12), 2e-12}) {
        const bool parallel = d <= 1e-12;
        expect(group({x, line({0, 0, 0}, {1, d, 0})}), parallel ? Json{{0, 1}} : Json{{0}, {1}},
               "native parallel squared tolerance preserves its inclusive boundary");
    }
    expect(
        group({spline()}), {{0}, {1}},
        "source linear B-spline knot intervals remain separate even when geometrically collinear");
    expect(group({x, spline(), x}), {{0}, {1}, {2}, {3}},
           "B-spline intervals flush previous groups and reset straight-run state");
    expect(group({spline({2, 2, 4, 8, 8})}), {{0}, {1}},
           "nonunit source knots retain native active-interval count");
    expect(group({spline(nullptr, true)}), {{0}, {1}, {2}},
           "periodic B-spline classification uses its source active domain without opening");
    auto clustered = spline({0, 0, 1e-15, 1, 1});
    expect(group({clustered}), {{0}},
           "near-equal native knot compression changes interval count without geometric fitting");
    auto collapsed = spline({0, 0, 1e-16, 2e-16, 2e-16});
    expect(group({x, collapsed, x}), {{0}, {1}},
           "ignored knot-validation failure still flushes pending groups when no active interval "
           "remains");
    constexpr double pi = 3.141592653589793;
    expect(group({arc(pi / 2)}), {{0}}, "short ellipse occupies one grouped interval");
    expect(group({arc(pi)}), {{0, 1}}, "half ellipse keeps its converted spans in one group");
    expect(group({arc(2 * pi)}), {{0, 1, 2}},
           "native periodic full ellipse keeps all three converted spans together");
    expect(group({x, arc(pi), y, arc(-2 * pi), spline()}), {{0}, {1, 2}, {3}, {4, 5, 6}, {7}, {8}},
           "mixed primitive boundaries preserve ellipse groups and B-spline singleton spans");
    for (unsigned type : {0u, 1u, 2u, 3u})
        expect(group({x, y}, type), {{0}, {1}},
               "native classifier accepts nonparity group kinds without a closure check");
    expect(group(Json::array()), Json::array(), "empty valid path succeeds with zero groups");
    check(!classify(nullptr).success && !classify(group({x}, 4)).success,
          "null and parity roots return native failure");
    auto failed = classify(group({x, y, poly({0, 0, 0})}));
    check(!failed.success && Json(failed.groups) == Json{{0}} &&
              failed.report["discarded_pending_indices"] == Json{1},
          "invalid polyline discards pending indices while retaining previously completed groups");
    auto part = [&](const Json &source, bool direct, std::size_t member, std::size_t count) {
        TubeBudget b;
        return partition_tube_facet_path(source, direct, member, count, b);
    };
    const auto selected = group({x, x, ignored, y, y});
    auto filled = part(selected, false, 2, 6);
    check(filled.success && Json(filled.groups) == Json{{0, 1}, {2}, {3}, {4, 5}} &&
              filled.report["selected_member_patch_count"] == 2,
          "non-direct partition preserves outer groups and inserts singleton generated middle "
          "patches");
    auto shortage = part(selected, false, 2, 4);
    check(!shortage.success && Json(shortage.groups) == Json{{0, 1}},
          "nonpositive generated middle fails after retaining appended prefix groups");
    check(!part(selected, false, 2, 1).success,
          "patch count smaller than prefix does not underflow middle allocation");
    check(Json(part(selected, false, 0, 6).groups) == Json{{0}, {1}, {2}, {3}, {4, 5}},
          "selected first member leaves an empty prefix and offsets the suffix");
    check(Json(part(selected, false, 4, 6).groups) == Json{{0, 1}, {2}, {3}, {4}, {5}},
          "selected last member preserves prefix and fills remaining singleton patches");
    auto direct = part(group({x, x, y}), true, 999, 0);
    check(direct.success && Json(direct.groups) == Json{{0, 1}, {2}},
          "direct branch ignores selected index and generated count as native caller does");
    auto bad_suffix = part(group({x, ignored, poly({0, 0, 0})}), false, 1, 8);
    check(!bad_suffix.success && bad_suffix.groups.empty(),
          "suffix classification failure precedes appending any prefix output");
    const auto source_saved = selected;
    part(selected, false, 2, 6);
    check(selected == source_saved, "path grouping leaves source structure and arrays immutable");
    bool caught = false;
    try {
        TubeBudget b;
        b.max_work = 0;
        classify_tube_facet_path(selected, b);
    } catch (const std::exception &) {
        caught = true;
    }
    check(caught, "classification resource limit is not converted to native false");
    caught = false;
    try {
        part(selected, false, 99, 8);
    } catch (const std::exception &) {
        caught = true;
    }
    check(caught, "undefined selected source index is rejected without repairing the native input");
    auto future = std::async(std::launch::async, [&] { return part(selected, false, 2, 6); });
    const auto concurrent = part(selected, false, 2, 6), other = future.get();
    check(concurrent.groups == other.groups && concurrent.report == other.report,
          "path classification uses no mutable global knot-validation counters");
    const auto profile = group({line({-1, -1, 0}, {1, -1, 0}), line({1, -1, 0}, {1, 1, 0}),
                                line({1, 1, 0}, {-1, 1, 0}), line({-1, 1, 0}, {-1, -1, 0})},
                               2);
    const auto straight = group({line({0, 0, 0}, {0, 0, 4})});
    const auto bent = group({line({0, 0, 0}, {0, 0, 4}), line({0, 0, 4}, {2, 0, 4})});
    for (const auto &trace : {straight, bent}) {
        TubeBudget b;
        auto generated = generate_tube_facet_groups(profile, trace, b);
        const auto classified = classify_generated_tube_facet_groups(generated, b);
        check(classified.success &&
                  Json(classified.groups) == (trace == straight ? Json{{0}} : Json{{0}, {1}}) &&
                  classified.report["caller"] == "native_generated_facet_groups",
              "actual source-to-surface groups provide native path-classification call context");
        generated.groups[0].clear();
        bool missing_first = false;
        try {
            classify_generated_tube_facet_groups(generated, b);
        } catch (const std::exception &) {
            missing_first = true;
        }
        check(missing_first,
              "outer consumer rejects an absent first member instead of inventing a count");
    }
    return n;
}
