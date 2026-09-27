#include "native_tube_facet_uv.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
bool near(double a, double b) {
    return std::abs(a - b) < 2e-12;
}
BsplineCurve profile(Json knots = nullptr, unsigned count = 2, bool closed = false) {
    Json poles = Json::array();
    for (unsigned i = 0; i < count; ++i) {
        poles.push_back(i);
        poles.push_back(0);
        poles.push_back(0);
    }
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", 2},
                                    {"closed", closed},
                                    {"poles", poles},
                                    {"knots", knots},
                                    {"weights", nullptr}});
}
BsplineSurface surface(bool quadratic, bool second = false, double z = 0) {
    Json poles = Json::array();
    const unsigned count = quadratic ? 3 : 2;
    for (unsigned v = 0; v < 2; ++v)
        for (unsigned u = 0; u < count; ++u) {
            poles.push_back(double(u) / (count - 1) + (second ? 2. * v - 1 : 0));
            poles.push_back(quadratic ? v - (u == 2 ? .4 : .2)
                                      : (second ? double(v) - .5 : 2. * v - 1));
            poles.push_back(z);
        }
    return BsplineSurface::from_bgfb({{"_type", "BsplineSurface"},
                                      {"numPolesU", count},
                                      {"numPolesV", 2},
                                      {"orderU", count},
                                      {"orderV", 2},
                                      {"closedU", false},
                                      {"closedV", false},
                                      {"knotsU", nullptr},
                                      {"knotsV", nullptr},
                                      {"poles", poles},
                                      {"weights", nullptr},
                                      {"boundaries", nullptr},
                                      {"numRulesU", 0},
                                      {"numRulesV", 0},
                                      {"holeOrigin", 0}});
}
} // namespace
unsigned native_tube_facet_uv_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool failed = false;
        try {
            fn();
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, "UV sampling rejects invalid or exhausted requests explicitly");
    };
    const TubeFacetUvQuery linear = [](double u) { return Point2{u, 1 - u}; };
    auto tree = [&](const TubeFacetUvQuery &q, double tolerance = .001,
                    std::array<double, 2> interval = {0, 1}) {
        TubeBudget b;
        return sample_tube_facet_uv_tree(q, interval, tolerance, b);
    };
    const auto flat = tree(linear);
    check(flat.success && flat.tree.size() == 3 && flat.first.size() == 3,
          "accepted interval retains both quarter probes and center");
    check(flat.first == std::vector<Point3>{{.25, .25, 0}, {.5, .5, 0}, {.75, .75, 0}} &&
              flat.second == std::vector<Point3>{{.25, .75, 0}, {.5, .5, 0}, {.75, .25, 0}},
          "inorder collector includes every node with zero Z and separate V parameters");
    Json fractions = Json::array();
    for (const auto &v : flat.report["queries"])
        fractions.push_back(v["fraction"]);
    check(fractions == Json({.5, 0, 1, .25, .75}),
          "tree queries center, both endpoints, then left and right quarter in native order");
    check(flat.tree[0].left == 1 && flat.tree[0].right == 2 && flat.tree[1].depth == 1 &&
              flat.tree[2].depth == 1 && !flat.tree[1].left,
          "tree storage retains source allocation order separately from collected order");
    const TubeFacetUvQuery quadratic = [](double u) { return Point2{u * u, 2 - u}; };
    const auto curved = tree(quadratic);
    check(curved.success && curved.first.size() == 63 && curved.report["deepest_probe"] == 5,
          "quadratic error bound retains all quarter probes through four refinements");
    for (std::size_t i = 0; i < curved.first.size(); ++i) {
        const double u = double(i + 1) / 64;
        check(curved.first[i] == Point3{u, u * u, 0} && curved.second[i] == Point3{u, 2 - u, 0},
              "quadratic UV samples agree with independent analytic dyadic values");
    }
    Json prefix = Json::array();
    for (unsigned i = 0; i < 8; ++i)
        prefix.push_back(curved.report["queries"][i]["fraction"]);
    check(prefix == Json({.5, 0, 1, .25, .75, 0, .125, .375}),
          "left recursion requeries original low and reuses parent center as high");
    const auto boundary = tree(quadratic, .25);
    const auto below = tree(quadratic, std::nextafter(.25, 0.));
    check(
        boundary.tree.size() == 3 && below.tree.size() == 7,
        "maximum center error equal to tolerance is accepted, strictly smaller tolerance refines");
    const auto second_curved = tree([](double u) { return Point2{2 - u, u * u}; });
    check(second_curved.second == curved.first && second_curved.first == curved.second,
          "second curve errors independently trigger the same refinement");
    const auto nonunit = tree(linear, .001, {2, 4});
    check(nonunit.first == std::vector<Point3>{{2.5, 2.5, 0}, {3, 3, 0}, {3.5, 3.5, 0}},
          "tree retains raw nonunit fractions rather than normalizing the interval");
    const auto degenerate = tree(linear, .001, {.5, .5});
    check(degenerate.success && degenerate.first == std::vector<Point3>(3, {.5, .5, 0}),
          "equal interval and repeated UV values are not deduplicated");
    for (std::size_t fail = 0; fail < 8; ++fail) {
        std::size_t calls = 0;
        const auto failed = tree([&](double u) -> std::optional<Point2> {
            if (calls++ == fail)
                return std::nullopt;
            return quadratic(u);
        });
        check(!failed.success && failed.tree.empty() && failed.first.empty() &&
                  failed.second.empty() && calls == fail + 1 &&
                  failed.report["failure"] == "native_sample_failure",
              "failure at root, endpoint, child or recursive probe discards the whole temporary "
              "tree");
    }
    const auto depth = tree([](double u) { return Point2{u == 0 ? 0. : 1., 0}; });
    check(!depth.success && depth.report["failure"] == "native_depth_limit" &&
              depth.report["deepest_probe"] == 302 && depth.report["queries"].size() == 908 &&
              depth.first.empty() && depth.tree.empty(),
          "native depth 301 fails only after both depth302 children are queried and allocated");
    TubeBudget nodes;
    nodes.max_control_points = 2;
    rejects([&] { sample_tube_facet_uv_tree(linear, {0, 1}, .001, nodes); });
    TubeBudget work;
    work.max_work = 0;
    rejects([&] { sample_tube_facet_uv_tree(linear, {0, 1}, .001, work); });
    rejects([&] { tree(linear, -1); });
    rejects([&] { tree(linear, .001, {1, 0}); });
    rejects([&] { tree([](double) { return Point2{NAN, 0}; }); });
    auto segments = [&](const BsplineCurve &p, const TubeFacetUvQuery &q) {
        TubeBudget b;
        return sample_tube_facet_uv_segments(p, q, b);
    };
    const auto one = segments(profile(), linear);
    check(one.success && one.first.size() == 1 &&
              one.first[0] ==
                  std::vector<Point3>{
                      {0, 0, 0}, {.25, .25, 0}, {.5, .5, 0}, {.75, .75, 0}, {1, 1, 0}},
          "one segment contains low, complete tree and explicit high equal to one");
    const auto two_profile = profile({0, 0, .5, 1, 1}, 3);
    const auto two = segments(two_profile, linear);
    check(two.success && two.first.size() == 2 && two.first[0].size() == 5 &&
              two.first[1].size() == 5 && two.first[0].back() == two.first[1].front(),
          "next interval low supplies previous segment end as well as its own start");
    check(two.report["endpoint_queries"] ==
              Json::array({{{"fraction", 0}, {"parameters", {0, 1}}},
                           {{"fraction", .5}, {"parameters", {.5, .5}}},
                           {{"fraction", 1}, {"parameters", {1, 0}}}}),
          "outer endpoint queries follow low per interval and terminal high only");
    const auto raw = segments(profile({2, 2, 3, 4, 4}, 3), linear);
    check(raw.success && raw.first.size() == 2 && raw.first[0].back()[0] == 3 &&
              raw.first[1].size() == 4 && raw.first[1].back()[0] == 3.75,
          "nonunit final high is not repaired into an explicit endpoint");
    const auto stop = segments(profile({0, 0, 1, 2, 2}, 3), linear);
    check(stop.success && stop.first.size() == 1 && stop.first[0].back()[0] == 1,
          "exact high one terminates even when more profile intervals exist");
    const auto periodic = segments(profile(nullptr, 3, true), linear);
    check(periodic.success && !periodic.first.empty() && periodic.first[0][0][0] == 0 &&
              periodic.first.back().back()[0] == 1,
          "periodic profile uses compressed active intervals, excluding exterior knots");
    for (std::size_t fail : std::array<std::size_t, 5>{0, 2, 6, 7, 12}) {
        std::size_t calls = 0;
        const auto failed = segments(two_profile, [&](double u) -> std::optional<Point2> {
            if (calls++ == fail)
                return std::nullopt;
            return linear(u);
        });
        check(
            !failed.success && failed.first.empty() && failed.second.empty() && calls == fail + 1,
            "outer low, tree, next low and final high failures clear both complete segment tables");
    }
    TubeBudget output_limit;
    output_limit.max_control_points = 9;
    rejects([&] { sample_tube_facet_uv_segments(profile(), linear, output_limit); });
    const auto curved_surface = surface(true);
    const auto saved_poles = curved_surface.poles();
    const std::array<Point3, 2> plane{{{0, 0, 0}, {0, 1, 0}}};
    TubeBudget actual_budget;
    const auto actual = sample_tube_facet_uv_seam(curved_surface, curved_surface, profile(), plane,
                                                  true, actual_budget);
    check(actual.success && actual.first.size() == 1 && actual.first[0].size() == 33 &&
              actual.report["surface_queries"]["fallback_queries"] == 0,
          "actual quadratic surfaces use analytic plane seam and adaptive subdivision");
    for (const auto &p : actual.first[0])
        check(near(p[1], .2 + .2 * p[0] * p[0]) && p[2] == 0,
              "actual surface UV samples satisfy the independent quadratic equation");
    check(actual.first == actual.second && curved_surface.poles() == saved_poles,
          "both UV lists preserve surface source data and common plane intersections");
    const std::array<Point3, 2> offset{{{0, .4, 0}, {0, 1, 0}}};
    TubeBudget fallback_budget;
    const auto fallback = sample_tube_facet_uv_seam(surface(false), surface(false, true), profile(),
                                                    offset, true, fallback_budget);
    check(fallback.success && fallback.first[0].size() == 5 &&
              fallback.report["surface_queries"]["fallback_queries"] == 7,
          "every adaptive and outer query uses complete curve-pair fallback when required");
    for (const auto &p : fallback.first[0])
        check(near(p[1], .5), "actual pair fallback samples have analytic center intersection");
    TubeBudget failure_budget;
    const auto spatial_failure = sample_tube_facet_uv_seam(surface(false), surface(false, true, 1),
                                                           profile(), offset, true, failure_budget);
    check(!spatial_failure.success && spatial_failure.first.empty() &&
              spatial_failure.report["surface_queries"]["last_failure"]["native_result"] == false,
          "actual skew isocurves propagate native failure without a partial boundary");
    auto future = std::async(std::launch::async, [&] {
        TubeBudget b;
        return sample_tube_facet_uv_seam(curved_surface, curved_surface, profile(), plane, true, b);
    });
    const auto concurrent = future.get();
    check(
        concurrent.first == actual.first && concurrent.report == actual.report &&
            curved_surface.poles() == saved_poles,
        "surface sampling is deterministic with independently owned budgets and immutable inputs");
    return n;
}
