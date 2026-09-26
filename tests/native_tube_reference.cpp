#include "native_tube_reference.hpp"
#include "native_curve_area.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json group(unsigned type, Json values) {
    Json members = Json::array();
    for (const auto &v : values)
        members.push_back({{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", members}};
}
Json poly(std::initializer_list<Point3> points) {
    Json data = Json::array();
    for (auto p : points)
        for (double x : p)
            data.push_back(x);
    return {{"_type", "LineString"}, {"points", data}};
}
Json line(Point3 a, Point3 b, bool spline = false) {
    if (spline)
        return {{"_type", "BsplineCurve"}, {"order", 2},
                {"closed", false},         {"poles", {a[0], a[1], a[2], b[0], b[1], b[2]}},
                {"weights", nullptr},      {"knots", nullptr}};
    Json s;
    for (unsigned i = 0; i < 3; ++i) {
        s[std::string("point0") + "XYZ"[i]] = a[i];
        s[std::string("point1") + "XYZ"[i]] = b[i];
    }
    return {{"_type", "LineSegment"}, {"segment", s}};
}
} // namespace
unsigned native_tube_reference_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
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
    auto point_near = [](Point3 a, Point3 b) {
        for (unsigned i = 0; i < 3; ++i)
            if (std::abs(a[i] - b[i]) > 2e-12 * std::max({1., std::abs(a[i]), std::abs(b[i])}))
                return false;
        return true;
    };
    const auto triangle = poly({{4, 0, 0}, {4, 3, 0}, {4, 0, 6}});
    for (unsigned boundary : {0u, 1u}) {
        auto source = group(boundary, {triangle});
        source["curves"][0]["primitive_id"] = {{"native", 17}};
        const auto before = source;
        TubeBudget budget;
        auto result = prepare_tube_reference_profile(source, budget);
        check(source == before, "reference profile source remains unchanged");
        check(result.area_valid && result.profile.at("type") == 2,
              "open reference profile promoted before area query");
        check(result.report.at("closing_member") == 1 && result.profile.at("curves").size() == 2,
              "open reference profile appends one closing line");
        check(result.profile.at("curves")[1] == Json{{"geometry", line({4, 0, 6}, {4, 0, 0})}},
              "closing line uses last to first with no inherited descriptor");
        check(result.profile.at("curves")[0] == source.at("curves")[0],
              "reference copy retains original primitive descriptor");
        check(point_near(result.reference, {4, 1, 2}) && result.plane_origin == result.reference &&
                  point_near(result.plane_normal, {1, 0, 0}),
              "triangle reference has independently known centroid and plane");
        check(result.report.at("reference_source") == "working_profile_area" &&
                  !result.report.contains("fallback_endpoints"),
              "successful area does not query fallback endpoints");
        result.profile["curves"][0]["primitive_id"]["native"] = 99;
        check(source == before, "working profile is an independent copy");
    }
    const auto closed = group(1, {poly({{4, 0, 0}, {4, 3, 0}, {4, 0, 6}, {4, 0, 0}})});
    {
        TubeBudget budget;
        const auto r = prepare_tube_reference_profile(closed, budget);
        check(r.area_valid && r.profile.at("type") == 2 &&
                  r.report.at("closing_member").is_null() && r.profile.at("curves").size() == 1,
              "already closed open group still promoted without extra line");
    }
    for (unsigned type : {2u, 3u}) {
        const auto source = group(type, {triangle});
        TubeBudget budget;
        const auto r = prepare_tube_reference_profile(source, budget);
        check(r.profile == source && !r.report.at("boundary_promoted").get<bool>() &&
                  r.report.at("closing_member").is_null(),
              "closed-region type does not request geometric closure repair");
        check(r.area_valid && point_near(r.reference, {4, 1, 2}),
              "implicit-reference triangle area is retained");
    }
    for (unsigned type : {4u, 5u}) {
        auto first = group(1, {triangle});
        const auto root = group(
            type, {first, group(2, {poly({{0, 0, 0}, {100, 0, 0}, {0, 100, 0}, {0, 0, 0}})})});
        TubeBudget budget;
        const auto r = prepare_tube_reference_profile(root, budget);
        check(r.area_valid && point_near(r.reference, {4, 1, 2}),
              "reference selects first child instead of largest area child");
        check(r.report.at("source_path") == "profile.curves[0].geometry" &&
                  r.profile.at("curves").size() == 2 &&
                  root.at("curves")[0].at("geometry") == first,
              "first child has an independent closure work copy");
        auto unused_unknown = group(type, {closed, Json{{"_type", "Unsupported"}}});
        TubeBudget second;
        check(prepare_tube_reference_profile(unused_unknown, second).area_valid,
              "successful first child reference does not inspect unused sibling");
    }
    const auto failed_first = group(1, {line({2, 4, 6}, {6, 4, 6})});
    const auto failed_root =
        group(4, {failed_first, group(1, {line({100, 200, 300}, {10, 20, 30})})});
    {
        TubeBudget budget;
        const auto r = prepare_tube_reference_profile(failed_root, budget);
        check(!r.area_valid && r.reference == Point3{6, 12, 18},
              "area fallback endpoints belong to original root not selected first child");
        check(r.plane_origin == Point3{} && r.plane_normal == Point3{0, 0, 1},
              "area fallback preserves unmoved default plane");
        check(r.report.at("fallback_endpoints") ==
                      Json::array({Point3{2, 4, 6}, Point3{10, 20, 30}}) &&
                  !r.report.at("fallback_division_guarded").get<bool>(),
              "ordinary fallback records root endpoints and division");
        check(r.profile.at("curves").size() == 2 && r.report.at("closing_member") == 1,
              "degenerate area does not discard reference closing line");
    }
    for (const auto &empty : {group(0, Json::array()), group(1, {nullptr}), group(1, {poly({})})}) {
        TubeBudget budget;
        const auto r = prepare_tube_reference_profile(empty, budget);
        check(!r.area_valid && r.reference == Point3{} && r.profile.at("type") == 2,
              "missing endpoints retain zero fallback and boundary promotion");
        check(!r.report.at("working_endpoints_found").get<bool>() &&
                  !r.report.at("fallback_endpoints_found").get<bool>() &&
                  r.report.at("closing_member").is_null(),
              "no endpoints does not produce a zero-length closing line");
    }
    {
        const auto nested = group(1, {nullptr, group(1, {nullptr, triangle}), nullptr});
        const auto before = nested;
        TubeBudget budget;
        const auto r = prepare_tube_reference_profile(nested, budget);
        check(r.area_valid && point_near(r.reference, {4, 1, 2}) && nested == before,
              "recursive clone skips nulls only in work copy");
        check(r.report.at("skipped_null_sources").size() == 3 &&
                  r.report.at("members").size() == 2 &&
                  r.report.at("members")[0].at("source_path") == "profile.curves[1].geometry" &&
                  r.report.at("members")[0].at("working_path") ==
                      "reference_profile.curves[0].geometry",
              "null compaction retains source-to-working provenance");
        check(r.profile.at("curves")[0].at("geometry").at("type") == 1,
              "only selected group boundary is promoted not nested groups");
    }
    for (bool spline : {false, true}) {
        TubeBudget budget;
        const auto source = group(1, {line({2, 0, 0}, {8, 0, 0}, spline)});
        const auto r = prepare_tube_reference_profile(source, budget);
        check(!r.area_valid && r.reference == Point3{5, 0, 0},
              "line and degree-one spline have original endpoint fallback");
    }
    {
        TubeBudget budget;
        const auto r =
            prepare_tube_reference_profile(group(1, {line({2, 3, 4}, {8, 9, 10}, true)}), budget);
        check(r.area_valid && r.report.at("reference_source") == "working_profile_area",
              "nonzero quadrature residue is not reclassified by geometric degeneracy");
    }
    {
        TubeBudget budget;
        const auto r =
            prepare_tube_reference_profile(group(1, {line({1, 0, 0}, {1, 1e-11, 0})}), budget);
        check(r.report.at("closing_member").is_null() && r.profile.at("type") == 2,
              "relative native endpoint tolerance controls reference closing line");
    }
    // Type 6 has no source-area branch; it permits testing guarded endpoint
    // division without area tensor arithmetic obscuring the native threshold.
    for (double sum : {2e12 - 1, 2e12, 2e12 + 1, -2e12}) {
        TubeBudget budget;
        const auto r = prepare_tube_reference_profile(
            group(6, {line({sum * .5, 4, -6}, {sum * .5, 8, -10})}), budget);
        const bool guard = std::abs(sum) >= 2e12;
        check(!r.area_valid && r.report.at("fallback_division_guarded") == guard,
              "guarded midpoint includes exact division threshold");
        check(r.reference == (guard ? Point3{sum, 12, -16} : Point3{sum * .5, 6, -8}),
              "guard affects all coordinates not only largest component");
    }
    for (unsigned axis : {1u, 2u}) {
        Point3 a{3, 4, 5};
        a[axis] = -1e12;
        TubeBudget budget;
        const auto r = prepare_tube_reference_profile(group(6, {line(a, a)}), budget);
        check(r.reference == Point3{2 * a[0], 2 * a[1], 2 * a[2]},
              "midpoint guard considers each coordinate and absolute sign");
    }
    rejects(
        [&] {
            TubeBudget b;
            prepare_tube_reference_profile(group(6, {line({1e308, 0, 0}, {1e308, 0, 0})}), b);
        },
        "nonfinite endpoint sum is not silently replaced by stable midpoint");
    for (const auto &bad : {Json(nullptr), group(4, Json::array()), group(5, {nullptr}),
                            group(4, {triangle}), group(1, {Json{{"_type", "Unsupported"}}})})
        rejects(
            [&] {
                TubeBudget b;
                prepare_tube_reference_profile(bad, b);
            },
            "missing reference or unsupported geometry fails explicitly");
    {
        auto source = group(1, {triangle});
        TubeBudget b;
        b.max_control_points = 4;
        rejects([&] { prepare_tube_reference_profile(source, b); },
                "closing controls participate in total copy budget");
        b = {};
        b.max_control_points = 2;
        rejects([&] { prepare_tube_reference_profile(source, b); },
                "source copy control budget checked before allocation");
        b = {};
        const auto good = prepare_tube_reference_profile(source, b);
        const auto work = b.work;
        b = {};
        b.max_work = work - 1;
        rejects([&] { prepare_tube_reference_profile(source, b); },
                "work budget includes clone endpoints and area");
        b = {};
        b.max_work = work;
        check(prepare_tube_reference_profile(source, b).report == good.report,
              "exact work budget is deterministic");
        source["opaque_descriptor"] = std::string(4096, 'x');
        b = {};
        b.max_work = 2048;
        rejects([&] { prepare_tube_reference_profile(source, b); },
                "opaque descriptor bytes are included in clone budget");
    }
    {
        Json deep = group(1, {triangle});
        for (unsigned i = 0; i < 260; ++i)
            deep = group(1, {deep});
        rejects(
            [&] {
                TubeBudget b;
                prepare_tube_reference_profile(deep, b);
            },
            "recursive working copy has bounded depth");
    }
    std::vector<std::future<TubeReferenceProfile>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async, [&] {
            TubeBudget b;
            return prepare_tube_reference_profile(failed_root, b);
        }));
    TubeBudget b;
    const auto baseline = prepare_tube_reference_profile(failed_root, b);
    for (auto &job : jobs) {
        const auto r = job.get();
        check(r.profile == baseline.profile && r.reference == baseline.reference &&
                  r.report == baseline.report,
              "concurrent reference preparation uses independent work state");
    }
    return checks;
}
