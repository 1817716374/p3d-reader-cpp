#include "native_tube_path_source.hpp"
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
    Json p = Json::array();
    for (const auto &v : points)
        for (double x : v)
            p.push_back(x);
    return {{"_type", "LineString"}, {"points", p}};
}
Json line(Point3 a, Point3 b) {
    Json s;
    for (unsigned i = 0; i < 3; ++i) {
        s[std::string("point0") + "XYZ"[i]] = a[i];
        s[std::string("point1") + "XYZ"[i]] = b[i];
    }
    return {{"_type", "LineSegment"}, {"segment", s}};
}
} // namespace
unsigned native_tube_path_source_tests() {
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
    const auto chain = poly({{1, 2, 3}, {4, 5, 6}, {4, 5, 6}, {7, 8, 9}});
    for (unsigned type : {0u, 1u, 2u, 3u, 4u, 5u}) {
        auto source = group(type, {chain});
        source["curves"][0]["native_id"] = 123;
        source["curves"][0]["geometry"]["source_note"] = "polyline";
        const auto original = source;
        TubeBudget b;
        auto r = prepare_tube_path_source(source, b);
        check(source == original && r.path.at("type") == type,
              "path source immutable and boundary type unchanged");
        check(r.path.at("curves").size() == 3 && r.report.at("working_control_points") == 6,
              "path polyline produces all adjacent segments");
        check(r.path.at("curves")[0] == Json{{"geometry", line({1, 2, 3}, {4, 5, 6})}} &&
                  r.path.at("curves")[1] == Json{{"geometry", line({4, 5, 6}, {4, 5, 6})}} &&
                  r.path.at("curves")[2] == Json{{"geometry", line({4, 5, 6}, {7, 8, 9})}},
              "path preserves zero segment and omits inherited primitive metadata");
        check(r.endpoints_found &&
                  r.source_endpoints == std::array<Point3, 2>{Point3{1, 2, 3}, Point3{7, 8, 9}},
              "path original endpoints retained");
        check(r.report.at("members")[2].at("source_segment") == 2 &&
                  r.report.at("members")[2].at("source_path") == "path.curves[0].geometry" &&
                  r.report.at("members")[2].at("working_path") == "working_path.curves[2].geometry",
              "path expanded members map to original polyline segments");
    }
    {
        const auto source = group(2, {poly({{0, 0, 0}, {1, 0, 0}, {0, 0, 0}})});
        TubeBudget b;
        const auto r = prepare_tube_path_source(source, b);
        check(r.path.at("curves").size() == 2 && r.source_endpoints[0] == r.source_endpoints[1],
              "closed path does not acquire another closing line");
    }
    const auto ordinary = line({10, 11, 12}, {13, 14, 15});
    const auto spline = Json{{"_type", "BsplineCurve"}, {"order", 2},
                             {"closed", false},         {"poles", {2., 0., 0., 6., 0., 0.}},
                             {"weights", {2., 3.}},     {"knots", {7., 7., 9., 9.}}};
    auto nested = group(4, {nullptr, group(3, {nullptr, chain, spline}), ordinary});
    nested["curves"][1]["native_id"] = 100;
    nested["curves"][1]["geometry"]["curves"][2]["native_id"] = 101;
    nested["curves"][2]["native_id"] = 102;
    {
        TubeBudget b;
        const auto before = nested;
        auto r = prepare_tube_path_source(nested, b);
        const auto &child = r.path.at("curves")[0];
        check(r.path.at("curves").size() == 2 && child.at("geometry").at("type") == 3 &&
                  child.at("geometry").at("curves").size() == 4,
              "nested path groups retained after recursive polyline expansion");
        check(child.size() == 1 && !child.contains("native_id") &&
                  child.at("geometry").at("curves")[3].at("native_id") == 101 &&
                  r.path.at("curves")[1].at("native_id") == 102,
              "new child wrapper discards descriptor while ordinary clones retain it");
        check(child.at("geometry").at("curves")[3].at("geometry") == spline,
              "path B-spline clone preserves weights and original knot domain");
        check(r.source_endpoints == std::array<Point3, 2>{Point3{1, 2, 3}, Point3{13, 14, 15}} &&
                  r.report.at("skipped_null_sources").size() == 2,
              "source endpoint traversal precedes recursive null compaction");
        check(r.report.at("members")[0].at("descriptor_policy") == "new_primitive" &&
                  r.report.at("members")[4].at("descriptor_policy") == "cloned",
              "recursive clone provenance distinguishes recreated wrappers");
        r.path["curves"][1]["native_id"] = 999;
        check(nested == before, "path clone output does not share mutable source JSON");
    }
    {
        auto source = group(1, {poly({{1, 2, 3}}), ordinary, poly({{90, 91, 92}})});
        TubeBudget b;
        const auto r = prepare_tube_path_source(source, b);
        check(r.path.at("curves").size() == 1 && r.path.at("curves")[0].at("geometry") == ordinary,
              "singleton polylines disappear only in working geometry");
        check(r.endpoints_found &&
                  r.source_endpoints == std::array<Point3, 2>{Point3{1, 2, 3}, Point3{90, 91, 92}},
              "singleton polylines still determine original path endpoints");
        check(r.report.at("polylines_without_segments").size() == 2 &&
                  r.report.at("members")[0].at("source_path") == "path.curves[1].geometry",
              "empty conversions preserve original index provenance");
    }
    for (const auto &source :
         {group(1, Json::array()), group(1, {nullptr}), group(1, {poly({})})}) {
        TubeBudget b;
        const auto r = prepare_tube_path_source(source, b);
        check(r.path.at("curves").empty() && !r.endpoints_found &&
                  r.source_endpoints == std::array<Point3, 2>{},
              "missing source endpoints retain native zero initialization");
    }
    {
        TubeBudget b;
        const auto r = prepare_tube_path_source(group(5, {group(1, {poly({})})}), b);
        check(r.path.at("curves").size() == 1 &&
                  r.path.at("curves")[0].at("geometry").at("curves").empty(),
              "empty nested group survives path expansion");
    }
    const auto profile = group(1, {poly({{0, 0, 0}, {3, 0, 0}, {0, 6, 0}})});
    {
        TubeBudget b;
        const auto r = prepare_tube_facet_sources(profile, nested, b);
        check(r.reference.area_valid && r.reference.reference == Point3{1, 2, 0} &&
                  r.path.path.at("curves").size() == 2,
              "facet preparation joins reference and path source stages");
        check(r.reference.report.at("work_used").get<std::size_t>() <
                      r.path.report.at("work_used").get<std::size_t>() &&
                  r.path.report.at("work_used") == b.work,
              "reference and path preparation use one ordered work budget");
        TubeBudget limited;
        limited.max_work = r.reference.report.at("work_used").get<std::size_t>();
        rejects([&] { prepare_tube_facet_sources(profile, nested, limited); },
                "path preparation cannot reset preceding reference work");
    }
    for (const auto &source : {Json(nullptr), group(1, {Json{{"_type", "Unknown"}}}),
                               group(1, {Json{{"_type", "LineString"}, {"points", {1, 2}}}})})
        rejects(
            [&] {
                TubeBudget b;
                prepare_tube_path_source(source, b);
            },
            "unknown and malformed path sources rejected explicitly");
    {
        TubeBudget b;
        rejects([&] { prepare_tube_facet_sources(profile, nullptr, b); },
                "null path checked before reference processing");
        check(b.work == 0, "null path does not consume reference budget");
        b.max_control_points = 5;
        rejects([&] { prepare_tube_path_source(group(1, {chain}), b); },
                "path budget includes all six expanded controls");
        b = {};
        b.max_control_points = 6;
        check(prepare_tube_path_source(group(1, {chain}), b).report.at("working_control_points") ==
                  6,
              "exact expanded control budget accepted");
        auto source = group(1, {chain, chain});
        b = {};
        b.max_control_points = 11;
        rejects([&] { prepare_tube_path_source(source, b); },
                "path control budget accumulates across source members");
        b = {};
        const auto complete = prepare_tube_path_source(nested, b);
        const auto work = b.work;
        b = {};
        b.max_work = work - 1;
        rejects([&] { prepare_tube_path_source(nested, b); }, "path work budget is cumulative");
        b = {};
        b.max_work = work;
        check(prepare_tube_path_source(nested, b).report == complete.report,
              "exact path work budget deterministic");
    }
    {
        auto source = group(1, {chain});
        source["curves"][0]["ignored_descriptor"] = std::string(100000, 'a');
        TubeBudget b;
        b.max_work = 1000;
        check(prepare_tube_path_source(source, b).path.at("curves").size() == 3,
              "new lines never inspect discarded source descriptor");
    }
    {
        Json source = group(1, {chain});
        for (unsigned i = 0; i < 260; ++i)
            source = group(1, {source});
        rejects(
            [&] {
                TubeBudget b;
                prepare_tube_path_source(source, b);
            },
            "deep path recursion is bounded");
    }
    {
        TubeBudget b;
        const auto r = prepare_tube_path_source(group(1, {ordinary, ordinary}), b);
        check(r.path.at("curves").size() == 2 && r.report.at("members").size() == 2,
              "equal source primitives not deduplicated");
    }
    std::vector<std::future<TubeFacetSources>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async, [&] {
            TubeBudget b;
            return prepare_tube_facet_sources(profile, nested, b);
        }));
    TubeBudget b;
    const auto expected = prepare_tube_facet_sources(profile, nested, b);
    for (auto &job : jobs) {
        const auto r = job.get();
        check(r.reference.report == expected.reference.report &&
                  r.path.report == expected.path.report && r.path.path == expected.path.path,
              "concurrent source preparation uses independent state");
    }
    return count;
}
