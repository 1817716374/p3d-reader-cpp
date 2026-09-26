#include "native_tube_path_selection.hpp"
#include "native_curve_planarity.hpp"
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
    for (auto a : points)
        for (double x : a)
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
unsigned native_tube_path_selection_tests() {
    unsigned count = 0;
    auto check = [&](bool v, const char *why) {
        ++count;
        require(v, why);
    };
    auto near = [](double a, double b) { return std::abs(a - b) < 1e-11; };
    auto rejects = [&](auto fn, const char *why) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    const auto profile = group(2, {poly({{0, -1, -1}, {0, 1, -1}, {0, 0, 2}})});
    const auto fallback = group(0, {line({-1, 0, 0}, {1, 0, 0})});
    auto select = [&](const Json &p, const Json &path) {
        TubeBudget b;
        return prepare_tube_facet_path(p, path, b);
    };
    for (unsigned boundary : {0u, 1u, 2u, 3u}) {
        const auto path = group(boundary, {line({-2, 3, 0}, {2, 3, 0})});
        const auto before = path;
        for (bool area : {false, true}) {
            const auto r = select(area ? profile : fallback, path);
            check(r.sources.reference.area_valid == area &&
                      r.selection.report.at("query") ==
                          (area ? "plane_intersection" : "closest_point"),
                  "reference area chooses native query branch");
            check(r.selection.index == 0 && near(r.selection.fraction, .5) &&
                      near(r.selection.candidate_squared_distance, 9),
                  "line interior candidate selection");
            check(path == before && r.selection.curves.size() == 1 &&
                      r.selection.report.at("candidate_count") == 2,
                  "immutable path and final endpoint comparison");
        }
    }
    {
        auto r = select(profile, group(1, {poly({{0, 10, 0}, {0, 0, 0}, {0, 10, 0}})}));
        check(r.selection.index == 0 && r.selection.fraction == 0 &&
                  r.selection.candidate_point == Point3{0, 10, 0},
              "coplanar segments have no isolated candidates and no closest fallback");
        check(r.selection.report.at("candidate_source") == "original_start" &&
                  r.selection.report.at("members")[0].at("all_parameter_segments") == 1,
              "coplanar branch report");
        r = select(fallback, group(1, {poly({{0, 10, 0}, {0, 0, 0}, {0, 10, 0}})}));
        check(r.selection.index == 1 && r.selection.fraction == 0 &&
                  r.selection.report.at("candidate_index") == 0 &&
                  r.selection.report.at("candidate_fraction") == 1,
              "closest at shared endpoint moves to next member");
    }
    {
        auto r =
            select(profile, group(1, {line({-2, 3, 0}, {2, 3, 0}), line({-2, 1, 0}, {2, 1, 0})}));
        check(r.selection.index == 1 && near(r.selection.fraction, .5) &&
                  near(r.selection.candidate_squared_distance, 1),
              "all plane candidates compared across members");
        r = select(profile, group(1, {line({-2, 1, 0}, {2, 1, 0}), line({-2, -1, 0}, {2, -1, 0})}));
        check(r.selection.index == 0 && near(r.selection.fraction, .5),
              "equal plane distance retains first candidate");
        r = select(profile, group(1, {line({-3, 0, 0}, {-1, 0, 0})}));
        check(r.selection.index == 0 && r.selection.fraction == 1 &&
                  r.selection.report.at("candidate_source") == "original_end",
              "no plane hit can still select original end");
    }
    {
        auto r =
            select(fallback, group(1, {line({-2, 0, 0}, {0, 0, 0}), line({10, 0, 0}, {20, 0, 0})}));
        check(r.selection.index == 1 && r.selection.fraction == 0 &&
                  r.selection.candidate_point == Point3{0, 0, 0},
              "end remapping retained across geometric gap");
        check(r.selection.curves[1].point_at(0) == Point3{10, 0, 0} &&
                  r.selection.report.at("moved_to_next_member") == true,
              "candidate point is not silently replaced by remapped point");
        check(r.location.point == Point3{10, 0, 0} && r.location.tangent == Point3{10, 0, 0},
              "selected point and tangent use remapped member across a gap");
        for (double delta : {2e-10, 4e-10}) {
            const double x = 1 - delta;
            r = select(group(0, {line({x, 0, 0}, {x, 0, 0})}),
                       group(1, {line({0, 0, 0}, {1, 0, 0}), line({2, 0, 0}, {3, 0, 0})}));
            check(r.selection.index == (delta < 3e-10 ? 1 : 0) &&
                      r.selection.report.at("moved_to_next_member") == (delta < 3e-10),
                  "native near-one relative tolerance");
        }
    }
    {
        auto r = select(fallback, group(1, {poly({{0, 0, 0}}), line({10, 0, 0}, {20, 0, 0})}));
        check(r.selection.curves.size() == 1 && r.selection.index == 0 &&
                  r.selection.fraction == 0 &&
                  r.selection.report.at("candidate_source") == "original_start",
              "original singleton start survives despite no working segment");
        check(r.selection.candidate_point == Point3{0, 0, 0} &&
                  r.selection.curves[0].point_at(0) == Point3{10, 0, 0},
              "original default start not recomputed on working path");
        r = select(fallback, group(1, {line({10, 0, 0}, {20, 0, 0}), poly({{0, 0, 0}})}));
        check(r.selection.fraction == 1 && r.selection.candidate_point == Point3{0, 0, 0} &&
                  r.selection.report.at("candidate_source") == "original_end",
              "original singleton end compared after all working segments");
    }
    {
        const Json spline{{"_type", "BsplineCurve"}, {"order", 2},
                          {"closed", false},         {"poles", {-1., 0., 0., 1., 0., 0.}},
                          {"weights", {-1., 1.}},    {"knots", {2., 2., 5., 5.}}};
        auto r = select(profile, group(1, {spline}));
        check(near(r.selection.fraction, .5) && r.selection.candidate_point == Point3{0, 0, 0} &&
                  r.selection.report.at("candidate_projection_succeeded") == false,
              "zero-weight plane intersection still participates as native zero point");
        check(r.selection.curves[0].weights() == std::vector<double>{-1, 1} &&
                  r.selection.curves[0].knot_domain() == std::array<double, 2>{2, 5},
              "path conversion preserves source weights and domain");
    }
    {
        auto r = select(fallback,
                        group(1, {nullptr, poly({{-2, 1, 0}, {0, 1, 0}, {0, 1, 0}, {2, 1, 0}})}));
        check(r.selection.curves.size() == 3 &&
                  r.selection.report.at("converted_control_points") == 6,
              "null skipped and zero-length segment retained");
        check(r.selection.index == 1 && r.selection.fraction == 0,
              "equal zero segment does not overwrite prior selection before remap");
        check(r.sources.path.report.at("members")[2].at("source_segment") == 2,
              "selection indexes remain joinable to original path provenance");
    }
    const auto path = group(1, {line({-2, 0, 0}, {2, 0, 0}), line({3, 0, 0}, {4, 0, 0})});
    TubeBudget b;
    const auto sources = prepare_tube_facet_sources(fallback, path, b);
    auto before = sources.path.path;
    const auto prior = b.work;
    const auto r = select_tube_path_candidate(sources, b);
    check(b.work > prior && sources.path.path == before,
          "selection shares budget and does not mutate prepared sources");
    const auto location = evaluate_tube_path_selection(r, b);
    auto planarity = curve_detail::native_primitive_planarity(
        sources.path.path.at("curves").at(r.index).at("geometry"), b.max_control_points,
        {b.work, b.max_work});
    planarity["working_index"] = r.index;
    const auto total = b.work;
    TubeBudget exact;
    exact.max_work = total;
    auto all = prepare_tube_facet_path(fallback, path, exact);
    check(exact.work == total && all.selection.report == r.report,
          "combined preparation uses identical cumulative budget and report");
    check(all.selected_member_planarity == planarity, "combined selected member planarity");
    check(all.location.report == location.report, "combined selected point tangent");
    rejects(
        [&] {
            TubeBudget small;
            small.max_work = total - 1;
            prepare_tube_facet_path(fallback, path, small);
        },
        "cumulative work exhaustion");
    rejects(
        [&] {
            TubeBudget small;
            small.max_control_points = 3;
            select_tube_path_candidate(sources, small);
        },
        "cumulative converted control limit");
    rejects([&] { select(profile, group(1, {})); }, "empty working path explicitly undefined");
    rejects([&] { select(profile, group(4, {line({0, 0, 0}, {1, 0, 0})})); },
            "non-simple path boundary rejected");
    rejects([&] { select(profile, group(1, {group(1, {line({0, 0, 0}, {1, 0, 0})})})); },
            "nested member not silently flattened for conversion");
    rejects(
        [&] {
            select(fallback, group(1, {Json{{"_type", "BsplineCurve"},
                                            {"order", 2},
                                            {"closed", false},
                                            {"poles", {0., 0., 0., 0., 0., 0.}},
                                            {"weights", {0., 0.}},
                                            {"knots", nullptr}}}));
        },
        "undefined closest query not skipped as if successful");
    const auto expected = select(fallback, path).selection.report;
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(std::async(std::launch::async,
                                  [&] { return select(fallback, path).selection.report; }));
    for (auto &job : jobs)
        check(job.get() == expected, "parallel path selection is deterministic");
    return count;
}
