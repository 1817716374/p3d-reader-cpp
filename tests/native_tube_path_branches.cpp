#include "native_tube_path_branches.hpp"
#include "native_tube_transform.hpp"
#include "native_curve_planarity.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json line(Point3 a, Point3 b) {
    Json s;
    for (unsigned i = 0; i < 3; ++i) {
        s[std::string("point0") + "XYZ"[i]] = a[i];
        s[std::string("point1") + "XYZ"[i]] = b[i];
    }
    return {{"_type", "LineSegment"}, {"segment", s}};
}
Json group(Json values) {
    Json a = Json::array();
    for (const auto &v : values)
        a.push_back(v.is_null() ? Json() : Json{{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", 1}, {"curves", a}};
}
Json poly(Json points) {
    return {{"_type", "LineString"}, {"points", points}};
}
} // namespace
unsigned native_tube_path_branches_tests() {
    unsigned count = 0;
    auto check = [&](bool b, const char *why) {
        ++count;
        require(b, why);
    };
    auto near = [](double a, double b) { return std::abs(a - b) < 1e-9; };
    auto rejects = [&](auto fn, const char *why) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    auto plan = [&](std::size_t n, std::size_t i, double f, bool whole, bool member) {
        TubeBudget b;
        return plan_tube_path_branches(n, i, f, whole, member, b);
    };
    for (std::size_t n = 1; n <= 5; ++n)
        for (std::size_t index = 0; index < n; ++index) {
            const auto whole = plan(n, index, .4, true, true);
            check(whole.prefix.empty() && whole.suffix.size() == n,
                  "whole planar path always suffix");
            const auto member = plan(n, index, .4, false, true);
            const auto boundary = index == 0 ? 0 : index == n - 1 ? n : index;
            check(member.prefix.size() == boundary && member.suffix.size() == n - boundary,
                  "planar member native first middle last partition");
            auto parts = member.prefix;
            parts.insert(parts.end(), member.suffix.begin(), member.suffix.end());
            bool ordered = true;
            for (std::size_t i = 0; i < n; ++i)
                ordered &= parts[i].index == i && parts[i].whole;
            check(ordered, "whole-member partition preserves source order and duplicates");
            for (double f : {0., 5e-11, 1., 1. - 2e-10, .4}) {
                const auto p = plan(n, index, f, false, false);
                const bool start = f < 1e-10, end = f > 1 - 3e-10;
                check(p.prefix.size() == index + (start ? 0 : 1) &&
                          p.suffix.size() == n - index - (end ? 1 : 0),
                      "nonplanar start end interior distribution");
                if (!start && !end)
                    check(!p.prefix.back().whole && !p.suffix.front().whole &&
                              p.prefix.back().first == 0 && p.prefix.back().last == f &&
                              p.suffix.front().first == f && p.suffix.front().last == 1,
                          "interior split retains exact requested source fractions");
            }
        }
    check(!plan(1, 0, 2e-10, false, false).prefix[0].whole,
          "fraction outside near-zero tolerance is not snapped");
    check(!plan(1, 0, 1 - 4e-10, false, false).prefix[0].whole,
          "fraction outside near-one tolerance is not snapped");
    check(plan(1, 0, -.1, false, false).prefix[0].last == -.1,
          "native interval requests not silently clamped");
    auto whole_planarity = [&](const Json &j) {
        std::size_t work = 0;
        return curve_detail::native_curve_vector_planarity(j, 10000, {work, 10000000});
    };
    auto spatial = group({poly({0., 0., 0., 1., 0., 0., 1., 1., 0., 1., 1., 1.})});
    const auto saved = spatial;
    check(whole_planarity(spatial).at("planar") == false && spatial == saved,
          "original spatial LineString whole-path planarity");
    auto flat = group({poly({0., 0., 0., 1., 0., 0., 1., 1., 0.})});
    check(whole_planarity(flat).at("planar") == true, "flat original LineString");
    check(whole_planarity(
              group({nullptr, group({line({0, 0, 1}, {1, 0, 1})}), line({0, 1, 1}, {1, 1, 1})}))
                  .at("planar") == true,
          "nested frame and range traversal retains null members");
    auto empty = group({});
    check(whole_planarity(empty).at("planar") == false, "empty source frame is native false");
    empty["curves"] = nullptr;
    check(whole_planarity(empty).at("planar") == false,
          "absent source member array is native false");
    const double huge = std::numeric_limits<double>::max();
    Matrix4 identity{};
    for (unsigned i = 0; i < 4; ++i)
        identity[i][i] = 1;
    std::size_t used = 0;
    const auto disconnected = curve_detail::native_primitive_range(
        poly({0., 1., 2., huge, 0., 0., 3., 4., 5.}), identity, 100, {used, 1000000});
    check(disconnected.low == Point3{0, 1, 2} && disconnected.high == Point3{3, 4, 5},
          "LineString later disconnect repeats previous range contribution");
    rejects(
        [&] {
            std::size_t w = 0;
            curve_detail::native_primitive_range(poly({huge, 0., 0., 1., 2., 3.}), identity, 100,
                                                 {w, 1000000});
        },
        "first disconnect native undefined temporary not fabricated");
    auto build = [&](const Json &path, Point3 reference) {
        TubeBudget budget;
        return prepare_tube_facet_path_branches(group({line(reference, reference)}), path, budget);
    };
    auto result = build(flat, {1, .5, 0});
    check(result.whole_path_planarity.at("planar") == true &&
              result.plan.report.at("branch") == "whole_path_planar" &&
              !result.prefix.constructed && result.suffix.constructed.has_value(),
          "caller whole-path flag routes combined curve to suffix");
    check(result.suffix.constructed->point_at(0) == Point3{0, 0, 0} &&
              result.suffix.constructed->point_at(1) == Point3{1, 1, 0},
          "native combined whole path endpoints");
    result = build(spatial, {1, .5, 0});
    check(result.path.selection.index == 1 && near(result.path.selection.fraction, .5) &&
              result.path.location.point == Point3{1, .5, 0} &&
              result.path.location.tangent == Point3{0, 1, 0},
          "selected location before branch combination");
    check(result.prefix.reused_curve_index == 0 && !result.prefix.constructed &&
              result.suffix.constructed && !result.suffix.reused_curve_index,
          "single whole member reused and multi-member suffix constructed");
    check(result.report.at("status") == "complete" &&
              result.report.at("scope") == "branch_curve_assembly" &&
              result.report.at("subsequent_frames_evaluated") == false,
          "branch assembly does not claim later native frame construction");
    result = build(group({line({0, 0, 0}, {2, 0, 0})}), {1, 0, 0});
    check(result.suffix.reused_curve_index == 0 && !result.suffix.constructed,
          "single source does not get spurious combination or knot normalization");
    Json spline{
        {"_type", "BsplineCurve"}, {"order", 4},
        {"closed", false},         {"poles", {0., 0., 0., 1., 0., 0., 1., 1., 1., 2., 2., 0.}},
        {"weights", nullptr},      {"knots", nullptr}};
    auto c = BsplineCurve::from_bgfb(spline);
    result = build(group({spline}), c.point_at(.5));
    check(result.plan.report.at("branch") == "nonplanar_interior" &&
              result.report.at("status") == "not_evaluated" && !result.prefix.constructed &&
              !result.suffix.constructed,
          "unimplemented native subcurve is explicit without partial geometry");
    TubePathSelection selection;
    selection.curves.push_back(BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                                        {"order", 2},
                                                        {"closed", false},
                                                        {"weights", nullptr},
                                                        {"poles", {0., 0., 0., 4., 8., 12.}},
                                                        {"knots", {-3., -3., 5., 5.}}}));
    selection.fraction = .25;
    TubeBudget b;
    auto location = evaluate_tube_path_selection(selection, b);
    check(location.point == Point3{1, 2, 3} && location.tangent == Point3{4, 8, 12},
          "fraction tangent scales derivative in original nonunit knot domain");
    selection.curves[0] = BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                                   {"order", 2},
                                                   {"closed", false},
                                                   {"poles", {1., 0., 0., 3., 0., 0.}},
                                                   {"weights", {-1., 1.}},
                                                   {"knots", {-3., -3., 5., 5.}}});
    selection.fraction = .5;
    location = evaluate_tube_path_selection(selection, b);
    check(location.point == Point3{2, 0, 0} && location.tangent == Point3{-2, 0, 0} &&
              location.report.at("zero_weight_fallback") == true,
          "selected point exactly zero weight fallback differs from intersection zero projection");
    TubeBudget exact;
    const auto profile = group({line({1, .5, 0}, {1, .5, 0})});
    const auto expected = prepare_tube_facet_path_branches(profile, spatial, exact);
    const auto cost = exact.work;
    exact.work = 0;
    exact.max_work = cost;
    check(prepare_tube_facet_path_branches(profile, spatial, exact).report == expected.report,
          "exact full preparation shared budget");
    rejects(
        [&] {
            TubeBudget small;
            small.max_work = cost - 1;
            prepare_tube_facet_path_branches(profile, spatial, small);
        },
        "full preparation exhausted budget");
    rejects(
        [&] {
            std::size_t w = 0;
            curve_detail::native_curve_vector_planarity(spatial, 3, {w, 10000000});
        },
        "original source cumulative point limit");
    rejects([&] { plan(0, 0, 0, false, false); }, "empty branch source");
    rejects([&] { plan(2, 2, 0, false, false); }, "invalid selected member");
    rejects([&] { plan(2, 0, NAN, false, false); }, "nonfinite selected parameter");
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return build(spatial, {1, .5, 0}).report; }));
    for (auto &job : jobs)
        check(job.get() == expected.report, "parallel branch preparation");
    return count;
}
