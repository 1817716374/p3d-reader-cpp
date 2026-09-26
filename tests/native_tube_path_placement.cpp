#include "native_tube_path_placement.hpp"
#include "native_curve_affine.hpp"
#include "native_curve_segment.hpp"
#include "bspline_frame.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json line(Point3 a, Point3 b) {
    Json p;
    for (unsigned i = 0; i < 3; ++i) {
        p[std::string("point0") + "XYZ"[i]] = a[i];
        p[std::string("point1") + "XYZ"[i]] = b[i];
    }
    return {{"_type", "LineSegment"}, {"segment", p}};
}
Json group(Json values) {
    Json curves = Json::array();
    for (const auto &v : values)
        curves.push_back({{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", 1}, {"curves", curves}};
}
Json lines(std::vector<Point3> p) {
    Json curves = Json::array();
    for (std::size_t i = 1; i < p.size(); ++i)
        curves.push_back(line(p[i - 1], p[i]));
    return group(curves);
}
Point3 mapped_point(const Matrix4 &m, Point3 p, bool vector = false) {
    Point3 out{};
    for (unsigned i = 0; i < 3; ++i)
        out[i] = (vector ? 0 : m[i][3]) + m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2];
    return out;
}
bool near(Point3 a, Point3 b) {
    for (unsigned i = 0; i < 3; ++i)
        if (std::abs(a[i] - b[i]) > 1e-8 * (1 + std::abs(b[i])))
            return false;
    return true;
}
const BsplineCurve &geometry(const TubeFacetPathBranches &p, const TubePathBranch &b) {
    return b.reused_curve_index ? p.path.selection.curves[*b.reused_curve_index] : *b.constructed;
}
} // namespace
unsigned native_tube_path_placement_tests() {
    unsigned count = 0;
    auto check = [&](bool b, const char *why) {
        ++count;
        require(b, why);
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
    auto build = [&](const Json &path, Point3 reference) {
        TubeBudget b;
        return prepare_tube_facet_path_placement(group({line(reference, reference)}), path, b);
    };
    const auto straight = lines({{0, 0, 0}, {4, 0, 0}});
    auto result = build(straight, {2, 0, 0});
    check(result.success && !result.prefix_transform && result.suffix_transform,
          "whole-planar placement creates suffix transform only");
    const auto matrix = *result.suffix_transform;
    check(near(mapped_point(matrix, {2, 0, 0}), {0, 0, 0}) &&
              near(mapped_point(matrix, {1, 0, 0}, true), {0, 0, 1}) &&
              near(mapped_point(matrix, {0, 1, 0}, true), {1, 0, 0}) &&
              near(mapped_point(matrix, {0, 0, 1}, true), {0, 1, 0}),
          "straight native N B T frame columns and origin");
    check(result.report.at("relocation").at("accepted") == true &&
              result.report.at("face_patches_generated") == false,
          "placement scope and checked relocation");
    const auto spatial = lines({{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {1, 1, 1}});
    auto first = build(spatial, {.5, 0, 0});
    check(first.success && first.branches.path.selection.index == 0 && !first.prefix_transform &&
              near(mapped_point(*first.suffix_transform, {.5, 0, 0}), {0, 0, 0}),
          "first planar member placement");
    result = build(spatial, {1, .5, 0});
    check(result.success && result.prefix_transform && result.suffix_transform &&
              result.report.at("joint_method") == "aligned_joint",
          "middle planar nonparallel joint");
    check(near(mapped_point(*result.suffix_transform, {1, .5, 0}), {0, 0, 0}) &&
              near(mapped_point(*result.prefix_transform, {1, .5, 0}), {0, 0, 0}),
          "both joint placements map reference to origin");
    check(near(mapped_point(*result.suffix_transform, {0, 1, 0}, true), {0, 0, 1}) &&
              near(mapped_point(*result.prefix_transform, {0, 1, 0}, true), {0, 0, -1}),
          "joint forward and reverse traversal directions");
    check(result.branches.prefix.reused_curve_index == 0 &&
              geometry(result.branches, result.branches.prefix).point_at(0) == Point3{1, 0, 0} &&
              geometry(result.branches, result.branches.prefix).point_at(1) == Point3{0, 0, 0},
          "final prefix reversal preserves native working-member alias");
    for (Point3 target : std::vector<Point3>{{1, 1, .5}, {1, 1, 1}}) {
        auto last = build(spatial, target);
        check(last.success && last.prefix_transform && !last.suffix_transform &&
                  last.branches.path.selection.index == 2,
              "last planar member prefix-only branch");
        check(near(mapped_point(*last.prefix_transform, target), {0, 0, 0}),
              "last member reference placement");
        check(geometry(last.branches, last.branches.prefix).point_at(0) == Point3{1, 1, 1},
              "last planar entire prefix reverses after frame computation");
        check(last.report.contains("relocation") == (target[2] != 1), "near-end skips relocation");
    }
    const auto parallel = lines({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {2, 1, 0}, {2, 1, 1}});
    for (double x : {1., 1.5}) {
        auto p = build(parallel, {x, 0, 0});
        check(p.success && p.branches.path.selection.index == 1 &&
                  p.report.at("joint_method") ==
                      (x == 1 ? "parallel_at_start" : "parallel_relocated"),
              "parallel joint zero and relocated branches");
        check(near(mapped_point(*p.prefix_transform, {x, 0, 0}), {0, 0, 0}) &&
                  near(mapped_point(*p.suffix_transform, {x, 0, 0}), {0, 0, 0}),
              "parallel joint reference origin");
    }
    const auto almost = lines({{0, 0, 0}, {1, 0, 0}, {2, .001, 0}, {2, .001, 1}, {3, .001, 1}});
    auto close = build(almost, {1.5, .0005, 0});
    check(close.success &&
              close.report.at("joint_alignment").at("method") == "nearly_aligned_frame",
          "nonparallel small-angle joint retains native dot threshold branch");
    Json spline{
        {"_type", "BsplineCurve"}, {"order", 4},
        {"closed", false},         {"poles", {0., 0., 0., 1., 0., 0., 1., 1., 1., 2., 2., 0.}},
        {"weights", nullptr},      {"knots", nullptr}};
    const auto curve = BsplineCurve::from_bgfb(spline);
    for (double fraction : {0., .5, 1.}) {
        const auto at = curve.point_at(fraction);
        auto p = build(group({spline}), at);
        check(p.success && p.prefix_transform.has_value() == (fraction != 0) &&
                  p.suffix_transform.has_value() == (fraction != 1),
              "nonplanar near ends and split placement");
        if (p.prefix_transform) {
            check(near(mapped_point(*p.prefix_transform, at), {0, 0, 0}),
                  "nonplanar prefix frame at split");
            check(near(geometry(p.branches, p.branches.prefix).point_at(0), at),
                  "nonplanar prefix reversal at split");
        }
        if (p.suffix_transform)
            check(near(mapped_point(*p.suffix_transform, at), {0, 0, 0}),
                  "nonplanar suffix frame at split");
    }
    auto zero = build(lines({{0, 0, 0}, {0, 0, 0}}), {0, 0, 0});
    check(zero.success && zero.report.at("suffix_placement").at("inverse_succeeded") == false &&
              mapped_point(*zero.suffix_transform, {1, 2, 3}) == Point3{1, 2, 3},
          "native ignored singular inverse keeps identity result");
    auto tiny = spline;
    tiny["knots"] = {0., 0., 0., 0., 1.5e-10, 1.5e-10, 1.5e-10, 1.5e-10};
    rejects([&] { build(group({tiny}), curve.point_at(.5)); },
            "native empty object frame is not fabricated");
    TubeBudget budget;
    auto prepared =
        prepare_tube_facet_path_branches(group({line({2, 0, 0}, {2, 0, 0})}), straight, budget);
    prepared.path.location.point = {100, 100, 100};
    const auto failed = place_tube_facet_paths(prepared, budget);
    check(!failed.success && failed.report.at("status") == "native_failure" &&
              !failed.suffix_transform,
          "relocation mismatch returns native failure before constructing frame");
    check(native_path_points_equal({0, 0, 0}, {5e-11, 0, 0}) &&
              !native_path_points_equal({0, 0, 0}, {2e-10, 0, 0}),
          "point equality original coordinate-scale tolerance");
    check(native_path_vectors_parallel({1, 0, 0}, {-2, 0, 0}) &&
              native_path_vectors_parallel({0, 0, 0}, {1, 2, 3}) &&
              !native_path_vectors_parallel({1, 0, 0}, {1, 2e-12, 0}),
          "native parallel predicate includes opposite and zero vectors");
    for (double small : {.5e-12, 2e-12})
        check(native_path_vectors_parallel({1, 0, 0}, {1, small, 0}) == (small < 1e-12),
              "parallel angular threshold");
    rejects([&] { native_path_points_equal({NAN, 0, 0}, {0, 0, 0}); },
            "nonfinite relocation arithmetic");
    // Several joint orientations exercise both signs/quadrants of the native
    // atan2 rotation without duplicating the matrix-construction algorithm.
    for (double angle : {0., .5, 1.8, 3.6, 5.4}) {
        Point3 direction{0, std::cos(angle), std::sin(angle)};
        Point3 target{0, .5 * direction[1], .5 * direction[2]};
        auto path = lines({{-1, 0, 0},
                           {0, 0, 0},
                           direction,
                           {1, direction[1], direction[2]},
                           {1, direction[1] - direction[2], direction[2] + direction[1]}});
        auto p = build(path, target);
        check(p.success && p.branches.path.selection.index == 1,
              "spatial joint orientation selection");
        check(near(mapped_point(*p.prefix_transform, target), {0, 0, 0}) &&
                  near(mapped_point(*p.suffix_transform, target), {0, 0, 0}),
              "rotated joint origin alignment");
        check(near(mapped_point(*p.prefix_transform, direction, true), {0, 0, -1}) &&
                  near(mapped_point(*p.suffix_transform, direction, true), {0, 0, 1}),
              "rotated joint traversal axes");
    }
    auto rational = spline;
    rational["weights"] = {.7, 1.9, 2.3, .8};
    for (unsigned i = 0; i < 4; ++i)
        for (unsigned k = 0; k < 3; ++k)
            rational["poles"][3 * i + k] =
                rational["poles"][3 * i + k].get<double>() * rational["weights"][i].get<double>();
    const auto original = rational;
    const auto source = BsplineCurve::from_bgfb(rational);
    TubeBudget rb;
    auto ready = prepare_tube_facet_path_branches(
        group({line(source.point_at(.5), source.point_at(.5))}), group({rational}), rb);
    auto prefix_copy = geometry(ready, ready.prefix);
    auto suffix_copy = geometry(ready, ready.suffix);
    Json reversal;
    std::size_t reversal_work = 0;
    curve_detail::reverse_native_working_curve(prefix_copy, 10000, {reversal_work, 1000000},
                                               reversal);
    const auto expected_work = native_bspline_frame_working(suffix_copy, 0);
    auto placed = place_tube_facet_paths(ready, rb);
    check(geometry(placed.branches, placed.branches.prefix).poles() == prefix_copy.poles() &&
              geometry(placed.branches, placed.branches.suffix).poles() ==
                  expected_work.working_poles &&
              rational == original,
          "temporary reversed-frame working changes do not leak into original prefix");
    auto direct = source;
    curve_detail::reverse_native_working_curve(direct, 10000, {reversal_work, 1000000}, reversal);
    auto reversed_poles = source.poles();
    std::reverse(reversed_poles.begin(), reversed_poles.end());
    check(direct.poles() == reversed_poles,
          "direct open reversal has no extra knot-tolerance round trip");
    auto bad_open =
        BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                 {"order", 2},
                                 {"closed", true},
                                 {"poles", {0., 0., 0., 1., 0., 0., 1., 1., 0., 0., 1., 1.}},
                                 {"weights", nullptr},
                                 {"knots", {0., 2., 4., 6., 8., 10., 12.}}});
    auto invalid_temporary = ready;
    invalid_temporary.prefix.reused_curve_index.reset();
    invalid_temporary.prefix.constructed = bad_open;
    rejects(
        [&] {
            TubeBudget b;
            place_tube_facet_paths(invalid_temporary, b);
        },
        "failed opening into a new reverse destination does not use the original curve as "
        "fallback");
    rational["poles"][8] = 0;
    const auto flat_rational = BsplineCurve::from_bgfb(rational);
    auto flat_placement = build(group({rational}), flat_rational.point_at(.4));
    check(flat_placement.branches.whole_path_planarity.at("planar") == true,
          "rational whole-planar branch");
    const auto selected_frame = native_bspline_frame_working(
        flat_rational, flat_placement.report.at("resolved_fraction").get<double>());
    const auto start_frame = native_bspline_frame_working(
        curve_detail::with_poles(flat_rational, selected_frame.working_poles), 0);
    check(geometry(flat_placement.branches, flat_placement.branches.suffix).poles() ==
              start_frame.working_poles,
          "whole planar selected/start frames carry ordered source working state");
    const auto profile = group({line({1, .5, 0}, {1, .5, 0})});
    TubeBudget exact;
    const auto expected = prepare_tube_facet_path_placement(profile, spatial, exact);
    const auto cost = exact.work;
    exact.work = 0;
    exact.max_work = cost;
    check(prepare_tube_facet_path_placement(profile, spatial, exact).report == expected.report,
          "exact shared placement budget");
    rejects(
        [&] {
            TubeBudget b;
            b.max_work = cost - 1;
            prepare_tube_facet_path_placement(profile, spatial, b);
        },
        "placement work budget exhaustion");
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return build(spatial, {1, .5, 0}).report; }));
    for (auto &job : jobs)
        check(job.get() == expected.report, "parallel placement independent working curves");
    return count;
}
