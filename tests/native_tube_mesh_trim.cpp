#include "native_tube_mesh_trim.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineCurve scalar(double a, double b, Json knots = nullptr) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", 2},
                                    {"closed", false},
                                    {"knots", knots},
                                    {"poles", {a, 0, 0, b, 0, 0}},
                                    {"weights", nullptr}});
}
TubeMeshBoundaryCurves bounds(std::optional<BsplineCurve> lower = {},
                              std::optional<BsplineCurve> upper = {}) {
    TubeMeshBoundaryCurves b;
    b.success = true;
    b.lower = std::move(lower);
    b.upper = std::move(upper);
    return b;
}
TubeMeshTrimPlan plan(const TubeMeshBoundaryCurves &b, std::vector<double> v = {0, .25, .5, .75, 1},
                      std::vector<double> u = {0, .5, 1}, std::array<double, 2> interval = {0, 1}) {
    TubeBudget budget;
    return prepare_tube_mesh_trim_columns(b, u, v, interval, budget);
}
std::vector<double> values(const TubeMeshTrimColumn &c, const std::vector<double> &samples) {
    std::vector<double> out;
    for (std::int64_t i = std::int64_t(c.first_interior) - 1;
         i <= std::int64_t(c.last_interior) + 1; ++i)
        out.push_back(i == c.first_interior - 1  ? c.lower
                      : i == c.last_interior + 1 ? c.upper
                                                 : samples.at(std::size_t(i)));
    return out;
}
bool near(double a, double b) {
    return std::abs(a - b) < 2e-11;
}
Json surface() {
    return {{"_type", "BsplineSurface"},
            {"orderU", 2},
            {"orderV", 3},
            {"numPolesU", 2},
            {"numPolesV", 3},
            {"closedU", false},
            {"closedV", false},
            {"poles", {0, 0, 0, 1, 0, 0, 0, .5, 0, 1, 1, 0, 0, 1, 1, 1, 2, 1}},
            {"weights", nullptr},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"numRulesU", 0},
            {"numRulesV", 0},
            {"holeOrigin", 0},
            {"boundaries", nullptr}};
}
} // namespace
unsigned native_tube_mesh_trim_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool thrown = false;
        try {
            fn();
        } catch (const std::exception &) {
            thrown = true;
        }
        check(thrown, "native trim preparation invalid data or budgets reject");
    };
    const std::vector<double> v{0, .25, .5, .75, 1};
    auto regular = plan(bounds());
    check(regular.success && regular.vertex_count == 15,
          "missing two bounds use original endpoint values");
    for (std::size_t i = 0; i < 3; ++i) {
        const auto &c = regular.columns[i];
        check(c.first_interior == 1 && c.last_interior == 3 && c.offset == 5 * i && c.count == 5 &&
                  values(c, v) == v,
              "default signed range reserves two boundary vertices");
    }
    auto cut = plan(bounds(scalar(.2, .2), scalar(.8, .8)));
    check(cut.success && values(cut.columns[0], v) == std::vector<double>{.2, .25, .5, .75, .8},
          "non-grid bounds retain all interior V samples plus exact endpoints");
    auto exact = plan(bounds(scalar(.25, .25), scalar(.75, .75)));
    check(exact.columns[0].first_interior == 2 && exact.columns[0].last_interior == 2 &&
              exact.columns[0].lower_snapped &&
              values(exact.columns[0], v) == std::vector<double>{.25, .5, .75},
          "exact lower advances index and exact upper excludes its grid row");
    const double below = .25 - 5e-11, above = .25 + 5e-11, upper = .75 - 5e-11;
    auto snapped = plan(bounds(scalar(below, below), scalar(upper, upper)));
    check(snapped.columns[0].lower_snapped && snapped.columns[0].lower == .25 &&
              snapped.columns[0].upper == upper,
          "only lower snaps to the first not-less sample");
    auto one_sided = plan(bounds(scalar(above, above)));
    check(!one_sided.columns[0].lower_snapped && one_sided.columns[0].lower == above &&
              one_sided.columns[0].first_interior == 2,
          "lower tolerance does not search the preceding sample");
    auto no_lower = plan(bounds({}, scalar(.6, .6)));
    auto no_upper = plan(bounds(scalar(.6, .6), {}));
    check(values(no_lower.columns[0], v) == std::vector<double>{0, .25, .5, .6} &&
              values(no_upper.columns[0], v) == std::vector<double>{.6, .75, 1},
          "independent missing-side defaults");
    auto duplicate = plan(bounds(scalar(.25, .25)), {0, .25, .25, 1});
    check(values(duplicate.columns[0], {0, .25, .25, 1}) == std::vector<double>{.25, .25, 1},
          "duplicate V samples retain distinct original row positions");
    auto repeated = plan(bounds(scalar(.1, .1)), v, {.5, 0, .5});
    check(repeated.columns[0].local_u == .5 && repeated.columns[1].local_u == 0 &&
              repeated.columns[2].local_u == .5,
          "source U order and repeated columns remain untouched");
    auto sub =
        plan(bounds(scalar(.1, .9, {-3, -3, 7, 7}), scalar(1, 1)), v, {0, .5, 1}, {.25, .75});
    check(near(sub.columns[0].lower, .3) && near(sub.columns[1].lower, .5) &&
              near(sub.columns[2].lower, .7),
          "local U maps to original interval before full scalar-curve knot mapping");
    auto low_out = plan(bounds(scalar(-.2, -.2), scalar(0, 0)));
    check(low_out.columns[0].first_interior == 0 && low_out.columns[0].last_interior == -1 &&
              values(low_out.columns[0], v) == std::vector<double>{-.2, 0},
          "negative row sentinel supports exact below-domain lower value");
    auto point = plan(bounds(scalar(1, 1), scalar(1, 1)));
    check(point.success && point.columns[0].count == 1 &&
              values(point.columns[0], v) == std::vector<double>{1},
          "coincident end boundaries produce one lower-precedence vertex");
    auto inverted = plan(bounds(scalar(.9, .9), scalar(.1, .1)));
    check(inverted.success && inverted.columns[0].count == 0 && inverted.vertex_count == 0,
          "reversed limits retain empty native loops without reordering or inventing geometry");
    auto crossing = plan(bounds(scalar(.6, .6), scalar(.5, .5)));
    check(crossing.success && values(crossing.columns[0], v) == std::vector<double>{.6},
          "single overlapping row chooses lower branch even when lower exceeds upper");
    auto outside = plan(bounds(scalar(.2, 1 + 1e-12)));
    check(!outside.success && outside.columns.empty() && outside.vertex_count == 0 &&
              outside.report["side"] == "lower" && outside.report["column"] == 2,
          "late out-of-range lower aborts before publishing any partial column plan");
    outside = plan(bounds({}, scalar(1 + 1e-12, 1 + 1e-12)));
    check(!outside.success && outside.report["side"] == "upper",
          "upper above last sample fails even within near tolerance");
    // Deliberate zero evaluated W uses the existing native curve caller's
    // exactly-zero fallback. Surface sampling has a different zero-W rule.
    auto zero_curve = BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                               {"order", 2},
                                               {"closed", false},
                                               {"knots", nullptr},
                                               {"poles", {.2, 0, 0, .2, 0, 0}},
                                               {"weights", {0, 0}}});
    auto zero = plan(bounds(zero_curve));
    check(zero.success && zero.columns[0].lower == .2 &&
              zero.report["curve_zero_weight_fallbacks"] == 3,
          "scalar boundary queries preserve native curve zero-W fallback");
    SweptBodyPatchGroup group;
    group.patches = {
        {surface(), {{{0, .1}, {.5, .2}, {1, .1}, {1, .9}, {.5, .8}, {0, .9}, {0, .1}}}}};
    TubeBudget build;
    auto prepared = prepare_tube_mesh_grid(group, false, .01, .1, 100000, build);
    check(prepared.success && prepared.patches[0].preparation.boundaries.lower &&
              prepared.patches[0].preparation.boundaries.upper,
          "actual runtime polygon creates two native scalar boundary functions");
    TubeBudget eval;
    auto actual = evaluate_tube_mesh_trim_vertices(prepared.patches[0], prepared.section, 0, eval);
    check(actual.plan.success && actual.vertices.size() == actual.plan.vertex_count &&
              !actual.vertices.empty(),
          "prepared original patch feeds variable trimmed-column vertex evaluation");
    for (const auto &column : actual.plan.columns) {
        check(actual.vertices[column.offset].geometry_v == column.lower &&
                  actual.vertices[column.offset + column.count - 1].geometry_v == column.upper,
              "trimmed columns include original lower and upper boundary geometry values");
        for (std::size_t j = 0; j < column.count; ++j) {
            const auto &node = actual.vertices[column.offset + j];
            double u = column.local_u, w = node.geometry_v;
            check(near(node.point[0], u) && near(node.point[1], (1 + u) * w) &&
                      near(node.point[2], w * w),
                  "variable column points match independent analytic surface");
            const auto normal = Point3{2 * w * w, -2 * w, 1 + u};
            const double norm =
                std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
            check(near(node.normal[0], normal[0] / norm) &&
                      near(node.normal[1], normal[1] / norm) &&
                      near(node.normal[2], normal[2] / norm),
                  "trimmed geometry normal keeps local DU cross DV orientation");
        }
    }
    auto failed = prepared.patches[0];
    failed.preparation.boundaries.upper = scalar(2, 2);
    TubeBudget failed_budget;
    auto failed_vertices =
        evaluate_tube_mesh_trim_vertices(failed, prepared.section, 0, failed_budget);
    check(!failed_vertices.plan.success && failed_vertices.vertices.empty(),
          "no vertex evaluation follows failed boundary planning");
    auto singleton = prepared.patches[0];
    singleton.preparation.boundaries = bounds(scalar(.6, .6), scalar(.5, .5));
    singleton.path.parameters = v;
    TubeBudget sb;
    auto single = evaluate_tube_mesh_trim_vertices(singleton, prepared.section, 0, sb);
    check(single.vertices.size() == single.plan.columns.size() &&
              single.vertices[0].geometry_v == .6,
          "actual sampling respects lower precedence for one-record columns");
    rejects([&] { plan(bounds(), {0, 1, .5}); });
    rejects([&] { plan(bounds(), {0}); });
    rejects([&] { plan(bounds(), v, {0}); });
    rejects([&] {
        TubeBudget b;
        b.max_work = 0;
        prepare_tube_mesh_trim_columns(bounds(), {0, 1}, v, {0, 1}, b);
    });
    rejects([&] {
        TubeBudget b;
        b.max_control_points = 8;
        prepare_tube_mesh_trim_columns(bounds(), {0, .5, 1}, v, {0, 1}, b);
    });
    auto work = [&]() {
        TubeBudget b;
        return evaluate_tube_mesh_trim_vertices(prepared.patches[0], prepared.section, 0, b)
            .vertices.size();
    };
    auto async = std::async(std::launch::async, work);
    check(async.get() == work() &&
              prepared.patches[0].preparation.boundaries.lower->point_at(0)[0] == .1,
          "parallel trimmed sampling leaves original boundary curves immutable");
    return n;
}
