#include "native_tube_transform.hpp"
#include "native_curve_affine.hpp"
#include <p3d/solid.hpp>
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Matrix4 identity() {
    Matrix4 m{};
    for (unsigned i = 0; i < 4; ++i)
        m[i][i] = 1;
    return m;
}
Json curve(Json poles = {2., 3., 4., 5., 6., 7.}, Json weights = nullptr, bool closed = false) {
    return {{"_type", "BsplineCurve"}, {"order", 2},         {"closed", closed},
            {"poles", poles},          {"weights", weights}, {"knots", nullptr}};
}
Json group(const Json &c) {
    return {{"_type", "CurveVector"}, {"type", 1}, {"curves", Json::array({{{"geometry", c}}})}};
}
std::shared_ptr<const BsplineCurve> view(const Json &c) {
    return std::make_shared<const BsplineCurve>(BsplineCurve::from_bgfb(c));
}
TubeCurveTransform transform(const TubeCurveViews &v, const Matrix4 *m, bool clone) {
    TubeBudget b;
    return transform_tube_curve_list(v, m, clone, b);
}
} // namespace
unsigned native_tube_transform_tests() {
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
    const auto source = view(curve()), equal_but_distinct = view(curve());
    auto m = identity();
    m[0][3] = 10;
    auto r = transform({source, source, equal_but_distinct}, &m, false);
    check(r.report["native_result"] == true && r.curves[0] == r.curves[1] &&
              r.curves[2] != r.curves[0],
          "in-place native transform preserves only actual source pointer aliases");
    check(r.curves[0]->poles()[0][0] == 22 && r.curves[2]->poles()[0][0] == 12 &&
              source->poles()[0][0] == 2,
          "repeated native references accumulate transforms while immutable source remains "
          "unchanged");
    check(r.report["first_reference_indices"] == Json::array({0, 0, 2}) &&
              r.report["working_copies"] == 2,
          "working copy report distinguishes native alias from equal geometry");
    r = transform({source, source, equal_but_distinct}, &m, true);
    check(r.curves[0] != r.curves[1] && r.curves[0] != source && r.curves[1] != r.curves[2],
          "clone mode creates a distinct native curve for every occurrence");
    for (const auto &c : r.curves)
        check(c->poles()[0][0] == 12, "cloning starts every occurrence from original controls");
    r = transform({source, nullptr, source, equal_but_distinct}, &m, false);
    check(r.report["native_result"] == false && r.report["failure_index"] == 1 &&
              r.curves.size() == 4,
          "in-place null failure retains entire native list");
    check(
        r.curves[0] == r.curves[2] && r.curves[2]->poles()[0][0] == 12 &&
            r.curves[3] == equal_but_distinct,
        "alias after failure sees earlier mutation but unrelated unvisited source remains shared");
    r = transform({source, nullptr, source}, &m, true);
    check(r.report["native_result"] == false && r.curves.size() == 1 &&
              r.curves[0]->poles()[0][0] == 12,
          "clone failure retains only successfully cloned prefix");
    for (bool clone : {false, true}) {
        auto id = identity();
        for (const Matrix4 *matrix :
             {static_cast<const Matrix4 *>(nullptr), static_cast<const Matrix4 *>(&id)}) {
            r = transform({source, source}, matrix, clone);
            check(r.report["transform_skipped"] == true && r.curves[0]->poles() == source->poles(),
                  "null or identity matrix skips native control arithmetic");
            check(clone ? (r.curves[0] != source && r.curves[0] != r.curves[1])
                        : (r.curves[0] == source && r.curves[1] == source),
                  "identity transform still respects native clone choice");
        }
    }
    auto small = identity();
    small[0][3] = std::nextafter(1e-10, 0.);
    check(transform({source}, &small, false).curves[0] == source,
          "near-identity translation inside strict bound preserves source identity");
    small[0][3] = 1e-10;
    check(transform({source}, &small, false).curves[0] != source,
          "translation exactly at native identity bound executes transform");
    auto weighted = view(curve({2., 3., 4., 5., 6., 7.}, {0., -2.}));
    r = transform({weighted}, &m, false);
    check(r.curves[0]->weights() == weighted->weights() && r.curves[0]->poles()[0][0] == 2 &&
              r.curves[0]->poles()[1][0] == -15,
          "weighted transforms retain zero and negative weights without deweighting");
    auto rounding = identity();
    rounding[0] = {1., 1., 0., -1e16};
    const auto polynomial = curve({1e16, 1., 0., 1e16, 2., 0.});
    auto rational = polynomial;
    rational["weights"] = {1., 1.};
    auto poly_result = transform({view(polynomial)}, &rounding, false);
    auto rat_result = transform({view(rational)}, &rounding, false);
    check(poly_result.curves[0]->poles()[0][0] == 1 && rat_result.curves[0]->poles()[0][0] == 0,
          "native unweighted and explicit-unit-weight kernels have different translation addition "
          "order");
    for (const auto &data : {polynomial, rational}) {
        const Json body{{"_type", "P3DSweptBody"}, {"profile", group(data)}, {"path", nullptr}};
        const auto placed = transform_bgfb_curve_solid(body, rounding);
        check(placed.status == "transformed" &&
                  placed.transformed["profile"]["curves"][0]["geometry"]["poles"][0] ==
                      (data["weights"].is_null() ? 1. : 0.),
              "public source-solid transform uses the same native B-spline arithmetic branches");
    }
    auto periodic = view(curve({2., 3., 4., 5., 6., 7., 8., 2., 1.}, nullptr, true));
    r = transform({periodic}, &m, true);
    check(r.curves[0]->closed() && r.curves[0]->source_knots().empty() &&
              r.curves[0]->knots() == periodic->knots(),
          "control transform preserves closed state and implicit knot representation");
    auto signed_zero = BsplineCurve::from_bgfb(curve({-0., 0., 0., 1., 0., 0.}));
    auto changed = signed_zero.poles();
    changed[0][0] = 0.;
    auto replaced = curve_detail::with_poles(signed_zero, changed);
    check(std::signbit(signed_zero.poles()[0][0]) && !std::signbit(replaced.poles()[0][0]),
          "control replacement does not erase a native signed-zero change");
    auto ignored_row = m;
    ignored_row[3] = {3, 4, 5, std::numeric_limits<double>::quiet_NaN()};
    check(transform({source}, &ignored_row, false).curves[0]->poles() ==
              transform({source}, &m, false).curves[0]->poles(),
          "native three-row transform does not consume carrier matrix last row");
    auto invalid = m;
    invalid[0][0] = std::numeric_limits<double>::infinity();
    check(transform({}, &invalid, true).report["native_result"] == true,
          "empty native list never evaluates unused matrix");
    check(transform({nullptr}, &invalid, false).report["native_result"] == false,
          "null source failure precedes matrix evaluation");
    rejects([&] { transform({source}, &invalid, false); },
            "consumed nonfinite matrix is rejected explicitly");
    TubeBudget measured;
    transform_tube_curve_list({source, source}, &m, false, measured);
    TubeBudget bounded;
    bounded.max_work = measured.work - 1;
    rejects([&] { transform_tube_curve_list({source, source}, &m, false, bounded); },
            "alias transformations share one cumulative work budget");
    TubeBudget controls;
    controls.max_control_points = 2;
    check(
        transform_tube_curve_list({source, source}, &m, false, controls).curves[0]->poles()[0][0] ==
            22,
        "native alias state allocates one bounded working curve");
    controls = {};
    controls.max_control_points = 3;
    rejects([&] { transform_tube_curve_list({source, source}, &m, true, controls); },
            "cloned duplicate output storage is bounded per actual native copy");
    const auto original = std::make_shared<const Json>(group(curve()));
    TubeBudget partition_budget;
    const auto partition = partition_tube_facet_profile(original, partition_budget);
    for (bool clone : {false, true}) {
        TubeBudget b;
        const auto branches = prepare_tube_facet_branches(partition.groups[0][0], &m, clone, b);
        check(branches.transformed[0]->poles()[0][0] == 12 &&
                  branches.original[0]->poles()[0][0] == (clone ? 2 : 12),
              "facet preparation connects conversion and native branch transform");
        check((branches.original[0] == branches.transformed[0]) == !clone,
              "original and transformed branch alias only when native clone flag is false");
    }
    auto future = std::async(std::launch::async, [&] {
        return transform({source, source}, &m, false).curves[0]->poles();
    });
    check(future.get() == transform({source, source}, &m, false).curves[0]->poles() &&
              source->poles()[0][0] == 2 && *original == group(curve()),
          "independent transforms share immutable sources safely across concurrent calls");
    return count;
}
