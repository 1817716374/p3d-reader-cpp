#include "native_tube_curvature.hpp"
#include "native_curve_range.hpp"
#include "native_curve_affine.hpp"
#include "bspline_frame.hpp"
namespace p3d::swept_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native facet curvature nonfinite arithmetic");
    return x;
}
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
struct Query {
    BsplineCurve working;
    TubeBudget &budget;
    Json visits = Json::array();
    std::array<double, 2> evaluate(double fraction, Point3 *point = nullptr) {
        const auto n = working.poles().size(), order = std::size_t(working.order());
        require(order >= 2 && order <= 26 && n <= budget.max_control_points,
                "native facet curvature source order/control limit");
        for (unsigned i = 0; i < 16; ++i)
            charge(budget, n);
        charge(budget, working.knots().size());
        charge(budget, 8 * order * order * order + 128);
        const auto f = native_bspline_frame_working(working, fraction);
        working = curve_detail::with_poles(working, f.working_poles);
        const double curvature = f.report.at("curvature").get<double>();
        if (point)
            for (unsigned i = 0; i < 3; ++i)
                (*point)[i] = f.report.at("frame").at(i).at(3).get<double>();
        visits.push_back({{"fraction", fraction},
                          {"curvature", curvature},
                          {"point_requested", point != nullptr}});
        return {fraction, curvature};
    }
};
struct Tree {
    Query query;
    double tolerance;
    std::vector<TubeCurvatureSample> nodes;
    unsigned deepest = 0;
    std::size_t add(double fraction, unsigned depth, bool point) {
        require(nodes.size() < query.budget.max_control_points,
                "native curvature tree node budget exceeded");
        charge(query.budget, 1);
        TubeCurvatureSample node;
        node.fraction = fraction;
        node.depth = depth;
        if (point)
            node.point = Point3{};
        node.curvature = query.evaluate(fraction, point ? &*node.point : nullptr)[1];
        deepest = std::max(deepest, depth);
        nodes.push_back(std::move(node));
        return nodes.size() - 1;
    }
    bool split(std::size_t i, std::optional<std::size_t> left_bound,
               std::optional<std::size_t> right_bound) {
        charge(query.budget, 1);
        // Missing boundary ancestors trigger another endpoint query, even if
        // the same endpoint has already been evaluated in another recursion.
        const auto a = left_bound ? std::array<double, 2>{nodes[*left_bound].fraction,
                                                          nodes[*left_bound].curvature}
                                  : query.evaluate(0);
        const auto z = right_bound ? std::array<double, 2>{nodes[*right_bound].fraction,
                                                           nodes[*right_bound].curvature}
                                   : query.evaluate(1);
        const double center = nodes[i].fraction;
        const auto depth = nodes[i].depth;
        const auto left = add(finite((a[0] + center) * .5), depth + 1, true);
        nodes[i].left = left;
        const auto right = add(finite((z[0] + center) * .5), depth + 1, true);
        nodes[i].right = right;
        // Native checks depth only after allocating/evaluating both children.
        if (depth > 100)
            return false;
        const bool refine =
            std::abs(finite(finite(z[1] * .25 + a[1] * .75) - nodes[left].curvature)) > tolerance ||
            std::abs(finite(finite((z[1] + a[1]) * .5) - nodes[i].curvature)) > tolerance ||
            std::abs(finite(finite(z[1] * .75 + a[1] * .25) - nodes[right].curvature)) > tolerance;
        if (!refine)
            return true;
        return split(left, left_bound, i) && split(right, i, right_bound);
    }
    void collect(std::size_t i, std::vector<std::array<double, 2>> &out) {
        charge(query.budget, 1);
        const auto &node = nodes[i];
        if (!node.left || !node.right || !nodes[*node.left].left || !nodes[*node.right].right)
            return;
        collect(*node.left, out);
        out.push_back({node.fraction, node.curvature});
        collect(*node.right, out);
    }
};
bool near_zero(double x) {
    return std::abs(x) <= finite((std::abs(x) + 1) * 1e-10);
}
} // namespace
TubeCurvatureSampling sample_tube_facet_curvature(const BsplineCurve &source, double tolerance,
                                                  TubeBudget &budget) {
    require(!std::isnan(tolerance) && tolerance >= 0, "native facet curvature tolerance");
    Tree tree{{source, budget}, tolerance, {}, 0};
    const auto root = tree.add(.5, 0, false);
    TubeCurvatureSampling out;
    out.success = tree.split(root, {}, {});
    if (out.success) {
        tree.collect(root, out.interior);
        out.tree = std::move(tree.nodes);
    } // On native depth failure the complete temporary tree is freed.
    out.working_poles = tree.query.working.poles();
    out.report = {{"scope", "native_facet_curvature_sampling"},
                  {"native_result", out.success},
                  {"tolerance", std::isfinite(tolerance) ? Json(tolerance) : Json(nullptr)},
                  {"unbounded_tolerance", std::isinf(tolerance)},
                  {"deepest_probe", tree.deepest},
                  {"queries", std::move(tree.query.visits)},
                  {"interior_samples", out.interior},
                  {"failure", out.success ? Json(nullptr) : Json("native_depth_limit")},
                  {"work_used", budget.work}};
    return out;
}
TubeFacetSampling prepare_tube_facet_sampling(const BsplineCurve &trace, TubeBudget &budget) {
    const auto range = curve_detail::native_curve_range(trace, budget.max_control_points,
                                                        {budget.work, budget.max_work});
    require(range.present, "native facet sampling has no finite path range");
    Point3 delta{};
    for (unsigned i = 0; i < 3; ++i)
        delta[i] = finite(range.high[i] - range.low[i]);
    const double diagonal = finite(
        std::sqrt(finite(finite(delta[1] * delta[1] + delta[0] * delta[0]) + delta[2] * delta[2])));
    const double scaled = finite(diagonal * 20);
    const double tolerance =
        scaled == 0 ? std::numeric_limits<double>::infinity() : finite(1 / scaled);
    TubeFacetSampling out;
    out.sampling = sample_tube_facet_curvature(trace, tolerance, budget);
    out.report = {{"scope", "native_facet_sampling_preparation"},
                  {"range_low", range.low},
                  {"range_high", range.high},
                  {"range_diagonal", diagonal},
                  {"surface_generated", false}};
    if (!out.sampling.success) {
        out.report["status"] = "native_failure";
        out.report["work_used"] = budget.work;
        return out;
    }
    Query endpoint{curve_detail::with_poles(trace, out.sampling.working_poles), budget};
    out.start_curvature = endpoint.evaluate(0)[1];
    out.end_curvature = endpoint.evaluate(1)[1];
    out.sampling.working_poles = endpoint.working.poles();
    out.ruled_fallback = out.sampling.interior.empty() &&
                         (near_zero(out.start_curvature) || near_zero(out.end_curvature));
    out.success = true;
    out.report["status"] = "complete";
    out.report["endpoint_queries"] = std::move(endpoint.visits);
    out.report["ruled_fallback"] = out.ruled_fallback;
    out.report["work_used"] = budget.work;
    return out;
}
} // namespace p3d::swept_detail
