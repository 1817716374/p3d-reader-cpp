#include "native_tube_facet_uv.hpp"
namespace p3d::swept_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native facet UV sampling nonfinite arithmetic");
    return x;
}
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
struct Tree {
    const TubeFacetUvQuery &query;
    std::array<double, 2> interval;
    double tolerance;
    TubeBudget &budget;
    std::vector<TubeFacetUvNode> nodes;
    Json visits = Json::array();
    unsigned deepest = 0;
    std::string failure{};
    std::optional<TubeFacetUvNode> evaluate(double u) {
        charge(budget, 8);
        const auto p = query(finite(u));
        if (p)
            for (double x : *p)
                finite(x);
        visits.push_back({{"fraction", u}, {"parameters", p ? Json(*p) : Json()}});
        if (!p) {
            failure = "native_sample_failure";
            return std::nullopt;
        }
        TubeFacetUvNode out;
        out.fraction = u;
        out.parameters = *p;
        return out;
    }
    std::size_t add(TubeFacetUvNode node, unsigned depth) {
        require(nodes.size() < budget.max_control_points, "native facet UV tree node budget");
        charge(budget, 1);
        node.depth = depth;
        deepest = std::max(deepest, depth);
        nodes.push_back(std::move(node));
        return nodes.size() - 1;
    }
    bool split(std::size_t i, std::optional<std::size_t> low, std::optional<std::size_t> high) {
        charge(budget, 1);
        // These bounds are exactly the predecessor/successor ancestors found
        // by the native parent/is-left links. Missing ancestors cause fresh
        // original-interval endpoint queries on every recursive visit.
        const auto a = low ? std::optional<TubeFacetUvNode>{nodes[*low]} : evaluate(interval[0]);
        if (!a)
            return false;
        const auto z = high ? std::optional<TubeFacetUvNode>{nodes[*high]} : evaluate(interval[1]);
        if (!z)
            return false;
        const auto center = nodes[i];
        auto l = evaluate(finite((center.fraction + a->fraction) * .5));
        if (!l)
            return false;
        const auto left = add(*l, center.depth + 1);
        nodes[i].left = left;
        auto r = evaluate(finite((center.fraction + z->fraction) * .5));
        if (!r)
            return false;
        const auto right = add(*r, center.depth + 1);
        nodes[i].right = right;
        // Native allocates and evaluates both children before this check.
        if (center.depth > 300) {
            failure = "native_depth_limit";
            return false;
        }
        bool refine = false;
        for (unsigned side = 0; side < 2 && !refine; ++side) {
            const double x = a->parameters[side], y = z->parameters[side];
            // Do not fold 3*.25 to .75 or fuse multiply-add: native rounding
            // occurs after x*3 before its multiplication by .25.
            refine = std::abs(finite(finite(finite(x * 3) * .25 + y * .25) - l->parameters[side])) >
                         tolerance ||
                     std::abs(finite(finite((y + x) * .5) - center.parameters[side])) > tolerance ||
                     std::abs(finite(finite(finite(y * 3) * .25 + x * .25) - r->parameters[side])) >
                         tolerance;
        }
        if (!refine)
            return true; // Keep both quarter probes in the accepted tree.
        return split(left, low, i) && split(right, i, high);
    }
    void collect(std::size_t i, TubeFacetUvTree &out) {
        charge(budget, 3);
        const auto &node = nodes[i];
        if (node.left)
            collect(*node.left, out);
        out.first.push_back({node.fraction, node.parameters[0], 0});
        out.second.push_back({node.fraction, node.parameters[1], 0});
        if (node.right)
            collect(*node.right, out);
    }
};
} // namespace
TubeFacetUvTree sample_tube_facet_uv_tree(const TubeFacetUvQuery &query,
                                          const std::array<double, 2> &interval, double tolerance,
                                          TubeBudget &b) {
    require(bool(query), "native facet UV sampling missing evaluator");
    require(finite(interval[0]) <= finite(interval[1]), "native facet UV interval order");
    require(finite(tolerance) >= 0, "native facet UV tolerance");
    Tree tree{query, interval, tolerance, b, {}};
    TubeFacetUvTree out;
    // The native root allocation precedes its query. Enforce its storage
    // allowance first; child storage allowances are checked after queries.
    require(b.max_control_points > 0, "native facet UV tree root budget");
    const auto root = tree.evaluate(finite((interval[1] + interval[0]) * .5));
    if (root) {
        tree.add(*root, 0);
        out.success = tree.split(0, {}, {});
    }
    if (out.success) {
        tree.collect(0, out);
        out.tree = std::move(tree.nodes);
    } // Any native failure frees the entire temporary tree, not a prefix.
    out.report = {{"scope", "native_facet_uv_tree"},
                  {"native_result", out.success},
                  {"interval", interval},
                  {"tolerance", tolerance},
                  {"deepest_probe", tree.deepest},
                  {"queries", std::move(tree.visits)},
                  {"interior_count", out.first.size()},
                  {"failure", out.success ? Json() : Json(tree.failure)},
                  {"work_used", b.work}};
    return out;
}
TubeFacetUvSegments sample_tube_facet_uv_segments(const BsplineCurve &profile,
                                                  const TubeFacetUvQuery &query, TubeBudget &b) {
    require(bool(query), "native facet UV segments missing evaluator");
    const auto plan = native_tube_facet_knot_data(profile, b);
    const auto values = plan.at("values").get<std::vector<double>>();
    const auto begin = plan.at("left_active_index").get<std::size_t>();
    const auto end = plan.at("right_active_index").get<std::size_t>();
    require(begin <= end && end < values.size(), "native facet UV active knot plan");
    TubeFacetUvSegments out;
    Json intervals = Json::array(), endpoint_queries = Json::array();
    std::string failure;
    std::size_t output_points = 0;
    auto endpoint = [&](double u) {
        charge(b, 8);
        auto p = query(u);
        if (p)
            for (double x : *p)
                finite(x);
        endpoint_queries.push_back({{"fraction", u}, {"parameters", p ? Json(*p) : Json()}});
        return p;
    };
    auto append = [&](std::vector<Point3> &a, std::vector<Point3> &z, double u, const Point2 &p) {
        require(output_points <= b.max_control_points && 2 <= b.max_control_points - output_points,
                "native facet UV segment output budget");
        charge(b, 2);
        output_points += 2;
        a.push_back({u, p[0], 0});
        z.push_back({u, p[1], 0});
    };
    for (std::size_t i = begin; i < end; ++i) {
        const double low = values[i], high = values[i + 1];
        const auto p = endpoint(low);
        if (!p) {
            failure = "native_low_sample_failure";
            break;
        }
        // The next interval's low query supplies the previous segment's end.
        // It is not an independently evaluated preceding high endpoint.
        if (!out.first.empty() && !out.second.empty())
            append(out.first.back(), out.second.back(), low, *p);
        std::vector<Point3> a, z;
        append(a, z, low, *p);
        auto sampled = sample_tube_facet_uv_tree(query, {low, high}, .001, b);
        intervals.push_back(std::move(sampled.report));
        if (!sampled.success) {
            failure = "native_tree_failure";
            break;
        }
        for (std::size_t j = 0; j < sampled.first.size(); ++j)
            append(a, z, sampled.first[j][0], {sampled.first[j][1], sampled.second[j][1]});
        out.first.push_back(std::move(a));
        out.second.push_back(std::move(z));
        if (high == 1) {
            const auto last = endpoint(high);
            if (!last) {
                failure = "native_high_sample_failure";
                break;
            }
            append(out.first.back(), out.second.back(), high, *last);
            break;
        }
    }
    out.success = failure.empty();
    if (!out.success) {
        out.first.clear();
        out.second.clear();
    }
    out.report = {{"scope", "native_facet_uv_segments"},
                  {"native_result", out.success},
                  {"knot_plan", plan},
                  {"intervals", std::move(intervals)},
                  {"endpoint_queries", std::move(endpoint_queries)},
                  {"segment_count", out.first.size()},
                  {"failure", out.success ? Json() : Json(failure)},
                  {"work_used", b.work}};
    return out;
}
TubeFacetUvSegments sample_tube_facet_uv_seam(const BsplineSurface &a, const BsplineSurface &z,
                                              const BsplineCurve &profile,
                                              const std::array<Point3, 2> &plane, bool same,
                                              TubeBudget &b) {
    std::size_t plane_queries = 0, fallback_queries = 0;
    Json last_failure = nullptr;
    const TubeFacetUvQuery query = [&](double u) {
        auto sampled = sample_tube_facet_seam(a, z, u, plane, same, b);
        ++plane_queries;
        if (sampled.report.at("method") == "curve_pair_fallback")
            ++fallback_queries;
        if (!sampled.parameters)
            last_failure = std::move(sampled.report);
        return sampled.parameters;
    };
    auto out = sample_tube_facet_uv_segments(profile, query, b);
    out.report["surface_queries"] = {{"plane_queries", plane_queries},
                                     {"fallback_queries", fallback_queries},
                                     {"last_failure", std::move(last_failure)}};
    return out;
}
} // namespace p3d::swept_detail
