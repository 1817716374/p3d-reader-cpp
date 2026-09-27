#include "native_curve_sampling.hpp"
#include "native_bezier_support.hpp"
#include "native_pcurve_points.hpp"
namespace p3d::curve_detail {
namespace {
using bezier_support::finite;
bool near_zero(double v) {
    return std::abs(v) <= finite((std::abs(v) + 1) * 1e-10);
}
Point3 add(const Point3 &a, const Point3 &b) {
    return {finite(a[0] + b[0]), finite(a[1] + b[1]), finite(a[2] + b[2])};
}
Point3 multiply(Point3 p, double f) {
    for (auto &v : p)
        v = finite(v * f);
    return p;
}
Point3 divide(Point3 p, double d) {
    const double scale = std::max({std::abs(p[0]), std::abs(p[1]), std::abs(p[2])});
    if (finite(scale * 1e-12) < std::abs(d))
        return multiply(p, finite(1 / d));
    return p; // Native operator/ retains numerator when its scale guard fails.
}
double distance_squared(const Point3 &a, const Point3 &b, bool xy = false) {
    const double x = finite(a[0] - b[0]), y = finite(a[1] - b[1]);
    const double xy2 = finite(y * y + x * x);
    if (xy)
        return xy2;
    const double z = finite(a[2] - b[2]);
    return finite(xy2 + z * z);
}
struct Sampler {
    const std::vector<const BsplineCurve *> &curves;
    double chord, angle;
    std::size_t max_nodes;
    BezierWork work;
    NativeCurveSampleTree out;
    std::size_t evaluations = 0, refinements = 0;
    unsigned deepest = 0;
    NativeCurveSample sample(double f) {
        NativeCurveSample result;
        result.parameter = finite(f);
        work.charge(curves.size());
        result.points.reserve(curves.size());
        result.tangents.reserve(curves.size());
        for (const auto *c : curves) {
            work.charge(c->knots().size() + 8 * std::size_t(c->order()) * c->order() + 16);
            auto value = detail::pcurve_point_tangent(*c, f);
            const auto domain = c->knot_domain();
            const double span = finite(domain[1] - domain[0]);
            for (auto &v : value.tangent)
                v = finite(v * span);
            for (double v : value.value.point)
                finite(v);
            result.points.push_back(value.value.point);
            result.tangents.push_back(value.tangent);
            ++evaluations;
        }
        return result;
    }
    std::size_t node(double f, unsigned depth, std::optional<std::size_t> parent) {
        require(out.nodes.size() < max_nodes, "native curve sample node budget");
        auto value = sample(f);
        const auto index = out.nodes.size();
        out.nodes.push_back({std::move(value), parent, {}, {}, depth, false});
        deepest = std::max(deepest, depth);
        return index;
    }
    bool visit(std::size_t index, const NativeCurveSample &low, const NativeCurveSample &high) {
        work.charge(1);
        auto &n = out.nodes[index]; // deque append preserves references.
        n.left = node(finite(finite(low.parameter + n.sample.parameter) * .5), n.depth + 1, index);
        n.right =
            node(finite(finite(high.parameter + n.sample.parameter) * .5), n.depth + 1, index);
        if (n.depth > 30) {
            out.report["failure"] = "native_depth_limit";
            return false;
        }
        bool refine = false;
        const bool chord_off = near_zero(chord), angle_off = near_zero(angle);
        for (std::size_t i = 0; i < curves.size(); ++i) {
            work.charge(128);
            if (chord_off && angle_off) {
                out.report["failure"] = "both_tolerances_near_zero";
                return false;
            }
            const auto &l = out.nodes[*n.left].sample, &r = out.nodes[*n.right].sample;
            if (!chord_off) {
                const auto &a = low.points[i], &b = high.points[i];
                const double tolerance2 = finite(chord * chord);
                // Match native scalar/vector operations, including each
                // guarded division and the XY-only right-quarter predicate.
                const auto mid = divide(add(a, b), 2);
                if (distance_squared(n.sample.points[i], mid) > tolerance2)
                    refine = true;
                else {
                    const auto quarter = add(divide(multiply(a, 3), 4), divide(b, 4));
                    if (distance_squared(l.points[i], quarter) > tolerance2)
                        refine = true;
                    else {
                        const auto quarter3 = add(divide(a, 4), divide(multiply(b, 3), 4));
                        refine = distance_squared(r.points[i], quarter3, true) > tolerance2;
                    }
                }
            }
            if (!refine && !angle_off) {
                const auto &t = low.tangents[i];
                refine = native_curve_vector_angle(t, high.tangents[i]) > angle ||
                         native_curve_vector_angle(t, r.tangents[i]) > angle ||
                         native_curve_vector_angle(t, n.sample.tangents[i]) > angle ||
                         native_curve_vector_angle(t, l.tangents[i]) > angle;
            }
            if (refine)
                break;
        }
        if (!refine)
            return true;
        n.refined = true;
        ++refinements;
        return visit(*n.left, low, n.sample) && visit(*n.right, n.sample, high);
    }
    void collect(std::size_t i) {
        work.charge(1);
        const auto &n = out.nodes[i];
        if (!n.refined)
            return;
        collect(*n.left);
        out.parameters.push_back(n.sample.parameter);
        collect(*n.right);
    }
};
} // namespace
NativeCurveSampleTree native_curve_sample_tree(const std::vector<const BsplineCurve *> &curves,
                                               std::array<double, 2> interval,
                                               double chord_tolerance, double angle_tolerance,
                                               std::size_t max_controls, std::size_t max_nodes,
                                               BezierWork work) {
    finite(interval[0]);
    finite(interval[1]);
    finite(chord_tolerance);
    finite(angle_tolerance);
    require(curves.size() <= max_controls && max_nodes > 0,
            "native sample curve/node count budget");
    std::size_t controls = 0;
    for (const auto *curve : curves) {
        work.charge(1);
        require(curve && curve->order() <= 26 && curve->poles().size() <= max_controls - controls,
                "native curve sample source/control budget");
        controls += curve->poles().size(); // Repeated references still cost evaluation work.
    }
    Sampler s{curves, chord_tolerance, angle_tolerance, max_nodes, work, {}, 0, 0, 0};
    s.out.report = {{"scope", "native_shared_curve_sample_tree"},
                    {"right_quarter_distance", "xy_squared"}};
    const auto root = s.node(finite(finite(interval[0] + interval[1]) * .5), 0, {});
    const auto low = s.sample(interval[0]), high = s.sample(interval[1]);
    s.out.success = s.visit(root, low, high);
    s.out.report["allocated_nodes"] = s.out.nodes.size();
    if (s.out.success) {
        s.out.parameters.push_back(interval[0]);
        s.collect(root);
        s.out.parameters.push_back(interval[1]);
    } else {
        // Native destroys the whole tree on a false recursive return.
        s.out.nodes.clear();
        s.out.parameters.clear();
    }
    s.out.report["status"] = s.out.success ? "sampled" : "native_failure";
    s.out.report["point_tangent_evaluations"] = s.evaluations;
    s.out.report["refined_nodes"] = s.refinements;
    s.out.report["deepest_allocated_level"] = s.deepest;
    s.out.report["work_used"] = work.used;
    return std::move(s.out);
}
} // namespace p3d::curve_detail
