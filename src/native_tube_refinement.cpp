#include "native_tube_refinement.hpp"
#include "native_bezier.hpp"
#include "native_curve_affine.hpp"
#include "loft_curve.hpp"
namespace p3d::swept_detail {
namespace {
using H = std::array<double, 4>;
using Index = std::int64_t;
double finite(double x) {
    require(std::isfinite(x), "native bulk knot refinement nonfinite arithmetic");
    return x;
}
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
template <class T> T &at(std::vector<T> &v, Index i) {
    require(i >= 0 && std::uint64_t(i) < v.size(), "native bulk refinement storage index");
    return v[std::size_t(i)];
}
template <class T> const T &at(const std::vector<T> &v, Index i) {
    require(i >= 0 && std::uint64_t(i) < v.size(), "native bulk refinement source index");
    return v[std::size_t(i)];
}
Index span(const std::vector<double> &knots, unsigned order, double u, TubeBudget &budget) {
    Index low = order - 1, high = Index(knots.size()) - order;
    if (std::abs(finite(u - at(knots, high))) < 1e-10)
        return std::abs(finite(u - knots.back())) < 1e-10 ? high - 1 : high;
    for (;;) {
        charge(budget, 1);
        const Index middle = (low + high) / 2;
        if (at(knots, middle) <= u && u < at(knots, middle + 1))
            return middle;
        require(middle != low && middle != high, "native knot span search does not progress");
        if (u < at(knots, middle))
            high = middle;
        else
            low = middle;
    }
}
H interpolate(const H &a, const H &b, double fraction) {
    H out{};
    // GePoint4d::interpolate switches its arithmetic anchor at exactly 0.5.
    const bool from_b = fraction > .5;
    const double f = from_b ? finite(fraction - 1) : fraction;
    for (unsigned i = 0; i < 4; ++i)
        out[i] = finite(finite(finite(b[i] - a[i]) * f) + (from_b ? b[i] : a[i]));
    return out;
}
} // namespace
TubeCurve refine_tube_facet_curve(const BsplineCurve &source, const std::vector<double> &insertions,
                                  TubeBudget &budget) {
    const auto order = source.order();
    const auto n = source.poles().size();
    require(!source.closed() && order >= 2 && order <= 26 && n <= budget.max_control_points &&
                n <= std::size_t(INT32_MAX) - order &&
                insertions.size() <= budget.max_control_points - n &&
                insertions.size() <= std::size_t(INT32_MAX) - n - order,
            "native bulk refinement open curve/order/control limit");
    require(!insertions.empty() && std::is_sorted(insertions.begin(), insertions.end()),
            "native bulk refinement requires a sorted nonempty insertion list");
    const auto &knots = source.knots();
    const auto domain = source.knot_domain();
    for (double u : insertions) {
        charge(budget, 1);
        require(std::isfinite(u) && u >= domain[0] && u <= domain[1],
                "native bulk insertion outside active knot domain");
    }
    const Index p = order - 1, last = Index(n) - 1, r = Index(insertions.size()) - 1;
    const Index m = last + p + 1;
    const Index a = span(knots, order, insertions.front(), budget);
    const Index b = span(knots, order, insertions.back(), budget) + 1;
    charge(budget, n + insertions.size() + knots.size());
    std::vector<H> old, result(n + insertions.size());
    old.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const auto &xyz = source.poles()[i];
        old.push_back({xyz[0], xyz[1], xyz[2], source.rational() ? source.weights()[i] : 1.});
    }
    std::vector<double> out_knots(knots.size() + insertions.size());
    for (Index j = 0; j <= a - p; ++j)
        at(result, j) = at(old, j);
    for (Index j = b - 1; j <= last; ++j)
        at(result, j + r + 1) = at(old, j);
    for (Index j = 0; j <= a; ++j)
        at(out_knots, j) = at(knots, j);
    for (Index j = b + p; j <= m; ++j)
        at(out_knots, j + r + 1) = at(knots, j);
    Index i = b + p - 1, k = b + p + r;
    std::size_t copied_near_knots = 0, interpolations = 0;
    for (Index j = r; j >= 0; --j) {
        const double u = at(insertions, j);
        while (at(knots, i) >= u && i > a) {
            charge(budget, 5);
            at(result, k - p - 1) = at(old, i - p - 1);
            at(out_knots, k) = at(knots, i);
            --k;
            --i;
        }
        at(result, k - p - 1) = at(result, k - p);
        for (Index l = 1; l <= p; ++l) {
            charge(budget, 24);
            const Index index = k - p + l;
            const double difference = finite(at(out_knots, k + l) - u);
            if (std::abs(difference) < 1e-10) {
                at(result, index - 1) = at(result, index);
                ++copied_near_knots;
            } else {
                const double alpha =
                    finite(difference / finite(at(out_knots, k + l) - at(knots, i - p + l)));
                at(result, index - 1) =
                    interpolate(at(result, index), at(result, index - 1), alpha);
                ++interpolations;
            }
        }
        at(out_knots, k--) = u;
    }
    loft_detail::Curve output;
    output.degree = order - 1;
    output.rational = source.rational();
    output.knots = std::move(out_knots);
    output.poles = std::move(result);
    auto refined = BsplineCurve::from_bgfb(output.table());
    return {std::move(refined),
            {{"scope", "native_facet_bulk_knot_refinement"},
             {"inserted_knots", insertions},
             {"first_span", a},
             {"last_span", b - 1},
             {"near_knot_copies", copied_near_knots},
             {"interpolations", interpolations},
             {"work_used", budget.work}}};
}
TubeFacetRefinedTrace prepare_tube_facet_trace(const BsplineCurve &trace, TubeBudget &budget) {
    require(!trace.closed() && trace.poles().size() == trace.order(),
            "native facet trace preparation requires an open Bezier");
    for (std::size_t i = 0; i < trace.knots().size(); ++i)
        require(trace.knots()[i] == (i < trace.order() ? 0. : 1.),
                "native facet trace preparation requires normalized Bezier knots");
    TubeFacetRefinedTrace out;
    out.sampling = prepare_tube_facet_sampling(trace, budget);
    out.report = {{"scope", "native_facet_trace_refinement"}, {"surface_generated", false}};
    if (!out.sampling.success) {
        out.report["status"] = "native_failure";
        out.report["work_used"] = budget.work;
        return out;
    }
    out.trace = curve_detail::with_poles(trace, out.sampling.sampling.working_poles);
    if (!out.sampling.ruled_fallback) {
        std::vector<double> insertions;
        for (const auto &pair : out.sampling.sampling.interior)
            insertions.push_back(pair[0]);
        if (!insertions.empty()) {
            auto refined = refine_tube_facet_curve(*out.trace, insertions, budget);
            out.trace = std::move(refined.curve);
            out.report["refinement"] = std::move(refined.report);
        }
        for (unsigned i = 0; i < trace.order(); ++i)
            out.curvature_knots.push_back({0, out.sampling.start_curvature});
        out.curvature_knots.insert(out.curvature_knots.end(),
                                   out.sampling.sampling.interior.begin(),
                                   out.sampling.sampling.interior.end());
        for (unsigned i = 0; i < trace.order(); ++i)
            out.curvature_knots.push_back({1, out.sampling.end_curvature});
        charge(budget, out.curvature_knots.size());
        require(out.curvature_knots.size() == out.trace->knots().size(),
                "native facet parameter/curvature knot count");
        const auto degree = trace.order() - 1;
        for (std::size_t i = 0; i < out.trace->poles().size(); ++i) {
            std::array<double, 2> mean{};
            for (unsigned j = 1; j <= degree; ++j) {
                charge(budget, 2);
                for (unsigned k = 0; k < 2; ++k)
                    mean[k] = finite(mean[k] + out.curvature_knots[i + j][k]);
            }
            for (double &x : mean)
                x = finite(x / degree);
            out.greville.push_back(mean);
        }
    }
    out.success = true;
    out.report["status"] = "complete";
    out.report["ruled_fallback"] = out.sampling.ruled_fallback;
    out.report["greville_parameter_curvature"] = out.greville;
    out.report["work_used"] = budget.work;
    return out;
}
} // namespace p3d::swept_detail
