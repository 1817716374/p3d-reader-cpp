#include "native_tube_mesh_trim.hpp"
#include "native_pcurve_points.hpp"
#include "native_bezier_support.hpp"
namespace p3d::swept_detail {
using curve_detail::bezier_support::finite;
TubeMeshTrimPlan prepare_tube_mesh_trim_columns(const TubeMeshBoundaryCurves &bounds,
                                                const std::vector<double> &u,
                                                const std::vector<double> &v,
                                                std::array<double, 2> interval,
                                                TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    require(bounds.success && u.size() >= 2 && v.size() >= 2 && u.size() <= INT32_MAX &&
                v.size() <= INT32_MAX - 2 && u.size() <= budget.max_control_points &&
                v.size() <= budget.max_control_points,
            "native trimmed grid parameter counts or boundary status");
    work.charge(u.size());
    work.charge(v.size());
    for (double x : u)
        finite(x);
    for (double x : v)
        finite(x);
    require(std::is_sorted(v.begin(), v.end()), "native trimmed grid V search order");
    finite(interval[0]);
    finite(interval[1]);
    TubeMeshTrimPlan out;
    out.report = {{"scope", "native_swept_trimmed_columns"},
                  {"status", "native_failure"},
                  {"mesh_generated", false}};
    out.columns.reserve(u.size());
    std::size_t fallbacks = 0;
    auto evaluate = [&](const BsplineCurve &c, double f) {
        require(c.poles().size() <= budget.max_control_points && c.order() <= 26,
                "native trimmed boundary curve budget");
        work.charge(c.knots().size());
        work.charge(16 * c.order() * c.order() + 8 * c.order());
        const auto sample = detail::pcurve_point(c, f);
        fallbacks += sample.zero_weight_fallback;
        return finite(sample.point[0]);
    };
    auto index = [&](double x) {
        auto at = std::lower_bound(v.begin(), v.end(), x, [&](double a, double b) {
            work.charge(1);
            return a < b;
        });
        return std::size_t(at - v.begin());
    };
    auto fail = [&](const char *side, std::size_t column) {
        out.columns.clear();
        out.vertex_count = 0;
        out.report["failure"] = "boundary_above_last_sample";
        out.report["side"] = side;
        out.report["column"] = column;
        out.report["work_used"] = budget.work;
        return std::move(out);
    };
    for (std::size_t i = 0; i < u.size(); ++i) {
        work.charge(16);
        TubeMeshTrimColumn c;
        c.local_u = u[i];
        c.source_u = finite(finite(finite(interval[1] - interval[0]) * u[i]) + interval[0]);
        c.first_interior = 1;
        c.last_interior = std::int32_t(v.size()) - 2;
        c.lower = v.front();
        c.upper = v.back();
        if (bounds.lower) {
            c.lower = evaluate(*bounds.lower, c.source_u);
            const auto at = index(c.lower);
            if (at == v.size())
                return fail("lower", i);
            // Base428 receives sampled V first and evaluated boundary second.
            c.lower_snapped =
                std::abs(finite(c.lower - v[at])) <=
                finite(finite(finite(std::abs(v[at]) + 1) + std::abs(c.lower)) * 1e-10);
            c.first_interior = std::int32_t(at) + int(c.lower_snapped);
            if (c.lower_snapped)
                c.lower = v[at];
        }
        if (bounds.upper) {
            c.upper = evaluate(*bounds.upper, c.source_u);
            const auto at = index(c.upper);
            if (at == v.size())
                return fail("upper", i);
            c.last_interior = std::int32_t(at) - 1; // Upper does not call almostEqual.
        }
        const auto declared = std::int64_t(c.last_interior) - c.first_interior + 3;
        c.count = declared > 0 ? std::size_t(declared) : 0;
        c.offset = out.vertex_count;
        require(c.count <= budget.max_control_points - out.vertex_count,
                "native trimmed grid cumulative vertex budget");
        out.vertex_count += c.count;
        out.columns.push_back(c);
    }
    out.success = true;
    out.report["status"] = "prepared";
    out.report["vertices"] = out.vertex_count;
    out.report["curve_zero_weight_fallbacks"] = fallbacks;
    out.report["work_used"] = budget.work;
    return out;
}
TubeMeshTrimVertices evaluate_tube_mesh_trim_vertices(const TubeMeshSampledPatch &patch,
                                                      const TubeSectionMeshSamples &section,
                                                      std::size_t strip, TubeBudget &budget) {
    const auto &prep = patch.preparation;
    require(prep.success && prep.boundaries.success && patch.path.success && section.success &&
                prep.strips.size() == section.interval_samples.size() && strip < prep.strips.size(),
            "native trimmed vertices require valid strip correspondence");
    require(prep.boundaries.lower || prep.boundaries.upper,
            "native trimmed vertex route requires a boundary function");
    const auto &k = section.knots;
    require(k.left <= k.right && strip < k.right - k.left && k.right < k.compressed.size(),
            "native trimmed vertex source interval");
    const std::array<double, 2> interval{k.compressed[k.left + strip],
                                         k.compressed[k.left + strip + 1]};
    TubeMeshTrimVertices out;
    out.plan = prepare_tube_mesh_trim_columns(prep.boundaries, section.interval_samples[strip],
                                              patch.path.parameters, interval, budget);
    if (!out.plan.success)
        return out;
    out.vertices.reserve(out.plan.vertex_count);
    for (const auto &c : out.plan.columns) {
        const auto first = std::int64_t(c.first_interior) - 1,
                   last = std::int64_t(c.last_interior) + 1;
        for (auto row = first; row <= last; ++row) {
            // When both boundaries share a single row, the lower branch wins.
            double v = c.lower;
            if (row != first) {
                if (row == last)
                    v = c.upper;
                else {
                    require(row >= 0 && std::size_t(row) < patch.path.parameters.size(),
                            "native trimmed vertex interior sample index");
                    v = patch.path.parameters[std::size_t(row)];
                }
            }
            const auto a =
                evaluate_tube_mesh_vertex(prep.strips[strip], c.local_u, v, interval, 0, 1, budget);
            out.vertices.push_back({std::int32_t(row), v, a.point, a.normal, a.normal_fallback});
        }
    }
    out.plan.report["work_used"] = budget.work;
    return out;
}
} // namespace p3d::swept_detail
