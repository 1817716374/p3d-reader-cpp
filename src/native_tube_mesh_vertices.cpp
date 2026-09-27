#include "native_tube_mesh_vertices.hpp"
#include "native_surface_sample.hpp"
#include "native_bezier_support.hpp"
namespace p3d::swept_detail {
using curve_detail::bezier_support::finite;
TubeMeshVertex evaluate_tube_mesh_vertex(const BsplineSurface &s, double u, double v,
                                         std::array<double, 2> interval, std::size_t patch_index,
                                         std::size_t patch_count, TubeBudget &budget) {
    require(s.poles().size() <= budget.max_control_points, "native vertex control budget");
    require(patch_count && patch_count <= INT32_MAX && patch_index < patch_count,
            "native vertex patch position");
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    require(s.u().order() <= 26 && s.v().order() <= 26, "native vertex surface order");
    work.charge(s.u().knots().size());
    work.charge(s.v().knots().size());
    work.charge(16 * (s.u().order() * s.u().order() + s.v().order() * s.v().order()));
    work.charge(32 * s.u().order() * s.v().order() + 40);
    const auto sample = detail::native_surface_sample(s, u, v);
    TubeMeshVertex out;
    out.point = sample.point;
    const auto &a = sample.du, &b = sample.dv;
    Point3 n{finite(a[1] * b[2] - a[2] * b[1]), finite(b[0] * a[2] - a[0] * b[2]),
             finite(a[0] * b[1] - b[0] * a[1])};
    const auto length = finite(std::sqrt(finite((n[0] * n[0] + n[1] * n[1]) + n[2] * n[2])));
    out.normal_fallback = !(length > 0);
    if (out.normal_fallback)
        n = {1, 0, 0};
    else {
        const auto inverse = finite(1 / length);
        for (auto &x : n)
            x = finite(x * inverse);
    }
    out.normal = n;
    // Native source UVs retain the un-clamped caller parameters even when
    // point evaluation clamps. Keep the two V divisions before their sum.
    const auto low = finite(interval[0]), high = finite(interval[1]);
    out.parameter = {finite(finite(finite(high - low) * u) + low),
                     finite(double(patch_index) / double(patch_count) + v / double(patch_count))};
    return out;
}
TubeMeshRegularVertices evaluate_tube_mesh_regular_vertices(
    const TubeMeshSampledPatch &patch, const TubeSectionMeshSamples &section,
    std::size_t strip_index, std::size_t patch_index, std::size_t patch_count, TubeBudget &budget) {
    const auto &prep = patch.preparation;
    require(prep.success && prep.boundaries.success && patch.path.success && section.success,
            "native regular vertices require successful sampling preparation");
    require(!prep.boundaries.lower && !prep.boundaries.upper,
            "native regular vertices cannot consume boundary functions");
    require(prep.strips.size() == section.interval_samples.size() &&
                strip_index < prep.strips.size(),
            "native regular vertex strip correspondence");
    const auto &u = section.interval_samples[strip_index], &v = patch.path.parameters;
    require(u.size() >= 2 && v.size() >= 2 && u.size() <= INT32_MAX && v.size() <= INT32_MAX,
            "native regular vertex parameter counts");
    require(u.size() <= budget.max_control_points / v.size(),
            "native regular vertex output budget");
    const auto &knots = section.knots;
    require(knots.left <= knots.right && strip_index < knots.right - knots.left &&
                knots.right < knots.compressed.size(),
            "native regular vertex source U interval");
    const auto first = knots.left + strip_index;
    TubeMeshRegularVertices out{u.size(), v.size(), {}};
    out.vertices.reserve(u.size() * v.size());
    for (const double y : v)
        for (const double x : u)
            out.vertices.push_back(
                evaluate_tube_mesh_vertex(prep.strips[strip_index], x, y,
                                          {knots.compressed[first], knots.compressed[first + 1]},
                                          patch_index, patch_count, budget));
    return out;
}
} // namespace p3d::swept_detail
