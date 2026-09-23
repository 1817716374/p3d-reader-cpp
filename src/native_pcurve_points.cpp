#include "internal.hpp"
#include "native_pcurve_points.hpp"

namespace p3d::detail {
namespace {
struct Blend {
    std::int64_t first;
    std::vector<double> values;
};
// The point/tangent and surface-point callers use fixed 26-value arrays.
// Keep their native clamp, interval selection and divide-before-multiply order.
Blend blend(unsigned order, const std::vector<double> &knots, int pole_shift, double upper,
            double parameter) {
    require(order <= 26, "native PCurve point evaluation supports order at most 26");
    require(std::isfinite(parameter), "native PCurve knot parameter overflow");
    const double t = std::min(upper, std::max(knots[order - 1], parameter));
    std::size_t index = 1;
    while (index < knots.size() && t >= knots[index] && upper > knots[index])
        ++index;
    require(index < knots.size() && index >= order - 1 && index + order - 2 < knots.size(),
            "native PCurve knot window");
    Blend result{std::int64_t(index) - order + pole_shift, std::vector<double>(order)};
    result.values[0] = 1;
    std::array<double, 26> left{}, right{};
    for (unsigned j = 1; j < order; ++j) {
        left[j - 1] = t - knots[index - j];
        right[j - 1] = knots[index + j - 1] - t;
        double saved = 0;
        for (unsigned r = 0; r < j; ++r) {
            const double denominator = right[r] + left[j - 1 - r];
            double value = result.values[r];
            if (denominator != 0)
                value /= denominator;
            result.values[r] = saved + right[r] * value;
            saved = left[j - 1 - r] * value;
        }
        result.values[j] = saved;
    }
    for (double value : result.values)
        require(std::isfinite(value), "native PCurve non-finite blending coefficient");
    return result;
}
std::size_t pole(std::int64_t index, std::size_t count, bool closed) {
    if (closed) {
        index %= std::int64_t(count);
        if (index < 0)
            index += std::int64_t(count);
    }
    require(index >= 0 && std::uint64_t(index) < count, "native PCurve pole index");
    return std::size_t(index);
}
Point3 finite(Point3 p) {
    for (double x : p)
        require(std::isfinite(x), "native PCurve non-finite evaluated point");
    return p;
}
} // namespace

NativePCurvePoint pcurve_point(const BsplineCurve &curve, double fraction) {
    require(std::isfinite(fraction), "native PCurve non-finite curve fraction");
    const auto domain = curve.knot_domain();
    const double t = (1 - fraction) * domain[0] + fraction * domain[1];
    const auto b = blend(curve.order(), curve.knots(), curve.periodic_pole_shift(), domain[1], t);
    NativePCurvePoint result{};
    double weight = 0;
    for (unsigned i = 0; i < curve.order(); ++i) {
        const auto index = pole(b.first + i, curve.poles().size(), curve.closed());
        for (unsigned k = 0; k < 3; ++k)
            result.point[k] += b.values[i] * curve.poles()[index][k];
        if (curve.rational())
            weight += b.values[i] * curve.weights()[index];
    }
    if (curve.rational()) {
        // This native curve-point caller replaces exactly zero W with one.
        // The surface-point caller below deliberately has no such fallback.
        result.zero_weight_fallback = weight == 0;
        if (result.zero_weight_fallback)
            weight = 1;
        for (double &x : result.point)
            x /= weight;
    }
    result.point = finite(result.point);
    return result;
}

Point3 pcurve_surface_point(const BsplineSurface &surface, double u, double v) {
    require(std::isfinite(u) && std::isfinite(v), "native PCurve non-finite surface fraction");
    const auto &du = surface.u(), &dv = surface.v();
    const auto bu = blend(du.order(), du.knots(), du.periodic_pole_shift(), 1,
                          (1 - u) * du.knot_domain()[0] + u);
    const auto bv = blend(dv.order(), dv.knots(), dv.periodic_pole_shift(), 1,
                          (1 - v) * dv.knot_domain()[0] + v);
    Point3 result{};
    double weight = 0;
    // Native traversal is U outside, V inside; the source control net is still
    // indexed by v * numPolesU + u. Polynomial outputs are not divided by sum(N).
    for (unsigned i = 0; i < du.order(); ++i) {
        const auto ui = pole(bu.first + i, du.pole_count(), du.closed());
        for (unsigned j = 0; j < dv.order(); ++j) {
            const auto vi = pole(bv.first + j, dv.pole_count(), dv.closed());
            const auto index = vi * du.pole_count() + ui;
            const double c = bu.values[i] * bv.values[j];
            for (unsigned k = 0; k < 3; ++k)
                result[k] += c * surface.poles()[index][k];
            if (surface.rational())
                weight += c * surface.weights()[index];
        }
    }
    if (surface.rational()) {
        require(weight != 0, "native PCurve surface point has zero evaluated weight");
        for (double &x : result)
            x /= weight;
    }
    return finite(result);
}
} // namespace p3d::detail
