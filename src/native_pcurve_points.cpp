#include "internal.hpp"
#include "native_pcurve_points.hpp"
#include "native_surface_iso.hpp"
#include "native_surface_sample.hpp"

namespace p3d::detail {
namespace {
struct Blend {
    std::int64_t first;
    std::vector<double> values;
    std::vector<double> derivatives{};
};
// The point/tangent and surface-point callers use fixed 26-value arrays.
// Keep their native clamp, interval selection and divide-before-multiply order.
// Derivative recurrence adapted from Bentley imodel-native bsputil.cpp.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
Blend blend(unsigned order, const std::vector<double> &knots, int pole_shift, double upper,
            double parameter, bool derivatives = false) {
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
    if (derivatives)
        result.derivatives.resize(order);
    std::array<double, 26> left{}, right{};
    for (unsigned j = 1; j < order; ++j) {
        left[j - 1] = t - knots[index - j];
        right[j - 1] = knots[index + j - 1] - t;
        double saved = 0;
        double saved_derivative = 0;
        for (unsigned r = 0; r < j; ++r) {
            const double denominator = right[r] + left[j - 1 - r];
            double value = result.values[r];
            if (denominator != 0)
                value /= denominator;
            result.values[r] = saved + right[r] * value;
            saved = left[j - 1 - r] * value;
            if (derivatives) {
                double derivative = result.derivatives[r];
                if (denominator != 0)
                    derivative /= denominator;
                result.derivatives[r] = (right[r] * derivative + saved_derivative) - value;
                saved_derivative = left[j - 1 - r] * derivative + value;
            }
        }
        result.values[j] = saved;
        if (derivatives)
            result.derivatives[j] = saved_derivative;
    }
    for (double value : result.values)
        require(std::isfinite(value), "native PCurve non-finite blending coefficient");
    for (double value : result.derivatives)
        require(std::isfinite(value), "native PCurve non-finite blending derivative");
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
NativePCurvePoint evaluate(const Blend &b, std::size_t count, bool closed,
                           const std::vector<Point3> &poles, const std::vector<double> &weights,
                           std::size_t first, std::size_t stride) {
    NativePCurvePoint result{};
    double weight = 0;
    for (std::size_t i = 0; i < b.values.size(); ++i) {
        const auto index = first + stride * pole(b.first + i, count, closed);
        for (unsigned k = 0; k < 3; ++k)
            result.point[k] += b.values[i] * poles[index][k];
        if (!weights.empty())
            weight += b.values[i] * weights[index];
    }
    if (!weights.empty()) {
        result.weight = weight;
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
} // namespace

NativePCurvePoint pcurve_point(const BsplineCurve &curve, double fraction) {
    require(std::isfinite(fraction), "native PCurve non-finite curve fraction");
    const auto domain = curve.knot_domain();
    const double t = (1 - fraction) * domain[0] + fraction * domain[1];
    const auto b = blend(curve.order(), curve.knots(), curve.periodic_pole_shift(), domain[1], t);
    return evaluate(b, curve.poles().size(), curve.closed(), curve.poles(), curve.weights(), 0, 1);
}

NativePCurvePointTangent pcurve_point_tangent(const BsplineCurve &curve, double fraction) {
    require(std::isfinite(fraction), "native PCurve non-finite curve fraction");
    const auto domain = curve.knot_domain();
    const double t = (1 - fraction) * domain[0] + fraction * domain[1];
    const auto b =
        blend(curve.order(), curve.knots(), curve.periodic_pole_shift(), domain[1], t, true);
    NativePCurvePointTangent result;
    result.value =
        evaluate(b, curve.poles().size(), curve.closed(), curve.poles(), curve.weights(), 0, 1);
    for (unsigned i = 0; i < curve.order(); ++i) {
        const auto index = pole(b.first + i, curve.poles().size(), curve.closed());
        if (curve.rational())
            result.weight_derivative += b.derivatives[i] * curve.weights()[index];
        for (unsigned k = 0; k < 3; ++k)
            result.tangent[k] += b.derivatives[i] * curve.poles()[index][k];
    }
    if (curve.rational()) {
        const double divisor = result.value.zero_weight_fallback ? 1 : result.value.weight;
        require(std::isfinite(divisor) && std::isfinite(result.weight_derivative),
                "native PCurve nonfinite tangent weight");
        for (unsigned k = 0; k < 3; ++k)
            result.tangent[k] =
                (result.tangent[k] - result.weight_derivative * result.value.point[k]) / divisor;
    }
    result.tangent = finite(result.tangent);
    return result;
}

// Isocurve construction adapted from Bentley imodel-native bspconv.cpp,
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Changes: shared read-only basis, direct strided evaluation, bounded output.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
namespace {
NativeIsoCurve iso_curve(const BsplineSurface &surface, double fraction, bool constant_u,
                         curve_detail::BezierWork work, std::size_t max_control_points) {
    const auto &u = surface.u(), &v = surface.v();
    const auto nu = u.pole_count(), nv = v.pole_count();
    const auto &fixed = constant_u ? u : v, &varying = constant_u ? v : u;
    const auto count = varying.pole_count();
    require(std::isfinite(fraction), "native isocurve nonfinite fraction");
    require(nu >= 2 && nv >= 2 && nu <= INT32_MAX && nv <= INT32_MAX &&
                count <= max_control_points && fixed.order() <= 26,
            "native isocurve order or control budget exceeded");
    work.charge(u.knots().size());
    work.charge(v.knots().size());
    work.charge(std::size_t(8) * fixed.order() * fixed.order());
    const auto cost = std::size_t(8) * fixed.order() + 12;
    require(count <= (work.limit - work.used) / cost, "native isocurve work budget exceeded");
    work.charge(count * cost);
    const auto domain = fixed.knot_domain();
    const double t = fraction * domain[1] + (1 - fraction) * domain[0];
    // Every source row/column uses the same read-only basis. Direct indexed
    // evaluation avoids transposing or copying/reparsing the control grid.
    const auto b = blend(fixed.order(), fixed.knots(), fixed.periodic_pole_shift(), domain[1], t);
    Json xyz = Json::array(), weights = Json::array();
    std::size_t fallbacks = 0;
    for (std::size_t i = 0; i < count; ++i) {
        auto p = evaluate(b, fixed.pole_count(), fixed.closed(), surface.poles(), surface.weights(),
                          constant_u ? i * nu : i, constant_u ? 1 : nu);
        fallbacks += p.zero_weight_fallback;
        if (surface.rational()) {
            require(std::isfinite(p.weight), "native isocurve nonfinite evaluated weight");
            for (auto &x : p.point)
                x *= p.weight;
            p.point = finite(p.point);
            weights.push_back(p.weight);
        }
        for (double x : p.point)
            xyz.push_back(x);
    }
    return {
        BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                 {"order", varying.order()},
                                 {"closed", varying.closed()},
                                 {"knots", varying.knots()},
                                 {"poles", std::move(xyz)},
                                 {"weights", surface.rational() ? std::move(weights) : Json()}}),
        fallbacks};
}
} // namespace
NativeIsoCurve native_iso_v_curve(const BsplineSurface &surface, double fraction,
                                  curve_detail::BezierWork work, std::size_t max_control_points) {
    return iso_curve(surface, fraction, false, work, max_control_points);
}
NativeIsoCurve native_iso_u_curve(const BsplineSurface &surface, double fraction,
                                  curve_detail::BezierWork work, std::size_t max_control_points) {
    return iso_curve(surface, fraction, true, work, max_control_points);
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

// Tensor-product derivatives adapted from Bentley imodel-native bspsurf.cpp,
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Changes: original P3D knot mapping/order, immutable arrays, checked indexing.
NativeSurfaceSample native_surface_sample(const BsplineSurface &surface, double u, double v) {
    require(std::isfinite(u) && std::isfinite(v), "native surface nonfinite fraction");
    const auto &su = surface.u(), &sv = surface.v();
    const auto bu = blend(su.order(), su.knots(), su.periodic_pole_shift(), 1,
                          (1 - u) * su.knot_domain()[0] + u, true);
    const auto bv = blend(sv.order(), sv.knots(), sv.periodic_pole_shift(), 1,
                          (1 - v) * sv.knot_domain()[0] + v, true);
    // The native surface loop applies remainder to positive indices even for
    // open axes. A negative open index would access invalid memory: reject it.
    auto index = [](std::int64_t value, std::size_t count, bool closed) {
        require(count && count <= INT32_MAX, "native surface control index extent");
        if (value < 0 && closed)
            value += std::int64_t(count);
        require(value >= 0, "native surface negative control index");
        return std::size_t(value) % count;
    };
    NativeSurfaceSample out;
    out.weight = surface.rational() ? 0 : 1;
    // U outside, V inside, matching the original accumulation order. The
    // derivative is in knots, without the caller fraction's scale factor.
    for (unsigned i = 0; i < su.order(); ++i) {
        const auto ui = index(bu.first + i, su.pole_count(), su.closed());
        for (unsigned j = 0; j < sv.order(); ++j) {
            const auto vi = index(bv.first + j, sv.pole_count(), sv.closed());
            const auto at = vi * su.pole_count() + ui;
            const double c = bu.values[i] * bv.values[j],
                         cu = bv.values[j] * bu.derivatives[i],
                         cv = bu.values[i] * bv.derivatives[j];
            for (unsigned k = 0; k < 3; ++k) {
                const auto p = surface.poles()[at][k];
                out.point[k] += c * p;
                out.du[k] += cu * p;
                out.dv[k] += cv * p;
            }
            if (surface.rational()) {
                const auto w = surface.weights()[at];
                out.weight += c * w;
                out.weight_du += cu * w;
                out.weight_dv += cv * w;
            }
        }
    }
    require(std::isfinite(out.weight) && std::isfinite(out.weight_du) &&
                std::isfinite(out.weight_dv), "native surface nonfinite weight derivatives");
    if (surface.rational()) {
        require(out.weight != 0, "native surface zero evaluated weight");
        for (unsigned k = 0; k < 3; ++k) {
            out.point[k] /= out.weight;
            out.du[k] = (out.du[k] - out.point[k] * out.weight_du) / out.weight;
            out.dv[k] = (out.dv[k] - out.point[k] * out.weight_dv) / out.weight;
        }
    }
    out.point = finite(out.point);
    out.du = finite(out.du);
    out.dv = finite(out.dv);
    return out;
}
} // namespace p3d::detail
