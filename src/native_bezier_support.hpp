// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from MSBsplineCurve_ByBezier.cpp and bezierDPoint4d.cpp.
// Changes: bounded immutable support extraction and native knot tolerance.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#pragma once
#include "internal.hpp"

namespace p3d::curve_detail::bezier_support {
using H = std::array<double, 4>;
inline double finite(double x) {
    require(std::isfinite(x), "native Bezier support nonfinite arithmetic");
    return x;
}
inline bool null_interval(double a, double b) {
    return finite(((std::abs(a) + 1) + std::abs(b)) * 1e-14) > std::abs(finite(a - b));
}
inline void saturate(std::vector<H> &p, std::vector<double> k) {
    const auto degree = p.size() - 1;
    const double left = k[degree - 1], right = k[degree];
    while (k.front() < left) {
        for (std::size_t i = 0; k[i] < left; ++i) {
            const double f1 = finite((left - k[i]) / (k[i + degree] - k[i])), f0 = 1 - f1;
            for (unsigned axis = 0; axis < 4; ++axis)
                p[i][axis] = finite(f0 * p[i][axis] + f1 * p[i + 1][axis]);
            k[i] = k[i + 1];
        }
    }
    while (right < k.back()) {
        for (std::size_t i = k.size() - 1, j = degree; right < k[i]; --i, --j) {
            const double f1 = finite((right - k[i]) / (k[i - degree] - k[i])), f0 = 1 - f1;
            for (unsigned axis = 0; axis < 4; ++axis)
                p[j][axis] = finite(f0 * p[j][axis] + f1 * p[j - 1][axis]);
            k[i] = k[i - 1];
        }
    }
}
// Caller validates the order, source index, non-null interval and work budget.
// No endpoint is replaced by an adjacent span's endpoint, including at breaks.
inline std::vector<H> extract(const BsplineCurve &curve, std::size_t i) {
    const auto order = curve.order(), degree = order - 1;
    const auto n = curve.poles().size();
    std::vector<H> p;
    p.reserve(order);
    for (unsigned j = 0; j < order; ++j) {
        auto index = std::int64_t(i + j) + curve.periodic_pole_shift();
        if (curve.closed()) {
            index %= std::int64_t(n);
            if (index < 0)
                index += std::int64_t(n);
        }
        require(index >= 0 && std::uint64_t(index) < n, "native Bezier support index");
        const auto &xyz = curve.poles()[std::size_t(index)];
        p.push_back(
            {xyz[0], xyz[1], xyz[2], curve.rational() ? curve.weights()[std::size_t(index)] : 1.});
    }
    const auto &k = curve.knots();
    saturate(p, std::vector<double>(k.begin() + i + 1, k.begin() + i + 1 + 2 * degree));
    return p;
}
} // namespace p3d::curve_detail::bezier_support
