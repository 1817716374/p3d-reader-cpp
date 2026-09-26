// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Adapted from bezeval.cpp, bezierDPoint4d.cpp, cv_properties.cpp and
// BSIQuadrature.cpp. P3D arithmetic/fallbacks, bounded immutable storage and
// explicit work accounting replace upstream runtime types. See THIRD_PARTY.md.
#include "native_bezier.hpp"

namespace p3d::curve_detail {
void BezierWork::charge(std::size_t n) const {
    require(used <= limit && n <= limit - used, "native Bezier work budget exceeded");
    used += n;
}
namespace {
double finite(double x) {
    require(std::isfinite(x), "native Bezier nonfinite arithmetic");
    return x;
}
Point3 subtract(const Point3 &a, const Point3 &b) {
    return {finite(a[0] - b[0]), finite(a[1] - b[1]), finite(a[2] - b[2])};
}
double dot(const Point3 &a, const Point3 &b) {
    return finite((a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]);
}
Point3 cross(const Point3 &a, const Point3 &b) {
    return {finite(a[1] * b[2] - a[2] * b[1]), finite(a[2] * b[0] - a[0] * b[2]),
            finite(a[0] * b[1] - a[1] * b[0])};
}
double normalize(Point3 &p) {
    const double length = finite(std::sqrt((p[0] * p[0] + p[1] * p[1]) + p[2] * p[2]));
    if (length > 0) {
        const double scale = finite(1 / length);
        for (double &x : p)
            x = finite(x * scale);
    } else
        p = {1, 0, 0};
    return length;
}
void validate(const std::vector<BezierPole> &poles, bool weights) {
    require(poles.size() >= 2 && poles.size() <= 26, "native Bezier order must be 2..26");
    for (const auto &p : poles)
        for (unsigned k = 0; k < (weights ? 4u : 3u); ++k)
            finite(p[k]);
}
unsigned candidate(double count, unsigned multiplier) {
    count = finite(std::ceil(count));
    require(count >= 0 && count <= double(INT32_MAX / std::max(1u, multiplier)),
            "native Bezier edge count is outside signed integer range");
    return unsigned(count) * multiplier;
}
} // namespace

BezierPointTangent native_bezier_point_tangent(const std::vector<BezierPole> &poles,
                                               double fraction, BezierWork work) {
    validate(poles, true);
    finite(fraction);
    const auto degree = poles.size() - 1;
    work.charge(4 * poles.size() * poles.size() + 24);
    const double v = finite(1 - fraction);
    BezierPole value{}, derivative{};
    for (unsigned axis = 0; axis < 4; ++axis) {
        std::array<double, 26> values{};
        for (std::size_t k = 0; k <= degree; ++k)
            values[k] = poles[k][axis];
        for (std::size_t j = 1; j <= degree; ++j) {
            if (j == degree)
                derivative[axis] = finite(double(degree) * (values[j] - values[j - 1]));
            for (std::size_t k = degree; k >= j; --k)
                values[k] = finite(v * values[k - 1] + fraction * values[k]);
        }
        value[axis] = values[degree];
    }
    BezierPointTangent result;
    result.weight_fallback = !(std::abs(value[3]) > 1e-12);
    const double reciprocal = result.weight_fallback ? 0. : finite(1 / value[3]);
    const double square = finite(reciprocal * reciprocal);
    const double factor = finite(-derivative[3] * square);
    for (unsigned axis = 0; axis < 3; ++axis) {
        result.point[axis] = finite(value[axis] * reciprocal);
        result.tangent[axis] = finite(value[axis] * factor + derivative[axis] * reciprocal);
    }
    return result;
}

unsigned native_bezier_edge_count(const std::vector<BezierPole> &poles, bool weights_are_one,
                                  double chord_tolerance, double angle_tolerance,
                                  double maximum_edge_length, BezierWork work) {
    if (poles.size() < 2)
        return 0;
    validate(poles, !weights_are_one);
    finite(chord_tolerance);
    finite(angle_tolerance);
    finite(maximum_edge_length);
    work.charge(24 * poles.size());
    std::array<Point3, 26> xyz{}, directions{};
    for (std::size_t i = 0; i < poles.size(); ++i) {
        if (!weights_are_one && poles[i][3] == 0)
            continue; // Native GetProjectedXYZ writes zero at W==0.
        const double scale = weights_are_one || poles[i][3] == 1 ? 1. : finite(1 / poles[i][3]);
        for (unsigned k = 0; k < 3; ++k)
            xyz[i][k] = finite(poles[i][k] * scale);
    }
    double longest = 0, min_cosine = 1;
    for (std::size_t i = 1; i < poles.size(); ++i) {
        directions[i - 1] = subtract(xyz[i], xyz[i - 1]);
        longest = std::max(longest, normalize(directions[i - 1]));
    }
    for (std::size_t i = 1; i + 1 < poles.size(); ++i)
        min_cosine = std::min(min_cosine, dot(directions[i - 1], directions[i]));
    // Native acos is not clamped. Roundoff below -1 yields NaN, whose ordered
    // comparisons skip angle/chord refinement; length refinement still applies.
    const double angle = std::acos(min_cosine);
    unsigned count = unsigned(poles.size() - 1);
    if (chord_tolerance > 0) {
        const double height = longest * std::sin(angle * .5);
        if (height > chord_tolerance)
            count = std::max(count, candidate(1.25 * std::sqrt(height / chord_tolerance), 1));
    }
    if (angle_tolerance > 0 && angle > angle_tolerance)
        count = std::max(count, candidate(angle / angle_tolerance, unsigned(poles.size() - 2)));
    if (maximum_edge_length > 0 && longest > maximum_edge_length)
        count =
            std::max(count, candidate(longest / maximum_edge_length, unsigned(poles.size() - 1)));
    return count;
}

BezierMoments native_bezier_moments(const std::vector<BezierPole> &poles, double u0, double u1,
                                    const Point3 &origin, BezierWork work) {
    validate(poles, true);
    finite(u0);
    finite(u1);
    for (double x : origin)
        finite(x);
    work.charge(1000);
    BezierMoments result;
    auto evaluate = [&](double u) {
        const auto p = native_bezier_point_tangent(poles, u, work);
        ++result.evaluations;
        result.weight_fallbacks += p.weight_fallback;
        return p;
    };
    const auto p0 = evaluate(u0).point, p1 = evaluate(u1).point;
    auto unit = subtract(p1, p0);
    normalize(unit);
    auto integrand = [&](double u) {
        const auto p = evaluate(u);
        const auto v = subtract(p.point, p0);
        const double along = dot(v, unit), slope = dot(p.tangent, unit);
        Point3 edge{}, average{}, perpendicular{}, centroid{};
        for (unsigned k = 0; k < 3; ++k) {
            edge[k] = finite(p0[k] + unit[k] * along);
            average[k] = finite(p.tangent[k] * .5 + unit[k] * (.5 * slope));
            perpendicular[k] = finite(v[k] + unit[k] * (-along));
            centroid[k] = finite((edge[k] + .5 * (p.point[k] - edge[k])) - origin[k]);
        }
        const auto normal = cross(perpendicular, average);
        std::array<double, 12> f{};
        for (unsigned r = 0; r < 3; ++r) {
            f[r] = normal[r];
            for (unsigned c = 0; c < 3; ++c)
                f[3 + 3 * r + c] = finite(centroid[r] * normal[c]);
        }
        return f;
    };
    const double q = 2 * std::sqrt(10. / 7), b = 13 * std::sqrt(70.), a1 = std::sqrt(5 - q) / 3,
                 a2 = std::sqrt(5 + q) / 3, w1 = (322 + b) / 900, w2 = (322 - b) / 900;
    const std::array<double, 5> nodes{-a2, -a1, 0, a1, a2}, weights{w2, w1, 128. / 225, w1, w2};
    auto integrate = [&](double left, double right, std::array<double, 12> &sums) {
        const double scale = finite((right - left) * .5);
        for (unsigned i = 0; i < 5; ++i) {
            const double u = finite((nodes[i] - (-1.)) * scale + left);
            const double weight = finite(scale * weights[i]);
            const auto f = integrand(u);
            for (unsigned k = 0; k < 12; ++k)
                sums[k] = finite(sums[k] + f[k] * weight);
        }
    };
    std::array<double, 12> coarse{}, fine{};
    integrate(u0, u1, coarse);
    const double middle = finite(.5 * (u0 + u1));
    integrate(u0, middle, fine);
    integrate(middle, u1, fine);
    const auto v0 = subtract(p0, origin), v1 = subtract(p1, origin);
    const auto triangle = cross(v0, v1);
    for (unsigned r = 0; r < 3; ++r) {
        result.normal[r] = finite(fine[r] + triangle[r] * .5);
        const double centroid = finite(v0[r] * (1. / 3) + v1[r] * (1. / 3));
        for (unsigned c = 0; c < 3; ++c)
            result.centroid_tensor[r][c] =
                finite(fine[3 + 3 * r + c] + (.5 * centroid) * triangle[c]);
    }
    return result;
}

BezierMoments native_bezier_area(const std::vector<BezierPole> &poles, bool weights_are_one,
                                 const Point3 &origin, BezierWork work) {
    validate(poles, true);
    BezierMoments result;
    result.edges = native_bezier_edge_count(poles, weights_are_one, 0, .3, 0, work);
    const double step = 1. / result.edges;
    for (unsigned i = 0; i < result.edges; ++i) {
        const auto m = native_bezier_moments(poles, i * step, (i + 1) * step, origin, work);
        result.evaluations += m.evaluations;
        result.weight_fallbacks += m.weight_fallbacks;
        for (unsigned r = 0; r < 3; ++r) {
            result.normal[r] = finite(result.normal[r] + m.normal[r]);
            for (unsigned c = 0; c < 3; ++c)
                result.centroid_tensor[r][c] =
                    finite(result.centroid_tensor[r][c] + m.centroid_tensor[r][c]);
        }
    }
    return result;
}
} // namespace p3d::curve_detail
