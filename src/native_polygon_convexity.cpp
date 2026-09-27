// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from polygon3d.cpp. Original P3D arithmetic and strict area ratio.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_polygon_convexity.hpp"
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
namespace p3d::swept_detail {
NativePolygonConvexity native_polygon_convexity(const std::vector<Point3> &points,
                                                TubeBudget &budget) {
    using curve_detail::bezier_support::finite;
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    require(points.size() <= budget.max_control_points && points.size() <= INT32_MAX,
            "native convexity point extent");
    work.charge(points.size());
    for (const auto &p : points)
        for (double x : p)
            finite(x);
    auto count = points.size();
    while (count > 2 && points[count - 1] == points[0]) {
        work.charge(1);
        --count;
    }
    NativePolygonConvexity out;
    auto finish = [&] {
        out.report = {{"convex", out.convex},
                      {"source_point_count", points.size()},
                      {"effective_point_count", count},
                      {"trailing_copies_removed", points.size() - count},
                      {"unit_normal", out.unit_normal},
                      {"positive_turn_sum", out.positive_turn_sum},
                      {"negative_turn_sum", out.negative_turn_sum},
                      {"relative_turn_tolerance", 1e-12}};
        return out;
    };
    if (count < 3)
        return finish();
    auto delta = [&](const Point3 &a, const Point3 &b) {
        work.charge(3);
        return Point3{finite(a[0] - b[0]), finite(a[1] - b[1]), finite(a[2] - b[2])};
    };
    auto cross = [&](const Point3 &a, const Point3 &b) {
        work.charge(9);
        return Point3{finite(finite(a[1] * b[2]) - finite(a[2] * b[1])),
                      finite(finite(b[0] * a[2]) - finite(a[0] * b[2])),
                      finite(finite(a[0] * b[1]) - finite(b[0] * a[1]))};
    };
    auto squared = [&](const Point3 &p) {
        work.charge(5);
        return finite(finite(finite(p[0] * p[0]) + finite(p[1] * p[1])) + finite(p[2] * p[2]));
    };
    Point3 largest{};
    double max_square = 0;
    auto a = delta(points[1], points[0]);
    for (std::size_t i = 2; i < count; ++i) {
        const auto b = delta(points[i], points[0]), c = cross(a, b);
        const double square = squared(c);
        if (square > max_square) {
            max_square = square;
            largest = c;
        }
        a = b;
    }
    const double magnitude = std::sqrt(squared(largest));
    if (magnitude > 0) {
        work.charge(4);
        const auto inverse = finite(1 / magnitude);
        for (unsigned i = 0; i < 3; ++i)
            out.unit_normal[i] = finite(largest[i] * inverse);
    } else
        out.unit_normal = {1, 0, 0};
    a = delta(points[0], points[count - 1]);
    for (std::size_t i = 1; i <= count; ++i) {
        const auto b = delta(points[i % count], points[i - 1]), c = cross(a, b);
        work.charge(6);
        const double dot =
            finite(finite(finite(c[1] * out.unit_normal[1]) + finite(c[0] * out.unit_normal[0])) +
                   finite(c[2] * out.unit_normal[2]));
        if (dot >= 0)
            out.positive_turn_sum = finite(out.positive_turn_sum + dot);
        else
            out.negative_turn_sum = finite(out.negative_turn_sum + dot);
        a = b;
    }
    work.charge(1);
    out.convex = finite(1e-12 * out.positive_turn_sum) > std::abs(out.negative_turn_sum);
    return finish();
}
} // namespace p3d::swept_detail
