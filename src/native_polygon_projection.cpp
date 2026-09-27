#include "native_polygon_projection.hpp"
// Polygon normal and triad algorithms also follow Bentley imodel-native
// polygon3d.cpp / PolygonOps.cpp. Native P3D branch and arithmetic differences
// are preserved. Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
bool disconnect(const Point3 &p) {
    const double marker = std::numeric_limits<double>::max();
    return p[0] == marker || p[1] == marker || p[2] == marker;
}
Point3 difference(const Point3 &a, const Point3 &b) {
    return {finite(a[0] - b[0]), finite(a[1] - b[1]), finite(a[2] - b[2])};
}
Point3 cross(const Point3 &a, const Point3 &b) {
    return {finite(finite(a[1] * b[2]) - finite(a[2] * b[1])),
            finite(finite(a[2] * b[0]) - finite(a[0] * b[2])),
            finite(finite(a[0] * b[1]) - finite(a[1] * b[0]))};
}
double squared(const Point3 &a) {
    return finite(finite(finite(a[1] * a[1]) + finite(a[0] * a[0])) + finite(a[2] * a[2]));
}
double normalize(Point3 &a) {
    const double length = std::sqrt(squared(a));
    if (length > 0) {
        const double scale = finite(1 / length);
        for (auto &x : a)
            x = finite(x * scale);
    }
    return length; // GePoint3d: leave a zero-length vector unchanged.
}
Point3 triangle_normal(const std::vector<Point3> &p, unsigned &origin, bool &median) {
    const double d01 = squared(difference(p[1], p[0])), d02 = squared(difference(p[2], p[0])),
                 d12 = squared(difference(p[2], p[1]));
    double d1, d2;
    if (d02 > d12) {
        if (d01 > d02) {
            origin = 2;
            d1 = d02;
            d2 = d12;
        } else {
            origin = 1;
            d1 = d12;
            d2 = d01;
        }
    } else {
        origin = 0;
        d1 = d01;
        d2 = d02;
    }
    const auto &base = p[origin], &end1 = p[(origin + 1) % 3], &end2 = p[(origin + 2) % 3];
    const auto a = difference(end1, base), b = difference(end2, base);
    auto n = cross(a, b);
    median = squared(n) <= finite(finite(d1 * 1e-5) * d2);
    if (median) {
        Point3 v{};
        for (unsigned k = 0; k < 3; ++k)
            v[k] = finite(finite(finite(finite(end2[k] - end1[k]) * 0.5) + end1[k]) - base[k]);
        const auto n1 = cross(a, v), n2 = cross(v, b);
        n = finite(squared(n1) * d2) >= finite(squared(n2) * d1) ? n1 : n2;
    }
    return n;
}
Matrix3 axes(Point3 normal, Point3 edge, bool &normalized) {
    // squareAndNormalizeColumns(primary=2, secondary=0). Unlike the curve
    // frame helper this uses GePoint3d normalization and ignores third length.
    const double primary_length = normalize(normal);
    auto third = cross(normal, edge);
    normalize(third);
    edge = cross(third, normal);
    normalized = normalize(edge) != 0;
    if (!normalized) {
        if (primary_length == 0)
            return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        // getNormalizedTriad: reference Y near Z, otherwise reference Z.
        const double threshold = finite(std::sqrt(squared(normal)) * 0.015625);
        const Point3 reference = threshold > std::abs(normal[0]) && threshold > std::abs(normal[1])
                                     ? Point3{0, 1, 0}
                                     : Point3{0, 0, 1};
        edge = cross(reference, normal);
        third = cross(normal, edge);
        normalize(edge);
        normalize(third);
        normalize(normal);
    }
    Matrix3 out{};
    for (unsigned i = 0; i < 3; ++i) {
        out[i][0] = edge[i];
        out[i][1] = third[i];
        out[i][2] = normal[i];
    }
    return out;
}
} // namespace
NativePolygonProjection prepare_native_polygon_projection(const std::vector<Point3> &points,
                                                          TubeBudget &budget) {
    require(points.size() <= budget.max_control_points && points.size() <= INT32_MAX,
            "native polygon projection point extent");
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(points.size());
    for (const auto &p : points)
        for (double x : p)
            finite(x);
    NativePolygonProjection out;
    for (unsigned i = 0; i < 4; ++i)
        out.local_to_world[i][i] = out.world_to_local[i][i] = 1;
    std::size_t count = 0, selected_edge = 0, attempts = 0, markers = 0;
    while (count < points.size() && !disconnect(points[count])) {
        work.charge(1);
        ++count;
    }
    Point3 normal{};
    double area = 0, threshold = 0;
    unsigned triangle_origin = 0;
    bool median = false, normalized = false;
    if (count >= 3) {
        if (count == 3) {
            work.charge(160);
            normal = triangle_normal(points, triangle_origin, median);
        } else {
            auto previous = difference(points[1], points[0]);
            for (std::size_t k = 2; k < count; ++k) {
                work.charge(24);
                const auto current = difference(points[k], points[0]);
                const auto term = cross(previous, current);
                for (unsigned i = 0; i < 3; ++i)
                    normal[i] = finite(term[i] + normal[i]);
                previous = current;
            }
        }
        area = finite(normalize(normal) * 0.5);
        threshold = finite(std::sqrt(area) * 0.001);
        for (std::size_t k = 1; k < count && !out.frame_succeeded; ++k) {
            work.charge(24);
            auto edge = difference(points[k], points[0]);
            if (!(normalize(edge) > threshold))
                continue;
            work.charge(512);
            ++attempts;
            const auto matrix = axes(normal, edge, normalized);
            for (unsigned i = 0; i < 3; ++i) {
                for (unsigned j = 0; j < 3; ++j)
                    out.local_to_world[i][j] = matrix[i][j];
                out.local_to_world[i][3] = points[0][i];
            }
            const auto inverse = native_matrix_inverse(matrix);
            // ab0d0 leaves identity on failure. Its sum starts at X, whereas
            // the subsequent in-place point multiply starts at Y.
            out.world_to_local = {};
            for (unsigned i = 0; i < 4; ++i)
                out.world_to_local[i][i] = 1;
            if (inverse.inverted) {
                out.frame_succeeded = true;
                selected_edge = k;
                for (unsigned i = 0; i < 3; ++i) {
                    for (unsigned j = 0; j < 3; ++j)
                        out.world_to_local[i][j] = inverse.matrix[i][j];
                    out.world_to_local[i][3] =
                        finite(finite(finite(-points[0][0] * inverse.matrix[i][0]) +
                                      finite(-points[0][1] * inverse.matrix[i][1])) +
                               finite(-points[0][2] * inverse.matrix[i][2]));
                }
            }
        }
    }
    if (out.frame_succeeded) {
        work.charge(points.size());
        out.points = points;
        for (auto &p : out.points) {
            work.charge(1);
            if (disconnect(p)) {
                ++markers;
                continue;
            }
            work.charge(24);
            Point3 local{};
            for (unsigned i = 0; i < 3; ++i) {
                const auto &row = out.world_to_local[i];
                local[i] = finite(finite(finite(finite(p[1] * row[1]) + finite(p[0] * row[0])) +
                                         finite(p[2] * row[2])) +
                                  row[3]);
            }
            p = local;
        }
    }
    out.report = {{"scope", "native_polygon_projection_preparation"},
                  {"frame_succeeded", out.frame_succeeded},
                  {"coordinate_selector", 0},
                  {"first_loop_points", count},
                  {"normal", normal},
                  {"native_area", area},
                  {"edge_length_threshold", threshold},
                  {"triangle_origin", count == 3 ? Json(triangle_origin) : Json()},
                  {"triangle_median_used", median},
                  {"selected_edge", out.frame_succeeded ? Json(selected_edge) : Json()},
                  {"frame_attempts", attempts},
                  {"square_normalization_succeeded", normalized},
                  {"disconnect_markers_preserved", markers},
                  {"triangulated", false},
                  {"work_used", budget.work}};
    return out;
}
} // namespace p3d::swept_detail
