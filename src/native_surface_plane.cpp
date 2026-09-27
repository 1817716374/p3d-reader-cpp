// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Adapted from DPoint3dOps.cpp, eigensys3d.cpp and refdmatrix4d.cpp;
// native P3D arithmetic, axis ordering and actual-point ranges. See THIRD_PARTY.md.
#include "native_surface_plane.hpp"
#include "native_bezier.hpp"
namespace p3d::swept_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native surface principal extents nonfinite arithmetic");
    return x;
}
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
Matrix3 identity() {
    return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
}
struct Eigen {
    Matrix3 axes = identity();
    Point3 values{};
    unsigned rotations = 0, sweeps = 0;
    bool converged = false;
};
Eigen jacobi(Matrix3 a, TubeBudget &budget) {
    Eigen e;
    Point3 b{}, z{};
    for (unsigned i = 0; i < 3; ++i)
        b[i] = e.values[i] = a[i][i];
    for (unsigned sweep = 0; sweep < 50; ++sweep) {
        charge(budget, 100);
        e.sweeps = sweep;
        double sum = 0;
        for (unsigned p = 0; p < 3; ++p)
            for (unsigned q = p + 1; q < 3; ++q)
                sum = finite(sum + std::abs(a[p][q]));
        if (sum == 0) {
            e.converged = true;
            return e;
        }
        const double threshold = sweep < 3 ? finite(.2 * sum) / 9 : 0;
        for (unsigned p = 0; p < 2; ++p)
            for (unsigned q = p + 1; q < 3; ++q) {
                const double g = finite(100 * std::abs(a[p][q]));
                if (sweep > 3 && finite(std::abs(e.values[p]) + g) == std::abs(e.values[p]) &&
                    finite(std::abs(e.values[q]) + g) == std::abs(e.values[q]))
                    a[p][q] = 0;
                else if (std::abs(a[p][q]) > threshold) {
                    double h = finite(e.values[q] - e.values[p]), t;
                    if (finite(std::abs(h) + g) == std::abs(h))
                        t = finite(a[p][q] / h);
                    else {
                        const double theta = finite(finite(.5 * h) / a[p][q]);
                        t = finite(1 / finite(std::sqrt(finite(1 + finite(theta * theta))) +
                                              std::abs(theta)));
                        if (theta < 0)
                            t = -t;
                    }
                    const double c = finite(1 / std::sqrt(finite(1 + finite(t * t))));
                    const double s = finite(t * c), tau = finite(s / (1 + c));
                    h = finite(t * a[p][q]);
                    z[p] = finite(z[p] - h);
                    z[q] = finite(z[q] + h);
                    e.values[p] = finite(e.values[p] - h);
                    e.values[q] = finite(e.values[q] + h);
                    a[p][q] = 0;
                    auto rotate = [&](double &left, double &right) {
                        const double u = left, v = right;
                        left = finite(u - finite(s * finite(v + finite(u * tau))));
                        right = finite(v + finite(s * finite(u - finite(v * tau))));
                    };
                    for (unsigned j = 0; j < p; ++j)
                        rotate(a[j][p], a[j][q]);
                    for (unsigned j = p + 1; j < q; ++j)
                        rotate(a[p][j], a[j][q]);
                    for (unsigned j = q + 1; j < 3; ++j)
                        rotate(a[p][j], a[q][j]);
                    for (unsigned j = 0; j < 3; ++j)
                        rotate(e.axes[j][p], e.axes[j][q]);
                    ++e.rotations;
                }
            }
        for (unsigned i = 0; i < 3; ++i) {
            b[i] = finite(b[i] + z[i]);
            e.values[i] = b[i];
            z[i] = 0;
        }
        e.sweeps = sweep + 1;
    }
    return e; // Native caller also accepts the final iterate after 50 sweeps.
}
void sort(Eigen &e) {
    for (const auto ij : {std::array<unsigned, 2>{0, 1}, {0, 2}, {1, 2}})
        if (e.values[ij[0]] < e.values[ij[1]]) {
            std::swap(e.values[ij[0]], e.values[ij[1]]);
            for (auto &row : e.axes)
                std::swap(row[ij[0]], row[ij[1]]);
        }
    const auto &a = e.axes;
    const double det = finite(((((a[0][1] * a[1][2]) * a[2][0] + (a[1][1] * a[0][0]) * a[2][2]) +
                                (a[1][0] * a[0][2]) * a[2][1]) -
                               (a[0][0] * a[1][2]) * a[2][1]) -
                              (a[0][1] * a[1][0]) * a[2][2] - (a[1][1] * a[0][2]) * a[2][0]);
    if (det < 0)
        for (auto &r : e.axes)
            r[2] *= -1;
}
Point3 transform(const Matrix3 &a, const Point3 &t, const Point3 &p) {
    Point3 q;
    for (unsigned i = 0; i < 3; ++i)
        q[i] = finite(((a[i][1] * p[1] + a[i][0] * p[0]) + a[i][2] * p[2]) + t[i]);
    return q;
}
Point3 translation(const Matrix3 &inverse, const Point3 &origin) {
    Point3 t;
    for (unsigned i = 0; i < 3; ++i)
        t[i] = finite((inverse[i][1] * -origin[1] + inverse[i][0] * -origin[0]) +
                      inverse[i][2] * -origin[2]);
    return t;
}
struct Range {
    Point3 low, high;
};
Range range(const std::vector<Point3> &points, const Matrix3 &a, const Point3 &t, TubeBudget &b) {
    const double big = std::numeric_limits<double>::max();
    Range out{{big, big, big}, {-big, -big, -big}};
    for (const auto &p : points) {
        charge(b, 1);
        const auto q = transform(a, t, p);
        // Native disconnect sentinel is skipped after transformation.
        if (std::find(q.begin(), q.end(), big) != q.end())
            continue;
        for (unsigned i = 0; i < 3; ++i) {
            if (q[i] < out.low[i])
                out.low[i] = q[i];
            if (q[i] > out.high[i])
                out.high[i] = q[i];
        }
    }
    return out;
}
Point3 diagonal(const Range &r) {
    Point3 d;
    for (unsigned i = 0; i < 3; ++i)
        d[i] = finite(r.high[i] - r.low[i]);
    return d;
}
} // namespace
Json native_surface_plane(const BsplineSurface &surface, TubeBudget &budget) {
    const auto &poles = surface.poles();
    require(poles.size() <= budget.max_control_points, "native surface principal control budget");
    charge(budget, poles.size());
    Json report{{"status", "computed"}, {"planar", false}, {"principal_extents_succeeded", false}};
    if (poles.empty())
        return report;
    std::vector<Point3> points = poles;
    if (surface.rational())
        for (std::size_t i = 0; i < points.size(); ++i) {
            const double inverse = finite(1 / surface.weights()[i]);
            for (double &x : points[i])
                x = finite(x * inverse);
        }
    Point3 origin{};
    for (const auto &p : points)
        for (unsigned i = 0; i < 3; ++i)
            origin[i] = finite(origin[i] + p[i]);
    const double reciprocal = 1 / double(points.size());
    for (double &x : origin)
        x = finite(x * reciprocal);
    Matrix4 products{};
    for (const auto &p : points) {
        charge(budget, 1);
        Point3 d;
        for (unsigned i = 0; i < 3; ++i)
            d[i] = finite(p[i] - origin[i]);
        for (unsigned i = 0; i < 3; ++i) {
            for (unsigned j = i; j < 3; ++j)
                products[i][j] = finite(products[i][j] + finite(d[i] * d[j]));
            products[i][3] = finite(products[i][3] + d[i]);
        }
    }
    products[3][3] = double(points.size());
    for (unsigned i = 0; i < 4; ++i)
        for (unsigned j = 0; j < i; ++j)
            products[i][j] = products[j][i];
    // The inertia conversion's identity sandwich still performs the products
    // and additions. Keep its zero/sign behavior rather than bypassing it.
    Matrix4 left = products, result{};
    const auto id = identity();
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 4; ++j)
            left[i][j] = ((id[i][0] * products[0][j] + id[i][1] * products[1][j]) +
                          id[i][2] * products[2][j]) +
                         0. * products[3][j];
    for (unsigned i = 0; i < 4; ++i) {
        for (unsigned j = 0; j < 3; ++j)
            result[i][j] =
                ((id[j][1] * left[i][1] + id[j][0] * left[i][0]) + id[j][2] * left[i][2]) +
                0. * left[i][3];
        result[i][3] = left[i][3];
    }
    Point3 first{result[0][3], result[1][3], result[2][3]}, shift{};
    if (!(double(points.size()) >
          finite(std::max({std::abs(first[0]), std::abs(first[1]), std::abs(first[2])}) * 1e-12))) {
        report["reason"] = "inertia_centroid_division_failed";
        return report;
    }
    for (unsigned i = 0; i < 3; ++i)
        shift[i] = finite(reciprocal * first[i]);
    Matrix3 q{}, inertia{};
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j) {
            q[i][j] = finite(result[i][j] + finite((first[i] * -1) * shift[j]));
            inertia[i][j] = q[i][j] * -1;
        }
    inertia[0][0] = finite(q[1][1] + q[2][2]);
    inertia[1][1] = finite(q[0][0] + q[2][2]);
    inertia[2][2] = finite(q[1][1] + q[0][0]);
    auto e = jacobi(inertia, budget);
    sort(e);
    for (unsigned i = 0; i < 3; ++i)
        origin[i] = finite(origin[i] + (0. + shift[i]));
    Matrix3 fast_inverse{};
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            fast_inverse[i][j] = e.axes[j][i];
    const auto initial = range(points, fast_inverse, translation(fast_inverse, origin), budget);
    const auto d0 = diagonal(initial);
    unsigned z = 0;
    if (std::abs(d0[0]) > std::abs(d0[1]))
        z = 1;
    if (std::abs(d0[z]) > std::abs(d0[2]))
        z = 2;
    const unsigned x = (z + 1) % 3, y = (z + 2) % 3;
    Matrix3 axes{};
    for (unsigned i = 0; i < 3; ++i)
        axes[i] = {e.axes[i][x], e.axes[i][y], e.axes[i][z]};
    if (axes[2][2] < 0)
        for (auto &r : axes) {
            r[0] = -r[0];
            r[2] = -r[2];
        }
    if (axes[0][0] < 0)
        for (auto &r : axes) {
            r[0] = -r[0];
            r[1] = -r[1];
        }
    const auto inverse = native_matrix_inverse(axes);
    const auto t = inverse.inverted ? translation(inverse.matrix, origin) : Point3{};
    const auto ordered = range(points, inverse.matrix, t, budget);
    const auto d = diagonal(ordered);
    auto corner = transform(axes, origin, ordered.low);
    for (auto &r : axes)
        for (unsigned j = 0; j < 3; ++j)
            r[j] = finite(r[j] * d[j]);
    // Compare the initial principal-axis range, not the reordered extents.
    if (d0[1] > d0[0])
        for (unsigned i = 0; i < 3; ++i) {
            corner[i] = finite(corner[i] + axes[i][1]);
            std::swap(axes[i][0], axes[i][1]);
            axes[i][0] = -axes[i][0];
        }
    Matrix4 extent{};
    extent[3][3] = 1;
    Point3 lengths{};
    for (unsigned j = 0; j < 3; ++j) {
        lengths[j] = finite(std::sqrt(
            finite((axes[0][j] * axes[0][j] + axes[1][j] * axes[1][j]) + axes[2][j] * axes[2][j])));
        for (unsigned i = 0; i < 3; ++i)
            extent[i][j] = axes[i][j];
    }
    for (unsigned i = 0; i < 3; ++i)
        extent[i][3] = corner[i];
    const double xy = finite(lengths[1] + lengths[0]), xyz = finite(xy + lengths[2]);
    const double tolerance = finite(((std::abs(xy) + 1) + std::abs(xyz)) * 1e-10);
    report.update({{"principal_extents_succeeded", true},
                   {"planar", std::abs(xyz - xy) <= tolerance},
                   {"centroid", origin},
                   {"principal_moments", e.values},
                   {"principal_axes", e.axes},
                   {"jacobi_converged", e.converged},
                   {"jacobi_sweeps", e.sweeps},
                   {"jacobi_rotations", e.rotations},
                   {"ordered_inverse_succeeded", inverse.inverted},
                   {"extent_transform", extent},
                   {"extent_lengths", lengths},
                   {"rounded_thickness", xyz - xy},
                   {"tolerance", tolerance},
                   {"work_used", budget.work}});
    return report;
}
} // namespace p3d::swept_detail
