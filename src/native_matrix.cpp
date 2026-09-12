#include "internal.hpp"

namespace p3d {
namespace {
double squared(const Point3 &v) {
    const auto result = (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2];
    require(std::isfinite(result), "native block matrix arithmetic overflow");
    return result;
}
double inner(const Point3 &a, const Point3 &b) {
    return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
}
double rotate_pair(Point3 &a, Point3 &b, Point3 &u, Point3 &v) {
    const auto aa = squared(a), bb = squared(b);
    const auto difference = aa - bb, twice_dot = 2 * inner(a, b);
    const auto tolerance = (aa + bb) * 1e-12;
    double c = 1, s = 0;
    if (!(tolerance > std::abs(twice_dot) && tolerance > std::abs(difference))) {
        const auto radius = std::sqrt(difference * difference + twice_dot * twice_dot);
        require(std::isfinite(radius), "native block orthogonalization overflow");
        if (radius != 0) {
            const auto x = difference / radius, y = twice_dot / radius;
            if (x >= 0) {
                c = std::sqrt((1 + x) * 0.5);
                s = y / (c + c);
            } else {
                s = std::sqrt((1 - x) * 0.5);
                if (!(y > 0))
                    s = -s;
                c = y / (s + s);
            }
        }
    }
    for (unsigned i = 0; i < 3; ++i) {
        const auto ai = a[i], bi = b[i], ui = u[i], vi = v[i];
        a[i] = bi * s + ai * c;
        b[i] = ai * -s + bi * c;
        u[i] = ui * c + vi * s;
        v[i] = vi * c + ui * -s;
    }
    return std::abs(s);
}
} // namespace
NativeOrthogonalFactors native_orthogonalize_columns(const std::array<Point3, 3> &m) {
    NativeOrthogonalFactors f;
    f.columns = m;
    for (unsigned sweep = 0; sweep < 10; ++sweep) {
        auto error = rotate_pair(f.columns[0], f.columns[1], f.rotation[0], f.rotation[1]);
        error += rotate_pair(f.columns[1], f.columns[2], f.rotation[1], f.rotation[2]);
        error += rotate_pair(f.columns[0], f.columns[2], f.rotation[0], f.rotation[2]);
        if (error <= 1e-14) {
            f.converged = true;
            break;
        }
    }
    return f;
}
NativeMatrixInverse native_matrix_inverse(const Matrix3 &m) {
    NativeMatrixInverse out;
    double largest = 0;
    for (const auto &row : m)
        for (auto x : row) {
            require(std::isfinite(x), "nonfinite native inverse input");
            largest = std::max(largest, std::abs(x));
        }
    if (largest == 0) {
        out.method = "zero_matrix_identity";
        return out;
    }
    auto a = m;
    const auto reciprocal = 1 / largest;
    require(std::isfinite(reciprocal), "native inverse normalization overflow");
    for (auto &row : a)
        for (auto &x : row)
            x *= reciprocal;
    Matrix3 cofactors{};
    cofactors[0] = {a[2][2] * a[1][1] - a[2][1] * a[1][2], a[0][2] * a[2][1] - a[2][2] * a[0][1],
                    a[1][2] * a[0][1] - a[0][2] * a[1][1]};
    const auto determinant =
        (cofactors[0][1] * a[1][0] + cofactors[0][0] * a[0][0]) + cofactors[0][2] * a[2][0];
    if (std::abs(determinant) > 1e-8) {
        cofactors[1] = {a[1][2] * a[2][0] - a[2][2] * a[1][0],
                        a[2][2] * a[0][0] - a[0][2] * a[2][0],
                        a[0][2] * a[1][0] - a[1][2] * a[0][0]};
        cofactors[2] = {a[2][1] * a[1][0] - a[1][1] * a[2][0],
                        a[0][1] * a[2][0] - a[2][1] * a[0][0],
                        a[1][1] * a[0][0] - a[0][1] * a[1][0]};
        const auto factor = 1 / (determinant * largest);
        for (auto &row : cofactors)
            for (auto &x : row)
                x *= factor;
        out.matrix = cofactors;
        out.inverted = true;
        out.method = "scaled_cofactors";
    } else {
        std::array<Point3, 3> columns{};
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j)
                columns[j][i] = m[i][j];
        const auto f = native_orthogonalize_columns(columns);
        if (!f.converged) {
            out.method = "nonconverged_identity";
            return out;
        }
        Point3 lengths{};
        for (unsigned i = 0; i < 3; ++i) {
            const auto &v = f.columns[i];
            lengths[i] = (v[1] * v[1] + v[0] * v[0]) + v[2] * v[2];
            require(std::isfinite(lengths[i]), "native inverse factor overflow");
        }
        const auto hi = *std::max_element(lengths.begin(), lengths.end());
        const auto lo = *std::min_element(lengths.begin(), lengths.end());
        if (!(hi > 0 && lo > hi * 1e-26 && std::sqrt(lo / hi) > 1e-15)) {
            out.method = "rank_deficient_identity";
            return out;
        }
        auto right = f.columns;
        for (unsigned i = 0; i < 3; ++i) {
            const auto factor = 1 / lengths[i];
            for (auto &x : right[i])
                x *= factor;
        }
        // f.rotation stores columns of V. Here right stores rows of
        // diag(1/|A*V|^2)*(A*V)^T, so the product is A^-1.
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j)
                out.matrix[i][j] =
                    (f.rotation[1][i] * right[1][j] + f.rotation[0][i] * right[0][j]) +
                    f.rotation[2][i] * right[2][j];
        out.inverted = true;
        out.method = "orthogonal_factors";
    }
    for (const auto &row : out.matrix)
        for (auto x : row)
            require(std::isfinite(x), "nonfinite native inverse result");
    return out;
}
} // namespace p3d
