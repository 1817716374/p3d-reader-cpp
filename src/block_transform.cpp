#include "internal.hpp"

namespace p3d {
namespace {
using Columns = std::array<Point3, 3>;
double squared(const Point3 &v) {
    const auto result = (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2];
    require(std::isfinite(result), "native block matrix arithmetic overflow");
    return result;
}
double inner(const Point3 &a, const Point3 &b) {
    return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
}
Point3 crossed(const Point3 &a, const Point3 &b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
void scale(Point3 &a, double s) {
    for (auto &x : a)
        x *= s;
}
double normalize(Point3 &a) {
    const auto length = std::sqrt(squared(a));
    if (length > 0)
        scale(a, 1 / length);
    return length;
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
struct Factors {
    Columns columns, rotation{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    bool converged = false;
};
Factors orthogonalize(const Columns &m) {
    Factors f;
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
bool native_invertible(const Columns &m) {
    double largest = 0;
    for (const auto &column : m)
        for (auto x : column) {
            require(std::isfinite(x), "nonfinite native block matrix");
            largest = std::max(largest, std::abs(x));
        }
    if (largest == 0)
        return false;
    auto a = m;
    for (auto &column : a)
        scale(column, 1 / largest);
    const auto cofactor0 = a[2][2] * a[1][1] - a[2][1] * a[1][2];
    const auto cofactor1 = a[2][0] * a[1][2] - a[2][2] * a[1][0];
    const auto cofactor2 = a[2][1] * a[1][0] - a[2][0] * a[1][1];
    const auto determinant = (cofactor1 * a[0][1] + cofactor0 * a[0][0]) + cofactor2 * a[0][2];
    if (std::abs(determinant) > 1e-8)
        return true;
    const auto f = orthogonalize(m);
    if (!f.converged)
        return false;
    std::array<double, 3> lengths{};
    for (unsigned i = 0; i < 3; ++i) {
        const auto &v = f.columns[i];
        lengths[i] = (v[1] * v[1] + v[0] * v[0]) + v[2] * v[2];
        require(std::isfinite(lengths[i]), "native block inverse factor overflow");
    }
    const auto hi = *std::max_element(lengths.begin(), lengths.end());
    const auto lo = *std::min_element(lengths.begin(), lengths.end());
    return hi > 0 && lo > hi * 1e-26 && std::sqrt(lo / hi) > 1e-15;
}
void repair_column(Point3 &missing, Point3 &next, Point3 &last) {
    auto a = next, b = last;
    const auto length_a = normalize(a), length_b = normalize(b);
    auto c = crossed(a, b);
    a = crossed(b, c);
    scale(a, length_a);
    scale(b, length_b);
    if (length_a == length_b)
        scale(c, length_a);
    missing = c;
    next = a;
    last = b;
}
Point3 geometric_cross(const Point3 &a, const Point3 &b) {
    auto v = crossed(a, b);
    const auto length = std::sqrt(squared(v));
    if (length != 0) {
        const auto factor = std::sqrt(std::sqrt(squared(a) * squared(b))) / length;
        require(std::isfinite(factor), "native block rank completion overflow");
        scale(v, factor);
    }
    return v;
}
// This helper mirrors the native triad's first output. The rank-one caller
// supplies the first row of its rotation factor, not the surviving source
// column; preserve that call-site convention rather than substituting an SVD.
Point3 triad_first(Point3 source) {
    auto length = std::sqrt(squared(source));
    const auto tolerance = length * 0.015625;
    if (length == 0) {
        source[2] = 1;
        length = 1;
    }
    const Point3 axis = tolerance > std::abs(source[0]) && tolerance > std::abs(source[1])
                            ? Point3{0, 1, 0}
                            : Point3{0, 0, 1};
    auto first = crossed(axis, source);
    normalize(first);
    scale(first, length);
    return first;
}
unsigned augment_rank(Columns &m, bool &converged) {
    const auto f = orthogonalize(m);
    converged = f.converged;
    std::array<double, 3> lengths{};
    for (unsigned i = 0; i < 3; ++i)
        lengths[i] = std::sqrt(squared(f.columns[i]));
    const auto tolerance = ((lengths[0] + lengths[1]) + lengths[2]) * 1e-12;
    std::vector<unsigned> absent, present;
    for (unsigned i = 0; i < 3; ++i)
        (lengths[i] <= tolerance ? absent : present).push_back(i);
    if (absent.size() == 3) {
        m = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    } else if (absent.size() == 1) {
        const auto k = absent[0];
        const auto added = geometric_cross(f.columns[(k + 1) % 3], f.columns[(k + 2) % 3]);
        for (unsigned column = 0; column < 3; ++column)
            for (unsigned row = 0; row < 3; ++row)
                m[column][row] += added[row] * f.rotation[k][column];
    } else if (absent.size() == 2) {
        const auto k = present[0];
        const Point3 first_row{f.rotation[0][0], f.rotation[1][0], f.rotation[2][0]};
        const auto next_row = triad_first(first_row);
        const auto &u = f.rotation[(k + 1) % 3], &v = f.rotation[(k + 2) % 3];
        for (unsigned column = 0; column < 3; ++column)
            for (unsigned row = 0; row < 3; ++row)
                m[column][row] =
                    (first_row[row] * u[column] + m[column][row]) + next_row[row] * v[column];
    }
    return unsigned(present.size());
}
} // namespace
Json native_block_transform(const Bytes &base) {
    Reader r(base, 164);
    Matrix4 source{{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    Columns m{};
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned column = 0; column < 3; ++column)
            m[column][row] = source[row][column] = r.f64();
    for (unsigned row = 0; row < 3; ++row) {
        source[row][3] = r.f64();
        require(std::isfinite(source[row][3]), "nonfinite native block translation");
    }
    Json result = {{"reader_profile", "bimbase_2025_block_transform_input"},
                   {"scope", "record_input_if_accepted"},
                   {"status", "resolved"},
                   {"source_matrix", source},
                   {"matrix_source_offset", 164},
                   {"translation_source_offset", 236},
                   {"repair", "unchanged"},
                   {"repaired_column_indices", Json::array()}};
    if (!native_invertible(m)) {
        for (unsigned i = 0; i < 3; ++i)
            if (squared(m[i]) < 1e-6) {
                repair_column(m[i], m[(i + 1) % 3], m[(i + 2) % 3]);
                result["repaired_column_indices"].push_back(i);
            }
        result["repair"] = "short_columns";
        if (!native_invertible(m)) {
            bool converged;
            result["rank_before_augmentation"] = augment_rank(m, converged);
            result["rank_factors_converged"] = converged;
            result["repair"] = "rank_augmentation";
        }
    }
    auto effective = source;
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned column = 0; column < 3; ++column) {
            require(std::isfinite(m[column][row]), "nonfinite repaired native block matrix");
            effective[row][column] = m[column][row];
        }
    result["matrix"] = effective;
    return result;
}
} // namespace p3d
