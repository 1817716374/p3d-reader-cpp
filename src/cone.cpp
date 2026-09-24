#include <p3d/solid.hpp>
#include "geometry.hpp"

namespace p3d {
namespace {
constexpr double pi = 3.141592653589793238462643383279502884;
struct Cone {
    Point3 a, b, u, v;
    double ra, rb;
    bool capped;
    explicit Cone(const Json &table) {
        require(table.at("_type") == "DgnCone", "expected BGFB DgnCone");
        const auto &d = table.at("detail");
        auto number = [&](const std::string &key) {
            const double value = d.at(key).get<double>();
            require(std::isfinite(value), "BGFB cone nonfinite parameter");
            return value;
        };
        auto point = [&](const char *prefix) {
            return Point3{number(std::string(prefix) + "X"), number(std::string(prefix) + "Y"),
                          number(std::string(prefix) + "Z")};
        };
        a = point("centerA");
        b = point("centerB");
        u = point("vector0");
        v = point("vector90");
        ra = number("radiusA");
        rb = number("radiusB");
        capped = d.at("capped").get<bool>();
    }
    Matrix4 frame() const {
        auto f = identity();
        for (unsigned k = 0; k < 3; ++k) {
            f[k][0] = u[k];
            f[k][1] = v[k];
            f[k][2] = b[k] - a[k];
        }
        return f;
    }
};
double length(const Point3 &p) {
    return std::hypot(p[0], p[1], p[2]);
}
Matrix4 inverse_linear(const Matrix4 &f) {
    long double a[3][6]{};
    for (unsigned r = 0; r < 3; ++r) {
        for (unsigned c = 0; c < 3; ++c)
            a[r][c] = f[r][c];
        a[r][r + 3] = 1;
    }
    for (unsigned c = 0; c < 3; ++c) {
        unsigned pivot = c;
        for (unsigned r = c + 1; r < 3; ++r)
            if (std::abs(a[r][c]) > std::abs(a[pivot][c]))
                pivot = r;
        require(a[pivot][c] != 0 && std::isfinite(a[pivot][c]), "BGFB cone singular source frame");
        for (unsigned j = 0; j < 6; ++j)
            std::swap(a[c][j], a[pivot][j]);
        const auto divisor = a[c][c];
        for (unsigned j = 0; j < 6; ++j)
            a[c][j] /= divisor;
        for (unsigned r = 0; r < 3; ++r)
            if (r != c) {
                const auto factor = a[r][c];
                for (unsigned j = 0; j < 6; ++j)
                    a[r][j] -= factor * a[c][j];
            }
    }
    auto out = identity();
    for (unsigned r = 0; r < 3; ++r)
        for (unsigned c = 0; c < 3; ++c) {
            out[r][c] = double(a[r][c + 3]);
            require(std::isfinite(out[r][c]), "BGFB cone inverse frame overflow");
        }
    return out;
}
} // namespace
ConeTransformResult transform_bgfb_cone(const Json &table, const Matrix4 &matrix) {
    ConeTransformResult out;
    try {
        const Cone source(table);
        for (const auto &row : matrix)
            for (double x : row)
                require(std::isfinite(x), "BGFB cone nonfinite transform");
        require(matrix[3] == std::array<double, 4>{0, 0, 0, 1},
                "BGFB cone transform must be affine");
        const auto a = transform(matrix, source.a), b = transform(matrix, source.b);
        auto u = vector_transform(matrix, source.u), v = vector_transform(matrix, source.v);
        const auto alpha = length(u), beta = length(v);
        require(std::isfinite(alpha) && std::isfinite(beta) && alpha > 0 && beta > 0,
                "BGFB cone collapsed or overflowing section direction");
        for (unsigned k = 0; k < 3; ++k) {
            u[k] /= alpha;
            v[k] /= beta;
        }
        auto target = identity();
        for (unsigned k = 0; k < 3; ++k) {
            target[k][0] = u[k] * alpha;
            target[k][1] = v[k] * alpha;
            target[k][2] = b[k] - a[k];
        }
        auto effective = multiply(target, inverse_linear(source.frame()));
        const auto mapped = vector_transform(effective, source.a);
        for (unsigned k = 0; k < 3; ++k)
            effective[k][3] = a[k] - mapped[k];
        for (const auto &row : effective)
            for (double x : row)
                require(std::isfinite(x), "BGFB cone effective transform overflow");
        const auto det = determinant(effective);
        require(std::isfinite(det) && det != 0, "BGFB cone singular effective transform");
        auto changed = table;
        auto &d = changed["detail"];
        auto store_point = [&](const char *prefix, const Point3 &p) {
            for (unsigned k = 0; k < 3; ++k) {
                require(std::isfinite(p[k]), "BGFB cone transformed center overflow");
                d[std::string(prefix) + "XYZ"[k]] = p[k];
            }
        };
        store_point("centerA", a);
        store_point("centerB", b);
        store_point("vector0", u);
        store_point("vector90", v);
        const auto ra = source.ra * alpha, rb = source.rb * alpha;
        require(std::isfinite(ra) && std::isfinite(rb), "BGFB cone transformed radius overflow");
        d["radiusA"] = ra;
        d["radiusB"] = rb;
        out.transformed = std::move(changed);
        out.geometry_transform = effective;
        out.report = {{"radius_scale", alpha},
                      {"second_direction_length", beta},
                      {"rule", "normalize_both_directions_scale_radii_by_first"}};
        out.status = "transformed";
    } catch (const std::exception &e) {
        out.transformed = nullptr;
        out.report["reason"] = e.what();
    }
    return out;
}
SolidMeshResult mesh_bgfb_cone(const Json &table, const PolyfaceMeshOptions &options,
                               unsigned segments) {
    SolidMeshResult out;
    out.source = table;
    try {
        const Cone cone(table);
        require(segments >= 3, "BGFB cone needs at least three angular segments");
        require((cone.ra != 0 || cone.rb != 0) &&
                    (cone.ra == 0 || cone.rb == 0 || (cone.ra < 0) == (cone.rb < 0)),
                "BGFB cone with two zero or opposite-sign radii not supported");
        const auto sign = determinant(cone.frame());
        require(std::isfinite(sign) && sign != 0, "BGFB cone degenerate or overflowing frame");
        const std::size_t n = segments, na = cone.ra == 0 ? 1 : n, nb = cone.rb == 0 ? 1 : n;
        const std::size_t caps =
            cone.capped ? std::size_t(cone.ra != 0) + std::size_t(cone.rb != 0) : 0;
        require(na + nb <= options.max_points, "BGFB cone point budget");
        require(n * (na == 1 || nb == 1 ? 4 : 5) + caps * (n + 1) <= options.max_corners,
                "BGFB cone corner budget");
        require(n * (na == 1 || nb == 1 ? 1 : 2) + caps * (n - 2) <= options.max_triangles,
                "BGFB cone triangle budget");
        Json points = Json::array(), indices = Json::array();
        for (unsigned end = 0; end < 2; ++end) {
            const auto &center = end ? cone.b : cone.a;
            const auto radius = end ? cone.rb : cone.ra;
            for (std::size_t i = 0; i < (end ? nb : na); ++i) {
                const auto theta = 2 * pi * double(i) / double(n);
                for (unsigned k = 0; k < 3; ++k) {
                    const auto p = center[k] + radius * (cone.u[k] * std::cos(theta) +
                                                         cone.v[k] * std::sin(theta));
                    require(std::isfinite(p), "BGFB cone sampled position overflow");
                    points.push_back(p);
                }
            }
        }
        auto face = [&](std::vector<std::size_t> corners, std::array<std::int64_t, 3> id) {
            if (sign < 0)
                std::reverse(corners.begin(), corners.end());
            for (auto i : corners)
                indices.push_back(i + 1);
            indices.push_back(0);
            out.face_indices.push_back(id);
        };
        if (cone.capped && na != 1) {
            std::vector<std::size_t> corners;
            for (std::size_t i = n; i > 0; --i)
                corners.push_back(i - 1);
            face(std::move(corners), {-1, 0, 0});
        }
        if (cone.capped && nb != 1) {
            std::vector<std::size_t> corners;
            for (std::size_t i = 0; i < n; ++i)
                corners.push_back(na + i);
            face(std::move(corners), {-1, 1, 0});
        }
        for (std::size_t i = 0; i < n; ++i) {
            const auto j = (i + 1) % n;
            if (na == 1)
                face({0, na + j, na + i}, {0, 0, 0});
            else if (nb == 1)
                face({i, j, na}, {0, 0, 0});
            else
                face({i, j, na + j, na + i}, {0, 0, 0});
        }
        Json polyface = {{"_type", "Polyface"},
                         {"meshStyle", 1},
                         {"numPerFace", 0},
                         {"point", std::move(points)},
                         {"pointIndex", std::move(indices)}};
        out.derived = mesh_bgfb_polyface(polyface, options);
        const double bound = std::max(std::abs(cone.ra), std::abs(cone.rb)) *
                             std::hypot(length(cone.u), length(cone.v)) *
                             (2 * std::pow(std::sin(pi / (2 * double(n))), 2));
        out.derived.report.update(
            {{"source_kind", "DgnCone"},
             {"scope", "derived_ruled_cone_faces"},
             {"native_tessellation_equivalence", "not_established"},
             {"circle_segments", segments},
             {"source_chord_error_bound", std::isfinite(bound) ? Json(bound) : Json()},
             {"sampling_rule", "uniform_source_angular_parameters"}});
        if (out.derived.status != "meshed")
            out.face_indices.clear();
    } catch (const std::exception &e) {
        out.derived = {};
        out.derived.report["reason"] = e.what();
        out.face_indices.clear();
    }
    return out;
}
} // namespace p3d
