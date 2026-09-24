#include <p3d/solid.hpp>
#include "geometry.hpp"

namespace p3d {
namespace {
constexpr double tau = 6.283185307179586476925286766559005768;
constexpr double full_circle_threshold = 6.283185307178586;
double length(const Point3 &p) {
    return std::hypot(p[0], p[1], p[2]);
}
Point3 normalized(Point3 p) {
    const double scale = length(p);
    require(std::isfinite(scale) && scale > 0, "BGFB torus degenerate or overflowing direction");
    for (auto &x : p)
        x /= scale;
    return p;
}
Point3 cross(const Point3 &a, const Point3 &b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
struct Torus {
    Point3 center, u, v, x, y, normal;
    double major, minor, sweep, u_length;
    bool capped;
    explicit Torus(const Json &table) {
        require(table.at("_type") == "DgnTorusPipe", "expected BGFB DgnTorusPipe");
        const auto &d = table.at("detail");
        auto number = [&](const std::string &key) {
            const double a = d.at(key).get<double>();
            require(std::isfinite(a), "BGFB torus nonfinite parameter");
            return a;
        };
        auto point = [&](const char *prefix) {
            return Point3{number(std::string(prefix) + "X"), number(std::string(prefix) + "Y"),
                          number(std::string(prefix) + "Z")};
        };
        center = point("center");
        u = point("vectorX");
        v = point("vectorY");
        major = number("majorRadius");
        minor = number("minorRadius");
        sweep = number("sweepRadians");
        capped = d.at("capped").get<bool>();
        u_length = length(u);
        x = normalized(u);
        normal = normalized(cross(x, normalized(v)));
        y = normalized(cross(normal, x));
    }
    bool full() const {
        return std::abs(sweep) > full_circle_threshold;
    }
    bool caps() const {
        return capped && !full();
    }
    double effective_sweep() const {
        return std::copysign(std::min(std::abs(sweep), tau), sweep);
    }
    Matrix4 frame() const {
        auto f = identity();
        const double sign = sweep < 0 ? -1. : 1.;
        for (unsigned k = 0; k < 3; ++k) {
            f[k][0] = u[k];
            f[k][1] = u_length * y[k];
            f[k][2] = sign * normal[k];
            f[k][3] = center[k];
        }
        return f;
    }
    Json info() const {
        Json faces = Json::array();
        if (capped) {
            faces.push_back({-1, 0, 0});
            faces.push_back({-1, 1, 0});
        }
        faces.push_back({0, 0, 0});
        return {{"status", "evaluated"},
                {"native_full_circle", full()},
                {"has_caps", caps()},
                {"native_is_closed_solid", capped || full()},
                {"enumerated_face_indices", faces},
                {"source_sweep_radians", sweep},
                {"effective_sweep_radians", effective_sweep()},
                {"sweep_frame", frame()},
                {"axis_rule", "rotate_source_start_section_about_orthogonal_axis"}};
    }
};
std::size_t product(std::size_t a, std::size_t b) {
    require(b == 0 || a <= std::numeric_limits<std::size_t>::max() / b,
            "BGFB torus count overflow");
    return a * b;
}
std::size_t sum(std::size_t a, std::size_t b) {
    require(a <= std::numeric_limits<std::size_t>::max() - b, "BGFB torus count overflow");
    return a + b;
}
} // namespace
Json native_bgfb_torus_parameters(const Json &table) {
    try {
        return Torus(table).info();
    } catch (const std::exception &e) {
        return {{"status", "not_evaluated"}, {"reason", e.what()}};
    }
}
SolidTransformResult transform_bgfb_torus(const Json &table, const Matrix4 &matrix) {
    SolidTransformResult out;
    try {
        const Torus original(table);
        for (const auto &row : matrix)
            for (double value : row)
                require(std::isfinite(value), "BGFB torus nonfinite transform");
        require(matrix[3] == std::array<double, 4>{0, 0, 0, 1},
                "BGFB torus transform must be affine");
        const auto center = transform(matrix, original.center);
        auto u = vector_transform(matrix, original.u), v = vector_transform(matrix, original.v);
        const auto alpha = length(u), beta = length(v);
        u = normalized(u);
        v = normalized(v);
        auto changed = table;
        auto &d = changed["detail"];
        auto store_point = [&](const char *prefix, const Point3 &p) {
            for (unsigned k = 0; k < 3; ++k) {
                require(std::isfinite(p[k]), "BGFB torus transformed center overflow");
                d[std::string(prefix) + "XYZ"[k]] = p[k];
            }
        };
        store_point("center", center);
        store_point("vectorX", u);
        store_point("vectorY", v);
        const auto major = original.major * alpha, minor = original.minor * alpha;
        require(std::isfinite(major) && std::isfinite(minor),
                "BGFB torus transformed radius overflow");
        require((original.major == 0 || major != 0) && (original.minor == 0 || minor != 0),
                "BGFB torus transformed radius underflow");
        d["majorRadius"] = major;
        d["minorRadius"] = minor;
        const Torus next(changed);
        auto target = next.frame();
        auto inverse = identity();
        // The sweep frame has orthogonal columns, with |u| on the first two.
        for (unsigned k = 0; k < 3; ++k) {
            inverse[0][k] = original.x[k] / original.u_length;
            inverse[1][k] = original.y[k] / original.u_length;
            inverse[2][k] = (original.sweep < 0 ? -1. : 1.) * original.normal[k];
            for (unsigned c = 0; c < 3; ++c)
                target[k][c] *= alpha;
            target[k][3] = 0;
        }
        auto effective = multiply(target, inverse);
        const auto old_center = vector_transform(effective, original.center);
        for (unsigned k = 0; k < 3; ++k)
            effective[k][3] = center[k] - old_center[k];
        for (const auto &row : effective)
            for (double value : row)
                require(std::isfinite(value), "BGFB torus effective transform overflow");
        const double det = determinant(effective);
        require(std::isfinite(det) && det != 0, "BGFB torus singular effective transform");
        out.transformed = std::move(changed);
        out.geometry_transform = effective;
        out.report = {{"radius_scale", alpha},
                      {"second_direction_length", beta},
                      {"rule", "normalize_both_directions_scale_both_radii_by_first"}};
        out.status = "transformed";
    } catch (const std::exception &e) {
        out.transformed = nullptr;
        out.report["reason"] = e.what();
    }
    return out;
}
SolidMeshResult mesh_bgfb_torus(const Json &table, const PolyfaceMeshOptions &options,
                                unsigned segments) {
    SolidMeshResult out;
    out.source = table;
    try {
        const Torus torus(table);
        require(segments >= 3, "BGFB torus needs at least three circle segments");
        require(torus.minor != 0 && std::abs(torus.major) > std::abs(torus.minor),
                "BGFB torus requires a nonzero tube radius and a non-self-intersecting ring");
        require(torus.sweep != 0, "BGFB torus zero sweep");
        const auto frame = torus.frame();
        const double det = determinant(frame);
        require(std::isfinite(det) && det != 0, "BGFB torus degenerate or overflowing sweep frame");
        const double sweep = torus.effective_sweep();
        const std::size_t n = segments,
                          bands = std::max<std::size_t>(
                              1, std::size_t(std::ceil(std::abs(sweep) / tau * double(n))));
        // Only an exact or natively clamped full revolution shares seam vertices.
        // The native full-circle tolerance alone does not change the source angle.
        const bool joined = std::abs(sweep) == tau;
        const std::size_t rows = joined ? bands : sum(bands, 1), caps = torus.caps() ? 2 : 0;
        const auto quads = product(bands, n);
        require(product(rows, n) <= options.max_points, "BGFB torus point budget");
        require(sum(product(quads, 5), product(caps, sum(n, 1))) <= options.max_corners,
                "BGFB torus corner budget");
        require(sum(product(quads, 2), product(caps, n - 2)) <= options.max_triangles,
                "BGFB torus triangle budget");
        Json points = Json::array(), indices = Json::array();
        for (std::size_t row = 0; row < rows; ++row) {
            const double theta = sweep * (double(row) / double(bands));
            for (std::size_t i = 0; i < n; ++i) {
                const double phi = tau * (double(i) / double(n));
                const double radial = torus.major + torus.minor * std::cos(phi);
                const auto p = transform(frame, {radial * std::cos(theta), radial * std::sin(theta),
                                                 torus.minor * std::sin(phi)});
                for (double value : p) {
                    require(std::isfinite(value), "BGFB torus sampled position overflow");
                    points.push_back(value);
                }
            }
        }
        auto face = [&](std::vector<std::size_t> corners, std::array<std::int64_t, 3> id) {
            if (torus.major < 0)
                std::reverse(corners.begin(), corners.end());
            for (auto i : corners)
                indices.push_back(i + 1);
            indices.push_back(0);
            out.face_indices.push_back(id);
        };
        if (caps) {
            std::vector<std::size_t> first, last;
            for (std::size_t i = 0; i < n; ++i)
                first.push_back(i);
            for (std::size_t i = n; i > 0; --i)
                last.push_back((rows - 1) * n + i - 1);
            face(std::move(first), {-1, 0, 0});
            face(std::move(last), {-1, 1, 0});
        }
        for (std::size_t row = 0; row < bands; ++row)
            for (std::size_t i = 0; i < n; ++i) {
                const auto j = (i + 1) % n, a = row * n, b = ((row + 1) % rows) * n;
                face({a + i, b + i, b + j, a + j}, {0, 0, 0});
            }
        const Json polyface = {{"_type", "Polyface"},
                               {"meshStyle", 1},
                               {"numPerFace", 0},
                               {"point", std::move(points)},
                               {"pointIndex", std::move(indices)}};
        out.derived = mesh_bgfb_polyface(polyface, options);
        out.derived.report.update({{"source_kind", "DgnTorusPipe"},
                                   {"scope", "derived_rotated_torus_section"},
                                   {"native_tessellation_equivalence", "not_established"},
                                   {"circle_segments", segments},
                                   {"sweep_bands", bands},
                                   {"seam_joined", joined},
                                   {"sampling_rule", "uniform_source_section_and_rotational_sweep"},
                                   {"native_parameters", torus.info()}});
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
