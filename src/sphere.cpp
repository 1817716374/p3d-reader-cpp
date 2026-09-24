#include <p3d/solid.hpp>
#include "geometry.hpp"

namespace p3d {
namespace {
constexpr double pi = 3.141592653589793238462643383279502884;
constexpr double half_pi = pi / 2;
struct Sphere {
    Matrix4 frame = identity();
    double start, sweep, end;
    bool capped;
    explicit Sphere(const Json &table) {
        require(table.at("_type") == "DgnSphere", "expected BGFB DgnSphere");
        const auto &d = table.at("detail");
        const auto &t = d.at("localToWorld");
        for (unsigned r = 0; r < 3; ++r)
            for (unsigned c = 0; c < 4; ++c) {
                const std::string key = std::string("a") + "xyz"[r] + "xyzw"[c];
                frame[r][c] = t.at(key).get<double>();
                require(std::isfinite(frame[r][c]), "BGFB sphere nonfinite frame");
            }
        start = d.at("startLatitudeRadians").get<double>();
        sweep = d.at("latitudeSweepRadians").get<double>();
        end = start + sweep;
        capped = d.at("capped").get<bool>();
        require(std::isfinite(start) && std::isfinite(sweep) && std::isfinite(end),
                "BGFB sphere nonfinite or overflowing latitude");
    }
    bool cap(unsigned end_index) const {
        return capped && std::abs(end_index ? end : start) < half_pi - 1e-12;
    }
    Json info(bool force_north) const {
        auto a = std::clamp(start, -half_pi, half_pi);
        auto b = std::clamp(end, -half_pi, half_pi);
        if (force_north && a > b)
            std::swap(a, b);
        const auto za = std::sin(a), zb = std::sin(b);
        const bool partial_z = std::abs(zb - za) < 1.9999999999;
        return {{"status", "evaluated"},
                {"local_to_world", frame},
                {"source_latitudes", {start, end}},
                {"caps", {cap(0), cap(1)}},
                {"native_is_closed_solid", capped || !partial_z},
                {"sweep_info",
                 {{"latitudes", {a, b}},
                  {"z", {za, zb}},
                  {"force_sweep_north", force_north},
                  {"native_return_value", partial_z}}}};
    }
};
std::size_t product(std::size_t a, std::size_t b) {
    require(b == 0 || a <= std::numeric_limits<std::size_t>::max() / b,
            "BGFB sphere count overflow");
    return a * b;
}
std::size_t sum(std::size_t a, std::size_t b) {
    require(a <= std::numeric_limits<std::size_t>::max() - b, "BGFB sphere count overflow");
    return a + b;
}
} // namespace
Json native_bgfb_sphere_parameters(const Json &table, bool force_sweep_north) {
    try {
        return Sphere(table).info(force_sweep_north);
    } catch (const std::exception &e) {
        return {{"status", "not_evaluated"}, {"reason", e.what()}};
    }
}
SolidMeshResult mesh_bgfb_sphere(const Json &table, const PolyfaceMeshOptions &options,
                                 unsigned segments) {
    SolidMeshResult out;
    out.source = table;
    try {
        const Sphere sphere(table);
        require(segments >= 3, "BGFB sphere needs at least three longitude segments");
        require(sphere.start >= -half_pi && sphere.start <= half_pi && sphere.end >= -half_pi &&
                    sphere.end <= half_pi && sphere.start != sphere.end,
                "BGFB sphere mesh requires distinct latitudes within the polar interval");
        const auto det = determinant(sphere.frame);
        require(std::isfinite(det) && det != 0, "BGFB sphere degenerate or overflowing frame");
        const bool pole_a = std::abs(sphere.start) == half_pi;
        const bool pole_b = std::abs(sphere.end) == half_pi;
        const std::size_t poles = std::size_t(pole_a) + std::size_t(pole_b);
        const std::size_t n = segments;
        const std::size_t bands = std::max<std::size_t>(
            1, std::size_t(std::ceil(std::abs(sphere.sweep) / (2 * pi) * double(n))));
        require(bands >= poles, "BGFB sphere insufficient latitude bands");
        const std::size_t caps = std::size_t(sphere.cap(0)) + std::size_t(sphere.cap(1));
        const auto points_count = sum(product(sum(bands, 1) - poles, n), poles);
        const auto corners_count =
            sum(product(n, sum(product(bands - poles, 5), product(poles, 4))),
                product(caps, sum(n, 1)));
        const auto triangles_count =
            sum(product(n, product(bands, 2) - poles), product(caps, n - 2));
        require(points_count <= options.max_points, "BGFB sphere point budget");
        require(corners_count <= options.max_corners, "BGFB sphere corner budget");
        require(triangles_count <= options.max_triangles, "BGFB sphere triangle budget");
        const bool reverse = (det < 0) != (sphere.sweep < 0);
        Json points = Json::array(), indices = Json::array();
        std::vector<std::size_t> starts, counts;
        for (std::size_t row = 0; row <= bands; ++row) {
            const double latitude =
                row == 0       ? sphere.start
                : row == bands ? sphere.end
                               : sphere.start + sphere.sweep * (double(row) / double(bands));
            const bool pole = (row == 0 && pole_a) || (row == bands && pole_b);
            starts.push_back(points.size() / 3);
            counts.push_back(pole ? 1 : n);
            for (std::size_t i = 0; i < counts.back(); ++i) {
                const double longitude = 2 * pi * (double(i) / double(n));
                const double radius = pole ? 0 : std::cos(latitude);
                const Point3 local{radius * std::cos(longitude), radius * std::sin(longitude),
                                   std::sin(latitude)};
                const auto p = transform(sphere.frame, local);
                for (double x : p) {
                    require(std::isfinite(x), "BGFB sphere sampled position overflow");
                    points.push_back(x);
                }
            }
        }
        auto face = [&](std::vector<std::size_t> corners, std::array<std::int64_t, 3> id) {
            if (reverse)
                std::reverse(corners.begin(), corners.end());
            for (auto i : corners)
                indices.push_back(i + 1);
            indices.push_back(0);
            out.face_indices.push_back(id);
        };
        if (sphere.cap(0)) {
            std::vector<std::size_t> corners;
            for (std::size_t i = n; i > 0; --i)
                corners.push_back(i - 1);
            face(std::move(corners), {-1, 0, 0});
        }
        if (sphere.cap(1)) {
            std::vector<std::size_t> corners;
            for (std::size_t i = 0; i < n; ++i)
                corners.push_back(starts.back() + i);
            face(std::move(corners), {-1, 1, 0});
        }
        for (std::size_t row = 0; row < bands; ++row)
            for (std::size_t i = 0; i < n; ++i) {
                const auto j = (i + 1) % n, a = starts[row], b = starts[row + 1];
                if (counts[row] == 1)
                    face({a, b + j, b + i}, {0, 0, 0});
                else if (counts[row + 1] == 1)
                    face({a + i, a + j, b}, {0, 0, 0});
                else
                    face({a + i, a + j, b + j, b + i}, {0, 0, 0});
            }
        const Json polyface = {{"_type", "Polyface"},
                               {"meshStyle", 1},
                               {"numPerFace", 0},
                               {"point", std::move(points)},
                               {"pointIndex", std::move(indices)}};
        out.derived = mesh_bgfb_polyface(polyface, options);
        double norm = 0;
        for (unsigned r = 0; r < 3; ++r)
            for (unsigned c = 0; c < 3; ++c)
                norm = std::hypot(norm, sphere.frame[r][c]);
        const double step = 2 * pi / double(n) + std::abs(sphere.sweep) / double(bands);
        const double bound = norm * (step * step / 2);
        out.derived.report.update({{"source_kind", "DgnSphere"},
                                   {"scope", "derived_latitude_sphere_faces"},
                                   {"native_tessellation_equivalence", "not_established"},
                                   {"circle_segments", segments},
                                   {"latitude_bands", bands},
                                   {"sampling_rule", "uniform_source_longitude_and_latitude"},
                                   {"native_parameters", sphere.info(false)},
                                   {"source_parameter_interpolation_error_bound",
                                    std::isfinite(bound) ? Json(bound) : Json()}});
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
