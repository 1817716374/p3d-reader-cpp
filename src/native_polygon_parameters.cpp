#include "native_polygon_parameters.hpp"
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"

namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
using Range = std::array<Point2, 2>;
Matrix4 identity() {
    Matrix4 out{};
    for (unsigned k = 0; k < 4; ++k)
        out[k][k] = 1;
    return out;
}
Range from_origin(Point2 diagonal) {
    Range out{};
    for (unsigned k = 0; k < 2; ++k) {
        if (0 > diagonal[k])
            out[0][k] = diagonal[k];
        if (diagonal[k] > 0)
            out[1][k] = diagonal[k];
    }
    return out;
}
bool null_range(const Range &range) {
    // Native GeRange2d tests magnitude, not ordering of the endpoints.
    for (const auto &p : range)
        for (double value : p)
            if (std::abs(value) >= 1e100)
                return true;
    return false;
}
} // namespace
NativePolygonParameterMapping
map_native_polygon_parameter_range(const Range &input, std::uint32_t mode, Point2 factors) {
    for (const auto &p : input)
        for (double value : p)
            finite(value);
    Point2 lengths;
    for (unsigned k = 0; k < 2; ++k)
        lengths[k] = finite(finite(input[1][k] - input[0][k]) * finite(factors[k]));
    NativePolygonParameterMapping out;
    out.transform = identity();
    out.distance_range = from_origin(lengths);
    if (mode == 0) {
        out.parameter_range = from_origin({1, 1});
    } else if (mode == 1) {
        Point2 diagonal{1, 1};
        if (lengths[1] > lengths[0])
            diagonal[0] = finite(lengths[0] / lengths[1]);
        else if (lengths[0] > lengths[1])
            diagonal[1] = finite(lengths[1] / lengths[0]);
        out.parameter_range = from_origin(diagonal);
    } else if (mode == 2) {
        out.parameter_range = out.distance_range;
    } else {
        out.parameter_range = input;
        return out;
    }
    if (null_range(input) || null_range(out.parameter_range))
        return out;
    Point2 scale, translation;
    for (unsigned k = 0; k < 2; ++k) {
        const double target = finite(out.parameter_range[1][k] - out.parameter_range[0][k]);
        const double source = finite(input[1][k] - input[0][k]);
        // Strict native safe-division test. A collapsed target may succeed;
        // a collapsed source does not, including the zero/zero case.
        if (!(std::abs(source) > std::abs(target) * 1e-15))
            return out;
        scale[k] = finite(target / source);
        translation[k] = finite(out.parameter_range[0][k] - finite(scale[k] * input[0][k]));
    }
    for (unsigned k = 0; k < 2; ++k) {
        out.transform[k][k] = scale[k];
        out.transform[k][3] = translation[k];
    }
    out.transform_applied = true;
    return out;
}
NativePolygonParameters prepare_native_polygon_parameters(const std::vector<Point3> &points,
                                                          const Matrix4 &world_to_local,
                                                          std::uint32_t mode, TubeBudget &budget) {
    require(points.size() <= budget.max_control_points && points.size() <= INT32_MAX,
            "native polygon parameter point extent");
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned k = 0; k < 4; ++k)
            finite(world_to_local[row][k]);
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(points.size());
    NativePolygonParameters out;
    out.projected.reserve(points.size());
    const double marker = std::numeric_limits<double>::max();
    std::size_t disconnected = 0;
    for (const auto &point : points) {
        work.charge(30);
        for (double value : point)
            finite(value);
        if (point[0] >= marker || point[1] >= marker || point[2] >= marker) {
            ++disconnected;
            out.projected.push_back(out.projected.empty() ? Point2{0, 0} : out.projected.back());
            continue;
        }
        Point3 local;
        for (unsigned k = 0; k < 3; ++k) {
            const auto &row = world_to_local[k];
            local[k] = finite(finite(finite(finite(point[1] * row[1]) + finite(point[0] * row[0])) +
                                     finite(point[2] * row[2])) +
                              row[3]);
        }
        out.projected.push_back({local[0], local[1]});
    }
    out.parameters = out.projected;
    if (out.parameters.size() == 1)
        out.parameters.front() = {0, 0};
    if (out.parameters.size() > 1) {
        Range source{{{marker, marker}, {-marker, -marker}}};
        for (const auto &point : out.projected) {
            work.charge(6);
            for (unsigned k = 0; k < 2; ++k) {
                if (source[0][k] > point[k])
                    source[0][k] = point[k];
                if (point[k] > source[1][k])
                    source[1][k] = point[k];
            }
        }
        work.charge(60);
        out.mapping = map_native_polygon_parameter_range(source, mode, {1, 1});
        out.face_distance_range = out.mapping->distance_range;
        if (out.mapping->transform_applied) {
            const auto &m = out.mapping->transform;
            for (auto &point : out.parameters) {
                work.charge(12);
                if (point[0] == marker || point[1] == marker)
                    continue;
                const auto p = point;
                point[0] =
                    finite(finite(finite(p[1] * m[0][1]) + finite(p[0] * m[0][0])) + m[0][3]);
                point[1] =
                    finite(finite(finite(p[0] * m[1][0]) + finite(p[1] * m[1][1])) + m[1][3]);
            }
        }
        out.complete = true;
    }
    out.report = {{"scope", "native_polygon_parameters"},
                  {"complete", out.complete},
                  {"mode", mode},
                  {"point_count", points.size()},
                  {"disconnect_count", disconnected},
                  {"transform_applied", out.mapping && out.mapping->transform_applied},
                  {"face_distance_range_written", out.face_distance_range.has_value()},
                  {"builder_indices_generated", false}};
    if (!out.complete)
        out.report["reason"] = "native_face_distance_range_unwritten_for_fewer_than_two_points";
    return out;
}
} // namespace p3d::swept_detail
