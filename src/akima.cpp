#include "internal.hpp"
namespace p3d {
namespace {
constexpr double disconnect = std::numeric_limits<double>::max();
double range_size(const std::vector<Point3> &points) {
    Point3 low{disconnect, disconnect, disconnect}, high{-disconnect, -disconnect, -disconnect};
    // The native scale is taken before filtering and excludes the two support
    // points at either end. Its range builder ignores a disconnect in any axis.
    for (std::size_t i = 2; i + 2 < points.size(); ++i) {
        const auto &p = points[i];
        if (std::find(p.begin(), p.end(), disconnect) != p.end())
            continue;
        for (unsigned k = 0; k < 3; ++k) {
            low[k] = std::min(low[k], p[k]);
            high[k] = std::max(high[k], p[k]);
        }
    }
    double scale = 0;
    for (unsigned k = 0; k < 3; ++k)
        scale = std::max(scale, std::abs(high[k] - low[k]));
    require(std::isfinite(scale), "Akima central point range is empty or unrepresentable");
    return scale;
}
double squared_distance(Point3 a, Point3 b) {
    const double x = b[0] - a[0], y = b[1] - a[1], z = b[2] - a[2];
    const double value = (y * y + x * x) + z * z;
    require(std::isfinite(value), "Akima point distance overflow");
    return value;
}
Point3 tangent(const std::vector<Point3> &directions, std::size_t point) {
    Point3 result{};
    for (unsigned k = 0; k < 3; ++k) {
        const double a = directions[point - 2][k], b = directions[point - 1][k],
                     c = directions[point][k], d = directions[point + 1][k];
        const double left = std::abs(b - a), right = std::abs(d - c), sum = left + right;
        result[k] = sum < 1e-5 ? 0.5 * (b + c) : (right * b + left * c) / sum;
    }
    return result;
}
} // namespace
AkimaCurve AkimaCurve::from_bgfb(const Json &table) {
    require(table.at("_type") == "AkimaCurve", "expected BGFB AkimaCurve table");
    const auto &values = table.at("points");
    require(values.is_array() && values.size() % 3 == 0, "Akima XYZ triplets");
    const auto count = values.size() / 3;
    require(count >= 6 && count <= 5000, "native Akima point count must be between 6 and 5000");
    std::vector<Point3> source;
    source.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        Point3 p{};
        for (unsigned k = 0; k < 3; ++k) {
            const auto &v = values[3 * i + k];
            require(v.is_number(), "Akima coordinate is not numeric");
            p[k] = v.get<double>();
            require(std::isfinite(p[k]), "Akima coordinate is not finite");
        }
        source.push_back(p);
    }
    const double scale = range_size(source), relative = scale * 1e-5;
    const double threshold = std::max(1e-28, relative * relative);
    require(std::isfinite(threshold), "Akima proximity threshold overflow");
    std::vector<std::size_t> indices;
    std::vector<Point3> points;
    Json ignored = Json::array();
    for (std::size_t i = 0; i < count; ++i) {
        if (source[i][0] == disconnect) {
            ignored.push_back({{"source_index", i}, {"reason", "native_disconnect_x"}});
            continue;
        }
        if (!points.empty() && squared_distance(points.back(), source[i]) < threshold) {
            ignored.push_back({{"source_index", i}, {"reason", "native_adjacent_proximity"}});
            continue;
        }
        points.push_back(source[i]);
        indices.push_back(i);
    }
    require(points.size() >= 6, "native Akima has fewer than six points after filtering");
    std::vector<Point3> directions;
    std::vector<double> lengths;
    for (std::size_t i = 1; i < points.size(); ++i) {
        const double length = std::sqrt(squared_distance(points[i - 1], points[i]));
        require(length > 0 && std::isfinite(length), "Akima zero or non-finite chord length");
        Point3 direction{};
        const double inverse_length = 1 / length;
        for (unsigned k = 0; k < 3; ++k)
            direction[k] = (points[i][k] - points[i - 1][k]) * inverse_length;
        directions.push_back(direction);
        lengths.push_back(length);
    }
    Json poles = Json::array(), knots = Json::array({0., 0., 0., 0.});
    auto append = [&](Point3 p) {
        for (double x : p) {
            require(std::isfinite(x), "Akima derived pole overflow");
            poles.push_back(x);
        }
    };
    append(points[2]);
    auto first_tangent = tangent(directions, 2);
    double total = 0;
    for (std::size_t i = 2; i + 3 < points.size(); ++i)
        total += lengths[i];
    require(total > 0 && std::isfinite(total), "Akima total interpolation chord length overflow");
    double cumulative = 0;
    for (std::size_t i = 2; i + 3 < points.size(); ++i) {
        const auto last_tangent = tangent(directions, i + 1);
        Point3 first{}, last{};
        for (unsigned k = 0; k < 3; ++k) {
            first[k] = points[i][k] + first_tangent[k] * (lengths[i] / 3);
            last[k] = points[i + 1][k] - last_tangent[k] * (lengths[i] / 3);
        }
        append(first);
        append(last);
        append(points[i + 1]);
        first_tangent = last_tangent;
        cumulative += lengths[i];
        if (i + 4 < points.size())
            for (unsigned k = 0; k < 3; ++k)
                knots.push_back(cumulative / total);
    }
    for (unsigned k = 0; k < 4; ++k)
        knots.push_back(1.);
    AkimaCurve result(BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                               {"order", 4},
                                               {"closed", false},
                                               {"poles", poles},
                                               {"weights", nullptr},
                                               {"knots", knots}}));
    result.source_points_ = std::move(source);
    result.retained_indices_ = std::move(indices);
    result.report_ = {
        {"status", "valid"},
        {"representation", "derived_open_cubic_bspline"},
        {"source_point_count", count},
        {"retained_point_indices", result.retained_indices_},
        {"ignored_points", ignored},
        {"central_range_size", scale},
        {"proximity_threshold_squared", threshold},
        {"first_interpolated_source_index", result.retained_indices_[2]},
        {"last_interpolated_source_index", result.retained_indices_[points.size() - 3]},
        {"support_points_each_end", 2},
        {"knot_spacing", "interpolation_chord_length"},
        {"derived_pole_count", result.bspline_.poles().size()}};
    return result;
}
} // namespace p3d
