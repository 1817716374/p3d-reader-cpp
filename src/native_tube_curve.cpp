#include "native_tube.hpp"
#include "native_curve_conversion.hpp"

namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t count) {
    require(b.work <= b.max_work && count <= b.max_work - b.work,
            "native swept primitive work budget exceeded");
    b.work += count;
}
double number(const Json &v) {
    require(v.is_number(), "native swept primitive coordinate is not numeric");
    const double x = v.get<double>();
    require(std::isfinite(x), "native swept primitive nonfinite coordinate");
    return x;
}
Point3 point(const Json &j, const char *prefix) {
    const std::string p = prefix;
    return {number(j.at(p + "X")), number(j.at(p + "Y")), number(j.at(p + "Z"))};
}
TubeCurve finish(BsplineCurve c, const std::string &type, unsigned native_type, Json conversion,
                 TubeBudget &budget) {
    require(c.poles().size() <= budget.max_control_points,
            "native swept converted primitive control budget exceeded");
    Json report{{"scope", "native_swept_primitive_conversion"},
                {"source_type", type},
                {"native_primitive_type", native_type},
                {"order", c.order()},
                {"closed", c.closed()},
                {"control_points", c.poles().size()},
                {"conversion", std::move(conversion)},
                {"work_used", budget.work},
                {"source_geometry_reused", false}};
    return {std::move(c), std::move(report)};
}
} // namespace

TubeCurve convert_tube_primitive(const Json &value, TubeBudget &budget) {
    require(value.is_object() && value.contains("_type") && value.at("_type").is_string(),
            "native swept primitive requires a decoded geometry table");
    const auto type = value.at("_type").get<std::string>();
    if (type == "LineSegment") {
        const auto &s = value.at("segment");
        auto converted = fit_tube_linestring({point(s, "point0"), point(s, "point1")}, budget);
        return finish(std::move(converted.curve), type, 1, {{"method", "line_from_endpoints"}},
                      budget);
    }
    if (type == "LineString") {
        const auto &flat = value.at("points");
        require(flat.is_array() && flat.size() % 3 == 0 &&
                    flat.size() / 3 <= budget.max_control_points &&
                    flat.size() / 3 <= INT32_MAX - 2,
                "native swept LineString point layout/budget");
        charge(budget, flat.size());
        std::vector<Point3> points;
        points.reserve(flat.size() / 3);
        for (std::size_t i = 0; i < flat.size(); i += 3)
            points.push_back({number(flat[i]), number(flat[i + 1]), number(flat[i + 2])});
        auto converted = fit_tube_linestring(points, budget);
        return finish(std::move(converted.curve), type, 2, std::move(converted.report), budget);
    }
    if (type == "EllipticArc") {
        // Native conversion has at most seven controls; validate the actual
        // result count after that bounded allocation, allowing a three-control
        // arc with max_control_points=3.
        charge(budget, 128);
        return finish(curve_detail::ellipse_to_bspline(value), type, 3,
                      {{"method", "native_ellipse_conversion"}}, budget);
    }
    if (type == "BsplineCurve") {
        const auto &poles = value.at("poles"), &weights = value.at("weights"),
                   &knots = value.at("knots");
        require(poles.is_array() && poles.size() % 3 == 0 &&
                    poles.size() / 3 <= budget.max_control_points && poles.size() / 3 <= INT32_MAX,
                "native swept B-spline control layout/budget");
        const auto count = poles.size() / 3;
        require((weights.is_null() || (weights.is_array() && weights.size() <= count)) &&
                    (knots.is_null() || (knots.is_array() && knots.size() <= 3 * count)),
                "native swept B-spline auxiliary array budget");
        for (unsigned i = 0; i < 8; ++i)
            charge(budget, count);
        return finish(BsplineCurve::from_bgfb(value), type, 4, {{"method", "native_bspline_copy"}},
                      budget);
    }
    throw std::runtime_error("unsupported native swept primitive: " + type);
}
} // namespace p3d::swept_detail
