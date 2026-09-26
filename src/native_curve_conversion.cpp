#include "native_curve_conversion.hpp"
#include "native_pcurve_points.hpp"

namespace p3d::curve_detail {
namespace {
double number(const Json &j) {
    require(j.is_number(), "native boundary coordinate is not numeric");
    const double v = j.get<double>();
    require(std::isfinite(v), "native boundary coordinate is not finite");
    return v;
}
Point3 point(const Json &j, const std::string &prefix) {
    return {number(j.at(prefix + "X")), number(j.at(prefix + "Y")), number(j.at(prefix + "Z"))};
}
double magnitude(const Point3 &p) {
    const double length = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    require(std::isfinite(length), "native boundary axis length overflow");
    return length;
}
} // namespace
bool normalized_domain(double low, double high) {
    return std::abs(low) <= 1e-14 && std::abs(high - 1) <= 1e-14;
}
void fraction_knots(std::vector<double> &knots, double low, double high) {
    for (auto &u : knots) {
        u = (u - low) / (high - low);
        require(std::isfinite(u), "native knot normalization nonfinite result");
        if (std::abs(u) < 1e-12)
            u = 0;
        else if (std::abs(u - 1) < 1e-12)
            u = 1;
    }
}
std::optional<std::array<Point3, 2>> primitive_endpoints(const Json &v, const BsplineCurve *c) {
    const auto type = v.at("_type").get<std::string>();
    std::array<Point3, 2> out{};
    if (type == "LineSegment") {
        out = {point(v.at("segment"), "point0"), point(v.at("segment"), "point1")};
    } else if (type == "LineString" || type == "PointString") {
        const auto &p = v.at("points");
        require(p.is_array() && p.size() % 3 == 0, "native source endpoint XYZ layout");
        if (p.empty())
            return {};
        for (unsigned k = 0; k < 3; ++k) {
            out[0][k] = number(p[k]);
            out[1][k] = number(p[p.size() - 3 + k]);
        }
    } else if (type == "EllipticArc") {
        const auto &a = v.at("arc");
        const auto origin = point(a, "center"), x = point(a, "vector0"), y = point(a, "vector90");
        const double start = number(a.at("startRadians")),
                     end = start + number(a.at("sweepRadians"));
        require(std::isfinite(end), "native source ellipse endpoint angle overflow");
        for (unsigned k = 0; k < 3; ++k) {
            out[0][k] = (origin[k] + x[k] * std::cos(start)) + y[k] * std::sin(start);
            out[1][k] = (origin[k] + x[k] * std::cos(end)) + y[k] * std::sin(end);
        }
    } else if (c) {
        out = {detail::pcurve_point(*c, 0).point, detail::pcurve_point(*c, 1).point};
    } else
        return {};
    for (auto p : out)
        for (double x : p)
            require(std::isfinite(x), "native source endpoint nonfinite point");
    return out;
}
bool endpoint_pair_closed(const Point3 &a, const Point3 &b) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    const double distance = (dy * dy + dx * dx) + dz * dz;
    double scale = a[0] * a[0] + a[1] * a[1];
    scale += a[2] * a[2];
    scale += b[0] * b[0];
    scale += b[1] * b[1];
    scale += b[2] * b[2];
    scale += 1;
    require(std::isfinite(distance) && std::isfinite(scale), "native source closure overflow");
    return distance < scale * 1.0000000000000001e-20;
}
BsplineCurve ellipse_to_bspline(const Json &value) {
    const auto &a = value.at("arc");
    const auto center = point(a, "center"), x = point(a, "vector0"), y = point(a, "vector90");
    const double lx = magnitude(x), ly = magnitude(y);
    constexpr double pi = 3.141592653589793;
    auto angle = [&](double t) {
        const double s = lx * std::sin(t), c = ly * std::cos(t);
        if (std::abs(s) < 1e-5 || std::abs(c) < 1e-5)
            return t;
        const double adjusted = std::atan2(s, c);
        if (std::abs(t) < pi)
            return adjusted;
        return adjusted + std::copysign(std::ceil(std::abs(t) / pi) * pi, t);
    };
    const double start = angle(number(a.at("startRadians")));
    const double sweep = angle(number(a.at("sweepRadians")));
    require(std::isfinite(start) && std::isfinite(sweep) && std::isfinite(start + sweep),
            "native ellipse angle conversion overflow");
    const unsigned spans = std::abs(sweep) <= 2.0943951023931953   ? 1
                           : std::abs(sweep) <= 4.1887902047863905 ? 2
                                                                   : 3;
    const bool closed = std::abs(sweep) > 6.283185307178586;
    const double w = std::cos(std::abs(sweep) * .5 / spans), twice = 2 * w, blend = 2 * (1 + w);
    require(w != 0, "native ellipse control weight is zero");
    const double half_step = spans == 3 ? sweep / 6 : spans == 2 ? sweep * .25 : sweep * .5;
    std::vector<Point2> circle(2 * spans + 1);
    for (unsigned i = 0; i <= spans; ++i) {
        const double t = i == spans ? start + sweep : start + double(2 * i) * half_step;
        circle[2 * i] = {std::cos(t), std::sin(t)};
    }
    for (unsigned i = 0; i < spans; ++i) {
        const double t = start + double(2 * i + 1) * half_step;
        const Point2 mid{std::cos(t), std::sin(t)};
        for (unsigned k = 0; k < 2; ++k)
            circle[2 * i + 1][k] =
                (blend * mid[k] - (circle[2 * i][k] + circle[2 * i + 2][k])) / twice;
    }
    std::vector<double> weights, poles;
    for (unsigned i = 0; i < circle.size(); ++i) {
        const double weight = i % 2 ? w : 1;
        weights.push_back(weight);
        // setEllipticArc first weights the unit-circle controls; setbyGeEllipse3d
        // deweights those controls, applies the original basis, then reweights.
        const double u = (circle[i][0] * weight) / weight, v = (circle[i][1] * weight) / weight;
        for (unsigned k = 0; k < 3; ++k)
            poles.push_back(((center[k] + x[k] * u) + y[k] * v) * weight);
    }
    std::vector<double> inside;
    if (spans == 2)
        inside = {.5, .5};
    if (spans == 3)
        inside = closed ? std::vector<double>{0, 1. / 3, 1. / 3, 2. / 3, 2. / 3, 1}
                        : std::vector<double>{1. / 3, 1. / 3, 2. / 3, 2. / 3};
    std::vector<double> knots(weights.size() + (closed ? 5 : 3), 0);
    std::copy(inside.begin(), inside.end(), knots.begin() + 3);
    for (unsigned i = 0; i < 3; ++i) {
        if (closed) {
            knots[i] = knots[i + inside.size() + 1] - 1;
            knots[3 + inside.size() + i] = knots[2 + i] + 1;
        } else
            knots[3 + inside.size() + i] = 1;
    }
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", 3},
                                    {"closed", closed},
                                    {"poles", poles},
                                    {"weights", weights},
                                    {"knots", knots}});
}
} // namespace p3d::curve_detail
