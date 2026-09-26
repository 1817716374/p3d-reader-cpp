#pragma once
#include "internal.hpp"

namespace p3d::curve_detail {
// Native GeTransform identity predicate used by B-spline transforms.
inline bool bspline_identity(const Matrix4 &m) {
    bool identity = true;
    for (unsigned r = 0; r < 3; ++r) {
        identity &= m[r][3] > -1e-10 && m[r][3] < 1e-10;
        for (unsigned c = 0; c < 3; ++c)
            identity &= std::abs(m[r][c] - (r == c ? 1. : 0.)) <= 1e-12;
    }
    return identity;
}
inline Point3 affine_point(const Matrix4 &m, const Point3 &p, double weight) {
    Point3 result{};
    for (unsigned r = 0; r < 3; ++r) {
        result[r] = ((m[r][0] * p[0] + m[r][1] * p[1]) + m[r][2] * p[2]) + m[r][3] * weight;
        require(std::isfinite(result[r]), "native affine coordinate overflow");
    }
    return result;
}
// Preserve the source direction representation, including implicit knots and
// periodic indexing conventions. This never normalizes or opens the curve.
inline BsplineCurve with_poles(const BsplineCurve &source, const std::vector<Point3> &poles) {
    require(poles.size() == source.poles().size(), "native curve replacement control count");
    if (poles == source.poles())
        return source;
    Json flat = Json::array();
    for (const auto &p : poles)
        for (double x : p)
            flat.push_back(x);
    return BsplineCurve::from_bgfb(
        {{"_type", "BsplineCurve"},
         {"order", source.order()},
         {"closed", source.closed()},
         {"poles", std::move(flat)},
         {"weights", source.rational() ? Json(source.weights()) : Json()},
         {"knots", source.source_knots().empty() ? Json() : Json(source.source_knots())}});
}
} // namespace p3d::curve_detail
