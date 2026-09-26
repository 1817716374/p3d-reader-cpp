// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from imodel-native bspdsurf.cpp tube-frame/patch algorithms.
// Changes: bounded C++/JSON storage; P3D preserves profile Z; explicit native
// tolerances and finite-arithmetic checks; immutable inputs and frame reports.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_tube.hpp"

namespace p3d::swept_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native tube nonfinite arithmetic");
    return x;
}
void check(const Matrix3 &m) {
    for (auto &row : m)
        for (double x : row)
            finite(x);
}
double dot(const Point3 &a, const Point3 &b) {
    return finite((a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]);
}
Point3 cross(const Point3 &a, const Point3 &b) {
    return {finite(a[1] * b[2] - a[2] * b[1]), finite(a[2] * b[0] - a[0] * b[2]),
            finite(a[0] * b[1] - a[1] * b[0])};
}
void normalize(Point3 &a) {
    const double length = finite(std::sqrt((a[0] * a[0] + a[1] * a[1]) + a[2] * a[2]));
    if (length > 0) {
        const double inverse = finite(1 / length);
        for (auto &x : a)
            x = finite(x * inverse);
    }
}
Point3 tangent_at(const BsplineCurve &curve, double t) {
    const auto f = curve.native_frame_at(t);
    const auto &m = f.at("frame");
    return {m.at(0).at(0), m.at(1).at(0), m.at(2).at(0)};
}
Point3 unweight(Point3 p, double w) {
    // Native unWeightPoles divides by every stored weight, including zero.
    // Reject nonfinite results instead of exposing a partly generated surface.
    const double inverse = finite(1 / w);
    for (auto &x : p)
        x = finite(x * inverse);
    return p;
}
} // namespace

Matrix3 advance_tube_frame(const Matrix3 &previous, Point3 tangent, bool rigid) {
    check(previous);
    for (double x : tangent)
        finite(x);
    if (dot(tangent, previous[2]) > .99999)
        return previous;
    if (rigid) {
        auto x = cross(previous[1], tangent);
        normalize(x);
        return {x, previous[1], tangent};
    }
    auto x = cross(previous[2], tangent);
    normalize(x);
    const double y_dot = dot(x, previous[1]), x_dot = dot(x, previous[0]);
    const double angle = x_dot == 0 && y_dot == 0 ? 0 : finite(std::atan2(y_dot, x_dot));
    auto y = cross(tangent, x);
    normalize(y);
    const double c = std::cos(angle), s = std::sin(angle);
    const Matrix3 twist{{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}}, pivot{x, y, tangent};
    Matrix3 result{};
    for (unsigned r = 0; r < 3; ++r)
        for (unsigned col = 0; col < 3; ++col)
            result[r][col] = finite((twist[r][0] * pivot[0][col] + twist[r][1] * pivot[1][col]) +
                                    twist[r][2] * pivot[2][col]);
    return result;
}

TubePatch tube_patch(const BsplineCurve &section, const BsplineCurve &segment, Matrix3 frame,
                     bool rigid, TubeBudget &budget) {
    const auto nu = section.poles().size(), nv = segment.poles().size();
    require(!segment.closed() && nv == segment.order() && segment.order() <= 26 &&
                section.order() <= 26,
            "native tube requires a Bezier segment and native curve orders");
    for (std::size_t i = 0; i < segment.knots().size(); ++i)
        require(segment.knots()[i] == (i < nv ? 0. : 1.),
                "native tube segment knots must be normalized Bezier knots");
    require(nv > 0 && nu <= budget.max_control_points / nv,
            "native tube control grid budget exceeded");
    require(budget.work <= budget.max_work, "native tube work budget invalid");
    const std::size_t count = nu * nv;
    require(count <= std::numeric_limits<std::size_t>::max() / 3 && nu <= INT32_MAX,
            "native tube control grid size overflow");
    // Curve evaluation touches its controls and computes derivatives; charge
    // a conservative bound as well as the generated tensor grid and input copy.
    const std::size_t evaluation_work = (nv + 1) * nv * nv * nv;
    require(count <= budget.max_work - budget.work, "native tube work budget exceeded");
    budget.work += count;
    require(evaluation_work <= budget.max_work - budget.work,
            "native tube frame work budget exceeded");
    budget.work += evaluation_work;
    check(frame);
    std::vector<Point3> profile;
    profile.reserve(nu);
    for (std::size_t j = 0; j < nu; ++j)
        profile.push_back(section.rational() ? unweight(section.poles()[j], section.weights()[j])
                                             : section.poles()[j]);
    // The P3D tubePatch preserves local profile Z, unlike the public Bentley
    // variant which flattens it. This matters for oblique sections.
    frame = advance_tube_frame(frame, tangent_at(segment, 0), rigid);
    const auto initial_frame = frame;
    Json poles = Json::array(), weights = Json::array(), nodes = Json::array(),
         frames = Json::array();
    auto &xyz = poles.get_ref<Json::array_t &>();
    xyz.reserve(count * 3);
    const bool rational = section.rational() || segment.rational();
    if (rational)
        weights.get_ref<Json::array_t &>().reserve(count);
    std::size_t compensated_rows = 0;
    for (std::size_t i = 0; i < nv; ++i) {
        // Native Greville summation for the callback's normalized Bezier knots.
        double node = 0;
        for (std::size_t k = 0; k < nv - 1; ++k)
            node += segment.knots()[i + 1 + k];
        node /= double(nv - 1);
        const auto tangent = tangent_at(segment, node);
        if (i)
            frame = advance_tube_frame(frame, tangent, rigid);
        nodes.push_back(node);
        frames.push_back(frame);
        const double v_weight = segment.rational() ? segment.weights()[i] : 1.;
        const auto origin = unweight(segment.poles()[i], v_weight);
        const double factor = finite(1 / v_weight);
        // Native comparison is strict tolerance > abs(factor - 1).
        const bool compensate = !(std::abs(factor - 1) < 1e-8);
        compensated_rows += compensate;
        for (std::size_t j = 0; j < nu; ++j) {
            auto p = profile[j];
            if (compensate)
                p[0] = finite(p[0] * factor);
            const double weight =
                finite((section.rational() ? section.weights()[j] : 1.) * v_weight);
            for (unsigned axis = 0; axis < 3; ++axis) {
                double x = finite(
                    ((frame[0][axis] * p[0] + frame[1][axis] * p[1]) + frame[2][axis] * p[2]) +
                    origin[axis]);
                if (rational)
                    x = finite(x * weight);
                poles.push_back(x);
            }
            if (rational)
                weights.push_back(weight);
        }
    }
    Json surface{{"_type", "BsplineSurface"},
                 {"orderU", section.order()},
                 {"orderV", segment.order()},
                 {"closedU", section.closed()},
                 {"closedV", false},
                 {"numPolesU", nu},
                 {"numPolesV", nv},
                 {"knotsU", section.knots()},
                 {"knotsV", segment.knots()},
                 {"poles", std::move(poles)},
                 {"weights", rational ? std::move(weights) : Json()},
                 {"numRulesU", nv},
                 {"numRulesV", nu},
                 {"holeOrigin", 0},
                 {"boundaries", nullptr}};
    // Validate storage, not watertightness, denominator regularity or closure.
    BsplineSurface::from_bgfb(surface);
    return {std::move(surface),
            frame,
            {{"scope", "native_bezier_tube_patch"},
             {"rigid_sweep", rigid},
             {"initial_frame_rows", initial_frame},
             {"greville_nodes", std::move(nodes)},
             {"frame_rows", std::move(frames)},
             {"weight_compensated_rows", compensated_rows},
             {"control_points", count},
             {"work_used", budget.work},
             {"profile_z_preserved", true},
             {"surface_validity", "not_certified"}}};
}
} // namespace p3d::swept_detail
