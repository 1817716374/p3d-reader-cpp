// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Adapted from CurvePrimitive ranges and CurveVector::IsPlanar. Native P3D
// angle, transform, range and cancellation rules; see THIRD_PARTY.md.
#include "native_curve_planarity.hpp"
#include "native_curve_affine.hpp"
namespace p3d::curve_detail {
namespace {
constexpr double tau = 6.283185307179586, full = 6.283185307178586;
double finite(double x) {
    require(std::isfinite(x), "native primitive range nonfinite arithmetic");
    return x;
}
double number(const Json &j) {
    require(j.is_number(), "native primitive range expected number");
    return finite(j.get<double>());
}
Point3 point(const Json &j, const std::string &prefix) {
    return {number(j.at(prefix + "X")), number(j.at(prefix + "Y")), number(j.at(prefix + "Z"))};
}
bool disconnect(const Point3 &p) {
    return std::find(p.begin(), p.end(), std::numeric_limits<double>::max()) != p.end();
}
Point3 transform(const Matrix4 &m, const Point3 &p, bool vector) {
    if (disconnect(p))
        return vector
                   ? Point3{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                            std::numeric_limits<double>::max()}
                   : p;
    Point3 out;
    for (unsigned i = 0; i < 3; ++i) {
        out[i] = finite((m[i][1] * p[1] + m[i][0] * p[0]) + m[i][2] * p[2]);
        if (!vector)
            out[i] = finite(out[i] + m[i][3]);
    }
    return out;
}
void extend(NativeCurveRange &r, const Point3 &p) {
    if (!r.present) {
        r.low = r.high = p;
        r.present = true;
    } else
        for (unsigned i = 0; i < 3; ++i) {
            if (p[i] < r.low[i])
                r.low[i] = p[i];
            if (p[i] > r.high[i])
                r.high[i] = p[i];
        }
}
// CVTTSD2SI's out-of-range sentinel, avoiding undefined C++ conversions.
std::int32_t native_int(double x) {
    return x >= -2147483648. && x < 2147483648. ? static_cast<std::int32_t>(x) : INT32_MIN;
}
void check_primitive(const Json &j, std::size_t limit, BezierWork work) {
    const auto t = j.value("_type", "");
    require(t == "LineSegment" || t == "EllipticArc" || t == "BsplineCurve",
            "native primitive planarity unsupported source type");
    work.charge(128);
    if (t == "BsplineCurve") {
        const auto &p = j.at("poles");
        require(p.is_array() && p.size() % 3 == 0 && p.size() / 3 <= limit,
                "native primitive planarity source control budget");
        for (unsigned i = 0; i < 128; ++i)
            work.charge(p.size());
    }
}
} // namespace
bool native_range_angle_in_sweep(double angle, double start, double sweep) {
    finite(angle);
    finite(start);
    finite(sweep);
    double delta = finite(sweep > 0 ? angle - start : start - angle);
    const double length = finite((sweep > 0 ? sweep : -sweep) + 1e-12);
    if (delta >= -1e-12 && delta <= length)
        return true;
    if (delta > 0) {
        const double shifted = finite(delta - tau);
        if (shifted >= -1e-12 && shifted <= length)
            return true;
        delta = finite(delta - double(native_int(delta / tau)) * tau);
    } else {
        const double shifted = finite(delta + tau);
        if (shifted >= -1e-12 && shifted <= length)
            return true;
        const std::uint32_t raw = 1u - std::uint32_t(native_int(delta / tau));
        const auto signed_value =
            raw <= INT32_MAX ? std::int64_t(raw) : std::int64_t(raw) - 4294967296LL;
        delta = finite(double(signed_value) * tau + delta);
    }
    return length > delta || delta > full;
}
NativeCurveRange native_primitive_range(const Json &j, const Matrix4 &m, std::size_t limit,
                                        BezierWork work) {
    check_primitive(j, limit, work);
    for (unsigned i = 0; i < 3; ++i)
        for (double x : m[i])
            finite(x);
    const auto type = j.at("_type").get<std::string>();
    if (type == "BsplineCurve") {
        auto c = BsplineCurve::from_bgfb(j);
        if (!bspline_identity(m)) {
            auto poles = c.poles();
            for (std::size_t i = 0; i < poles.size(); ++i)
                poles[i] = c.rational() ? affine_point(m, poles[i], c.weights()[i])
                                        : affine_polynomial_point(m, poles[i]);
            c = with_poles(c, poles);
        }
        return native_curve_range(c, limit, work);
    }
    NativeCurveRange out;
    out.segments = 1;
    if (type == "LineSegment") {
        for (const auto &key : {"point0", "point1"}) {
            const auto p = point(j.at("segment"), key);
            if (!disconnect(p))
                extend(out, transform(m, p, false));
        }
        return out;
    }
    const auto &a = j.at("arc");
    const auto c = transform(m, point(a, "center"), false);
    const auto u = transform(m, point(a, "vector0"), true);
    const auto v = transform(m, point(a, "vector90"), true);
    const double start = number(a.at("startRadians")), sweep = number(a.at("sweepRadians"));
    if (std::abs(sweep) > full)
        extend(out, c);
    else {
        for (double angle : {start, finite(start + sweep)}) {
            const double cs = std::cos(angle), sn = std::sin(angle);
            Point3 p;
            for (unsigned i = 0; i < 3; ++i)
                p[i] = finite((u[i] * cs + c[i]) + v[i] * sn);
            extend(out, p);
        }
    }
    for (unsigned i = 0; i < 3; ++i) {
        const double mag = finite(std::sqrt(finite(u[i] * u[i] + v[i] * v[i])));
        if (!(mag > 0))
            continue;
        const double x = u[i] / mag, y = v[i] / mag;
        const double angle = x == 0 && y == 0 ? 0 : std::atan2(y, x);
        const double extent = finite(x * u[i] + y * v[i]);
        for (unsigned side = 0; side < 2; ++side)
            if (native_range_angle_in_sweep(angle + (side ? tau / 2 : 0), start, sweep)) {
                const double value = finite(side ? c[i] - extent : extent + c[i]);
                if (value < out.low[i])
                    out.low[i] = value;
                if (value > out.high[i])
                    out.high[i] = value;
                ++out.extrema_evaluations;
            }
    }
    return out;
}
NativeRangeZ native_range_z(const NativeCurveRange &range) {
    require(range.present, "native range Z requires a nonempty range");
    NativeRangeZ out;
    for (unsigned i = 0; i < 3; ++i) {
        finite(range.low[i]);
        finite(range.high[i]);
        require(range.high[i] >= range.low[i], "native range Z inverted bounds");
        out.scale = std::max(out.scale, std::max(std::abs(range.low[i]), std::abs(range.high[i])));
        out.scale = std::max(out.scale, std::abs(finite(range.high[i] - range.low[i])));
    }
    const double shifted = finite((range.high[2] - range.low[2]) + out.scale);
    out.rounded_span = std::abs(finite(shifted - out.scale));
    out.tolerance = finite((std::abs(out.scale) + 1. + std::abs(shifted)) * 1e-10);
    out.planar = out.tolerance >= out.rounded_span;
    return out;
}
Json native_primitive_planarity(const Json &source, std::size_t limit, BezierWork work) {
    check_primitive(source, limit, work);
    const Json group{{"_type", "CurveVector"},
                     {"type", 0},
                     {"curves", Json::array({Json{{"geometry", source}}})}};
    const auto frame = native_curve_frame(group, 0);
    require(frame.at("status") != "not_evaluated",
            "native primitive planarity frame not evaluated");
    Json out{{"status", "computed"}, {"planar", false}, {"frame_query", frame}};
    if (frame.at("status") != "computed") {
        out["reason"] = "frame_failed";
        return out;
    }
    const auto world = frame.at("frame").get<Matrix4>();
    Matrix3 linear;
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned k = 0; k < 3; ++k)
            linear[i][k] = world[i][k];
    const auto inverse = native_matrix_inverse(linear);
    out["frame_inverse_succeeded"] = inverse.inverted;
    if (!inverse.inverted) {
        out["reason"] = "inverse_failed";
        return out;
    }
    Matrix4 local{};
    local[3][3] = 1;
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned k = 0; k < 3; ++k)
            local[i][k] = inverse.matrix[i][k];
        local[i][3] = finite((local[i][1] * -world[1][3] + local[i][0] * -world[0][3]) +
                             local[i][2] * -world[2][3]);
    }
    out["world_to_local"] = local;
    const auto range = native_primitive_range(source, local, limit, work);
    if (!range.present) {
        out["reason"] = "empty_range";
        return out;
    }
    const auto z = native_range_z(range);
    out.update({{"local_range", {range.low, range.high}},
                {"range_scale", z.scale},
                {"rounded_z_span", z.rounded_span},
                {"z_tolerance", z.tolerance},
                {"planar", z.planar},
                {"work_used", work.used}});
    return out;
}
} // namespace p3d::curve_detail
