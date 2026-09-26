// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from imodel-native bspdsurf.cpp tube-frame/patch/assembly algorithms,
// bsputil.cpp line intersection and bspcurv.cpp contiguous curve combination.
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

namespace {
Point3 difference(const Point3 &a, const Point3 &b) {
    return {finite(a[0] - b[0]), finite(a[1] - b[1]), finite(a[2] - b[2])};
}
// Returns closest points on the infinite control lines. Native segment status
// codes do not affect the assembly callback; there is no clipping or skew-gap
// tolerance check. On a parallel/degenerate pair the endpoints stay unchanged.
bool intersect_lines(const Point3 &p0, Point3 &p1, Point3 &p2, const Point3 &p3) {
    const auto aa = difference(p1, p0), bb = difference(p3, p2), cc = difference(p2, p0);
    const double a = dot(aa, cc), b = dot(aa, aa), c = dot(aa, bb), d = dot(bb, cc),
                 f = dot(bb, bb), denominator = finite(b * f - c * c);
    if (!(std::abs(denominator) > 1e-12))
        return false;
    const double u = finite((a * f - c * d) / denominator),
                 v = finite((c * a - b * d) / denominator);
    Point3 q0{}, q1{};
    for (unsigned k = 0; k < 3; ++k) {
        q0[k] = finite(p0[k] + aa[k] * u);
        q1[k] = finite(p2[k] + bb[k] * v);
    }
    p1 = q0;
    p2 = q1;
    return true;
}
bool same_point(const Point3 &a, const Point3 &b, double tolerance) {
    for (unsigned k = 0; k < 3; ++k)
        if (std::abs(finite(a[k] - b[k])) > tolerance)
            return false;
    return true;
}
double first_rows_range(const BsplineSurface &s) {
    Point3 low{}, high{};
    low.fill(std::numeric_limits<double>::max());
    high.fill(-std::numeric_limits<double>::max());
    for (std::size_t i = 0; i < 2 * s.u().pole_count(); ++i) {
        if (s.rational() && std::abs(s.weights()[i]) <= 1e-12)
            continue; // Native range helper's weight cutoff, not unWeightPoles.
        const auto p = s.rational() ? unweight(s.poles()[i], s.weights()[i]) : s.poles()[i];
        for (unsigned k = 0; k < 3; ++k) {
            low[k] = std::min(low[k], p[k]);
            high[k] = std::max(high[k], p[k]);
        }
    }
    double extent = 0;
    for (unsigned k = 0; k < 3; ++k)
        extent = std::max(extent, std::abs(finite(high[k] - low[k])));
    return extent;
}
void change_weights(std::vector<Point3> &poles, const std::vector<double> &weights,
                    std::size_t begin, std::size_t end, bool divide) {
    if (weights.empty())
        return;
    for (std::size_t i = begin; i < end; ++i) {
        const double factor = divide ? finite(1 / weights[i]) : weights[i];
        for (auto &x : poles[i])
            x = finite(x * factor);
    }
}
Json join_rows(std::vector<Point3> &a, std::size_t a0, std::size_t a1, std::vector<Point3> &b,
               std::size_t b0, std::size_t b1, std::size_t nu, double tolerance, bool average) {
    std::size_t equal = 0, intersections = 0, parallel = 0;
    for (std::size_t j = 0; j < nu; ++j) {
        auto &p1 = a[a1 + j], &p2 = b[b0 + j];
        if (same_point(p1, p2, tolerance)) {
            ++equal;
        } else if (intersect_lines(a[a0 + j], p1, p2, b[b1 + j])) {
            ++intersections;
            if (average) {
                for (unsigned k = 0; k < 3; ++k)
                    p1[k] = finite(p1[k] + .5 * (p2[k] - p1[k]));
                p2 = p1;
            }
        } else {
            ++parallel;
        }
    }
    return {{"within_tolerance", equal},
            {"line_intersections", intersections},
            {"parallel_or_degenerate", parallel}};
}
} // namespace

TubeAssembly append_tube_patch(const BsplineSurface &assembled, const BsplineSurface &patch,
                               std::size_t prior_segments, bool close_trace, TubeBudget &budget) {
    const auto nu = assembled.u().pole_count(), na = assembled.v().pole_count(),
               nb = patch.v().pole_count();
    const auto order = assembled.v().order();
    require(!assembled.v().closed() && !patch.v().closed() && order <= 26 &&
                patch.v().order() == order && nb == order && assembled.u().order() <= 26 &&
                patch.u().order() == assembled.u().order() && patch.u().pole_count() == nu &&
                patch.u().closed() == assembled.u().closed() &&
                patch.u().knots() == assembled.u().knots() &&
                patch.rational() == assembled.rational() && assembled.boundaries().is_null() &&
                patch.boundaries().is_null() && assembled.hole_origin() == 0 &&
                patch.hole_origin() == 0,
            "native tube assembly requires matching untrimmed tube patches");
    require(prior_segments > 0 && prior_segments <= INT32_MAX &&
                prior_segments <= (std::numeric_limits<std::size_t>::max() - 1) / (nb - 1) &&
                na == prior_segments * (nb - 1) + 1,
            "native tube assembly segment count does not match prior grid");
    const auto &ka = assembled.v().knots(), &kb = patch.v().knots();
    for (std::size_t k = 0; k < kb.size(); ++k)
        require(kb[k] == (k < nb ? 0. : 1.), "native tube patch must be normalized Bezier V");
    for (std::size_t k = 0; k < order; ++k)
        require(ka[k] == 0 && ka[na + k] == 1,
                "native tube assembly requires normalized clamped V knots");
    require(na <= std::numeric_limits<std::size_t>::max() - nb + 1,
            "native tube assembly row count overflow");
    const auto nv = na + nb - 1;
    require(nv <= INT32_MAX && nu <= INT32_MAX && nu <= budget.max_control_points / nv,
            "native tube assembly control grid budget exceeded");
    const auto count = nu * nv;
    require(count <= std::numeric_limits<std::size_t>::max() / 32,
            "native tube assembly work size overflow");
    const auto work = 5 * count + 16 * nu;
    require(budget.work <= budget.max_work && work <= budget.max_work - budget.work,
            "native tube assembly work budget exceeded");
    budget.work += work;
    // Tolerance is determined from the FIRST two rows of the previous surface,
    // not the rows at this join and not the extent of the whole surface.
    const double intersection_tolerance = finite(first_rows_range(assembled) * 1e-4),
                 point_tolerance = finite(intersection_tolerance * .25);
    auto a = assembled.poles(), b = patch.poles();
    auto weights = assembled.weights();
    const auto a0 = (na - 2) * nu, a1 = (na - 1) * nu;
    change_weights(b, patch.weights(), 0, 2 * nu, true);
    change_weights(a, weights, a0, na * nu, true);
    auto join = join_rows(a, a0, a1, b, 0, nu, nu, point_tolerance, false);
    change_weights(b, patch.weights(), 0, 2 * nu, false);
    change_weights(a, weights, a0, na * nu, false);
    // Force-contiguous combine keeps the left join row and drops the right
    // first row, including its weights. It does not average/rescale weights.
    a.insert(a.end(), b.begin() + nu, b.end());
    if (assembled.rational())
        weights.insert(weights.end(), patch.weights().begin() + nu, patch.weights().end());
    std::vector<double> knots;
    knots.reserve(nv + order);
    for (std::size_t k = 0; k < ka.size() - 1; ++k)
        knots.push_back(finite(ka[k] * double(prior_segments)));
    const double end = finite(ka[na] * double(prior_segments));
    for (std::size_t k = order; k < kb.size(); ++k)
        knots.push_back(finite(end + (kb[k] - kb[order - 1])));
    const double start = knots[order - 1], span = finite(knots[nv] - start);
    require(span > 0, "native tube assembly empty knot span");
    for (auto &k : knots)
        k = finite((k - start) / span);
    std::fill(knots.begin() + nv, knots.end(), 1.);
    Json closure = nullptr;
    if (close_trace) {
        change_weights(a, weights, 0, count, true);
        closure = join_rows(a, (nv - 2) * nu, (nv - 1) * nu, a, 0, nu, nu, point_tolerance, true);
        change_weights(a, weights, 0, count, false);
    }
    Json xyz = Json::array();
    xyz.get_ref<Json::array_t &>().reserve(count * 3);
    for (auto &p : a)
        for (auto x : p)
            xyz.push_back(x);
    Json surface{{"_type", "BsplineSurface"},
                 {"orderU", assembled.u().order()},
                 {"orderV", order},
                 {"closedU", assembled.u().closed()},
                 {"closedV", false},
                 {"numPolesU", nu},
                 {"numPolesV", nv},
                 {"knotsU", assembled.u().knots()},
                 {"knotsV", knots},
                 {"poles", std::move(xyz)},
                 {"weights", assembled.rational() ? Json(weights) : Json()},
                 {"numRulesU", nv},
                 {"numRulesV", nu},
                 {"holeOrigin", 0},
                 {"boundaries", nullptr}};
    BsplineSurface::from_bgfb(surface);
    return {std::move(surface),
            {{"scope", "native_tube_surface_assembly"},
             {"prior_segments", prior_segments},
             {"intersection_tolerance", intersection_tolerance},
             {"point_tolerance", point_tolerance},
             {"join", std::move(join)},
             {"closure", std::move(closure)},
             {"control_points", count},
             {"work_used", budget.work},
             {"surface_validity", "not_certified"}}};
}
} // namespace p3d::swept_detail
