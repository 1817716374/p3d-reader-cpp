// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Adapted from bspmisc.cpp (see THIRD_PARTY.md). Immutable surfaces, bounded
// storage and reports replace native allocations. P3D arithmetic is retained.
#include "native_tube.hpp"

namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    require(b.work <= b.max_work && n <= b.max_work - b.work,
            "native surface elevation work budget exceeded");
    b.work += n;
}
double finite(double x) {
    require(std::isfinite(x), "native surface elevation nonfinite arithmetic");
    return x;
}
BsplineCurve column(const BsplineSurface &s, std::size_t u) {
    Json poles = Json::array(), weights = Json::array();
    for (std::size_t v = 0; v < s.v().pole_count(); ++v) {
        const auto i = v * s.u().pole_count() + u;
        for (double x : s.poles()[i])
            poles.push_back(x);
        if (s.rational())
            weights.push_back(s.weights()[i]);
    }
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", s.v().order()},
                                    {"closed", s.v().closed()},
                                    {"knots", s.v().knots()},
                                    {"poles", std::move(poles)},
                                    {"weights", s.rational() ? std::move(weights) : Json()}});
}
struct Plan {
    std::vector<double> knots, matrix;
    double tolerance;
    std::size_t count;
};
Plan plan(const BsplineCurve &c, unsigned degree, std::size_t columns, TubeBudget &b) {
    auto working = c.poles();
    Plan p;
    p.tolerance = native_bspline_knot_tolerance(c, working);
    // This private tolerance-query copy is discarded. In particular, its
    // weight round trips must not feed the first column's derivative queries.
    const auto domain = c.knot_domain();
    const auto &knots = c.knots();
    struct Group {
        double value;
        std::size_t count;
    };
    std::vector<Group> groups;
    std::size_t filled = 0;
    for (std::size_t i = 0; i < knots.size(); ++i) {
        if (knots[i] < domain[0])
            continue;
        if (knots[i] > domain[1])
            break;
        if (groups.empty() || std::abs(knots[i] - knots[i - 1]) > p.tolerance)
            groups.push_back({knots[i], 1});
        else
            ++groups.back().count;
        ++filled;
    }
    require(filled == knots.size() && !groups.empty(),
            "native surface elevation has unwritten exterior knot storage");
    groups.back().value = domain[1];
    const unsigned order = degree + 1, delta = order - c.order();
    require(knots.size() <= INT32_MAX &&
                groups.size() <= (std::size_t(INT32_MAX) - knots.size()) / delta,
            "native surface elevation knot count overflow");
    const auto nk = knots.size() + groups.size() * delta;
    require(nk >= 2 * order && nk - order <= b.max_control_points / columns,
            "native surface elevation output control budget exceeded");
    p.count = nk - order;
    charge(b, nk);
    charge(b, (p.count - 1) * order);
    p.knots.reserve(nk);
    for (const auto &g : groups)
        p.knots.insert(p.knots.end(), g.count + delta, g.value);
    p.matrix.resize((p.count - 1) * order);
    // One symmetric block at a time; the coefficient matrix is shared by all
    // columns. Unlike curve elevation, the knot domain is never normalized.
    for (std::size_t i = 0; i < p.count; ++i) {
        charge(b, order * order * order);
        std::array<double, 26 * 26> symmetric{};
        for (unsigned j = 0; j <= degree; ++j)
            symmetric[j * order] = 1;
        for (unsigned j = 1; j <= degree; ++j)
            for (unsigned k = 1; k <= j; ++k)
                symmetric[j * order + k] = finite((p.knots[i + j - 1] - p.knots[i + degree]) *
                                                      symmetric[(j - 1) * order + k - 1] +
                                                  symmetric[(j - 1) * order + k]);
        if (!i)
            continue;
        for (unsigned j = 0; j <= degree; ++j) {
            auto &a = p.matrix[(i - 1) * order + j];
            a = (j % 2 ? -1. : 1.) * symmetric[degree * order + degree - j];
            for (unsigned k = j + 1; k <= degree; ++k)
                a /= k;
        }
    }
    return p;
}
} // namespace
TubeAssembly elevate_tube_surface_v(const BsplineSurface &s, unsigned degree, TubeBudget &b) {
    require(s.boundaries().is_null(), "native surface elevation requires an untrimmed surface");
    require(degree <= 25 && degree + 1 >= s.v().order(), "native surface elevation degree/state");
    const auto nu = s.u().pole_count(), nv = s.v().pole_count();
    require(s.poles().size() <= b.max_control_points && nu <= INT32_MAX && nv <= INT32_MAX,
            "native surface elevation source control budget exceeded");
    for (unsigned pass = 0; pass < 8; ++pass)
        charge(b, s.poles().size());
    charge(b, s.u().knots().size());
    charge(b, s.v().knots().size());
    Json out{{"_type", "BsplineSurface"},
             {"numPolesU", nu},
             {"numPolesV", nv},
             {"orderU", s.u().order()},
             {"orderV", s.v().order()},
             {"closedU", s.u().closed()},
             {"closedV", s.v().closed()},
             {"knotsU", s.u().knots()},
             {"knotsV", s.v().knots()},
             {"numRulesU", s.num_rules_u()},
             {"numRulesV", s.num_rules_v()},
             {"holeOrigin", s.hole_origin()},
             {"boundaries", nullptr}};
    Json report{{"scope", "native_surface_v_elevation"},
                {"source_degree", s.v().order() - 1},
                {"degree", degree},
                {"source_geometry_reused", false}};
    auto poles = s.poles();
    auto weights = s.weights();
    if (degree + 1 == s.v().order()) {
        report["method"] = "same_degree_copy";
    } else {
        require(!s.v().closed(), "native surface elevation periodic V is not implemented");
        const auto p = plan(column(s, 0), degree, nu, b);
        const unsigned order = degree + 1;
        poles.assign(nu * p.count, Point3{});
        weights.assign(s.rational() ? nu * p.count : 0, 0.);
        for (std::size_t u = 0; u < nu; ++u) {
            charge(b, nv);
            const auto c = column(s, u);
            auto working = c.poles();
            for (std::size_t i = 0; i < p.count; ++i) {
                for (unsigned pass = 0; pass < 8; ++pass)
                    charge(b, nv);
                charge(b, 16 * order * order * order);
                auto evaluation = native_bspline_evaluate_working(c, p.knots[i + degree], degree,
                                                                  true, std::move(working));
                working = std::move(evaluation.working_poles);
                if (!i)
                    continue;
                const auto index = (i - 1) * nu + u;
                for (unsigned j = 0; j <= degree; ++j) {
                    const unsigned q = degree - j;
                    const double factor = (q % 2 ? -1. : 1.) * p.matrix[(i - 1) * order + q];
                    for (unsigned axis = 0; axis < 3; ++axis)
                        poles[index][axis] =
                            finite(poles[index][axis] + factor * evaluation.homogeneous[j][axis]);
                    if (s.rational())
                        weights[index] =
                            finite(weights[index] + factor * evaluation.homogeneous[j][3]);
                }
            }
            // Native derivative queries carry working controls even at i=0.
            // The final control comes from that state; the first is restored
            // from the column saved before any query.
            poles[(p.count - 1) * nu + u] = working.back();
            poles[u] = c.poles().front();
            if (s.rational()) {
                weights[(p.count - 1) * nu + u] = c.weights().back();
                weights[u] = c.weights().front();
            }
        }
        out["numPolesV"] = p.count;
        out["orderV"] = order;
        out["knotsV"] = p.knots;
        report["method"] = "shared_first_column_plan";
        report["knot_tolerance"] = p.tolerance;
        report["knot_plan_source_column"] = 0;
        report["normalized_source_knots"] = false;
    }
    Json flat = Json::array();
    for (const auto &p : poles)
        for (double x : p)
            flat.push_back(x);
    out["poles"] = std::move(flat);
    out["weights"] = s.rational() ? Json(std::move(weights)) : Json();
    report["work_used"] = b.work;
    return {std::move(out), std::move(report)};
}
} // namespace p3d::swept_detail
