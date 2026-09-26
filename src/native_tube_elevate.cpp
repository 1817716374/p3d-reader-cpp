// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Adapted from bspcurv.cpp / bsputil.cpp (see THIRD_PARTY.md). Bounded local
// storage and reports replace native allocations; P3D arithmetic is retained.
#include "native_tube.hpp"
#include "loft_curve.hpp"

namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    require(b.work <= b.max_work && n <= b.max_work - b.work,
            "native elevation work budget exceeded");
    b.work += n;
}
double finite(double x) {
    require(std::isfinite(x), "native elevation nonfinite arithmetic");
    return x;
}
BsplineCurve table(unsigned order, const std::vector<Point3> &poles,
                   const std::vector<double> &weights, const std::vector<double> &knots) {
    Json flat = Json::array();
    for (auto p : poles)
        for (auto x : p)
            flat.push_back(x);
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"closed", false},
                                    {"poles", std::move(flat)},
                                    {"weights", weights.empty() ? Json(nullptr) : Json(weights)},
                                    {"knots", knots}});
}
bool normalized(double low, double high) {
    return std::abs(low) <= 1e-14 && std::abs(high - 1) <= 1e-14;
}
BsplineCurve opened(const BsplineCurve &c, TubeBudget &budget, Json &report) {
    require(c.poles().size() <= budget.max_control_points &&
                budget.max_control_points <= UINT32_MAX,
            "native combination opening control budget");
    for (unsigned i = 0; i < 8; ++i)
        charge(budget, c.poles().size());
    if (!c.closed()) {
        report = {{"method", "open_copy"}};
        return c;
    }
    // Opening performs at most order knot insertion passes over the controls.
    require(c.order() <= 26, "native combination opening order");
    for (unsigned i = 0; i < 8 * c.order(); ++i)
        charge(budget, c.poles().size());
    auto work =
        loft_detail::open_periodic_boundary(c, unsigned(budget.max_control_points), &report);
    return BsplineCurve::from_bgfb(work.table());
}
} // namespace

TubeCurve elevate_open_tube_curve(const BsplineCurve &source, unsigned degree, TubeBudget &budget) {
    require(!source.closed() && degree <= 25 && degree + 1 >= source.order(),
            "native open elevation degree/state");
    const auto count = source.poles().size();
    require(count <= budget.max_control_points && count <= INT32_MAX - source.order(),
            "native elevation source budget");
    for (unsigned i = 0; i < 8; ++i)
        charge(budget, count);
    Json report{{"scope", "native_open_curve_elevation"},
                {"source_degree", source.order() - 1},
                {"degree", degree},
                {"source_geometry_reused", false}};
    if (degree + 1 == source.order()) {
        report["method"] = "same_degree_copy";
        report["work_used"] = budget.work;
        return {source, std::move(report)};
    }
    report["method"] = "native_derivatives_and_symmetric_functions";
    auto knots = source.knots();
    const auto domain = source.knot_domain();
    const bool normalize = !normalized(domain[0], domain[1]);
    if (normalize)
        for (auto &u : knots) {
            u = finite((u - domain[0]) / (domain[1] - domain[0]));
            if (std::abs(u) < 1e-12)
                u = 0;
            else if (std::abs(u - 1) < 1e-12)
                u = 1;
        }
    const auto c = table(source.order(), source.poles(), source.weights(), knots);
    const auto first = c.poles().front();
    auto working = c.poles();
    const double tolerance = native_bspline_knot_tolerance(c, working);
    const auto cd = c.knot_domain();
    struct Group {
        double value;
        std::size_t count;
    };
    std::vector<Group> groups;
    std::size_t filled = 0;
    for (std::size_t i = 0; i < knots.size(); ++i) {
        if (knots[i] < cd[0])
            continue;
        if (knots[i] > cd[1])
            break;
        if (groups.empty() || std::abs(knots[i] - knots[i - 1]) > tolerance)
            groups.push_back({knots[i], 1});
        else
            ++groups.back().count;
        ++filled;
    }
    require(filled == knots.size(),
            "native elevation non-clamped knot allocation has unwritten entries");
    require(!groups.empty(), "native elevation has no knot groups");
    groups.back().value = cd[1];
    const auto delta = degree + 1 - source.order();
    require(groups.size() <= (std::size_t(INT32_MAX) - knots.size()) / delta,
            "native elevation knot count overflow");
    const auto nk = knots.size() + groups.size() * delta;
    require(nk >= 2 * (degree + 1) && nk - degree - 1 <= budget.max_control_points,
            "native elevation output control budget");
    const auto nc = nk - degree - 1;
    for (unsigned i = 0; i < 8; ++i)
        charge(budget, nk);
    std::vector<double> output_knots;
    output_knots.reserve(nk);
    for (auto g : groups)
        output_knots.insert(output_knots.end(), g.count + delta, g.value);
    std::vector<Point3> output(nc);
    std::vector<double> weights(source.rational() ? nc : 0);
    const unsigned order = degree + 1;
    // One derivative/symmetric block at a time: the source working poles carry
    // the native tolerance-query round trips through all nc calls, including i=0.
    for (std::size_t i = 0; i < nc; ++i) {
        for (unsigned p = 0; p < 8; ++p)
            charge(budget, count);
        charge(budget, 16 * order * order * order);
        auto evaluation = native_bspline_evaluate_working(c, output_knots[i + degree], degree, true,
                                                          std::move(working));
        working = std::move(evaluation.working_poles);
        std::array<double, 26 * 26> symmetric{};
        for (unsigned j = 0; j <= degree; ++j)
            symmetric[j * order] = 1;
        for (unsigned j = 1; j <= degree; ++j)
            for (unsigned k = 1; k <= j; ++k)
                symmetric[j * order + k] =
                    finite((output_knots[i + j - 1] - output_knots[i + degree]) *
                               symmetric[(j - 1) * order + k - 1] +
                           symmetric[(j - 1) * order + k]);
        if (!i)
            continue;
        std::array<double, 26> psi{};
        for (unsigned j = 0; j <= degree; ++j) {
            psi[j] = (j % 2 ? -1. : 1.) * symmetric[degree * order + degree - j];
            for (unsigned k = j + 1; k <= degree; ++k)
                psi[j] /= k;
        }
        for (unsigned j = 0; j <= degree; ++j) {
            const auto q = degree - j;
            const double factor = (q % 2 ? -1. : 1.) * psi[q];
            for (unsigned axis = 0; axis < 3; ++axis)
                output[i - 1][axis] =
                    finite(output[i - 1][axis] + factor * evaluation.homogeneous[j][axis]);
            if (source.rational())
                weights[i - 1] = finite(weights[i - 1] + factor * evaluation.homogeneous[j][3]);
        }
    }
    output.back() = working.back();
    output.front() = first;
    if (source.rational()) {
        weights.back() = source.weights().back();
        weights.front() = source.weights().front();
    }
    const bool restore = normalize && normalized(output_knots[degree], output_knots[nc]);
    if (restore)
        for (auto &u : output_knots)
            u = finite(domain[0] + (domain[1] - domain[0]) * u);
    report["normalized_source_knots"] = normalize;
    report["restored_source_domain"] = restore;
    report["knot_tolerance"] = tolerance;
    report["control_points"] = nc;
    report["work_used"] = budget.work;
    return {table(order, output, weights, output_knots), std::move(report)};
}

TubeCurve combine_tube_curves(const BsplineCurve &left, const BsplineCurve &right,
                              bool force_contiguous, bool reparameterize, TubeBudget &budget) {
    Json left_open, right_open, left_elevation = nullptr, right_elevation = nullptr;
    auto a = opened(left, budget, left_open), b = opened(right, budget, right_open);
    if (a.order() < b.order()) {
        auto e = elevate_open_tube_curve(a, b.order() - 1, budget);
        a = std::move(e.curve);
        left_elevation = std::move(e.report);
    } else if (b.order() < a.order()) {
        auto e = elevate_open_tube_curve(b, a.order() - 1, budget);
        b = std::move(e.curve);
        right_elevation = std::move(e.report);
    }
    auto result = combine_open_tube_curves(a, b, force_contiguous, reparameterize, budget);
    result.report = {{"scope", "native_curve_combination"},
                     {"left_opening", left_open},
                     {"right_opening", right_open},
                     {"left_elevation", left_elevation},
                     {"right_elevation", right_elevation},
                     {"combination", std::move(result.report)},
                     {"work_used", budget.work},
                     {"source_geometry_reused", false}};
    return result;
}
} // namespace p3d::swept_detail
