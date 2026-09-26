// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from imodel-native bspcurv.cpp prepareCurve, MSBsplineCurve_ByBezier.cpp
// support extraction, bezierDPoint4d.cpp knot saturation and bspdsurf.cpp tubeSurface.
// Changes: bounded immutable C++/JSON storage, native P3D tolerance, explicit
// source-span/leading-control reports and shared resource limits.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_tube.hpp"
#include "bspline_frame.hpp"
#include "native_curve_affine.hpp"

namespace p3d::swept_detail {
namespace {
using H = std::array<double, 4>;
double finite(double v) {
    require(std::isfinite(v), "native tube path nonfinite arithmetic");
    return v;
}
void charge(TubeBudget &budget, std::size_t amount) {
    require(budget.work <= budget.max_work && amount <= budget.max_work - budget.work,
            "native tube path work budget exceeded");
    budget.work += amount;
}
bool null_interval(double a, double b) {
    // Base isNullKnotInterval: strict comparison, sensitive to the raw knot
    // domain. Normalizing before this decision would change segment selection.
    const double tolerance = finite(((std::abs(a) + 1) + std::abs(b)) * 1e-14);
    return tolerance > std::abs(finite(a - b));
}
void saturate(std::vector<H> &p, std::vector<double> k) {
    const auto degree = p.size() - 1;
    const double left = k[degree - 1], right = k[degree];
    // Native two-sided knot insertion in the local support. The order <= 26
    // and sorted support bound both triangular passes by degree squared.
    while (k.front() < left) {
        for (std::size_t i = 0; k[i] < left; ++i) {
            const double f1 = finite((left - k[i]) / (k[i + degree] - k[i])), f0 = 1 - f1;
            for (unsigned axis = 0; axis < 4; ++axis)
                p[i][axis] = finite(f0 * p[i][axis] + f1 * p[i + 1][axis]);
            k[i] = k[i + 1];
        }
    }
    while (right < k.back()) {
        for (std::size_t i = k.size() - 1, j = degree; right < k[i]; --i, --j) {
            const double f1 = finite((right - k[i]) / (k[i - degree] - k[i])), f0 = 1 - f1;
            for (unsigned axis = 0; axis < 4; ++axis)
                p[j][axis] = finite(f0 * p[j][axis] + f1 * p[j - 1][axis]);
            k[i] = k[i - 1];
        }
    }
}
} // namespace

TubeTrace prepare_tube_trace(const BsplineCurve &trace, TubeBudget &budget) {
    const auto order = trace.order(), degree = order - 1;
    const auto n = trace.poles().size();
    require(order <= 26 && n <= INT32_MAX && n <= budget.max_control_points,
            "native tube trace order or control budget exceeded");
    const auto candidates = trace.closed() ? n : n - order + 1;
    charge(budget, candidates);
    const auto &knots = trace.knots();
    std::size_t count = 0;
    for (std::size_t i = 0; i < candidates; ++i)
        count += !null_interval(knots[i + degree], knots[i + order]);
    require(count > 0, "native tube trace has no non-null Bezier intervals");
    require(count <= budget.max_control_points / order &&
                count <= (std::size_t(INT32_MAX) - 1) / degree,
            "native tube trace Bezier storage budget exceeded");
    const auto per_span = std::size_t(8) * order * order + 4 * order;
    require(count <= (budget.max_work - budget.work) / per_span,
            "native tube trace extraction budget exceeded");
    charge(budget, count * per_span);
    TubeTrace result;
    result.segments.reserve(count);
    Json spans = Json::array(), replacements = Json::array();
    H previous_end{};
    for (std::size_t i = 0; i < candidates; ++i) {
        const double u0 = knots[i + degree], u1 = knots[i + order];
        if (null_interval(u0, u1))
            continue;
        std::vector<H> poles;
        poles.reserve(order);
        for (std::size_t j = 0; j < order; ++j) {
            auto index = std::int64_t(i + j) + trace.periodic_pole_shift();
            if (trace.closed()) {
                index %= std::int64_t(n);
                if (index < 0)
                    index += std::int64_t(n);
            }
            require(index >= 0 && std::uint64_t(index) < n, "native tube trace support index");
            const auto &p = trace.poles()[std::size_t(index)];
            poles.push_back(
                {p[0], p[1], p[2], trace.rational() ? trace.weights()[std::size_t(index)] : 1.});
        }
        saturate(poles,
                 std::vector<double>(knots.begin() + i + 1, knots.begin() + i + 1 + 2 * degree));
        if (!result.segments.empty()) {
            if (poles.front() != previous_end)
                replacements.push_back({{"segment", result.segments.size()},
                                        {"incoming_homogeneous", poles.front()},
                                        {"retained_homogeneous", previous_end}});
            poles.front() = previous_end;
        }
        previous_end = poles.back();
        Json xyz = Json::array(), weights = Json::array(), normalized_knots = Json::array();
        for (auto &p : poles) {
            for (unsigned axis = 0; axis < 3; ++axis)
                xyz.push_back(p[axis]);
            weights.push_back(p[3]);
        }
        for (unsigned j = 0; j < 2 * order; ++j)
            normalized_knots.push_back(j < order ? 0. : 1.);
        result.segments.push_back(
            BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                     {"order", order},
                                     {"closed", false},
                                     {"poles", std::move(xyz)},
                                     {"weights", trace.rational() ? std::move(weights) : Json()},
                                     {"knots", std::move(normalized_knots)}}));
        spans.push_back({{"support_index", i}, {"source_knot_interval", {u0, u1}}});
    }
    result.report = {{"scope", "native_tube_trace_preparation"},
                     {"source_closed", trace.closed()},
                     {"periodic_pole_shift", trace.periodic_pole_shift()},
                     {"candidate_intervals", candidates},
                     {"skipped_intervals", candidates - count},
                     {"segments", std::move(spans)},
                     {"replaced_leading_controls", std::move(replacements)},
                     {"prepared_control_count", count * degree + 1},
                     {"work_used", budget.work}};
    return result;
}

TubePatch tube_surface(const BsplineCurve &section, const BsplineCurve &trace, bool rigid,
                       TubeBudget &budget) {
    require(trace.order() <= 26 && section.order() <= 26 &&
                trace.poles().size() <= budget.max_control_points &&
                trace.poles().size() <= INT32_MAX &&
                section.poles().size() <= budget.max_control_points,
            "native tube surface input budget exceeded");
    charge(budget, std::size_t(trace.order()) * trace.order() * trace.order());
    for (unsigned i = 0; i < 13; ++i)
        charge(budget, trace.poles().size());
    // Native tubeSurface queries the source frame before processBspline makes
    // its prepared copy. Retain the frame query's weighted-control round trips.
    auto evaluated_frame = native_bspline_frame_working(trace, 0);
    const auto &source_frame = evaluated_frame.report;
    const auto working = curve_detail::with_poles(trace, evaluated_frame.working_poles);
    auto prepared = prepare_tube_trace(working, budget);
    const auto count = prepared.segments.size(), nu = section.poles().size();
    const auto nv = count * (trace.order() - 1) + 1;
    require(nu <= budget.max_control_points / nv && section.order() <= 26,
            "native tube surface control budget exceeded");
    // The initial Frenet frame belongs to the original curve, before null-span
    // skipping or shared-control preparation. Rows are N, B, T.
    Matrix3 frame{};
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned axis = 0; axis < 3; ++axis)
            frame[row][axis] = source_frame.at("frame").at(axis).at((row + 1) % 3);
    Json surface, pieces = Json::array(), joins = Json::array();
    for (std::size_t i = 0; i < count; ++i) {
        auto patch = tube_patch(section, prepared.segments[i], frame, rigid, budget);
        frame = patch.final_frame;
        pieces.push_back(std::move(patch.report));
        if (i == 0) {
            // Native first append is just a copy, even for a one-span closed
            // trace. Do not add a separate seam repair in that branch.
            surface = std::move(patch.surface);
        } else {
            auto combined = append_tube_patch(BsplineSurface::from_bgfb(surface),
                                              BsplineSurface::from_bgfb(patch.surface), i,
                                              trace.closed() && i + 1 == count, budget);
            surface = std::move(combined.surface);
            joins.push_back(std::move(combined.report));
        }
    }
    return {std::move(surface),
            frame,
            {{"scope", "native_bspline_tube_surface"},
             {"source_frame", source_frame},
             {"trace_preparation", std::move(prepared.report)},
             {"patches", std::move(pieces)},
             {"joins", std::move(joins)},
             {"work_used", budget.work},
             {"source_geometry_reused", false},
             {"surface_validity", "not_certified"}},
            std::move(evaluated_frame.working_poles)};
}
} // namespace p3d::swept_detail
