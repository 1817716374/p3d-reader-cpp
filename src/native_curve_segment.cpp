// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from imodel-native bspcurv_segmentCurve2 and bspknot_insertKnot.
// Changes: P3D branch/rounding behavior, immutable inputs, bounded storage/work.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_curve_segment.hpp"
#include "native_curve_affine.hpp"
#include "native_knot_normalize.hpp"
namespace p3d::curve_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native subcurve nonfinite arithmetic");
    return x;
}
loft_detail::Curve raw(const BsplineCurve &s) {
    loft_detail::Curve c;
    c.degree = s.order() - 1;
    c.rational = s.rational();
    c.knots = s.knots();
    c.poles.reserve(s.poles().size());
    for (std::size_t i = 0; i < s.poles().size(); ++i) {
        const auto &p = s.poles()[i];
        c.poles.push_back({p[0], p[1], p[2], s.rational() ? s.weights()[i] : 1.});
    }
    return c;
}
void charge_curve(const BsplineCurve &s, BezierWork work, bool opening = false) {
    work.charge(s.knots().size());
    for (unsigned i = 0; i < (opening ? 8 * (s.order() + 1) : 16); ++i)
        work.charge(s.poles().size());
}
bool open(BsplineCurve &s, double knot, unsigned limit, BezierWork work, Json &report) {
    charge_curve(s, work, true);
    const auto domain = s.knot_domain();
    const double edge = finite(domain[1] - domain[0]) * 1e-10;
    const double effective =
        knot < finite(domain[0] + edge) || knot > finite(domain[1] - edge) ? 0 : knot;
    if (effective < domain[0] || effective > domain[1]) {
        report = {{"success", false},
                  {"reason", "effective_opening_knot_outside_domain"},
                  {"requested_knot", knot},
                  {"effective_knot", effective}};
        return false;
    }
    auto c = loft_detail::open_periodic_boundary_at(s, knot, limit, &report);
    s = BsplineCurve::from_bgfb(c.table());
    report["success"] = true;
    return true;
}
bool reverse(BsplineCurve &s, unsigned limit, BezierWork work, Json &report) {
    const bool closed = s.closed();
    if (closed && !open(s, 0, limit, work, report["opening"]))
        return false;
    charge_curve(s, work);
    auto c = raw(s);
    std::reverse(c.poles.begin(), c.poles.end());
    std::reverse(c.knots.begin(), c.knots.end());
    // The native reversal ignores normalization failure. Its descending knots
    // are not evaluable storage; do not silently repair or label native false.
    require(normalize_native_knots(c.knots, c.poles.size(), c.degree + 1, false),
            "native reversal left descending knots after failed normalization");
    s = BsplineCurve::from_bgfb(c.table());
    if (closed) {
        work.charge(4 * std::size_t(s.order()) * s.order());
        charge_curve(s, work);
        auto closure = loft_detail::close_native_curve(s, limit);
        auto table = closure.curve.table();
        table["closed"] = closure.closed;
        s = BsplineCurve::from_bgfb(table);
        report["closure"] = std::move(closure.report);
        report["success"] = closure.success;
        return closure.success;
    }
    report["success"] = true;
    return true;
}
} // namespace
bool insert_open_native_knot(loft_detail::Curve &c, double knot, double tolerance, unsigned target,
                             unsigned limit, BezierWork work, Json &report) {
    const auto n = c.poles.size();
    const unsigned order = c.degree + 1;
    require(order >= 2 && order <= 26 && n >= order && n <= limit && n <= INT32_MAX &&
                c.knots.size() == n + order && std::is_sorted(c.knots.begin(), c.knots.end()) &&
                std::isfinite(knot) && std::isfinite(tolerance) && tolerance >= 0,
            "native knot insertion open storage/parameter bounds");
    for (unsigned i = 0; i < 8; ++i)
        work.charge(c.knots.size());
    for (double k : c.knots)
        finite(k);
    for (const auto &p : c.poles)
        for (double x : p)
            finite(x);
    report = {{"requested_knot", knot}, {"target_multiplicity", target}, {"success", false}};
    if (knot < c.knots[c.degree] || knot > c.knots[n]) {
        report["reason"] = "outside_domain";
        return false;
    }
    unsigned multiplicity = 0;
    double t = knot;
    for (double k : c.knots) {
        if (std::abs(finite(k - t)) <= tolerance) {
            t = k;
            ++multiplicity;
        } else if (multiplicity)
            break;
    }
    const unsigned added = target > multiplicity ? target - multiplicity : 0;
    report["snapped_knot"] = t;
    report["current_multiplicity"] = multiplicity;
    report["added"] = added;
    if (!added) {
        report["success"] = true;
        return true;
    }
    if (added > order) {
        report["reason"] = "added_multiplicity_exceeds_order";
        return false;
    }
    require(n + added <= limit && n + added <= INT32_MAX,
            "native knot insertion output control budget");
    work.charge(8 * std::size_t(order) * order);
    const auto right =
        std::size_t(std::upper_bound(c.knots.begin(), c.knots.end(), t) - c.knots.begin());
    require(right >= order && right <= n + order, "native open insertion relevant control range");
    const auto start = right - order;
    require(start <= n, "native open insertion control offset");
    std::vector<loft_detail::H> buffer(order + added);
    for (unsigned j = 0; j < order; ++j)
        buffer[added + j] = c.poles[(start + j) % n];
    for (unsigned step = 0; step < added; ++step) {
        buffer[step] = buffer[added];
        const unsigned width = c.degree - step;
        for (unsigned j = 0; j < width; ++j) {
            require(right + j < c.knots.size(), "native insertion knot stencil bounds");
            const double low = c.knots[right - width + j];
            const double a = finite(finite(t - low) / finite(c.knots[right + j] - low));
            for (unsigned axis = 0; axis < (c.rational ? 4u : 3u); ++axis) {
                auto &x = buffer[added + j][axis];
                x = finite(finite(a * finite(buffer[added + j + 1][axis] - x)) + x);
            }
        }
    }
    std::vector<loft_detail::H> poles(n + added);
    auto copy = [&](const auto &from, std::size_t at, std::size_t count, std::size_t to) {
        require(at <= from.size() && count <= from.size() - at && to <= poles.size() &&
                    count <= poles.size() - to,
                "native insertion copy bounds");
        std::copy_n(from.begin() + at, count, poles.begin() + to);
    };
    // P3D also takes the right-wrap branch for OPEN input; the newer upstream
    // `closed &&` guard is absent in the native implementation.
    if (start > n - order) {
        const auto shift = start + order - n;
        copy(buffer, order + added - shift, shift, 0);
        copy(c.poles, shift, n - order, shift);
        copy(buffer, 0, n + added - start, start);
        report["control_copy"] = "right_wrap";
    } else {
        copy(c.poles, 0, start, 0);
        copy(buffer, 0, order + added, start);
        copy(c.poles, start + order, n - start - order, start + order + added);
        report["control_copy"] = "interior";
    }
    c.knots.insert(c.knots.begin() + right, added, t);
    c.poles = std::move(poles);
    report["success"] = true;
    return true;
}
NativeCurveSegment native_curve_segment(const BsplineCurve &source, double first, double last,
                                        unsigned limit, BezierWork work) {
    require(source.order() >= 2 && source.order() <= 26 && source.poles().size() <= limit &&
                std::isfinite(first) && std::isfinite(last),
            "native subcurve source/parameter bounds");
    charge_curve(source, work);
    NativeCurveSegment out;
    out.working_poles = source.poles();
    const double tolerance = native_bspline_knot_tolerance(source, out.working_poles);
    auto current = with_poles(source, out.working_poles);
    const auto domain = source.knot_domain();
    const double span = finite(domain[1] - domain[0]);
    double u1 = finite(domain[0] + finite(span * first));
    double u2 = finite(domain[0] + finite(span * last));
    const double difference = finite(u2 - u1), distance = std::abs(difference);
    const bool reversed = difference < 0;
    out.report = {{"scope", "native_curve_segment"},
                  {"fraction0", first},
                  {"fraction1", last},
                  {"knot0", u1},
                  {"knot1", u2},
                  {"knot_tolerance", tolerance},
                  {"reversed", reversed},
                  {"source_geometry_modified", false}};
    auto finish = [&](bool success, const char *reason) {
        out.success = success;
        out.report["success"] = success;
        out.report["reason"] = reason;
        out.report["has_geometry"] = out.curve.has_value();
        out.report["work_used"] = work.used;
        return std::move(out);
    };
    if (distance < tolerance)
        return finish(false, "interval_below_knot_tolerance");
    if (distance >= span) {
        bool success = true;
        out.report["branch"] = "whole_copy";
        if (reversed)
            success = reverse(current, limit, work, out.report["reversal"]);
        if (current.closed() && std::abs(finite(u1 - domain[0])) > tolerance &&
            std::abs(finite(u1 - domain[1])) > tolerance)
            success = open(current, u1, limit, work, out.report["opening"]);
        out.curve = std::move(current);
        return finish(success, success ? "whole_copy" : "whole_copy_operation_failed");
    }
    if (current.closed()) {
        out.report["branch"] = "periodic_partial";
        while (u1 < domain[0]) {
            work.charge(1);
            const double next = finite(u1 + span);
            require(next > u1, "native subcurve wrapping made no progress");
            u1 = next;
        }
        while (u1 > domain[1]) {
            work.charge(1);
            const double next = finite(u1 - span);
            require(next < u1, "native subcurve wrapping made no progress");
            u1 = next;
        }
        if (!open(current, u1, limit, work, out.report["opening"]))
            return finish(false, "partial_opening_failed");
        if (reversed && !reverse(current, limit, work, out.report["working_reversal"]))
            return finish(false, "partial_working_reversal_failed");
        u1 = 0;
        u2 = distance; // Native raw distance, even when opening normalized the domain.
    } else {
        out.report["branch"] = "open_partial";
        u1 = std::clamp(u1, domain[0], domain[1]);
        u2 = std::clamp(u2, domain[0], domain[1]);
    }
    auto c = raw(current);
    out.report["insertion_knots"] = {u1, u2};
    if (!insert_open_native_knot(c, u1, tolerance, source.order(), limit, work,
                                 out.report["first_insertion"]))
        return finish(false, "first_insertion_failed");
    if (!insert_open_native_knot(c, u2, tolerance, source.order(), limit, work,
                                 out.report["last_insertion"]))
        return finish(false, "last_insertion_failed");
    if (u2 < u1)
        std::swap(u1, u2);
    work.charge(c.knots.size());
    std::size_t begin = 0, end;
    while (begin < c.knots.size() && std::abs(finite(c.knots[begin] - u1)) > tolerance)
        ++begin;
    require(begin < c.knots.size(), "native subcurve initial knot search out of bounds");
    end = begin + 1;
    while (end < c.knots.size() && std::abs(finite(c.knots[end] - u2)) > tolerance)
        ++end;
    require(end < c.knots.size(), "native subcurve final knot search out of bounds");
    out.report["first_knot_index"] = begin;
    out.report["final_knot_index"] = end;
    if (end - begin < source.order())
        return finish(false, "too_few_segment_controls");
    require(end <= c.poles.size() && end + source.order() <= c.knots.size(),
            "native subcurve copy bounds");
    c.poles = std::vector<loft_detail::H>(c.poles.begin() + begin, c.poles.begin() + end);
    c.knots = std::vector<double>(c.knots.begin() + begin, c.knots.begin() + end + source.order());
    if (!normalize_native_knots(c.knots, c.poles.size(), source.order(), false))
        return finish(false, "segment_domain_too_short_to_normalize");
    out.curve = BsplineCurve::from_bgfb(c.table());
    if (reversed) {
        // Unlike the working reversal above, the final native status is ignored.
        const bool result = reverse(*out.curve, limit, work, out.report["result_reversal"]);
        out.report["result_reversal_success"] = result;
    }
    return finish(true, "segment_constructed");
}
} // namespace p3d::curve_detail
