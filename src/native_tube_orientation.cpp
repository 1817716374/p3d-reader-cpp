// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Surface reversal adapted from imodel-native bspsurf.cpp. Native P3D ring
// orientation uses the area visitor and preserves its early-return rules.
// Changes: immutable inputs, bounded storage and explicit partial-state report.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_tube_orientation.hpp"
#include "native_knot_normalize.hpp"
#include "native_surface_iso.hpp"
#include "native_bspline_area.hpp"
#include "native_pcurve_points.hpp"

namespace p3d::swept_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native tube orientation nonfinite arithmetic");
    return x;
}
curve_detail::BezierWork work(TubeBudget &b) {
    return {b.work, b.max_work};
}
Point3 unit(Point3 p) {
    for (double v : p)
        finite(v);
    const double length = finite(std::sqrt(finite((p[0] * p[0] + p[1] * p[1]) + p[2] * p[2])));
    if (length > 0) {
        const double reciprocal = finite(1. / length);
        for (auto &x : p)
            x = finite(x * reciprocal);
        return p;
    }
    return {1, 0, 0};
}
} // namespace
TubeAssembly reverse_tube_surface(const BsplineSurface &s, bool reverse_u, TubeBudget &b) {
    require(s.boundaries().is_null(), "native tube reversal requires an untrimmed surface");
    const auto &u = s.u(), &v = s.v();
    require(s.poles().size() <= b.max_control_points && u.pole_count() <= INT32_MAX &&
                v.pole_count() <= INT32_MAX,
            "native tube reversal control budget exceeded");
    work(b).charge(s.poles().size());
    for (unsigned i = 0; i < 7; ++i)
        work(b).charge(s.poles().size());
    work(b).charge(u.knots().size());
    work(b).charge(v.knots().size());
    auto ku = u.knots(), kv = v.knots();
    auto &k = reverse_u ? ku : kv;
    const auto &d = reverse_u ? u : v;
    std::reverse(k.begin(), k.end());
    const bool normalized =
        curve_detail::normalize_native_knots(k, d.pole_count(), d.order(), d.closed());
    Json poles = Json::array(), weights = Json::array();
    for (std::size_t j = 0; j < v.pole_count(); ++j)
        for (std::size_t i = 0; i < u.pole_count(); ++i) {
            const auto index = reverse_u ? j * u.pole_count() + (u.pole_count() - 1 - i)
                                         : (v.pole_count() - 1 - j) * u.pole_count() + i;
            for (double x : s.poles()[index])
                poles.push_back(x);
            if (s.rational())
                weights.push_back(s.weights()[index]);
        }
    return {{{"_type", "BsplineSurface"},
             {"numPolesU", u.pole_count()},
             {"numPolesV", v.pole_count()},
             {"orderU", u.order()},
             {"orderV", v.order()},
             {"closedU", u.closed()},
             {"closedV", v.closed()},
             {"knotsU", std::move(ku)},
             {"knotsV", std::move(kv)},
             {"poles", std::move(poles)},
             {"weights", s.rational() ? std::move(weights) : Json()},
             {"boundaries", nullptr},
             {"numRulesU", s.num_rules_u()},
             {"numRulesV", s.num_rules_v()},
             {"holeOrigin", s.hole_origin()}},
            {{"direction", reverse_u ? "u" : "v"},
             {"knots_normalized", normalized},
             {"native_return", 0},
             {"evaluable_knot_order", normalized}}};
}
TubeOrientation orient_tube_surfaces(const std::vector<Json> &input, Point3 tangent,
                                     TubeBudget &b) {
    std::size_t total = 0;
    work(b).charge(input.size());
    for (const auto &j : input) {
        if (j.is_null())
            continue;
        require(j.is_object() && j.contains("poles") && j["poles"].is_array() &&
                    j["poles"].size() % 3 == 0,
                "native tube orientation surface table");
        const auto count = j["poles"].size() / 3;
        require(count <= b.max_control_points - total,
                "native tube orientation total control budget");
        total += count;
        for (unsigned i = 0; i < 8; ++i)
            work(b).charge(count);
    }
    tangent = unit(tangent);
    TubeOrientation out{input,
                        {{"scope", "native_tube_ring_orientation"},
                         {"unit_start_tangent", tangent},
                         {"native_result", true},
                         {"visited_rings", 0},
                         {"completed_all_rings", true},
                         {"stop_reason", nullptr},
                         {"rings", Json::array()}}};
    for (std::size_t i = 0; i < input.size(); ++i) {
        out.report["visited_rings"] = i + 1;
        if (input[i].is_null()) {
            out.report["native_result"] = false;
            out.report["completed_all_rings"] = false;
            out.report["stop_reason"] = "null_surface";
            break;
        }
        const auto surface = BsplineSurface::from_bgfb(input[i]);
        const auto iso = detail::native_iso_v_curve(surface, 0, work(b), b.max_control_points);
        const auto area = curve_detail::native_bspline_area(iso.curve, work(b));
        Json ring = {{"source_ring_index", i},
                     {"area", area.area},
                     {"area_valid", area.valid},
                     {"isocurve_order", iso.curve.order()},
                     {"reversed_u", false},
                     {"isocurve_zero_weight_fallbacks", iso.zero_weight_fallbacks}};
        if (!area.valid) {
            out.report["native_result"] = iso.curve.order() == 2;
            out.report["completed_all_rings"] = false;
            out.report["stop_reason"] = "area_failure";
            out.report["rings"].push_back(std::move(ring));
            break;
        }
        const auto normal = unit(area.normal);
        const double dot =
            finite((tangent[1] * normal[1] + tangent[0] * normal[0]) + tangent[2] * normal[2]);
        const bool positive = dot > 1e-14, reverse = i == 0 ? !positive : positive;
        ring["normal"] = normal;
        ring["tangent_dot_normal"] = dot;
        ring["reversed_u"] = reverse;
        if (reverse) {
            auto reversed = reverse_tube_surface(surface, true, b);
            out.surfaces[i] = std::move(reversed.surface);
            ring["reversal"] = std::move(reversed.report);
        }
        out.report["rings"].push_back(std::move(ring));
    }
    out.report["work_used"] = b.work;
    return out;
}
TubeOrientation orient_tube_surfaces_from_trace(const std::vector<Json> &surfaces,
                                                const BsplineCurve &trace, TubeBudget &b) {
    if (surfaces.empty()) {
        auto out = orient_tube_surfaces(surfaces, Point3{}, b);
        out.report["native_result"] = false;
        out.report["stop_reason"] = "empty_generated_surface_list";
        out.report["start_tangent_query"] = nullptr;
        return out;
    }
    require(trace.order() <= 26 && trace.poles().size() <= b.max_control_points,
            "native tube start tangent order or control budget exceeded");
    work(b).charge(trace.knots().size());
    work(b).charge(std::size_t(16) * trace.order() * trace.order());
    const auto query = detail::pcurve_point_tangent(trace, 0);
    const auto domain = trace.knot_domain();
    const double span = finite(domain[1] - domain[0]);
    auto fraction_tangent = query.tangent;
    for (auto &x : fraction_tangent)
        x = finite(x * span);
    const auto initial_unit = unit(fraction_tangent);
    auto out = orient_tube_surfaces(surfaces, initial_unit, b);
    out.report["start_tangent_query"] = {{"point", query.value.point},
                                         {"knot_tangent", query.tangent},
                                         {"raw_weight", query.value.weight},
                                         {"zero_weight_fallback", query.value.zero_weight_fallback},
                                         {"knot_domain", domain},
                                         {"fraction_tangent", fraction_tangent},
                                         {"first_unit_tangent", initial_unit}};
    return out;
}
} // namespace p3d::swept_detail
