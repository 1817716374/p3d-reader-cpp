#include "native_tube.hpp"
#include "loft_curve.hpp"

namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t amount) {
    require(b.work <= b.max_work && amount <= b.max_work - b.work,
            "native tube closure work budget exceeded");
    b.work += amount;
}
Json source_table(const BsplineSurface &s) {
    Json xyz = Json::array();
    for (const auto &p : s.poles())
        for (double x : p)
            xyz.push_back(x);
    return {{"_type", "BsplineSurface"},
            {"orderU", s.u().order()},
            {"orderV", s.v().order()},
            {"closedU", s.u().closed()},
            {"closedV", s.v().closed()},
            {"numPolesU", s.u().pole_count()},
            {"numPolesV", s.v().pole_count()},
            {"knotsU", s.u().knots()},
            {"knotsV", s.v().knots()},
            {"poles", std::move(xyz)},
            {"weights", s.rational() ? Json(s.weights()) : Json()},
            {"numRulesU", s.num_rules_u()},
            {"numRulesV", s.num_rules_v()},
            {"holeOrigin", s.hole_origin()},
            {"boundaries", s.boundaries()}};
}
} // namespace

TubeAssembly close_tube_surface_v(const BsplineSurface &s, TubeBudget &budget) {
    const auto nu = s.u().pole_count(), nv = s.v().pole_count();
    const auto order = s.v().order();
    require(nu && nv && nv <= INT32_MAX && nu <= INT32_MAX &&
                nu <= budget.max_control_points / nv && order <= 26,
            "native tube closure control budget/order exceeded");
    require(s.boundaries().is_null() && s.hole_origin() == 0,
            "native tube closure requires an untrimmed tube surface");
    const auto count = nu * nv;
    charge(budget, count);
    Json report{{"scope", "native_tube_v_closure"}, {"applied", false},
                {"native_return_code", 0},          {"input_v_poles", nv},
                {"columns", Json::array()},         {"surface_validity", "not_certified"}};
    if (s.v().closed()) {
        report["applied"] = true;
        report["method"] = "already_closed_copy";
        report["work_used"] = budget.work;
        return {source_table(s), std::move(report)};
    }
    // Account for column extraction, closure storage and serialization before
    // allocating. Orders are bounded above, and division avoids size overflow.
    for (unsigned i = 0; i < 5; ++i)
        charge(budget, count);
    require(nu <= (budget.max_work - budget.work) / (4 * order * order),
            "native tube closure triangular work budget exceeded");
    charge(budget, nu * 4 * order * order);
    std::vector<loft_detail::H> result;
    std::size_t out_nv = 0;
    bool out_closed = false;
    std::vector<double> out_knots;
    for (std::size_t u = 0; u < nu; ++u) {
        loft_detail::Curve c;
        c.degree = order - 1;
        c.rational = s.rational();
        c.knots = s.v().knots();
        c.poles.reserve(nv);
        for (std::size_t v = 0; v < nv; ++v) {
            const auto index = v * nu + u;
            const auto &p = s.poles()[index];
            c.poles.push_back({p[0], p[1], p[2], s.rational() ? s.weights()[index] : 1.});
        }
        auto closure = loft_detail::close_normalized_curve(std::move(c), unsigned(nv));
        auto &column = closure.curve;
        closure.report["u_index"] = u;
        closure.report["closed"] = closure.closed;
        closure.report["success"] = closure.success;
        report["columns"].push_back(std::move(closure.report));
        if (!closure.success) {
            report["method"] = "retained_input";
            report["reason"] = "column_closure_failed";
            report["native_return_code"] = 1;
            report["work_used"] = budget.work;
            return {source_table(s), std::move(report)};
        }
        if (u == 0) {
            out_nv = column.poles.size();
            out_closed = closure.closed;
            out_knots = column.knots;
            require(out_nv <= nv, "native tube closure unexpectedly increased control count");
            result.resize(nu * out_nv);
        } else if (column.poles.size() != out_nv) {
            // Native exits before replacing the surface but retains the zero
            // return code from the last successful closeCurve call.
            report["method"] = "retained_input";
            report["reason"] = "column_pole_count_mismatch";
            report["work_used"] = budget.work;
            return {source_table(s), std::move(report)};
        }
        // The first column owns V parameters. Later columns only have their
        // pole count compared by the native routine, not their knot values.
        report["columns"].back()["uses_first_column_parameters"] = true;
        for (std::size_t v = 0; v < out_nv; ++v)
            result[v * nu + u] = column.poles[v];
    }
    Json out = source_table(s), xyz = Json::array(), weights = Json::array();
    for (const auto &h : result) {
        for (unsigned axis = 0; axis < 3; ++axis)
            xyz.push_back(h[axis]);
        if (s.rational())
            weights.push_back(h[3]);
    }
    out["poles"] = std::move(xyz);
    out["weights"] = s.rational() ? std::move(weights) : Json();
    out["numPolesV"] = out_nv;
    out["closedV"] = out_closed;
    out["knotsV"] = std::move(out_knots);
    BsplineSurface::from_bgfb(out);
    report["applied"] = true;
    report["method"] = "column_curve_closure";
    report["output_v_poles"] = out_nv;
    report["output_closed_v"] = out_closed;
    report["work_used"] = budget.work;
    return {std::move(out), std::move(report)};
}
} // namespace p3d::swept_detail
