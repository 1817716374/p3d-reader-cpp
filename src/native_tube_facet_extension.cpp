#include "native_tube_facet_extension.hpp"
#include "native_tube_orientation.hpp"
namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    require(b.work <= b.max_work && n <= b.max_work - b.work,
            "native facet extension work budget exceeded");
    b.work += n;
}
double finite(double v) {
    require(std::isfinite(v), "native facet extension nonfinite arithmetic");
    return v;
}
const Point3 &read(const std::shared_ptr<TubeFacetSeamStorage<Point3>> &p) {
    require(p && p->alive, "native facet extension requires live tangent storage");
    return p->value;
}
BsplineCurve column(const BsplineSurface &s, std::size_t u) {
    Json xyz = Json::array(), weights = Json::array();
    for (std::size_t v = 0; v < s.v().pole_count(); ++v) {
        const auto index = v * s.u().pole_count() + u;
        for (double x : s.poles()[index])
            xyz.push_back(x);
        if (s.rational())
            weights.push_back(s.weights()[index]);
    }
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", s.v().order()},
                                    {"closed", s.v().closed()},
                                    {"knots", s.v().knots()},
                                    {"poles", std::move(xyz)},
                                    {"weights", s.rational() ? std::move(weights) : Json()}});
}
Json translated_profile(const BsplineDirection &u, const std::vector<Point3> &cartesian,
                        const std::vector<double> &weights, Point3 delta, TubeBudget &b) {
    const auto nu = cartesian.size();
    require(nu == u.pole_count() && (weights.empty() || weights.size() == nu) &&
                nu <= b.max_control_points / 2,
            "native extension profile layout/control budget");
    charge(b, 32 * nu);
    Json xyz = Json::array(), w = Json::array();
    // f210 constructor last flag=false multiplies Cartesian XYZ by W first;
    // c9d40 then translates homogeneous controls by delta*W in its second row.
    for (unsigned row = 0; row < 2; ++row)
        for (std::size_t i = 0; i < nu; ++i) {
            for (unsigned k = 0; k < 3; ++k) {
                double x = weights.empty() ? cartesian[i][k] : finite(cartesian[i][k] * weights[i]);
                if (row)
                    x = finite(x + (weights.empty() ? delta[k] : finite(delta[k] * weights[i])));
                xyz.push_back(x);
            }
            if (!weights.empty())
                w.push_back(weights[i]);
        }
    return {{"_type", "BsplineSurface"},
            {"orderU", u.order()},
            {"orderV", 2},
            {"numPolesU", nu},
            {"numPolesV", 2},
            {"closedU", u.closed()},
            {"closedV", false},
            {"knotsU", u.knots()},
            {"knotsV", {0., 0., 1., 1.}},
            {"poles", std::move(xyz)},
            {"weights", weights.empty() ? Json() : std::move(w)},
            {"numRulesU", 2},
            {"numRulesV", nu},
            {"holeOrigin", 0},
            {"boundaries", nullptr}};
}
Json extend(Json &table, bool second, const BsplineDirection &original_u,
            const std::vector<Point3> &cartesian, const std::vector<double> &weights,
            const Point3 &tangent, double amount, TubeBudget &b) {
    const auto s = BsplineSurface::from_bgfb(table);
    const bool ruled = s.v().order() == 2;
    const double used = ruled ? amount : std::max(.1, amount);
    Point3 delta{};
    for (unsigned k = 0; k < 3; ++k) {
        const double direction = second && !ruled ? -tangent[k] : tangent[k];
        delta[k] = finite(finite(direction * used) * 1.5);
    }
    Json report{{"method", ruled ? "move_control_row" : "translate_elevate_combine"},
                {"extension_parameter", used},
                {"displacement", delta}};
    if (ruled) {
        const auto nu = s.u().pole_count(), begin = second ? 0 : (s.v().pole_count() - 1) * nu;
        charge(b, 16 * nu);
        for (std::size_t i = 0; i < nu; ++i) {
            // Both native unweight and reweight use the BASE weight pointer,
            // including when the controls being moved belong to the last row.
            const double inverse = s.rational() ? finite(1. / s.weights()[i]) : 1.;
            for (unsigned k = 0; k < 3; ++k) {
                double x = s.rational() ? finite(s.poles()[begin + i][k] * inverse)
                                        : s.poles()[begin + i][k];
                x = finite(second ? x - delta[k] : x + delta[k]);
                if (s.rational())
                    x = finite(x * s.weights()[i]);
                table["poles"][3 * (begin + i) + k] = x;
            }
        }
    } else {
        require(s.num_rules_u() < INT32_MAX, "native extension rule count overflow");
        auto patch = translated_profile(original_u, cartesian, weights, delta, b);
        if (second)
            patch = reverse_tube_surface(BsplineSurface::from_bgfb(patch), false, b).surface;
        auto elevated =
            elevate_tube_surface_v(BsplineSurface::from_bgfb(patch), s.v().order() - 1, b);
        const auto e = BsplineSurface::from_bgfb(elevated.surface);
        auto combined =
            second ? combine_tube_surfaces_v(e, s, b) : combine_tube_surfaces_v(s, e, b);
        combined.surface["numRulesU"] = s.num_rules_u() + 1;
        table = std::move(combined.surface);
        report["elevation"] = std::move(elevated.report);
        report["combination"] = std::move(combined.report);
    }
    return report;
}
} // namespace
TubeAssembly combine_tube_surfaces_v(const BsplineSurface &a, const BsplineSurface &b,
                                     TubeBudget &budget) {
    require(a.boundaries().is_null() && b.boundaries().is_null() && !a.v().closed() &&
                !b.v().closed() && a.v().order() == b.v().order(),
            "native V combination requires open equal-order untrimmed surfaces");
    const auto nu = a.u().pole_count(), n1 = a.v().pole_count(), n2 = b.v().pole_count();
    require(b.u().pole_count() >= nu && n1 <= INT32_MAX && n2 <= INT32_MAX - n1,
            "native V combination column layout/count overflow");
    const auto nv = n1 + n2 - 1;
    require(a.poles().size() <= budget.max_control_points &&
                b.poles().size() <= budget.max_control_points &&
                nv <= budget.max_control_points / nu,
            "native V combination control budget exceeded");
    for (unsigned pass = 0; pass < 8; ++pass)
        charge(budget, nu * nv);
    std::vector<Point3> poles(nu * nv);
    std::vector<double> weights(a.rational() ? nu * nv : 0);
    std::vector<double> knots;
    for (std::size_t u = 0; u < nu; ++u) {
        auto joined = combine_open_tube_curves(column(a, u), column(b, u), true, false, budget);
        require(joined.curve.poles().size() == nv, "native V combination inconsistent column size");
        if (!u)
            knots = joined.curve.knots();
        for (std::size_t v = 0; v < nv; ++v) {
            poles[v * nu + u] = joined.curve.poles()[v];
            if (a.rational())
                weights[v * nu + u] = joined.curve.weights()[v];
        }
    }
    Json flat = Json::array();
    for (const auto &p : poles)
        for (double x : p)
            flat.push_back(x);
    return {{{"_type", "BsplineSurface"},
             {"numPolesU", nu},
             {"numPolesV", nv},
             {"orderU", a.u().order()},
             {"orderV", a.v().order()},
             {"closedU", a.u().closed()},
             {"closedV", false},
             {"knotsU", a.u().knots()},
             {"knotsV", std::move(knots)},
             {"poles", std::move(flat)},
             {"weights", a.rational() ? Json(std::move(weights)) : Json()},
             {"numRulesU", nv},
             {"numRulesV", nu},
             {"holeOrigin", a.hole_origin()},
             {"boundaries", nullptr}},
            {{"scope", "native_forced_surface_v_combination"},
             {"reparameterized", false},
             {"discarded_incoming_first_row", true},
             {"knot_source_column", 0},
             {"work_used", budget.work}}};
}
void extend_tube_facet_plane_seam(TubeFacetPlanePreparation &p, const TubeFacetSeamReferences &seam,
                                  TubeBudget &budget) {
    require(p.status == TubeFacetSeamStatus::pending_general && p.classifier == 1,
            "native facet extension requires completed plane preparation");
    const bool self = p.report.at("self_surface").get<bool>();
    auto first = p.first, second = self ? Json() : p.second;
    auto &other = self ? first : second;
    const auto original = BsplineSurface::from_bgfb(first);
    Json report;
    report["first"] = extend(first, false, original.u(), p.first_original, p.first_weights,
                             read(seam.incoming), p.first_extension, budget);
    report["second"] = extend(other, true, original.u(), p.second_original, p.second_weights,
                              read(seam.outgoing), p.second_extension, budget);
    p.first = std::move(first);
    p.second = self ? p.first : std::move(second);
    p.status = TubeFacetSeamStatus::complete;
    p.report["extension"] = std::move(report);
    p.report["reason"] = "surface_extension_applied";
    p.report["work_used"] = budget.work;
}
} // namespace p3d::swept_detail
