#include "native_tube_mesh_patch.hpp"
#include "native_bezier_support.hpp"
#include "loft_curve.hpp"
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
using Pole = std::array<double, 4>;
BsplineCurve row(const BsplineSurface &s, std::size_t v) {
    Json xyz = Json::array(), weights = Json::array();
    for (std::size_t u = 0; u < s.u().pole_count(); ++u) {
        const auto i = v * s.u().pole_count() + u;
        for (double x : s.poles()[i])
            xyz.push_back(x);
        if (s.rational())
            weights.push_back(s.weights()[i]);
    }
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", s.u().order()},
                                    {"closed", s.u().closed()},
                                    {"knots", s.u().knots()},
                                    {"poles", std::move(xyz)},
                                    {"weights", s.rational() ? std::move(weights) : Json()}});
}
} // namespace
TubeMeshPatchPreparation prepare_tube_mesh_patch(const BsplineSurface &surface,
                                                 const std::vector<std::vector<Point2>> &bounds,
                                                 TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    TubeMeshPatchPreparation out;
    out.report = {{"scope", "native_swept_mesh_patch_preparation"},
                  {"status", "native_failure"},
                  {"mesh_generated", false},
                  {"opening_rows", Json::array()}};
    out.boundaries = tube_mesh_boundary_curves(bounds, budget);
    if (!out.boundaries.success) {
        out.report["failure_step"] = "boundary_graphs";
        return out;
    }
    const auto order = surface.u().order();
    const std::size_t degree = order - 1, nv = surface.v().pole_count();
    require(order <= 26 && nv && surface.poles().size() <= budget.max_control_points &&
                budget.max_control_points <= UINT_MAX,
            "native Bezier strip order/control budget");
    std::vector<std::vector<Pole>> rows;
    rows.reserve(nv);
    std::optional<BsplineCurve> first;
    std::vector<double> knots;
    const auto domain = surface.u().knot_domain();
    // 48480 computes this value and passes it directly to openSurface U.
    // The curve opening kernel interprets it as a raw knot, including its
    // native near-end/out-of-domain reset. Do not replace it with domain[0].
    const double requested = finite(finite(0. - domain[0]) / finite(domain[1] - domain[0]));
    std::size_t nu = 0;
    for (std::size_t v = 0; v < nv; ++v) {
        work.charge(surface.u().knots().size() + surface.u().pole_count() * 8);
        auto c = row(surface, v);
        if (surface.u().closed()) {
            work.charge(c.poles().size() * 8 * std::size_t(order) * order);
            Json report;
            auto opened = loft_detail::open_periodic_boundary_at(
                c, requested, unsigned(budget.max_control_points / nv), &report);
            out.report["opening_rows"].push_back(std::move(report));
            c = BsplineCurve::from_bgfb(opened.table());
        }
        if (!first) {
            first = c;
            knots = c.knots();
            nu = c.poles().size();
        }
        // Native sizes storage from the first row and reuses its knots.
        // An incompatible later row would leave storage unwritten/overrun.
        require(c.poles().size() == nu, "native strip opening inconsistent row control count");
        std::vector<Pole> p;
        p.reserve(nu);
        for (std::size_t u = 0; u < nu; ++u) {
            const auto &a = c.poles()[u];
            p.push_back({a[0], a[1], a[2], c.rational() ? c.weights()[u] : 1.});
        }
        rows.push_back(std::move(p));
    }
    out.knots = curve_detail::native_curve_knot_data(*first, budget.max_control_points, work);
    const auto &kd = out.knots;
    require(kd.left < kd.right, "native strip has no active compressed interval");
    const auto count = kd.right - kd.left;
    require(nv <= budget.max_control_points / order &&
                count <= budget.max_control_points / (nv * order),
            "native Bezier strip output control budget");
    work.charge(count * nv * order);
    std::vector<std::vector<Pole>> data(count, std::vector<Pole>(nv * order));
    for (std::size_t v = 0; v < nv; ++v)
        for (std::size_t u = 0; u < order; ++u)
            data[0][v * order + u] = rows[v][u];
    std::size_t b = degree;
    std::vector<double> alpha(degree);
    for (std::size_t span = 0; span < count; ++span) {
        const auto g = kd.left + span, m = kd.multiplicities[g + 1];
        if (span)
            b += kd.multiplicities[g];
        // Full interior multiplicity underflows the unsigned copy start and
        // leaves the next strip unwritten. Do not invent a repair here.
        require(span + 1 == count || m <= degree,
                "native strip internal multiplicity leaves invalid output indexing");
        if (m < degree) {
            const double numerator = finite(kd.compressed[g + 1] - kd.compressed[g]);
            for (std::size_t j = degree; j > m; --j) {
                require(b + j < knots.size(), "native strip coefficient knot index");
                alpha[j - m - 1] = finite(numerator / finite(knots[b + j] - knots[b]));
            }
            const auto r = degree - m;
            for (std::size_t j = 1; j <= r; ++j) {
                for (std::size_t k = degree; k >= m + j; --k) {
                    const double a = alpha[k - m - j], z = finite(1 - a);
                    work.charge(nv * 16);
                    for (std::size_t v = 0; v < nv; ++v) {
                        auto &current = data[span][v * order + k];
                        const auto &previous = data[span][v * order + k - 1];
                        for (unsigned axis = 0; axis < 3; ++axis)
                            current[axis] =
                                finite(finite(a * current[axis]) + finite(z * previous[axis]));
                        if (surface.rational())
                            current[3] = finite(finite(z * previous[3]) + finite(a * current[3]));
                    }
                }
                if (span + 1 < count) {
                    work.charge(nv);
                    for (std::size_t v = 0; v < nv; ++v)
                        data[span + 1][v * order + r - j] = data[span][v * order + degree];
                }
            }
        }
        if (span + 1 < count) {
            work.charge(nv * (m + 1));
            for (std::size_t i = degree - m; i < order; ++i) {
                const auto source = b - degree + m + i;
                require(source < nu, "native strip source control index");
                for (std::size_t v = 0; v < nv; ++v)
                    data[span + 1][v * order + i] = rows[v][source];
            }
        }
    }
    std::vector<double> uk(order, 0);
    uk.insert(uk.end(), order, 1);
    for (auto &strip : data) {
        work.charge(strip.size() * 4);
        Json xyz = Json::array(), weights = Json::array();
        for (const auto &p : strip) {
            for (unsigned i = 0; i < 3; ++i)
                xyz.push_back(p[i]);
            weights.push_back(p[3]);
        }
        out.strips.push_back(BsplineSurface::from_bgfb(
            {{"_type", "BsplineSurface"},
             {"orderU", order},
             {"orderV", surface.v().order()},
             {"closedU", false},
             {"closedV", surface.v().closed()},
             {"numPolesU", order},
             {"numPolesV", nv},
             {"knotsU", uk},
             {"knotsV", surface.v().knots()},
             {"poles", std::move(xyz)},
             {"weights", surface.rational() ? std::move(weights) : Json()},
             {"numRulesU", 0},
             {"numRulesV", surface.num_rules_v()},
             {"holeOrigin", 0},
             {"boundaries", nullptr}}));
    }
    out.success = true;
    out.report["status"] = "prepared";
    out.report["strip_count"] = count;
    out.report["source_closed_u"] = surface.u().closed();
    out.report["opening_argument"] = requested;
    out.report["work_used"] = budget.work;
    return out;
}
} // namespace p3d::swept_detail
