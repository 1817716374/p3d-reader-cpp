#include "native_tube_facet_patch.hpp"
#include "native_curve_affine.hpp"
#include "native_bezier.hpp"
#include "bspline_frame.hpp"
namespace p3d::swept_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native facet patch nonfinite arithmetic");
    return x;
}
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
Point3 scale(Point3 p, double f) {
    for (auto &x : p)
        x = finite(x * f);
    return p;
}
Point3 add(Point3 a, const Point3 &b) {
    for (unsigned i = 0; i < 3; ++i)
        a[i] = finite(a[i] + b[i]);
    return a;
}
Point3 subtract(Point3 a, const Point3 &b) {
    for (unsigned i = 0; i < 3; ++i)
        a[i] = finite(a[i] - b[i]);
    return a;
}
Point3 cross(const Point3 &a, const Point3 &b) {
    return {finite(a[1] * b[2] - a[2] * b[1]), finite(a[2] * b[0] - a[0] * b[2]),
            finite(a[0] * b[1] - a[1] * b[0])};
}
double dot(const Point3 &a, const Point3 &b) {
    return finite((a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]);
}
Point3 guarded_divide(Point3 p, double w) {
    const double largest = std::max({std::abs(p[0]), std::abs(p[1]), std::abs(p[2])});
    return largest * 1e-12 >= std::abs(w) ? p : scale(p, finite(1 / w));
}
Point3 transpose_multiply(const Matrix3 &m, const Point3 &p) {
    return {finite((p[0] * m[0][0] + p[1] * m[1][0]) + p[2] * m[2][0]),
            finite((p[1] * m[1][1] + p[0] * m[0][1]) + p[2] * m[2][1]),
            finite((p[1] * m[1][2] + p[0] * m[0][2]) + p[2] * m[2][2])};
}
struct Frame {
    Point3 tangent, normal, binormal, point;
};
Frame query(BsplineCurve &working, double fraction, TubeBudget &b) {
    const auto n = working.poles().size(), order = std::size_t(working.order());
    require(order >= 2 && order <= 26 && n <= b.max_control_points,
            "native facet frame order/control budget");
    for (unsigned i = 0; i < 16; ++i)
        charge(b, n);
    charge(b, working.knots().size());
    charge(b, 8 * order * order * order + 128);
    const auto result = native_bspline_frame_working(working, fraction);
    working = curve_detail::with_poles(working, result.working_poles);
    Frame out;
    for (unsigned i = 0; i < 3; ++i) {
        const auto &row = result.report.at("frame").at(i);
        out.tangent[i] = row.at(0);
        out.normal[i] = row.at(1);
        out.binormal[i] = row.at(2);
        out.point[i] = row.at(3);
    }
    return out;
}
void validate(const BsplineCurve &section, const BsplineCurve &path, const Matrix3 &frame,
              TubeBudget &b) {
    require(section.order() >= 2 && section.order() <= 26 &&
                section.poles().size() <= b.max_control_points,
            "native facet section order/control budget");
    require(!path.closed() && path.poles().size() == path.order() && path.order() >= 2 &&
                path.order() <= 26 && path.poles().size() <= b.max_control_points,
            "native facet requires open Bezier path");
    for (std::size_t i = 0; i < path.knots().size(); ++i)
        require(path.knots()[i] == (i < path.order() ? 0. : 1.),
                "native facet requires normalized Bezier knots");
    for (const auto &row : frame)
        for (double x : row)
            finite(x);
    charge(b, section.poles().size() + path.poles().size());
}
std::size_t grid_size(std::size_t nu, std::size_t nv, TubeBudget &b) {
    require(nv && nu <= b.max_control_points / nv && nu <= std::size_t(INT32_MAX) / nv,
            "native facet control grid budget");
    const auto count = nu * nv;
    for (unsigned i = 0; i < 128; ++i)
        charge(b, count);
    return count;
}
Json surface_table(const BsplineCurve &section, unsigned order_v, std::size_t nv,
                   const std::vector<double> &knots_v, std::size_t rules_u, Json poles,
                   Json weights) {
    Json out{{"_type", "BsplineSurface"},
             {"orderU", section.order()},
             {"orderV", order_v},
             {"closedU", section.closed()},
             {"closedV", false},
             {"numPolesU", section.poles().size()},
             {"numPolesV", nv},
             {"knotsU", section.knots()},
             {"knotsV", knots_v},
             {"poles", std::move(poles)},
             {"weights", std::move(weights)},
             {"numRulesU", rules_u},
             {"numRulesV", section.poles().size()},
             {"holeOrigin", 0},
             {"boundaries", nullptr}};
    BsplineSurface::from_bgfb(out);
    return out;
}
Json ruled(const BsplineCurve &section, const BsplineCurve &input, const Matrix3 &frame,
           TubeBudget &b) {
    grid_size(section.poles().size(), 2, b);
    const auto first =
        guarded_divide(input.poles().front(), input.rational() ? input.weights().front() : 1.);
    const auto last =
        guarded_divide(input.poles().back(), input.rational() ? input.weights().back() : 1.);
    const auto delta = subtract(last, first);
    Matrix4 placement{};
    placement[3][3] = 1;
    for (unsigned r = 0; r < 3; ++r) {
        for (unsigned c = 0; c < 3; ++c)
            placement[r][c] = frame[c][r];
        placement[r][3] = first[r];
    }
    auto controls = section.poles();
    if (!curve_detail::bspline_identity(placement))
        for (std::size_t i = 0; i < controls.size(); ++i)
            controls[i] =
                section.rational()
                    ? curve_detail::affine_point(placement, controls[i], section.weights()[i])
                    : curve_detail::affine_polynomial_point(placement, controls[i]);
    Json poles = Json::array(), weights = section.rational() ? Json::array() : Json();
    for (unsigned row = 0; row < 2; ++row)
        for (std::size_t j = 0; j < controls.size(); ++j) {
            const double w = section.rational() ? section.weights()[j] : 1.;
            const auto p =
                !row ? controls[j] : add(controls[j], section.rational() ? scale(delta, w) : delta);
            for (double x : p)
                poles.push_back(x);
            if (section.rational())
                weights.push_back(w);
        }
    return surface_table(section, 2, 2, {0, 0, 1, 1}, 2, std::move(poles), std::move(weights));
}
} // namespace
TubeFacetPatch generate_tube_facet_patch(const BsplineCurve &section, const BsplineCurve &path,
                                         Matrix3 frame, bool rigid, TubeBudget &budget) {
    validate(section, path, frame, budget);
    TubeFacetPatch out;
    out.final_frame = frame;
    auto prepared = prepare_tube_facet_trace(path, budget);
    out.report = {{"scope", "native_adaptive_facet_patch"},
                  {"preparation", prepared.report},
                  {"rigid_sweep", rigid},
                  {"initial_frame_rows", frame}};
    if (!prepared.success) {
        out.working_trace_poles = std::move(prepared.sampling.sampling.working_poles);
        out.report["status"] = "native_failure";
        out.report["work_used"] = budget.work;
        return out;
    }
    auto working = std::move(*prepared.trace);
    if (prepared.sampling.ruled_fallback) {
        out.surface = ruled(section, path, frame, budget);
    } else {
        const auto nu = section.poles().size(), nv = working.poles().size();
        const auto count = grid_size(nu, nv, budget);
        auto profile = section.poles();
        if (section.rational())
            for (std::size_t j = 0; j < nu; ++j)
                profile[j] = scale(profile[j], finite(1 / section.weights()[j]));
        const bool rational = section.rational() || working.rational();
        std::vector<Point3> controls;
        controls.reserve(count);
        std::vector<double> weights;
        if (rational)
            weights.reserve(count);
        Json rows = Json::array();
        for (std::size_t i = 0; i < nv; ++i) {
            const double v_weight = working.rational() ? working.weights()[i] : 1.;
            const double inverse = finite(1 / v_weight);
            // Native code reads before and after its mutating frame query.
            scale(working.poles()[i], inverse);
            const auto f = query(working, prepared.greville[i][0], budget);
            if (i)
                frame = advance_tube_frame(frame, f.tangent, rigid);
            const auto origin = scale(working.poles()[i], inverse);
            rows.push_back(frame);
            for (std::size_t j = 0; j < nu; ++j) {
                const auto rotated = transpose_multiply(frame, profile[j]);
                const double a = dot(f.normal, rotated), bb = dot(f.binormal, rotated),
                             c = dot(f.tangent, rotated);
                auto adjusted =
                    subtract(f.normal, scale(subtract(origin, f.point), prepared.greville[i][1]));
                const double length =
                    finite(std::sqrt((adjusted[0] * adjusted[0] + adjusted[1] * adjusted[1]) +
                                     adjusted[2] * adjusted[2]));
                if (length > 0)
                    adjusted = scale(adjusted, finite(1 / length));
                const auto other = cross(adjusted, f.binormal);
                controls.push_back(
                    add(add(origin, scale(add(scale(adjusted, a), scale(other, c)), length)),
                        scale(f.binormal, bb)));
                if (rational)
                    weights.push_back(
                        finite((section.rational() ? section.weights()[j] : 1.) * v_weight));
            }
        }
        Json poles = Json::array();
        for (std::size_t i = 0; i < count; ++i) {
            // Native code applies weights only after all frame queries and rows.
            const auto p = rational ? scale(controls[i], weights[i]) : controls[i];
            for (double x : p)
                poles.push_back(x);
        }
        out.surface =
            surface_table(section, working.order(), nv, working.knots(), path.poles().size(),
                          std::move(poles), rational ? Json(weights) : Json());
        out.report["frame_rows"] = std::move(rows);
    }
    out.final_frame = frame;
    out.working_trace_poles = working.poles();
    out.success = true;
    out.report["status"] = "complete";
    out.report["ruled_fallback"] = prepared.sampling.ruled_fallback;
    out.report["work_used"] = budget.work;
    return out;
}
TubeFacetPatch tube_facet_patch(const BsplineCurve &section, const BsplineCurve &path,
                                Matrix3 frame, bool rigid, TubeBudget &budget) {
    validate(section, path, frame, budget);
    auto working = path;
    const auto first = query(working, 0, budget);
    frame = advance_tube_frame(frame, first.tangent, rigid);
    auto out = generate_tube_facet_patch(section, working, frame, rigid, budget);
    out.report["callback_working_poles"] = working.poles();
    return out;
}
} // namespace p3d::swept_detail
