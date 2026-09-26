#include "loft_curve.hpp"
#include "native_curve_range.hpp"
#include "native_curve_affine.hpp"

namespace p3d {
namespace {
double finite(double x) {
    require(std::isfinite(x), "nonfinite loft cap UV calculation");
    return x;
}
struct Range {
    Point3 low{}, high{};
    bool present = false;
    std::size_t spans = 0, skipped = 0, controls = 0, work_used = 0;
    std::size_t extrema = 0, rejected = 0, work_limit;
    unsigned limit;
    explicit Range(unsigned maximum) : work_limit(std::size_t(maximum) * 4096), limit(maximum) {}
    void curve(const Json &table, const Matrix4 &to_local) {
        const auto &flat = table.at("poles");
        require(flat.is_array() && flat.size() / 3 <= limit, "loft cap source control budget");
        curve_detail::BezierWork work{work_used, work_limit};
        work.charge(flat.size());
        auto curve = BsplineCurve::from_bgfb(table);
        if (!curve_detail::bspline_identity(to_local)) {
            work.charge(curve.poles().size() * 32);
            auto poles = curve.poles();
            for (std::size_t i = 0; i < poles.size(); ++i)
                poles[i] = curve.rational()
                               ? curve_detail::affine_point(to_local, poles[i], curve.weights()[i])
                               : curve_detail::affine_polynomial_point(to_local, poles[i]);
            curve = curve_detail::with_poles(curve, poles);
        }
        require(controls <= limit, "loft cap range control budget");
        const auto range = curve_detail::native_curve_range(curve, limit - controls, work);
        controls += range.segment_controls;
        spans += range.segments;
        skipped += range.skipped_intervals;
        extrema += range.extrema_evaluations;
        rejected += range.rejected_weights;
        if (range.present) {
            if (!present) {
                low = range.low;
                high = range.high;
                present = true;
            } else
                for (unsigned axis = 0; axis < 3; ++axis) {
                    if (range.low[axis] < low[axis])
                        low[axis] = range.low[axis];
                    if (range.high[axis] > high[axis])
                        high[axis] = range.high[axis];
                }
        }
    }
    void region(const Json &region, const Matrix4 &matrix, unsigned depth = 0) {
        require(depth < 80, "loft cap region depth");
        curve_detail::BezierWork{work_used, work_limit}.charge(1);
        for (const auto &entry : region.at("curves")) {
            const auto &g = entry.at("geometry");
            if (g.at("_type") == "CurveVector")
                this->region(g, matrix, depth + 1);
            else {
                require(g.at("_type") == "BsplineCurve", "unexpected native loft cap primitive");
                curve(g, matrix);
            }
        }
    }
};
bool almost(Point3 a, Point3 b) {
    double d2 = 0, scale = 1;
    for (unsigned i = 0; i < 3; ++i) {
        d2 += (a[i] - b[i]) * (a[i] - b[i]);
        scale += a[i] * a[i] + b[i] * b[i];
    }
    return finite(d2) < finite(scale * 1.0000000000000001e-20);
}
} // namespace

Json SectionLoft::native_cap_uv(bool top, double u, double v, unsigned max_control_points) const {
    require(max_control_points > 0 && std::isfinite(u) && std::isfinite(v),
            "invalid loft cap UV query options");
    Json out{{"status", "not_evaluated"},
             {"cap", top ? "top" : "bottom"},
             {"uv", {u, v}},
             {"containment", "not_evaluated"},
             {"planarity", "not_evaluated"},
             {"range_method", "native_bezier_extrema"},
             {"native_root_solver_reproduced", true}};
    try {
        const auto faces = native_faces(max_control_points);
        out["native_faces"] = faces.report;
        if (faces.report.at("status") != "complete") {
            if (faces.report.at("status") == "native_failure")
                out["status"] = "native_failure";
            return out;
        }
        const auto &cap = top ? faces.caps.top : faces.caps.bottom;
        if (cap.is_null()) {
            out["status"] = "native_failure";
            out["reason"] = "cap not present";
            return out;
        }
        out["frame_query"] = native_curve_frame(cap);
        if (out["frame_query"].at("status") != "computed") {
            out["status"] = out["frame_query"].at("status");
            return out;
        }
        auto to_world = out["frame_query"].at("frame").get<Matrix4>();
        Matrix3 rotation{};
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j)
                rotation[i][j] = to_world[i][j];
        const auto inverse = native_matrix_inverse(rotation);
        Matrix4 to_local{};
        to_local[3][3] = 1;
        for (unsigned i = 0; i < 3; ++i) {
            for (unsigned j = 0; j < 3; ++j)
                to_local[i][j] = inverse.matrix[i][j];
            if (inverse.inverted)
                to_local[i][3] = finite((inverse.matrix[i][1] * -to_world[1][3] +
                                         inverse.matrix[i][0] * -to_world[0][3]) +
                                        inverse.matrix[i][2] * -to_world[2][3]);
        }
        out["frame_inverse_succeeded"] = inverse.inverted;
        Range range(max_control_points);
        range.region(cap, to_local);
        require(range.present, "native cap range has no accepted finite points");
        out["local_range_before_normalization"] = {range.low, range.high};
        Point3 extent, difference;
        for (unsigned i = 0; i < 3; ++i) {
            extent[i] = finite(range.high[i] - range.low[i]);
            difference[i] = finite(range.high[i] - to_world[i][3]);
        }
        const bool shift = !almost(extent, difference);
        if (shift) {
            for (unsigned i = 0; i < 3; ++i) {
                const double delta =
                    (to_world[i][1] * range.low[1] + to_world[i][0] * range.low[0]) +
                    to_world[i][2] * range.low[2];
                to_world[i][3] = finite(to_world[i][3] + delta);
                to_local[i][3] = finite(to_local[i][3] - range.low[i]);
            }
            range.low = {0, 0, 0};
            range.high = extent;
        }
        const double zscale = finite(std::sqrt(finite(extent[0] * extent[1])));
        const bool scale = !(extent[0] == 1 && extent[1] == 1) && std::abs(extent[0]) > 1e-15 &&
                           std::abs(extent[1]) > 1e-15 && std::abs(zscale) > 1e-15;
        if (scale) {
            const Point3 factors{extent[0], extent[1], zscale};
            for (unsigned axis = 0; axis < 3; ++axis) {
                const double reciprocal = 1 / factors[axis];
                for (unsigned row = 0; row < 3; ++row)
                    to_world[row][axis] = finite(to_world[row][axis] * factors[axis]);
                for (unsigned col = 0; col < 4; ++col)
                    to_local[axis][col] = finite(to_local[axis][col] * reciprocal);
                range.low[axis] = finite(range.low[axis] * reciprocal);
                range.high[axis] = finite(range.high[axis] * reciprocal);
            }
        }
        Point3 point, du, dv;
        for (unsigned axis = 0; axis < 3; ++axis) {
            point[axis] =
                finite(((u * to_world[axis][0] + to_world[axis][3]) + v * to_world[axis][1]) +
                       0 * to_world[axis][2]);
            du[axis] = finite(to_world[axis][0]);
            dv[axis] = finite(to_world[axis][1]);
        }
        out.update({{"status", "computed"},
                    {"point", point},
                    {"u_direction", du},
                    {"v_direction", dv},
                    {"local_to_world", to_world},
                    {"world_to_local", to_local},
                    {"local_range", {range.low, range.high}},
                    {"origin_shifted", shift},
                    {"range_scaled", scale},
                    {"range_spans", range.spans},
                    {"skipped_near_zero_spans", range.skipped},
                    {"range_extrema_evaluations", range.extrema},
                    {"range_rejected_weights", range.rejected},
                    {"range_work_used", range.work_used}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
