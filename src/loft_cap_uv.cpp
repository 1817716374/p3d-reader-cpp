#include "loft_curve.hpp"

namespace p3d {
namespace {
using H = loft_detail::H;
using Polynomial = std::vector<long double>;
double finite(double x) {
    require(std::isfinite(x), "nonfinite loft cap UV calculation");
    return x;
}
H evaluate(std::vector<H> poles, double t) {
    for (std::size_t n = poles.size(); n > 1; --n)
        for (std::size_t i = 0; i + 1 < n; ++i)
            for (unsigned k = 0; k < 4; ++k)
                poles[i][k] = (1 - t) * poles[i][k] + t * poles[i + 1][k];
    return poles.front();
}
long double choose(unsigned n, unsigned k) {
    long double v = 1;
    for (unsigned i = 1; i <= k; ++i)
        v = v * (n + 1 - i) / i;
    return v;
}
Polynomial tangent(const std::vector<H> &poles, unsigned axis) {
    const auto degree = unsigned(poles.size() - 1);
    if (std::all_of(poles.begin(), poles.end(),
                    [](const H &h) { return std::abs(h[3] - 1) <= 1e-8; })) {
        Polynomial out(degree);
        for (unsigned i = 0; i < degree; ++i)
            out[i] = degree * (static_cast<long double>(poles[i + 1][axis]) - poles[i][axis]);
        return out;
    }
    Polynomial out(2 * degree);
    for (unsigned i = 0; i < degree; ++i)
        for (unsigned j = 0; j <= degree; ++j) {
            const long double dx = static_cast<long double>(poles[i + 1][axis]) - poles[i][axis];
            const long double dw = static_cast<long double>(poles[i + 1][3]) - poles[i][3];
            out[i + j] += degree * (dx * poles[j][3] - dw * poles[j][axis]) *
                          choose(degree - 1, i) * choose(degree, j) / choose(2 * degree - 1, i + j);
        }
    return out;
}
void roots(const Polynomial &p, double a, double b, std::vector<double> &out,
           std::uint64_t &budget) {
    require(budget > 0, "loft cap extrema root budget");
    --budget;
    unsigned changes = 0;
    int previous = 0;
    for (auto x : p) {
        require(std::isfinite(x), "nonfinite loft cap derivative polynomial");
        const int sign = x > 0 ? 1 : x < 0 ? -1 : 0;
        if (sign) {
            changes += previous && sign != previous;
            previous = sign;
        }
    }
    if (!previous) {
        // Native zero-polynomial roots are an evenly spaced grid of `order`
        // points. Their range contributions matter at the weight acceptance gate.
        if (p.size() > 1)
            for (std::size_t i = 0; i < p.size(); ++i)
                out.push_back(a + (b - a) * double(i) / double(p.size() - 1));
        return;
    }
    if (p.front() == 0)
        out.push_back(a);
    if (p.back() == 0)
        out.push_back(b);
    if (!changes)
        return;
    const double mid = a + (b - a) * .5;
    if (b - a <= 8 * std::numeric_limits<double>::epsilon() || mid == a || mid == b) {
        out.push_back(mid);
        return;
    }
    Polynomial work = p, left(p.size()), right(p.size());
    left.front() = work.front();
    right.back() = work.back();
    for (std::size_t n = p.size() - 1; n > 0; --n) {
        for (std::size_t i = 0; i < n; ++i)
            work[i] = (work[i] + work[i + 1]) * .5L;
        left[p.size() - n] = work.front();
        right[n - 1] = work[n - 1];
    }
    roots(left, a, mid, out, budget);
    roots(right, mid, b, out, budget);
}
bool identity(const Matrix4 &matrix) {
    for (unsigned row = 0; row < 3; ++row) {
        if (!(matrix[row][3] > -1e-10 && matrix[row][3] < 1e-10))
            return false;
        for (unsigned col = 0; col < 3; ++col)
            if (std::abs(matrix[row][col] - double(row == col)) > 1e-12)
                return false;
    }
    return true;
}
struct Range {
    Point3 low, high;
    bool present = false;
    std::size_t spans = 0, skipped = 0, controls = 0;
    std::uint64_t root_budget;
    unsigned limit;
    Range(unsigned maximum) : root_budget(std::uint64_t(maximum) * 64), limit(maximum) {}
    void extend(H h) {
        // Native endpoint and extremum range extension both use this absolute gate.
        if (!(std::abs(h[3]) > 1e-12))
            return;
        const double reciprocal = 1 / h[3];
        Point3 p{finite(h[0] * reciprocal), finite(h[1] * reciprocal), finite(h[2] * reciprocal)};
        if (!present) {
            low = high = p;
            present = true;
        } else
            for (unsigned i = 0; i < 3; ++i) {
                low[i] = std::min(low[i], p[i]);
                high[i] = std::max(high[i], p[i]);
            }
    }
    void curve(const Json &table, const Matrix4 &to_local) {
        auto curve = loft_detail::Curve::from_bspline(BsplineCurve::from_bgfb(table), limit);
        if (!identity(to_local))
            for (auto &pole : curve.poles) {
                const auto old = pole;
                for (unsigned row = 0; row < 3; ++row)
                    pole[row] = finite(((to_local[row][0] * old[0] + to_local[row][3] * old[3]) +
                                        to_local[row][1] * old[1]) +
                                       to_local[row][2] * old[2]);
            }
        const auto source_knots = curve.knots;
        for (auto t : source_knots)
            if (t > 0 && t < 1)
                curve.insert(t, curve.degree, limit);
        require(curve.poles.size() <= limit - controls, "loft cap range control budget");
        controls += curve.poles.size();
        for (std::size_t span = curve.degree; span < curve.poles.size(); ++span) {
            const auto a = curve.knots[span], b = curve.knots[span + 1];
            if (!(b > a))
                continue;
            if (b - a < ((std::abs(a) + 1) + std::abs(b)) * 1e-14) {
                ++skipped;
                continue;
            }
            ++spans;
            std::vector<H> poles(curve.poles.begin() + span - curve.degree,
                                 curve.poles.begin() + span + 1);
            extend(poles.front());
            extend(poles.back());
            if (poles.size() <= 2)
                continue;
            for (unsigned axis = 0; axis < 3; ++axis) {
                std::vector<double> values;
                roots(tangent(poles, axis), 0, 1, values, root_budget);
                for (auto t : values)
                    extend(evaluate(poles, t));
            }
        }
    }
    void region(const Json &region, const Matrix4 &matrix, unsigned depth = 0) {
        require(depth < 80, "loft cap region depth");
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
             {"range_method", "rational_bezier_numerical_extrema"},
             {"native_root_solver_reproduced", false}};
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
                    {"root_parameter_tolerance", 8 * std::numeric_limits<double>::epsilon()}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
