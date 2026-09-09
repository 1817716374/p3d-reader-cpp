#include "internal.hpp"
namespace p3d {
namespace {
double finite(double v) {
    require(std::isfinite(v), "native spiral fit numeric overflow");
    return v;
}
// Both the scalar second-derivative system and the final pole system are
// tridiagonal. Solve all coordinate components with one factorization.
template <std::size_t K>
std::vector<std::array<double, K>> solve(const std::vector<double> &a, std::vector<double> b,
                                         const std::vector<double> &c,
                                         std::vector<std::array<double, K>> rhs) {
    for (std::size_t i = 0; i < b.size(); ++i) {
        require(finite(b[i]) != 0, "native spiral fit singular system");
        if (i + 1 == b.size())
            break;
        const double r = finite(a[i + 1] / b[i]);
        b[i + 1] -= r * c[i];
        for (std::size_t k = 0; k < K; ++k)
            rhs[i + 1][k] -= r * rhs[i][k];
    }
    for (std::size_t i = b.size(); i-- > 0;)
        for (std::size_t k = 0; k < K; ++k) {
            if (i + 1 < b.size())
                rhs[i][k] -= c[i] * rhs[i + 1][k];
            rhs[i][k] = finite(rhs[i][k] / b[i]);
        }
    return rhs;
}
struct ScalarSpline {
    std::vector<double> t;
    std::vector<Point2> y, z;
    Point2 derivative(std::size_t i, double f) const {
        const double h = t[i + 1] - t[i], g = 1 - f;
        Point2 d{};
        for (unsigned k = 0; k < 2; ++k)
            d[k] =
                finite(((1 - 3 * g * g) * z[i][k] + (3 * f * f - 1) * z[i + 1][k]) * (h * h / 6) +
                       y[i + 1][k] - y[i][k]) /
                h;
        return d;
    }
};
ScalarSpline interpolate(const std::vector<double> &s, const std::vector<Point3> &points, Point2 d0,
                         Point2 m0, Point2 d1, Point2 m1) {
    const std::size_t n = s.size(), m = n + 2;
    ScalarSpline out;
    auto append = [&](double t, Point2 p) {
        out.t.push_back(t);
        out.y.push_back(p);
    };
    append(s[0], {points[0][0], points[0][1]});
    append((s[0] + s[1]) * .5,
           {(points[0][0] + points[1][0]) * .5, (points[0][1] + points[1][1]) * .5});
    for (std::size_t i = 1; i + 1 < n; ++i)
        append(s[i], {points[i][0], points[i][1]});
    append((s[n - 2] + s[n - 1]) * .5, {(points[n - 2][0] + points[n - 1][0]) * .5,
                                        (points[n - 2][1] + points[n - 1][1]) * .5});
    append(s.back(), {points.back()[0], points.back()[1]});
    auto &t = out.t;
    auto &y = out.y;
    std::vector<double> a(m), b(m), c(m);
    std::vector<Point2> rhs(m);
    for (std::size_t i = 3; i + 3 < m; ++i) {
        const double l = t[i] - t[i - 1], r = t[i + 1] - t[i];
        a[i] = l / 6;
        b[i] = (l + r) / 3;
        c[i] = r / 6;
        for (unsigned k = 0; k < 2; ++k)
            rhs[i][k] = (y[i + 1][k] - y[i][k]) / r - (y[i][k] - y[i - 1][k]) / l;
    }
    // Start and end each replace three rows. Keep their native order: for
    // three source points the end equations overwrite the shared middle row.
    // Boundary routines reset the auxiliary parameters sequentially. With only
    // two samples their initially coincident midpoints become distinct.
    t[1] = (t[0] + t[2]) * .5;
    const double h0 = t[1] - t[0], q0 = t[2] - t[1], r0 = t[3] - t[2];
    require(h0 > 0 && q0 > 0 && r0 > 0, "native spiral fit collapsed start interval");
    b[0] = 1;
    rhs[0] = m0;
    b[1] = h0;
    c[1] = h0 / 6;
    b[2] = (q0 + r0) / 3;
    c[2] = r0 / 6;
    for (unsigned k = 0; k < 2; ++k) {
        const double A = y[2][k] - y[0][k], B = y[3][k] - y[2][k];
        rhs[1][k] = A / h0 - 2 * d0[k] - 5 * h0 * m0[k] / 6;
        rhs[2][k] = B / r0 - A / q0 + d0[k] + q0 * m0[k] / 3;
    }
    t[m - 2] = (t[m - 1] + t[m - 3]) * .5;
    const double h1 = t[m - 1] - t[m - 2], q1 = t[m - 2] - t[m - 3], r1 = t[m - 3] - t[m - 4];
    for (std::size_t i = 1; i < m; ++i)
        require(finite(t[i]) > t[i - 1], "native spiral fit collapsed auxiliary interval");
    b[m - 1] = 1;
    rhs[m - 1] = m1;
    a[m - 2] = h1 / 6;
    b[m - 2] = h1;
    c[m - 2] = 0;
    a[m - 3] = r1 / 6;
    b[m - 3] = (r1 + q1) / 3;
    c[m - 3] = 0;
    for (unsigned k = 0; k < 2; ++k) {
        const double A = y[m - 1][k] - y[m - 3][k], B = y[m - 3][k] - y[m - 4][k];
        rhs[m - 2][k] = 2 * d1[k] - 5 * h1 * m1[k] / 6 - A / h1;
        rhs[m - 3][k] = h1 * m1[k] / 3 - d1[k] + A / h1 - B / r1;
    }
    out.z = solve(a, std::move(b), c, std::move(rhs));
    for (unsigned k = 0; k < 2; ++k) {
        y[1][k] = finite(y[0][k] + h0 * d0[k] + h0 * h0 * (out.z[1][k] / 6 + m0[k] / 3));
        y[m - 2][k] =
            finite(y[m - 1][k] - h1 * d1[k] + h1 * h1 * (out.z[m - 2][k] / 6 + m1[k] / 3));
    }
    return out;
}
double update_parameters(const ScalarSpline &axis, std::vector<double> &s) {
    double max_change = 0, previous_old = s[0];
    std::size_t span = 0;
    for (std::size_t i = 0; i + 1 < s.size(); ++i) {
        double length = 0;
        const unsigned spans = i == 0 || i + 2 == s.size() ? 2 : 1;
        for (unsigned j = 0; j < spans; ++j, ++span) {
            std::array<double, 21> speed{};
            for (unsigned k = 0; k <= 20; ++k) {
                const auto d = axis.derivative(span, k * .05);
                speed[k] = finite(std::sqrt(d[0] * d[0] + d[1] * d[1]));
            }
            double sum = 0;
            for (unsigned k = 0; k < 20; k += 2)
                sum += (4 * speed[k + 1] + speed[k]) + speed[k + 2];
            length = finite(length + sum * ((axis.t[span + 1] - axis.t[span]) / 60));
        }
        const double old = s[i + 1];
        max_change = std::max(max_change, std::abs(length - (old - previous_old)));
        s[i + 1] = finite(s[i] + length);
        require(s[i + 1] > s[i], "native spiral fit collapsed arc length");
        previous_old = old;
    }
    return max_change;
}
} // namespace
SpiralFit TransitionSpiral::native_fit(unsigned budget) const {
    const auto input = native_fit_input(budget);
    const auto points = input.at("local_points").get<std::vector<Point3>>();
    require(points.size() <= 998, "native spiral fit exceeds 998 source points");
    std::vector<double> s(points.size(), 0);
    for (std::size_t i = 1; i < s.size(); ++i) {
        const double x = points[i][0] - points[i - 1][0], y = points[i][1] - points[i - 1][1];
        s[i] = finite(s[i - 1] + std::sqrt(x * x + y * y));
        require(s[i] > s[i - 1], "native spiral fit coincident samples");
    }
    const double initial_length = s.back(), threshold = initial_length * 1e-6;
    const auto theta = input.at("endpoint_bearings").get<Point2>(),
               radius = input.at("endpoint_radii").get<Point2>();
    const Point2 d0{std::cos(theta[0]), std::sin(theta[0])},
        d1{std::cos(theta[1]), std::sin(theta[1])};
    Point2 m0{}, m1{};
    if (radius[0] != 0)
        m0 = {-d0[1] / radius[0], d0[0] / radius[0]};
    if (radius[1] != 0)
        m1 = {-d1[1] / radius[1], d1[0] / radius[1]};
    ScalarSpline axis;
    unsigned iterations = 0;
    double change = 0;
    do {
        require(iterations < 30, "native spiral fit did not converge in 30 iterations");
        axis = interpolate(s, points, d0, m0, d1, m1);
        change = update_parameters(axis, s);
        ++iterations;
    } while (!(change < threshold));
    // The converged axes precede the last parameter update. Do not refit them
    // with the updated s, and do not use the analytic spiral's derivatives here.
    const auto n = axis.t.size();
    const auto start_d = axis.derivative(0, 0), end_d = axis.derivative(n - 2, 1);
    const double total = axis.t.back();
    std::vector<double> parameters = axis.t;
    for (double &t : parameters)
        t /= total;
    std::vector<double> a(n), b(n, 1), c(n), knots(4, 0);
    std::vector<Point2> rhs = axis.y;
    for (unsigned k = 0; k < 2; ++k) {
        rhs.front()[k] += (parameters[1] - parameters[0]) / 3 * (start_d[k] * total);
        rhs.back()[k] -= (parameters[n - 1] - parameters[n - 2]) / 3 * (end_d[k] * total);
    }
    for (std::size_t i = 1; i + 1 < n; ++i) {
        const double hprev = i == 1 ? 0 : axis.t[i - 1] - axis.t[i - 2],
                     l = axis.t[i] - axis.t[i - 1], r = axis.t[i + 1] - axis.t[i],
                     hnext = i + 2 == n ? 0 : axis.t[i + 2] - axis.t[i + 1];
        a[i] = r * r / (hprev + l + r) / (l + r);
        b[i] = ((hprev + l) * r / (hprev + l + r) + (r + hnext) * l / (l + r + hnext)) / (l + r);
        c[i] = l * l / (l + r + hnext) / (l + r);
        knots.push_back(parameters[i]);
    }
    knots.insert(knots.end(), 4, 1);
    auto poles = solve(a, std::move(b), c, std::move(rhs));
    poles.insert(poles.begin(), axis.y.front());
    poles.push_back(axis.y.back());
    Json flat = Json::array();
    for (const auto &p : poles)
        for (unsigned k = 0; k < 3; ++k)
            flat.push_back(finite(transform_[4 * k] * p[0] + transform_[4 * k + 1] * p[1] +
                                  transform_[4 * k + 3]));
    return {BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                     {"order", 4},
                                     {"closed", false},
                                     {"poles", flat},
                                     {"weights", nullptr},
                                     {"knots", knots}}),
            {{"status", "valid"},
             {"representation", "derived_cubic_bspline"},
             {"iterations", iterations},
             {"iteration_limit", 30},
             {"source_point_count", points.size()},
             {"expanded_local_points", axis.y},
             {"expanded_parameters", parameters},
             {"parameter_length", total},
             {"updated_source_parameters", s},
             {"initial_chord_length", initial_length},
             {"max_interval_length_change", change},
             {"convergence_threshold", threshold},
             {"convergence_is_geometric_error_bound", false},
             {"derived_pole_count", poles.size()}}};
}
} // namespace p3d
