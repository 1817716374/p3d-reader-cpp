#include "loft_curve.hpp"
namespace p3d::loft_detail {
namespace {
bool same_endpoint(const Curve &c) {
    double delta = 0, scale = 0;
    for (unsigned j = 0; j < 3; ++j) {
        delta = std::max(delta, std::abs(c.poles.front()[j] - c.poles.back()[j]));
        scale = std::max({scale, std::abs(c.poles.front()[j]), std::abs(c.poles.back()[j])});
    }
    return delta < 1e-8 || delta < scale * 1e-8 + 1e-8;
}
// Remove the endpoint clamps, then test the overlapping homogeneous poles.
// Failure must leave the input unchanged: the caller uses the other closed
// knot representation, not a partially transformed control polygon.
bool regular(Curve &c) {
    const auto p = c.degree;
    const auto n = c.poles.size() - 1;
    if (n - p < p)
        return false;
    Curve trial = c;
    auto &u = trial.knots;
    auto &h = trial.poles;
    for (unsigned i = 0; i + 1 < p; ++i) {
        u[p - i - 1] = u[p - i] - (u[n - i + 1] - u[n - i]);
        for (int j = int(i); j >= 0; --j) {
            const auto k = p - 1 - i + unsigned(j);
            const double a = (u[p] - u[k]) / (u[p + j + 1] - u[k]);
            if (!std::isfinite(a) || a == 1)
                return false;
            const double b = 1 / (1 - a), d = -a * b;
            for (unsigned axis = 0; axis < (c.rational ? 4u : 3u); ++axis)
                h[j][axis] = h[j][axis] * b + h[j + 1][axis] * d;
        }
    }
    u[0] = u[1] - (u[n - p + 2] - u[n - p + 1]);
    for (unsigned i = 0; i + 1 < p; ++i) {
        u[n + i + 2] = u[n + i + 1] + (u[p + i + 1] - u[p + i]);
        for (int j = int(i); j >= 0; --j) {
            const auto k = n - j;
            const double a = (u[n + 1] - u[k]) / (u[n + 2 + i - j] - u[k]);
            if (!std::isfinite(a) || a == 0)
                return false;
            const double b = 1 / a, d = (a - 1) * b;
            for (unsigned axis = 0; axis < (c.rational ? 4u : 3u); ++axis)
                h[k][axis] = h[k][axis] * b + h[k - 1][axis] * d;
        }
    }
    u[n + p + 1] = u[n + p] + (u[2 * p] - u[2 * p - 1]);
    double maximum = 0;
    for (auto point : h)
        for (unsigned j = 0; j < 3; ++j) {
            require(std::isfinite(point[j]), "nonfinite loft periodic closure control");
            maximum = std::max(maximum, point[j]);
        }
    // These two native tests are asymmetric: XYZ uses the largest positive
    // stored component, and the weight difference is signed, not absolute.
    const double tolerance = maximum * 1e-10;
    const auto count = h.size() - p;
    for (unsigned i = 0; i < p; ++i) {
        const auto &a = h[i], &b = h[count + i];
        if (std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) > tolerance ||
            (c.rational && a[3] - b[3] > 1e-10))
            return false;
    }
    h.resize(count);
    c = std::move(trial);
    return true;
}
void special(Curve &c) {
    const auto n = c.poles.size(), order = std::size_t(c.degree + 1);
    std::vector<double> u(n + 2 * order - 1);
    std::copy_n(c.knots.begin() + (order + 1) / 2, n - 1, u.begin() + order);
    for (std::size_t i = 0; i < order; ++i) {
        u[i] = u[i + n] - 1;
        u[n + order - 1 + i] = u[order - 1 + i] + 1;
    }
    c.knots = std::move(u);
}
} // namespace
Curve close_reopen(Curve c, unsigned limit, Json &report) {
    c.check(limit);
    report = {{"method", "retained_open"}, {"input_pole_count", c.poles.size()}};
    if (c.degree == 1 && c.poles.size() == 2)
        report["reason"] = "two_pole_line";
    else if (std::abs(c.poles.front()[3] - c.poles.back()[3]) >= 1e-10)
        report["reason"] = "endpoint_weight_mismatch";
    else if (!same_endpoint(c))
        report["reason"] = "endpoint_position_mismatch";
    else {
        if (c.degree == 1) {
            const auto n = c.poles.size();
            c.knots[0] = c.knots[1] - (c.knots[n] - c.knots[n - 1]);
            c.knots[n + 1] = c.knots[n] + (c.knots[2] - c.knots[1]);
            c.poles.pop_back();
            report["method"] = "linear_periodic_reopened";
        } else if (regular(c))
            report["method"] = "regular_periodic_reopened";
        else {
            special(c);
            report["method"] = "special_periodic_reopened";
        }
        report["closed_pole_count"] = c.poles.size();
        auto table = c.table();
        table["closed"] = true;
        Json opening;
        c = open_periodic(BsplineCurve::from_bgfb(table), limit, &opening);
        report["opening"] = std::move(opening);
    }
    report["opened_pole_count"] = c.poles.size();
    return c;
}
} // namespace p3d::loft_detail
