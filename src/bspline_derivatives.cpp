#include "internal.hpp"

namespace p3d {
namespace {
using HPoint = std::array<double, 4>;

void finite(double x) {
    require(std::isfinite(x), "B-spline native derivatives: non-finite arithmetic");
}

// The native tolerance query temporarily unweights and reweights its poles.
// Reproduce that working representation without mutating the source object.
double knot_tolerance(const BsplineCurve &curve, std::vector<Point3> &working) {
    if (curve.rational()) {
        for (std::size_t i = 0; i < working.size(); ++i) {
            require(curve.weights()[i] != 0, "B-spline native derivatives: zero control weight");
            const double inverse = 1 / curve.weights()[i];
            finite(inverse);
            for (auto &x : working[i]) {
                x *= inverse;
                finite(x);
            }
        }
    }
    double length = 0;
    for (std::size_t i = 1; i < working.size(); ++i) {
        const auto &a = working[i - 1], &b = working[i];
        const double x = b[0] - a[0], y = b[1] - a[1], z = b[2] - a[2];
        length += std::sqrt(x * x + y * y + z * z);
        finite(length);
    }
    if (curve.rational())
        for (std::size_t i = 0; i < working.size(); ++i)
            for (auto &x : working[i]) {
                x *= curve.weights()[i];
                finite(x);
            }
    const auto domain = curve.knot_domain();
    double tolerance = (domain[1] - domain[0]) * (1e-10 / std::max(1., length));
    const auto &knots = curve.knots();
    for (std::size_t i = curve.order(); i <= knots.size() - curve.order(); ++i) {
        const double gap = knots[i] - knots[i - 1];
        if (gap < tolerance && gap > 1e-5)
            tolerance = gap / 10;
    }
    return std::max(1e-14, tolerance);
}

int native_pole_shift(const BsplineCurve &curve, const std::vector<Point3> &working) {
    const int candidate = curve.periodic_pole_shift();
    if (!candidate)
        return 0;
    Point3 low, high;
    low.fill(std::numeric_limits<double>::max());
    high.fill(-std::numeric_limits<double>::max());
    for (std::size_t i = 0; i < working.size(); ++i) {
        if (curve.rational() && std::abs(curve.weights()[i]) <= 1e-12)
            continue;
        const double inverse = curve.rational() ? 1 / curve.weights()[i] : 1.;
        for (unsigned axis = 0; axis < 3; ++axis) {
            const double x = working[i][axis] * inverse;
            finite(x);
            low[axis] = std::min(low[axis], x);
            high[axis] = std::max(high[axis], x);
        }
    }
    double extent = 0;
    for (unsigned axis = 0; axis < 3; ++axis)
        extent = std::max(extent, std::abs(high[axis] - low[axis]));
    const double tolerance = std::min(1., extent * 1e-5);
    for (unsigned axis = 0; axis < 3; ++axis)
        if (std::abs(working.front()[axis] - working.back()[axis]) > tolerance)
            return 0;
    if (curve.rational() && std::abs(curve.weights().front() - curve.weights().back()) >= 1e-10)
        return 0;
    return candidate;
}
} // namespace

std::vector<Point3> BsplineCurve::native_derivatives_at(double fraction,
                                                        unsigned derivative_order) const {
    require(order() <= 26 && derivative_order <= 24,
            "B-spline native derivatives: unsupported order");
    require(std::isfinite(fraction), "B-spline native derivatives: non-finite fraction");
    const auto domain = knot_domain();
    const double width = domain[1] - domain[0];
    double u = domain[0] + width * fraction;
    finite(u);
    if (closed()) {
        // Avoid the native unbounded repeated-add/subtract loop. Exact positive
        // periods retain the upper endpoint; negative periods retain the lower.
        if (u < domain[0]) {
            const double distance = domain[0] - u;
            finite(distance);
            const double remainder = std::fmod(distance, width);
            u = remainder == 0 ? domain[0] : domain[1] - remainder;
        } else if (u > domain[1]) {
            const double distance = u - domain[1];
            finite(distance);
            u = domain[0] + std::fmod(distance, width);
            if (u == domain[0])
                u = domain[1];
        }
    } else
        u = std::clamp(u, domain[0], domain[1]);

    auto working = poles_;
    const double tolerance = knot_tolerance(*this, working);
    const int pole_shift = native_pole_shift(*this, working);
    const auto &k = knots();
    const auto end = u == domain[1] ? std::lower_bound(k.begin(), k.end(), u)
                                    : std::upper_bound(k.begin(), k.end(), u);
    require(end != k.begin() && end != k.end(), "B-spline native derivatives: span");
    const auto span = std::size_t(end - k.begin() - 1);
    const unsigned degree = order() - 1;
    require(span >= degree && span + degree < k.size(), "B-spline native derivatives: span extent");
    const auto first = span - degree;
    std::array<HPoint, 26> p{};
    std::array<double, 26> left{}, right{};
    for (unsigned i = 0; i <= degree; ++i) {
        auto index = std::int64_t(first + i) + pole_shift;
        if (closed()) {
            index %= std::int64_t(working.size());
            if (index < 0)
                index += std::int64_t(working.size());
        }
        require(index >= 0 && std::uint64_t(index) < working.size(),
                "B-spline native derivatives: pole index");
        const auto pole = std::size_t(index);
        p[i] = {working[pole][0], working[pole][1], working[pole][2],
                rational() ? weights_[pole] : 1.};
        if (i) {
            left[i] = u - k[first + i];
            right[i] = k[first + i + degree] - u;
            finite(left[i]);
            finite(right[i]);
        }
    }
    const bool reverse = right[1] <= left[degree];
    if (reverse) {
        std::reverse(p.begin(), p.begin() + order());
        const auto old_left = left;
        for (unsigned i = 1; i <= degree; ++i)
            left[i] = right[degree + 1 - i];
        for (unsigned i = 1; i <= degree; ++i)
            right[i] = old_left[degree + 1 - i];
    }
    auto denominator = [&](double value) {
        finite(value);
        require(value >= tolerance, "B-spline native derivatives: knot tolerance failure");
        return value;
    };
    const unsigned components = rational() ? 4 : 3;
    for (unsigned level = 1; level <= degree; ++level)
        for (unsigned j = 0; j <= degree - level; ++j) {
            const double a = right[j + 1], b = left[j + level];
            const double d = denominator(a + b);
            for (unsigned axis = 0; axis < components; ++axis) {
                p[j][axis] = (a * p[j][axis] + b * p[j + 1][axis]) / d;
                finite(p[j][axis]);
            }
        }
    const auto count = std::min(derivative_order, degree);
    for (unsigned level = 1; level <= count; ++level)
        for (unsigned j = count; j >= level; --j) {
            const double d = denominator(right[j - level + 1] / (degree - level + 1));
            for (unsigned axis = 0; axis < components; ++axis) {
                const double difference =
                    reverse ? p[j - 1][axis] - p[j][axis] : p[j][axis] - p[j - 1][axis];
                p[j][axis] = difference / d;
                finite(p[j][axis]);
            }
        }
    std::vector<Point3> result(derivative_order + 1);
    for (unsigned i = 0; i <= count; ++i)
        std::copy_n(p[i].begin(), 3, result[i].begin());
    if (rational()) {
        require(p[0][3] != 0, "B-spline native derivatives: zero evaluated weight");
        const double inverse = 1 / p[0][3];
        finite(inverse);
        for (unsigned i = 0; i <= derivative_order; ++i) {
            Point3 sum{};
            std::uint64_t binomial = 1;
            for (unsigned j = 1; j <= i; ++j) {
                binomial = binomial * (i - j + 1) / j;
                const double weight = j <= count ? p[j][3] : 0.;
                const double factor = double(binomial) * weight;
                for (unsigned axis = 0; axis < 3; ++axis)
                    sum[axis] += factor * result[i - j][axis];
            }
            for (unsigned axis = 0; axis < 3; ++axis) {
                result[i][axis] = (result[i][axis] - sum[axis]) * inverse;
                finite(result[i][axis]);
            }
        }
    }
    return result;
}
} // namespace p3d
