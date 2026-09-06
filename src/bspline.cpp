#include "internal.hpp"
namespace p3d {
namespace {
std::vector<double> numbers(const Json &table, const char *key) {
    const auto &value = table.at(key);
    if (value.is_null())
        return {};
    require(value.is_array(), std::string("BGFB B-spline array: ") + key);
    std::vector<double> result;
    result.reserve(value.size());
    for (const auto &v : value) {
        require(v.is_number(), std::string("BGFB B-spline number: ") + key);
        double x = v.get<double>();
        require(std::isfinite(x), std::string("BGFB B-spline finite value: ") + key);
        result.push_back(x);
    }
    return result;
}
} // namespace
BsplineCurve BsplineCurve::from_bgfb(const Json &table) {
    require(table.at("_type") == "BsplineCurve", "expected BGFB BsplineCurve table");
    BsplineCurve c;
    const auto &order = table.at("order");
    require(order.is_number_integer() && order >= 2 && order <= std::numeric_limits<int>::max(),
            "BGFB B-spline order");
    c.order_ = order.get<unsigned>();
    require(table.at("closed").is_boolean(), "BGFB B-spline closed flag");
    c.closed_ = table.at("closed").get<bool>();
    const auto flat = numbers(table, "poles");
    require(flat.size() % 3 == 0 && flat.size() / 3 >= c.order_, "BGFB B-spline pole count/order");
    for (std::size_t i = 0; i < flat.size(); i += 3)
        c.poles_.push_back({flat[i], flat[i + 1], flat[i + 2]});
    const auto n = c.poles_.size();
    c.weights_ = numbers(table, "weights");
    require(c.weights_.empty() || c.weights_.size() == n, "BGFB B-spline weight count");
    c.source_knots_ = numbers(table, "knots");
    const std::size_t count = n + (c.closed_ ? 2 * std::size_t(c.order_) - 1 : c.order_);
    require(c.source_knots_.empty() || c.source_knots_.size() == count, "BGFB B-spline knot count");
    c.knots_ = c.source_knots_;
    if (c.knots_.empty()) {
        c.knots_.resize(count, 0);
        const auto interior = c.closed_ ? n - 1 : n - c.order_;
        const double step = 1.0 / double(interior + 1);
        double value = 0;
        for (std::size_t i = 0; i < interior; ++i) {
            value += step;
            c.knots_[c.order_ + i] = value;
        }
        for (std::size_t i = 0; i < c.order_; ++i) {
            if (c.closed_) {
                c.knots_[i] = c.knots_[i + interior + 1] - 1;
                c.knots_[c.order_ + interior + i] = c.knots_[c.order_ - 1 + i] + 1;
            } else
                c.knots_[c.order_ + interior + i] = 1;
        }
    }
    require(std::is_sorted(c.knots_.begin(), c.knots_.end()), "BGFB B-spline decreasing knots");
    const auto domain = c.knot_domain();
    require(domain[0] < domain[1] && std::isfinite(domain[1] - domain[0]),
            "BGFB B-spline empty or unrepresentable domain");
    // Native closed, clamped-like convention: exactly 'order' near-zero knots
    // straddle index order-1, with floor(order/2) on its right. Keep the source
    // knots/poles intact; only the evaluator's cyclic pole index is shifted.
    const auto d = std::size_t(c.order_ - 1);
    if (c.closed_ && c.knots_[d] == 0) {
        std::size_t right = 0, left = 0;
        while (d + 1 + right < 2 * std::size_t(c.order_) &&
               std::abs(c.knots_[d + 1 + right]) <= 1e-7)
            ++right;
        for (std::size_t i = d; i > 0 && std::abs(c.knots_[i - 1]) <= 1e-7; --i)
            ++left;
        if (right == c.order_ / 2 && left + 1 + right == c.order_)
            c.pole_shift_ = -int(c.order_ / 2);
    }
    return c;
}
std::array<double, 2> BsplineCurve::knot_domain() const {
    return {knots_[order_ - 1], knots_[knots_.size() - order_]};
}
std::array<double, 4> BsplineCurve::homogeneous_at(double fraction) const {
    require(std::isfinite(fraction) && fraction >= 0 && fraction <= 1,
            "B-spline fraction must be in [0, 1]");
    const auto domain = knot_domain();
    const double u = fraction == 0   ? domain[0]
                     : fraction == 1 ? domain[1]
                                     : (1 - fraction) * domain[0] + fraction * domain[1];
    // Choose a nonempty span even when several endpoint knots coincide.
    const auto end = u == domain[1] ? std::lower_bound(knots_.begin(), knots_.end(), u)
                                    : std::upper_bound(knots_.begin(), knots_.end(), u);
    require(end != knots_.begin() && end != knots_.end(), "B-spline span");
    const auto span = std::size_t(end - knots_.begin() - 1);
    const std::size_t degree = order_ - 1;
    require(span >= degree && span + degree < knots_.size(), "B-spline span extent");
    std::vector<std::array<double, 4>> work(order_);
    for (std::size_t j = 0; j <= degree; ++j) {
        auto index = std::int64_t(span - degree + j) + pole_shift_;
        if (closed_) {
            index %= std::int64_t(poles_.size());
            if (index < 0)
                index += std::int64_t(poles_.size());
        }
        require(index >= 0 && std::uint64_t(index) < poles_.size(), "B-spline pole index");
        const auto &p = poles_[std::size_t(index)];
        work[j] = {p[0], p[1], p[2], rational() ? weights_[std::size_t(index)] : 1};
    }
    for (std::size_t r = 1; r <= degree; ++r)
        for (std::size_t j = degree; j >= r; --j) {
            const auto i = span - degree + j;
            const double length = knots_[i + order_ - r] - knots_[i];
            require(length > 0 && std::isfinite(length), "B-spline degenerate knot interval");
            const double a = (u - knots_[i]) / length;
            for (unsigned axis = 0; axis < 4; ++axis)
                work[j][axis] = (1 - a) * work[j - 1][axis] + a * work[j][axis];
        }
    for (double x : work.back())
        require(std::isfinite(x), "B-spline non-finite homogeneous result");
    return work.back();
}
Point3 BsplineCurve::point_at(double fraction) const {
    const auto h = homogeneous_at(fraction);
    require(h[3] != 0, "B-spline zero evaluated weight");
    Point3 p{h[0] / h[3], h[1] / h[3], h[2] / h[3]};
    for (double x : p)
        require(std::isfinite(x), "B-spline non-finite Cartesian result");
    return p;
}
} // namespace p3d
