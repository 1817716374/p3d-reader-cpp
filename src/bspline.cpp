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
int integer(const Json &table, const char *key, int minimum = std::numeric_limits<int>::min()) {
    const auto &value = table.at(key);
    require(value.is_number_integer() && value >= minimum &&
                value <= std::numeric_limits<int>::max(),
            std::string("BGFB B-spline integer: ") + key);
    return value.get<int>();
}
bool flag(const Json &table, const char *key) {
    require(table.at(key).is_boolean(), std::string("BGFB B-spline flag: ") + key);
    return table.at(key).get<bool>();
}
std::vector<Point3> poles(const Json &table) {
    const auto flat = numbers(table, "poles");
    require(flat.size() % 3 == 0, "BGFB B-spline XYZ triplets");
    std::vector<Point3> result;
    result.reserve(flat.size() / 3);
    for (std::size_t i = 0; i < flat.size(); i += 3)
        result.push_back({flat[i], flat[i + 1], flat[i + 2]});
    return result;
}
} // namespace
BsplineDirection BsplineDirection::from_data(unsigned order, bool closed, std::size_t n,
                                             std::vector<double> source_knots) {
    require(order > 1 && n >= order, "BGFB B-spline pole count/order");
    BsplineDirection c;
    c.order_ = order;
    c.closed_ = closed;
    c.pole_count_ = n;
    c.source_knots_ = std::move(source_knots);
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
std::array<double, 2> BsplineDirection::knot_domain() const {
    return {knots_[order_ - 1], knots_[knots_.size() - order_]};
}
namespace {
struct BasisTerm {
    std::size_t pole;
    double value;
};
std::vector<BasisTerm> basis(const BsplineDirection &direction, double fraction) {
    require(std::isfinite(fraction) && fraction >= 0 && fraction <= 1,
            "B-spline fraction must be in [0, 1]");
    const auto domain = direction.knot_domain();
    const auto &knots = direction.knots();
    const double u = fraction == 0   ? domain[0]
                     : fraction == 1 ? domain[1]
                                     : (1 - fraction) * domain[0] + fraction * domain[1];
    // Choose a nonempty span even when several endpoint knots coincide.
    const auto end = u == domain[1] ? std::lower_bound(knots.begin(), knots.end(), u)
                                    : std::upper_bound(knots.begin(), knots.end(), u);
    require(end != knots.begin() && end != knots.end(), "B-spline span");
    const auto span = std::size_t(end - knots.begin() - 1);
    const std::size_t degree = direction.order() - 1;
    require(span >= degree && span + degree < knots.size(), "B-spline span extent");
    std::vector<BasisTerm> result(direction.order());
    std::vector<double> left(direction.order()), right(direction.order());
    result[0].value = 1;
    for (std::size_t j = 1; j <= degree; ++j) {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        double saved = 0;
        for (std::size_t r = 0; r < j; ++r) {
            const double length = right[r + 1] + left[j - r];
            require(length > 0 && std::isfinite(length), "B-spline degenerate knot interval");
            const double value = result[r].value;
            result[r].value = saved + (right[r + 1] / length) * value;
            saved = (left[j - r] / length) * value;
        }
        result[j].value = saved;
    }
    for (std::size_t j = 0; j <= degree; ++j) {
        auto index = std::int64_t(span - degree + j) + direction.periodic_pole_shift();
        if (direction.closed()) {
            index %= std::int64_t(direction.pole_count());
            if (index < 0)
                index += std::int64_t(direction.pole_count());
        }
        require(index >= 0 && std::uint64_t(index) < direction.pole_count(), "B-spline pole index");
        result[j].pole = std::size_t(index);
    }
    return result;
}
Point3 cartesian(const std::array<double, 4> &h) {
    require(h[3] != 0, "B-spline zero evaluated weight");
    Point3 p{h[0] / h[3], h[1] / h[3], h[2] / h[3]};
    for (double x : p)
        require(std::isfinite(x), "B-spline non-finite Cartesian result");
    return p;
}
std::array<double, 4> checked_homogeneous(const std::array<double, 4> &h) {
    for (double x : h)
        require(std::isfinite(x), "B-spline non-finite homogeneous result");
    return h;
}
} // namespace
BsplineCurve BsplineCurve::from_bgfb(const Json &table) {
    require(table.at("_type") == "BsplineCurve", "expected BGFB BsplineCurve table");
    BsplineCurve c;
    c.poles_ = p3d::poles(table);
    c.weights_ = numbers(table, "weights");
    require(c.weights_.empty() || c.weights_.size() == c.poles_.size(),
            "BGFB B-spline weight count");
    c.direction_ =
        BsplineDirection::from_data(unsigned(integer(table, "order", 2)), flag(table, "closed"),
                                    c.poles_.size(), numbers(table, "knots"));
    return c;
}
std::array<double, 4> BsplineCurve::homogeneous_at(double fraction) const {
    std::array<double, 4> result{};
    for (const auto &term : basis(direction_, fraction)) {
        for (unsigned a = 0; a < 3; ++a)
            result[a] += term.value * poles_[term.pole][a];
        result[3] += term.value * (rational() ? weights_[term.pole] : 1);
    }
    return checked_homogeneous(result);
}
Point3 BsplineCurve::point_at(double fraction) const {
    return cartesian(homogeneous_at(fraction));
}
BsplineSurface BsplineSurface::from_bgfb(const Json &table) {
    require(table.at("_type") == "BsplineSurface", "expected BGFB BsplineSurface table");
    BsplineSurface s;
    const auto nu = std::size_t(integer(table, "numPolesU", 2));
    const auto nv = std::size_t(integer(table, "numPolesV", 2));
    s.poles_ = p3d::poles(table);
    require(nu <= s.poles_.size() / nv && nu * nv == s.poles_.size(),
            "BGFB B-spline surface pole grid");
    s.weights_ = numbers(table, "weights");
    require(s.weights_.empty() || s.weights_.size() == s.poles_.size(),
            "BGFB B-spline surface weight count");
    s.u_ = BsplineDirection::from_data(unsigned(integer(table, "orderU", 2)),
                                       flag(table, "closedU"), nu, numbers(table, "knotsU"));
    s.v_ = BsplineDirection::from_data(unsigned(integer(table, "orderV", 2)),
                                       flag(table, "closedV"), nv, numbers(table, "knotsV"));
    s.num_rules_u_ = integer(table, "numRulesU");
    s.num_rules_v_ = integer(table, "numRulesV");
    s.hole_origin_ = integer(table, "holeOrigin");
    s.boundaries_ = table.at("boundaries");
    require(s.boundaries_.is_null() ||
                (s.boundaries_.is_object() &&
                 s.boundaries_.value("_type", std::string()) == "CurveVector"),
            "BGFB B-spline surface trim table");
    return s;
}
std::array<double, 4> BsplineSurface::homogeneous_at(double fraction_u, double fraction_v) const {
    const auto bu = basis(u_, fraction_u), bv = basis(v_, fraction_v);
    std::array<double, 4> result{};
    for (const auto &v : bv)
        for (const auto &u : bu) {
            const auto index = v.pole * u_.pole_count() + u.pole;
            const double coefficient = u.value * v.value;
            for (unsigned a = 0; a < 3; ++a)
                result[a] += coefficient * poles_[index][a];
            result[3] += coefficient * (rational() ? weights_[index] : 1);
        }
    return checked_homogeneous(result);
}
Point3 BsplineSurface::point_at(double fraction_u, double fraction_v) const {
    return cartesian(homogeneous_at(fraction_u, fraction_v));
}
} // namespace p3d
