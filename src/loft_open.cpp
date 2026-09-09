#include "loft_curve.hpp"
namespace p3d::loft_detail {
namespace {
// Boehm insertion on a finite lift of the cyclic curve, including its two
// cutting parameters. No Bezier expansion or extra interior knots are needed.
void clamp(std::vector<double> &u, std::vector<H> &poles, unsigned p, double t) {
    auto count = unsigned(std::count(u.begin(), u.end(), t));
    require(count <= p + 1, "periodic loft seam multiplicity");
    while (count < p + 1) {
        const auto k = std::size_t(std::upper_bound(u.begin(), u.end(), t) - u.begin() - 1);
        require(k >= p && k - count < poles.size(), "periodic loft insertion span");
        std::vector<H> q(poles.size() + 1);
        for (std::size_t i = 0; i <= k - p; ++i)
            q[i] = poles[i];
        for (std::size_t i = k - count; i < poles.size(); ++i)
            q[i + 1] = poles[i];
        for (std::size_t i = k - p + 1; i <= k - count; ++i) {
            const double den = u[i + p] - u[i];
            require(den > 0, "periodic loft insertion interval");
            const double a = (t - u[i]) / den;
            for (unsigned j = 0; j < 4; ++j)
                q[i][j] = (1 - a) * poles[i - 1][j] + a * poles[i][j];
        }
        poles = std::move(q);
        u.insert(u.begin() + k + 1, t);
        ++count;
    }
}
bool special_seam(const Curve &c) {
    Point3 low, high;
    low.fill(std::numeric_limits<double>::infinity());
    high.fill(-std::numeric_limits<double>::infinity());
    for (auto h : c.poles) {
        if (c.rational && h[3] <= 1e-12)
            continue;
        const auto p = cartesian(h);
        for (unsigned k = 0; k < 3; ++k) {
            low[k] = std::min(low[k], p[k]);
            high[k] = std::max(high[k], p[k]);
        }
    }
    double range = 0;
    for (unsigned k = 0; k < 3; ++k)
        range = std::max(range, std::abs(high[k] - low[k]));
    const double tolerance = std::min(1., range * 1e-5);
    // The native endpoint test uses stored weighted XYZ; only the range uses
    // deweighted points. The separate endpoint-weight comparison is strict.
    for (unsigned k = 0; k < 3; ++k)
        if (std::abs(c.poles.front()[k] - c.poles.back()[k]) > tolerance)
            return false;
    return !c.rational || std::abs(c.poles.front()[3] - c.poles.back()[3]) < 1e-10;
}
} // namespace
Curve open_periodic(const BsplineCurve &source, unsigned limit) {
    require(source.closed() && source.order() >= 2 && source.order() <= 26 &&
                source.poles().size() <= limit,
            "periodic loft degree/control budget");
    Curve c;
    c.degree = source.order() - 1;
    c.rational = source.rational();
    c.knots = source.knots();
    for (std::size_t i = 0; i < source.poles().size(); ++i) {
        const auto p = source.poles()[i];
        H h{p[0], p[1], p[2], source.rational() ? source.weights()[i] : 1};
        cartesian(h);
        c.poles.push_back(h);
    }
    const auto domain = source.knot_domain();
    const double length = domain[1] - domain[0];
    // The native callers pass the knot parameter zero, not fraction zero.
    require(domain[0] <= 0 && domain[1] >= 0,
            "native periodic opening parameter zero is outside the source knot domain");
    if (source.periodic_pole_shift() != 0) {
        require(special_seam(c),
                "special periodic loft seam fallback opening is not yet supported");
        const auto first = std::size_t(source.order() / 2);
        c.knots = std::vector<double>(c.knots.begin() + first,
                                      c.knots.begin() + first + c.poles.size() + source.order());
        const double a = c.knots[c.degree], b = c.knots[c.poles.size()];
        require(b > a, "special periodic loft knot domain");
        for (auto &k : c.knots)
            k = (k - a) / (b - a);
        c.check(limit);
        return c;
    }
    const auto n = c.poles.size();
    const auto multiplicity = std::size_t(std::count(c.knots.begin(), c.knots.end(), 0.));
    require(multiplicity <= source.order() && n + source.order() - multiplicity <= limit,
            "periodic opening control budget/multiplicity");
    // Ordinary periodic storage repeats knots by N and the active period.
    // Inconsistent exterior knots cannot be represented by this finite lift.
    for (std::size_t i = n; i < c.knots.size(); ++i) {
        const double expected = c.knots[i - n] + length;
        const double roundoff = 16 * std::numeric_limits<double>::epsilon() *
                                std::max({1., std::abs(expected), std::abs(c.knots[i])});
        require(std::abs(c.knots[i] - expected) <= roundoff,
                "inconsistent periodic loft exterior knots are not supported");
    }
    const double tolerance = std::max(1e-14, length * 1e-10 / std::max(1., c.polygon_length()));
    for (double k : c.knots)
        require(k == 0 || std::abs(k) > tolerance,
                "near-coincident periodic seam knots require native tolerance handling");
    auto knots = c.knots;
    for (std::size_t i = 0; i < n; ++i)
        knots.push_back(knots[knots.size() - n] + length);
    std::vector<H> poles;
    poles.reserve(knots.size() - source.order());
    for (std::size_t i = 0; i < knots.size() - source.order(); ++i)
        poles.push_back(c.poles[i % n]);
    clamp(knots, poles, c.degree, 0);
    clamp(knots, poles, c.degree, length);
    const auto first =
        std::size_t(std::lower_bound(knots.begin(), knots.end(), 0.) - knots.begin());
    const auto end =
        std::size_t(std::upper_bound(knots.begin(), knots.end(), length) - knots.begin());
    require(end > first + source.order(), "periodic loft cut interval");
    const auto count = end - first - source.order();
    require(count == n + source.order() - multiplicity && first + count <= poles.size(),
            "periodic loft native control count");
    c.knots = std::vector<double>(knots.begin() + first, knots.begin() + end);
    c.poles = std::vector<H>(poles.begin() + first, poles.begin() + first + count);
    for (auto &k : c.knots)
        k /= length;
    c.check(limit);
    return c;
}
} // namespace p3d::loft_detail
