#include "loft_curve.hpp"
namespace p3d::loft_detail {
namespace {
// Native cyclic insertion changes a local K-pole block. Its replacement can
// straddle either end of the stored pole array; it does not regenerate the
// exterior knots or extend a second copy of the complete curve.
void insert_cyclic(Curve &c, double t, unsigned added) {
    using Index = std::ptrdiff_t;
    const Index n = c.poles.size(), order = c.degree + 1;
    const Index b = std::upper_bound(c.knots.begin(), c.knots.end(), t) - c.knots.begin();
    const Index start = b - order;
    require(start > -order && start <= n, "periodic loft insertion control span");
    std::vector<H> block(order + added);
    for (Index i = 0; i < order; ++i)
        block[added + i] = c.poles[(start + i + n) % n];
    for (unsigned step = 0; step < added; ++step) {
        block[step] = block[added];
        const Index width = c.degree - step;
        require(b >= width && b + width <= Index(c.knots.size()),
                "periodic loft insertion knot span");
        for (Index j = 0; j < width; ++j) {
            const double lo = c.knots[b - width + j], den = c.knots[b + j] - lo;
            require(den > 0, "periodic loft insertion interval");
            const double a = (t - lo) / den;
            for (unsigned axis = 0; axis < (c.rational ? 4u : 3u); ++axis)
                block[added + j][axis] += a * (block[added + j + 1][axis] - block[added + j][axis]);
        }
    }
    std::vector<H> poles(n + added);
    auto copy = [&](const std::vector<H> &from, Index first, Index count, Index at) {
        require(first >= 0 && count >= 0 && first + count <= Index(from.size()) && at >= 0 &&
                    at + count <= Index(poles.size()),
                "periodic loft cyclic control copy");
        std::copy_n(from.begin() + first, count, poles.begin() + at);
    };
    if (start < 0) {
        copy(block, -start, order + added + start, 0);
        copy(c.poles, start + order, n - order, start + order + added);
        copy(block, 0, -start, n + added + start);
    } else if (start > n - order) {
        const Index wrap = start + order - n;
        copy(block, order + added - wrap, wrap, 0);
        copy(c.poles, wrap, n - order, wrap);
        copy(block, 0, n - start + added, start);
    } else {
        copy(c.poles, 0, start, 0);
        copy(block, 0, order + added, start);
        copy(c.poles, start + order, n - start - order, start + order + added);
    }
    c.poles = std::move(poles);
    c.knots.insert(c.knots.begin() + b, added, t);
}
double seam_tolerance(const Curve &c, double length) {
    double tolerance = length * 1e-10 / std::max(1., c.polygon_length());
    for (std::size_t i = c.degree + 1; i <= c.knots.size() - c.degree - 1; ++i) {
        const double delta = std::abs(c.knots[i] - c.knots[i - 1]);
        if (delta < tolerance && delta > 1e-5)
            tolerance = delta / 10;
    }
    return std::max(1e-14, tolerance);
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
Curve open_periodic(const BsplineCurve &source, unsigned limit, Json *report) {
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
    if (source.periodic_pole_shift() != 0 && special_seam(c)) {
        const auto first = std::size_t(source.order() / 2);
        c.knots = std::vector<double>(c.knots.begin() + first,
                                      c.knots.begin() + first + c.poles.size() + source.order());
        const double a = c.knots[c.degree], b = c.knots[c.poles.size()];
        require(b > a, "special periodic loft knot domain");
        for (auto &k : c.knots)
            k = (k - a) / (b - a);
        c.check(limit);
        if (report)
            *report = {{"method", "strip_exterior_knots"},
                       {"effective_seam_knot", a},
                       {"inserted_knot_count", 0},
                       {"pole_rotation", 0}};
        return c;
    }
    const double tolerance = seam_tolerance(c, length);
    double t = 0;
    unsigned multiplicity = 0;
    // Each match updates t before the next comparison. The original knot
    // values remain intact, including distinct values in the same cluster.
    for (double k : c.knots) {
        if (std::abs(k - t) <= tolerance) {
            t = k;
            ++multiplicity;
        } else if (multiplicity)
            break;
    }
    const auto added = source.order() > multiplicity ? source.order() - multiplicity : 0;
    require(c.poles.size() + added <= limit, "periodic opening control budget");
    if (added)
        insert_cyclic(c, t, added);
    std::size_t upper = 0;
    // This search still uses the requested zero, not the snapped parameter.
    while (upper < c.knots.size() && c.knots[upper] <= tolerance)
        ++upper;
    require(upper >= source.order(), "periodic loft opening knot span");
    const auto first = upper - source.order(), count = c.poles.size();
    require(first <= count, "periodic loft opening control rotation");
    const auto stop = count + c.degree;
    std::vector<double> knots(c.knots.begin() + first, c.knots.begin() + stop);
    for (std::size_t i = c.degree; knots.size() < count + source.order(); ++i) {
        require(i < c.knots.size(), "periodic loft opening wrap knot span");
        knots.push_back(c.knots[i] + length);
    }
    c.knots = std::move(knots);
    std::rotate(c.poles.begin(), c.poles.begin() + first, c.poles.end());
    const double a = c.knots[c.degree], b = c.knots[count];
    require(b - a >= 1e-10, "native periodic opening knot domain is too short to normalize");
    for (auto &k : c.knots)
        k = (k - a) / (b - a);
    // Native open-curve normalization explicitly fills the trailing K knots
    // with one. It does not similarly replace the leading exterior knots.
    std::fill(c.knots.begin() + count, c.knots.end(), 1.);
    c.check(limit);
    if (report)
        *report = {{"method", source.periodic_pole_shift() ? "cyclic_seam_fallback"
                                                           : "cyclic_knot_insertion"},
                   {"effective_seam_knot", a},
                   {"knot_tolerance", tolerance},
                   {"inserted_knot_count", added},
                   {"pole_rotation", first}};
    return c;
}
} // namespace p3d::loft_detail
