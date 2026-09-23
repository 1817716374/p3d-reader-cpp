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
double boundary_polygon_length(Curve &c) {
    // The native distance helper operates in place: reciprocal-weight scaling,
    // Cartesian distances, then reweighting. Even the round trip is observable.
    if (c.rational)
        for (auto &h : c.poles) {
            require(h[3] != 0, "native periodic opening cannot deweight a zero control weight");
            const double inverse = 1 / h[3];
            require(std::isfinite(inverse), "native periodic opening reciprocal weight overflow");
            for (unsigned k = 0; k < 3; ++k) {
                h[k] *= inverse;
                require(std::isfinite(h[k]), "native periodic opening deweight overflow");
            }
        }
    double length = 0;
    for (std::size_t i = 1; i < c.poles.size(); ++i) {
        const auto &a = c.poles[i - 1], &b = c.poles[i];
        const double x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2];
        length += std::sqrt((x * x + y * y) + z * z);
    }
    require(std::isfinite(length), "native periodic opening polygon distance overflow");
    if (c.rational)
        for (auto &h : c.poles)
            for (unsigned k = 0; k < 3; ++k) {
                h[k] *= h[3];
                require(std::isfinite(h[k]), "native periodic opening reweight overflow");
            }
    return length;
}
double seam_tolerance(Curve &c, double length, bool boundary) {
    double tolerance = boundary ? (1e-10 / std::max(1., boundary_polygon_length(c))) * length
                                : length * 1e-10 / std::max(1., c.polygon_length());
    for (std::size_t i = c.degree + 1; i <= c.knots.size() - c.degree - 1; ++i) {
        const double delta = std::abs(c.knots[i] - c.knots[i - 1]);
        if (delta < tolerance && delta > 1e-5)
            tolerance = delta / 10;
    }
    return std::max(1e-14, tolerance);
}
bool special_seam(const Curve &c) {
    Point3 low, high;
    const double largest = std::numeric_limits<double>::max();
    low.fill(largest);
    high.fill(-largest);
    for (auto h : c.poles) {
        if (c.rational && std::abs(h[3]) <= 1e-12)
            continue;
        Point3 p;
        if (!c.rational && (h[0] == largest || h[1] == largest || h[2] == largest))
            continue;
        const double inverse = c.rational ? 1 / h[3] : 1;
        for (unsigned k = 0; k < 3; ++k) {
            p[k] = h[k] * inverse;
            require(std::isfinite(p[k]), "native periodic seam range overflow");
        }
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
Curve open_periodic_impl(const BsplineCurve &source, unsigned limit, Json *report, bool boundary) {
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
        if (!boundary)
            cartesian(h);
        c.poles.push_back(h);
    }
    const auto domain = source.knot_domain();
    const double length = domain[1] - domain[0];
    // The native callers pass the knot parameter zero, not fraction zero.
    require(domain[0] <= 0 && domain[1] >= 0,
            "native periodic opening parameter zero is outside the source knot domain");
    auto check_result = [&] {
        if (!boundary) {
            c.check(limit);
            return;
        }
        // Validate working storage directly, without serializing and reparsing
        // a second complete JSON control array before the caller consumes it.
        require(c.knots.size() == c.poles.size() + source.order() &&
                    std::is_sorted(c.knots.begin(), c.knots.end()),
                "native periodic opening output knots");
        for (double k : c.knots)
            require(std::isfinite(k), "native periodic opening nonfinite output knot");
        for (const auto &h : c.poles)
            for (double value : h)
                require(std::isfinite(value), "native periodic opening nonfinite output control");
        const double span = c.knots[c.poles.size()] - c.knots[c.degree];
        require(span > 0 && std::isfinite(span), "native periodic opening output domain");
    };
    if (source.periodic_pole_shift() != 0 && special_seam(c)) {
        const auto first = std::size_t(source.order() / 2);
        c.knots = std::vector<double>(c.knots.begin() + first,
                                      c.knots.begin() + first + c.poles.size() + source.order());
        const double a = c.knots[c.degree], b = c.knots[c.poles.size()];
        require(b > a, "special periodic loft knot domain");
        // The direct native copy/strip branch never calls knot normalization.
        if (!boundary)
            for (auto &k : c.knots)
                k = (k - a) / (b - a);
        check_result();
        if (report)
            *report = {{"method", "strip_exterior_knots"},
                       {"effective_seam_knot", a},
                       {"inserted_knot_count", 0},
                       {"pole_rotation", 0}};
        return c;
    }
    const double tolerance = seam_tolerance(c, length, boundary);
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
    check_result();
    if (report)
        *report = {{"method", source.periodic_pole_shift() ? "cyclic_seam_fallback"
                                                           : "cyclic_knot_insertion"},
                   {"effective_seam_knot", a},
                   {"knot_tolerance", tolerance},
                   {"inserted_knot_count", added},
                   {"pole_rotation", first}};
    return c;
}
} // namespace
Curve open_periodic(const BsplineCurve &source, unsigned limit, Json *report) {
    return open_periodic_impl(source, limit, report, false);
}
Curve open_periodic_boundary(const BsplineCurve &source, unsigned limit, Json *report) {
    return open_periodic_impl(source, limit, report, true);
}
} // namespace p3d::loft_detail
