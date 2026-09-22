#include "internal.hpp"
#include "bspline_denominator.hpp"

namespace p3d {
namespace {
struct Interval {
    double lo, hi;
};
Interval exact(double x) {
    return {x, x};
}
Interval outward(double lo, double hi) {
    require(std::isfinite(lo) && std::isfinite(hi), "surface denominator interval overflow");
    const Interval result{std::nextafter(lo, -std::numeric_limits<double>::infinity()),
                          std::nextafter(hi, std::numeric_limits<double>::infinity())};
    require(std::isfinite(result.lo) && std::isfinite(result.hi),
            "surface denominator outward interval overflow");
    return result;
}
Interval add(Interval a, Interval b) {
    if (a.lo == 0 && a.hi == 0)
        return b;
    if (b.lo == 0 && b.hi == 0)
        return a;
    return outward(a.lo + b.lo, a.hi + b.hi);
}
Interval subtract(Interval a, Interval b) {
    if (b.lo == 0 && b.hi == 0)
        return a;
    return outward(a.lo - b.hi, a.hi - b.lo);
}
Interval multiply(Interval a, Interval b) {
    if ((a.lo == 0 && a.hi == 0) || (b.lo == 0 && b.hi == 0))
        return exact(0);
    const std::array<double, 4> products{a.lo * b.lo, a.lo * b.hi, a.hi * b.lo, a.hi * b.hi};
    for (const auto p : products)
        require(std::isfinite(p), "surface denominator interval product overflow");
    return outward(*std::min_element(products.begin(), products.end()),
                   *std::max_element(products.begin(), products.end()));
}
Interval ratio(double t, double a, double b) {
    require(b > a, "surface denominator degenerate insertion interval");
    if (t == a)
        return exact(0);
    if (t == b)
        return exact(1);
    const auto n = subtract(exact(t), exact(a)), d = subtract(exact(b), exact(a));
    require(d.lo > 0, "surface denominator insertion interval below precision");
    auto q = outward(n.lo / d.hi, n.hi / d.lo);
    // Knot insertion in the active window is a convex combination.
    require(t >= a && t <= b, "surface denominator insertion outside knot interval");
    q.lo = std::max(0., q.lo);
    q.hi = std::min(1., q.hi);
    return q;
}
struct Proof {
    std::size_t steps = 0, cells = 0, spans = 0;
    unsigned limit;
    void charge(std::size_t n = 1) {
        require(n <= limit - steps, "surface denominator work budget");
        steps += n;
    }
    Interval blend(Interval a, Interval b, Interval fraction) {
        charge();
        if (a.lo == b.lo && a.hi == b.hi)
            return a;
        if (fraction.lo == 0 && fraction.hi == 0)
            return a;
        if (fraction.lo == 1 && fraction.hi == 1)
            return b;
        return add(multiply(subtract(exact(1), fraction), a), multiply(fraction, b));
    }
    std::vector<std::size_t> active_spans(const BsplineDirection &d) {
        std::vector<std::size_t> out;
        const auto domain = d.knot_domain();
        const auto &knots = d.knots();
        for (std::size_t i = d.order() - 1; i < knots.size() - d.order(); ++i) {
            charge();
            if (knots[i] >= domain[0] && knots[i + 1] <= domain[1] && knots[i] < knots[i + 1])
                out.push_back(i);
        }
        require(!out.empty(), "surface denominator has no active spans");
        return out;
    }
    std::vector<Interval> extract(const BsplineDirection &d, std::size_t span,
                                  std::vector<Interval> values) {
        const std::size_t degree = d.order() - 1;
        const auto &source = d.knots();
        charge(2 * std::size_t(d.order()));
        std::vector<double> knots(source.begin() + span - degree,
                                  source.begin() + span + degree + 2);
        for (double t : {source[span], source[span + 1]}) {
            auto multiplicity = std::size_t(std::count(knots.begin(), knots.end(), t));
            while (multiplicity < degree) {
                const auto k = std::size_t(std::upper_bound(knots.begin(), knots.end(), t) -
                                           knots.begin() - 1);
                require(k >= degree && k - multiplicity < values.size(),
                        "surface denominator insertion span");
                charge(values.size() + 1);
                std::vector<Interval> next(values.size() + 1);
                for (std::size_t i = 0; i <= k - degree; ++i)
                    next[i] = values[i];
                for (std::size_t i = k - multiplicity; i < values.size(); ++i)
                    next[i + 1] = values[i];
                for (std::size_t i = k - degree + 1; i <= k - multiplicity; ++i)
                    next[i] =
                        blend(values[i - 1], values[i], ratio(t, knots[i], knots[i + degree]));
                values = std::move(next);
                knots.insert(knots.begin() + k + 1, t);
                ++multiplicity;
            }
        }
        const auto end = std::size_t(std::upper_bound(knots.begin(), knots.end(), source[span]) -
                                     knots.begin() - 1);
        require(end >= degree && end < values.size(), "surface denominator Bernstein window");
        return {values.begin() + end - degree, values.begin() + end + 1};
    }
    void verify(const std::vector<Interval> &net, std::size_t nu, std::size_t nv,
                unsigned depth = 0) {
        charge(net.size());
        ++cells;
        bool positive = true, negative = true;
        for (auto w : net) {
            positive &= w.lo > 0;
            negative &= w.hi < 0;
        }
        if (positive || negative)
            return;
        bool positive_corner = false, negative_corner = false;
        for (const auto i : {std::size_t(0), nu - 1, (nv - 1) * nu, nv * nu - 1}) {
            const auto w = net[i];
            require(w.lo != 0 || w.hi != 0, "surface denominator zero at patch corner");
            positive_corner |= w.lo > 0;
            negative_corner |= w.hi < 0;
        }
        require(!(positive_corner && negative_corner), "surface denominator crosses zero");
        require(depth < 64, "surface denominator sign unresolved at subdivision depth");
        charge(2 * net.size());
        std::vector<Interval> low(net.size()), high(net.size());
        bool constant_u = true, constant_v = true;
        for (std::size_t j = 0; j < nv; ++j)
            for (std::size_t i = 0; i < nu; ++i) {
                const auto w = net[j * nu + i];
                constant_u &= w.lo == net[j * nu].lo && w.hi == net[j * nu].hi;
                constant_v &= w.lo == net[i].lo && w.hi == net[i].hi;
            }
        // Do not duplicate the same unresolved polynomial along an axis on
        // which all coefficient intervals are already identical.
        const bool split_u = constant_v || (!constant_u && depth % 2 == 0);
        const auto count = split_u ? nu : nv, lines = split_u ? nv : nu;
        for (std::size_t line = 0; line < lines; ++line) {
            std::vector<Interval> work(count);
            auto index = [&](std::size_t i) { return split_u ? line * nu + i : i * nu + line; };
            for (std::size_t i = 0; i < count; ++i)
                work[i] = net[index(i)];
            low[index(0)] = work[0];
            high[index(count - 1)] = work[count - 1];
            for (std::size_t level = 1; level < count; ++level) {
                for (std::size_t i = 0; i < count - level; ++i)
                    work[i] = blend(work[i], work[i + 1], exact(.5));
                low[index(level)] = work[0];
                high[index(count - level - 1)] = work[count - level - 1];
            }
        }
        verify(low, nu, nv, depth + 1);
        verify(high, nu, nv, depth + 1);
    }
};
std::size_t source_index(const BsplineDirection &d, std::size_t span, std::size_t local) {
    auto i = std::int64_t(span - (d.order() - 1) + local) + d.periodic_pole_shift();
    if (d.closed()) {
        i %= std::int64_t(d.pole_count());
        if (i < 0)
            i += std::int64_t(d.pole_count());
    }
    require(i >= 0 && std::uint64_t(i) < d.pole_count(), "surface denominator pole index");
    return std::size_t(i);
}
} // namespace

Json certify_surface_denominator(const BsplineSurface &surface, unsigned max_steps) {
    if (!surface.rational())
        return {{"status", "verified"}, {"method", "polynomial"}};
    const auto &weights = surface.weights();
    const bool positive = weights.front() > 0;
    if (std::all_of(weights.begin(), weights.end(),
                    [&](double w) { return w != 0 && (w > 0) == positive; }))
        return {{"status", "verified"}, {"method", "source_weight_sign"}};
    Json result = {{"status", "unverified"}, {"method", "bernstein_interval_subdivision"}};
    Proof proof{0, 0, 0, max_steps};
    try {
        const auto &u = surface.u(), &v = surface.v();
        const auto us = proof.active_spans(u), vs = proof.active_spans(v);
        const std::size_t nu = u.order(), nv = v.order();
        require(nu <= max_steps / nv, "surface denominator control net budget");
        for (const auto sv : vs)
            for (const auto su : us) {
                proof.charge(nu * nv);
                std::vector<Interval> net(nu * nv);
                for (std::size_t j = 0; j < nv; ++j) {
                    std::vector<Interval> row(nu);
                    for (std::size_t i = 0; i < nu; ++i)
                        row[i] = exact(weights[source_index(v, sv, j) * u.pole_count() +
                                               source_index(u, su, i)]);
                    row = proof.extract(u, su, std::move(row));
                    std::copy(row.begin(), row.end(), net.begin() + j * nu);
                }
                for (std::size_t i = 0; i < nu; ++i) {
                    std::vector<Interval> column(nv);
                    for (std::size_t j = 0; j < nv; ++j)
                        column[j] = net[j * nu + i];
                    column = proof.extract(v, sv, std::move(column));
                    for (std::size_t j = 0; j < nv; ++j)
                        net[j * nu + i] = column[j];
                }
                proof.verify(net, nu, nv);
                ++proof.spans;
            }
        result["status"] = "verified";
    } catch (const std::exception &e) {
        result["reason"] = e.what();
    }
    result["verified_knot_rectangles"] = proof.spans;
    result["visited_cells"] = proof.cells;
    result["work_steps"] = proof.steps;
    return result;
}
} // namespace p3d
