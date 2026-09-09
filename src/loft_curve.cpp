#include "loft_curve.hpp"
namespace p3d::loft_detail {
Point3 cartesian(H h) {
    require(h[3] > 0 && std::isfinite(h[3]), "loft requires positive control weights");
    Point3 p{h[0] / h[3], h[1] / h[3], h[2] / h[3]};
    for (double x : p)
        require(std::isfinite(x), "nonfinite loft control point");
    return p;
}
namespace {
struct Group {
    double value;
    std::size_t start, count;
};
std::vector<Group> groups(const std::vector<double> &knots, double tolerance) {
    std::vector<Group> result;
    for (std::size_t i = 0; i < knots.size();) {
        const auto start = i++;
        while (i < knots.size() && knots[i] - knots[i - 1] <= tolerance)
            ++i;
        result.push_back({knots[start], start, i - start});
    }
    return result;
}
// Taylor coefficients of the homogeneous polynomial immediately to the left
// of t. Computing them by de Boor avoids a global, potentially singular solve.
std::vector<H> polynomial(const Curve &c, double t) {
    const auto &u = c.knots;
    const auto end =
        t == 0 ? std::upper_bound(u.begin(), u.end(), t) : std::lower_bound(u.begin(), u.end(), t);
    const auto span = std::size_t(end - u.begin() - 1);
    const auto p = c.degree;
    require(span >= p && span < c.poles.size(), "loft elevation span");
    std::vector<std::vector<H>> d(p + 1, std::vector<H>(p + 1));
    for (unsigned j = 0; j <= p; ++j)
        d[j][0] = c.poles[span - p + j];
    for (unsigned r = 1; r <= p; ++r)
        for (unsigned j = p; j >= r; --j) {
            const auto i = span - p + j;
            const double den = u[i + p - r + 1] - u[i];
            require(den > 0, "loft elevation interval");
            const double a = (t - u[i]) / den;
            for (int k = int(p); k >= 0; --k)
                for (unsigned axis = 0; axis < 4; ++axis) {
                    const double extra = k ? (d[j][k - 1][axis] - d[j - 1][k - 1][axis]) / den : 0;
                    d[j][k][axis] = (1 - a) * d[j - 1][k][axis] + a * d[j][k][axis] + extra;
                }
        }
    return d[p];
}
} // namespace
void Curve::check(unsigned limit) const {
    require(degree >= 1 && degree <= 25 && poles.size() >= degree + 1 && poles.size() <= limit,
            "loft curve degree/control budget");
    require(knots.size() == poles.size() + degree + 1 && std::is_sorted(knots.begin(), knots.end()),
            "loft curve knots");
    for (double k : knots)
        require(std::isfinite(k), "nonfinite loft knot");
    const auto g = groups(knots, 0);
    require(g.size() >= 2 && g.front().value == 0 && g.back().value == 1 &&
                g.front().count == degree + 1 && g.back().count == degree + 1,
            "loft requires clamped normalized curves");
    for (std::size_t i = 1; i + 1 < g.size(); ++i)
        require(g[i].count <= degree, "discontinuous loft curve is not supported");
    for (auto h : poles)
        cartesian(h);
}
Curve Curve::from_bspline(const BsplineCurve &b, unsigned limit) {
    if (b.closed())
        return open_periodic(b, limit);
    Curve c;
    c.degree = b.order() - 1;
    c.rational = b.rational();
    c.knots = b.knots();
    const auto domain = b.knot_domain();
    for (auto &k : c.knots)
        k = (k - domain[0]) / (domain[1] - domain[0]);
    for (std::size_t i = 0; i < b.poles().size(); ++i) {
        const auto p = b.poles()[i];
        c.poles.push_back({p[0], p[1], p[2], b.rational() ? b.weights()[i] : 1});
    }
    c.check(limit);
    return c;
}
double Curve::polygon_length() const {
    double length = 0;
    for (std::size_t i = 1; i < poles.size(); ++i) {
        const auto a = cartesian(poles[i - 1]), b = cartesian(poles[i]);
        length += std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    }
    require(std::isfinite(length), "loft control polygon length overflow");
    return length;
}
double Curve::knot_tolerance() const {
    return std::max(1e-14, 1e-10 / std::max(1., polygon_length()));
}
void Curve::insert(double t, unsigned target, unsigned limit) {
    require(t > 0 && t < 1 && target <= degree, "loft insertion domain/multiplicity");
    auto count = unsigned(std::count(knots.begin(), knots.end(), t));
    while (count < target) {
        require(poles.size() < limit, "loft knot insertion budget");
        const auto k =
            std::size_t(std::upper_bound(knots.begin(), knots.end(), t) - knots.begin() - 1);
        require(k >= degree && k - count < poles.size(), "loft knot insertion span");
        std::vector<H> q(poles.size() + 1);
        for (std::size_t i = 0; i <= k - degree; ++i)
            q[i] = poles[i];
        for (std::size_t i = k - count; i < poles.size(); ++i)
            q[i + 1] = poles[i];
        for (std::size_t i = k - degree + 1; i <= k - count; ++i) {
            const double den = knots[i + degree] - knots[i];
            require(den > 0, "loft knot insertion interval");
            const double a = (t - knots[i]) / den;
            for (unsigned j = 0; j < 4; ++j)
                q[i][j] = (1 - a) * poles[i - 1][j] + a * poles[i][j];
        }
        poles = std::move(q);
        knots.insert(knots.begin() + k + 1, t);
        ++count;
    }
}
void Curve::elevate(unsigned q, unsigned limit) {
    require(q >= degree && q <= 25, "loft elevation degree");
    if (q == degree)
        return;
    Curve out;
    out.degree = q;
    out.rational = rational;
    const auto g = groups(knots, knot_tolerance());
    for (const auto &item : g) {
        const auto n = item.count + q - degree;
        require(out.knots.size() + n <= std::size_t(limit) + q + 1, "loft elevation budget");
        out.knots.insert(out.knots.end(), n, item.value);
    }
    require(out.knots.size() >= 2 * (q + 1), "loft elevation collapsed knot domain");
    out.poles.resize(out.knots.size() - q - 1);
    std::vector<double> choose(degree + 1, 1);
    for (unsigned k = 1; k <= degree; ++k)
        choose[k] = choose[k - 1] * (q - k + 1) / k;
    std::vector<H> coefficients;
    double previous = 0;
    for (std::size_t i = 1; i + 1 < out.poles.size(); ++i) {
        const double t = out.knots[i + q];
        if (coefficients.empty() || t != previous) {
            coefficients = polynomial(*this, t);
            previous = t;
        }
        std::vector<double> symmetric(degree + 1);
        symmetric[0] = 1;
        for (unsigned j = 1; j <= q; ++j)
            for (unsigned k = degree; k > 0; --k)
                symmetric[k] += (out.knots[i + j] - t) * symmetric[k - 1];
        for (unsigned k = 0; k <= degree; ++k)
            for (unsigned axis = 0; axis < 4; ++axis)
                out.poles[i][axis] += coefficients[k][axis] * symmetric[k] / choose[k];
        if (!rational)
            out.poles[i][3] = 1;
    }
    out.poles.front() = poles.front();
    out.poles.back() = poles.back();
    out.check(limit);
    *this = std::move(out);
}
void compatible(std::vector<Curve *> curves, unsigned limit) {
    require(curves.size() >= 2 && curves.size() <= 5000, "loft compatible curve count");
    unsigned degree = 1;
    bool rational = false, bezier = true;
    for (auto c : curves) {
        c->check(limit);
        degree = std::max(degree, c->degree);
        rational |= c->rational;
        bezier &= c->poles.size() == c->degree + 1;
    }
    double tolerance = 1;
    for (auto c : curves) {
        c->elevate(degree, limit);
        c->rational = rational;
        tolerance = std::min(tolerance, c->knot_tolerance());
    }
    if (bezier)
        return;
    std::vector<std::vector<Group>> all;
    for (auto c : curves)
        all.push_back(groups(c->knots, tolerance));
    bool matching = true;
    for (const auto &g : all) {
        if (g.size() != all[0].size()) {
            matching = false;
            break;
        }
        for (std::size_t j = 1; j + 1 < g.size(); ++j)
            matching &= std::abs(g[j].value - all[0][j].value) <= tolerance &&
                        g[j].count == all[0][j].count;
    }
    if (matching) {
        for (std::size_t j = 1; j + 1 < all[0].size(); ++j) {
            double mean = 0;
            for (const auto &g : all)
                mean += g[j].value;
            mean /= curves.size();
            for (std::size_t i = 0; i < curves.size(); ++i)
                std::fill_n(curves[i]->knots.begin() + all[i][j].start, all[i][j].count, mean);
        }
    } else {
        std::size_t cursor = degree + 1;
        while (true) {
            std::vector<std::pair<double, std::size_t>> next;
            for (std::size_t i = 0; i < curves.size(); ++i) {
                require(cursor < curves[i]->knots.size(), "loft compatibility cursor");
                next.emplace_back(curves[i]->knots[cursor], i);
            }
            std::stable_sort(next.begin(), next.end());
            if (next.front().first == 1)
                break;
            std::size_t n = 1;
            double mean = next.front().first;
            while (n < next.size() && next[n].first - next[n - 1].first < tolerance)
                mean += next[n++].first;
            mean /= n;
            require(mean > 0 && mean < 1 && next[n - 1].first < 1,
                    "loft knot cluster reaches an endpoint");
            unsigned multiplicity = 0;
            for (std::size_t i = 0; i < n; ++i) {
                auto &u = curves[next[i].second]->knots;
                auto end = cursor + 1;
                while (end < u.size() && u[end] - u[end - 1] < tolerance)
                    ++end;
                const auto count = unsigned(end - cursor);
                require(count <= degree, "loft knot cluster is discontinuous");
                multiplicity = std::max(multiplicity, count);
                std::fill(u.begin() + cursor, u.begin() + end, mean);
            }
            for (auto c : curves)
                c->insert(mean, multiplicity, limit);
            cursor += multiplicity;
        }
    }
    for (auto c : curves) {
        c->check(limit);
        require(c->knots == curves.front()->knots, "loft compatibility left unequal knots");
    }
}
Curve append(Curve a, Curve b, bool length_weighted, unsigned limit) {
    a.elevate(std::max(a.degree, b.degree), limit);
    b.elevate(a.degree, limit);
    const auto x = cartesian(a.poles.back()), y = cartesian(b.poles.front());
    double distance = 0, extent = 0;
    for (unsigned k = 0; k < 3; ++k) {
        distance = std::max(distance, std::abs(x[k] - y[k]));
        extent = std::max({extent, std::abs(x[k]), std::abs(y[k])});
    }
    require(distance < 1e-8 || distance < extent * 1e-8 + 1e-8,
            "discontinuous composite loft guide is not supported");
    double cut = .5;
    if (length_weighted) {
        const double xlength = a.polygon_length(), ylength = b.polygon_length();
        require(std::isfinite(xlength + ylength), "degenerate composite loft guide length");
        const bool omit_a = xlength < 1e-12 || ylength / xlength > 1e10;
        const bool omit_b = ylength < 1e-12 || xlength / ylength > 1e10;
        require(!(omit_a && omit_b), "both composite loft curves have negligible length");
        if (omit_a)
            return b;
        if (omit_b)
            return a;
        cut = xlength / (xlength + ylength);
    }
    require(cut > 0 && cut < 1, "composite loft guide parameter precision exhausted");
    require(b.poles.size() - 1 <= limit - a.poles.size(), "composite loft guide control budget");
    a.knots.resize(a.knots.size() - a.degree - 1);
    for (auto &k : a.knots)
        k *= cut;
    a.knots.insert(a.knots.end(), a.degree, cut);
    for (std::size_t i = b.degree + 1; i < b.knots.size(); ++i)
        a.knots.push_back(cut + (1 - cut) * b.knots[i]);
    a.poles.insert(a.poles.end(), b.poles.begin() + 1, b.poles.end());
    a.rational |= b.rational;
    a.check(limit);
    return a;
}
Json Curve::table() const {
    Json xyz = Json::array(), weights = Json::array();
    for (auto h : poles) {
        for (unsigned k = 0; k < 3; ++k)
            xyz.push_back(h[k]);
        if (rational)
            weights.push_back(h[3]);
    }
    return {{"_type", "BsplineCurve"}, {"order", degree + 1},
            {"closed", false},         {"poles", xyz},
            {"knots", knots},          {"weights", rational ? weights : Json(nullptr)}};
}
} // namespace p3d::loft_detail
