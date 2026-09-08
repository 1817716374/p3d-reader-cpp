#include "internal.hpp"
namespace p3d {
namespace {
using H = std::array<double, 4>;
using Piece = std::vector<H>;
double number(const Json &v) {
    require(v.is_number(), "trim coordinate is not numeric");
    const double x = v.get<double>();
    require(std::isfinite(x), "trim coordinate is not finite");
    return x;
}
Point3 point(const Json &v, const std::string &prefix) {
    return {number(v.at(prefix + "X")), number(v.at(prefix + "Y")), number(v.at(prefix + "Z"))};
}
H homogeneous(Point3 p) {
    return {p[0], p[1], p[2], 1};
}
Point3 cartesian(H h) {
    require(h[3] != 0, "trim point has zero weight");
    Point3 p{h[0] / h[3], h[1] / h[3], h[2] / h[3]};
    for (double x : p)
        require(std::isfinite(x), "trim point is not finite");
    return p;
}
bool native_closed(Point3 a, Point3 b) {
    double d = 0, scale = 1;
    for (unsigned k = 0; k < 3; ++k) {
        d += (a[k] - b[k]) * (a[k] - b[k]);
        scale += a[k] * a[k] + b[k] * b[k];
    }
    return std::isfinite(d) && std::isfinite(scale) && d < scale * 1e-20;
}
double distance(Point2 p, Point2 a, Point2 b) {
    const long double x = static_cast<long double>(b[0]) - a[0],
                      y = static_cast<long double>(b[1]) - a[1];
    const long double length = x * x + y * y;
    const long double t =
        length == 0 ? 0
                    : std::max(0.L, std::min(1.L, ((static_cast<long double>(p[0]) - a[0]) * x +
                                                   (static_cast<long double>(p[1]) - a[1]) * y) /
                                                      length));
    const double d = double(std::hypot(static_cast<long double>(p[0]) - a[0] - t * x,
                                       static_cast<long double>(p[1]) - a[1] - t * y));
    require(std::isfinite(d), "trim distance overflow");
    return d;
}
std::vector<Piece> spans(const BsplineCurve &c, unsigned limit) {
    const auto order = std::size_t(c.order()), degree = order - 1;
    const auto &source = c.knots();
    const auto domain = c.knot_domain();
    std::vector<Piece> result;
    // Isolate each active span using only its local knot/control window. Avoid
    // repeatedly inserting into the entire source curve (quadratic in span count).
    for (std::size_t span = degree; span < source.size() - order; ++span) {
        if (source[span] < domain[0] || source[span + 1] > domain[1] ||
            source[span] >= source[span + 1])
            continue;
        require(result.size() < limit, "trim B-spline span limit");
        std::vector<double> knots(source.begin() + span - degree,
                                  source.begin() + span + degree + 2);
        std::vector<H> poles;
        for (std::size_t i = span - degree; i <= span; ++i) {
            auto j = (std::int64_t(i) + c.periodic_pole_shift()) % std::int64_t(c.poles().size());
            if (j < 0)
                j += std::int64_t(c.poles().size());
            const auto &p = c.poles()[std::size_t(j)];
            poles.push_back({p[0], p[1], p[2], c.rational() ? c.weights()[std::size_t(j)] : 1});
        }
        for (double t : std::array<double, 2>{source[span], source[span + 1]}) {
            auto multiplicity = std::size_t(std::count(knots.begin(), knots.end(), t));
            while (multiplicity < degree) {
                const auto k = std::size_t(std::upper_bound(knots.begin(), knots.end(), t) -
                                           knots.begin() - 1);
                require(k >= degree && k - multiplicity < poles.size(), "trim knot insertion span");
                std::vector<H> q(poles.size() + 1);
                for (std::size_t i = 0; i <= k - degree; ++i)
                    q[i] = poles[i];
                for (std::size_t i = k - multiplicity; i < poles.size(); ++i)
                    q[i + 1] = poles[i];
                for (std::size_t i = k - degree + 1; i <= k - multiplicity; ++i) {
                    const double den = knots[i + degree] - knots[i];
                    require(den > 0 && std::isfinite(den), "trim knot insertion interval");
                    const double a = (t - knots[i]) / den;
                    for (unsigned j = 0; j < 4; ++j)
                        q[i][j] = (1 - a) * poles[i - 1][j] + a * poles[i][j];
                }
                poles = std::move(q);
                knots.insert(knots.begin() + k + 1, t);
                ++multiplicity;
            }
        }
        const auto i = std::size_t(std::upper_bound(knots.begin(), knots.end(), source[span]) -
                                   knots.begin() - 1);
        require(i >= degree && i < poles.size(), "trim Bezier pole window");
        result.emplace_back(poles.begin() + i - degree, poles.begin() + i + 1);
    }
    return result;
}
std::vector<Piece> primitive(const Json &v, unsigned limit) {
    const auto type = v.at("_type").get<std::string>();
    if (type == "BsplineCurve")
        return spans(BsplineCurve::from_bgfb(v), limit);
    std::vector<Piece> result;
    if (type == "LineSegment") {
        const auto &s = v.at("segment");
        result.push_back({homogeneous(point(s, "point0")), homogeneous(point(s, "point1"))});
    } else if (type == "LineString") {
        const auto &a = v.at("points");
        require(a.is_array() && a.size() % 3 == 0, "trim line-string XYZ triplets");
        require(a.size() / 3 <= std::size_t(limit) + 1, "trim line-string segment limit");
        for (std::size_t i = 3; i < a.size(); i += 3) {
            Point3 p{}, q{};
            for (unsigned k = 0; k < 3; ++k) {
                p[k] = number(a[i - 3 + k]);
                q[k] = number(a[i + k]);
            }
            result.push_back({homogeneous(p), homogeneous(q)});
        }
    } else if (type == "EllipticArc") {
        const auto &a = v.at("arc");
        const auto c = point(a, "center"), x = point(a, "vector0"), y = point(a, "vector90");
        const double start = number(a.at("startRadians")), sweep = number(a.at("sweepRadians"));
        const double pi = 3.1415926535897932384626433832795;
        const double pieces = std::max(1.0, std::ceil(std::abs(sweep) / (2 * pi / 3)));
        require(pieces <= limit, "trim arc segment limit");
        const auto n = unsigned(pieces);
        auto at = [&](double t, double w) {
            H p{};
            for (unsigned k = 0; k < 3; ++k)
                p[k] = c[k] * w + x[k] * std::cos(t) + y[k] * std::sin(t);
            p[3] = w;
            return p;
        };
        for (unsigned i = 0; i < n; ++i) {
            const double t0 = start + sweep * (double(i) / n),
                         t1 = start + sweep * (double(i + 1) / n);
            result.push_back({at(t0, 1), at((t0 + t1) / 2, std::cos((t1 - t0) / 2)), at(t1, 1)});
        }
        if (sweep != 0 && std::remainder(sweep, 2 * pi) == 0)
            result.back().back() = result.front().front();
    } else
        throw std::runtime_error("unsupported trim primitive: " + type);
    return result;
}
struct Loop {
    std::string path;
    int source_type = 0;
    std::vector<Piece> pieces;
};
struct Builder {
    double tolerance;
    unsigned limit, segments = 0;
    bool complete = true;
    Json sources = Json::array(), ignored = Json::array(), errors = Json::array();
    std::vector<Loop> loops;
    Builder(double tolerance_, unsigned limit_) : tolerance(tolerance_), limit(limit_) {}
    void error(const std::string &path, const std::string &message) {
        complete = false;
        errors.push_back({{"source_path", path}, {"error", message}});
    }
    void collect(const Json &root, const std::string &path, unsigned depth = 0) {
        if (root.is_null())
            return;
        try {
            require(depth <= 80, "trim tree depth");
            require(root.at("_type") == "CurveVector", "expected trim CurveVector");
            const auto &tv = root.at("type");
            require(tv.is_number_integer() && tv >= 0 && tv <= 5, "unknown trim boundary type");
            const int type = tv.get<int>();
            const auto &curves = root.at("curves");
            require(curves.is_null() || curves.is_array(), "trim curve array");
            if (type == 4 || type == 5) {
                for (std::size_t i = 0; i < curves.size(); ++i) {
                    const auto p = path + "/curves/" + std::to_string(i) + "/geometry";
                    const auto &child = curves[i].at("geometry");
                    if (child.is_object() && child.value("_type", std::string()) == "CurveVector")
                        collect(child, p, depth + 1);
                    else
                        ignored.push_back(
                            {{"source_path", p}, {"reason", "region_member_is_not_curve_array"}});
                }
                return;
            }
            if (type == 0) {
                ignored.push_back({{"source_path", path}, {"reason", "boundary_type_none"}});
                return;
            }
            Loop loop{path, type, {}};
            for (std::size_t i = 0; i < curves.size(); ++i) {
                auto pieces = primitive(curves[i].at("geometry"), limit);
                loop.pieces.insert(loop.pieces.end(), std::make_move_iterator(pieces.begin()),
                                   std::make_move_iterator(pieces.end()));
                require(loop.pieces.size() <= limit, "trim primitive span limit");
            }
            if (loop.pieces.empty()) {
                ignored.push_back({{"source_path", path}, {"reason", "empty_boundary"}});
                return;
            }
            if (type == 1 && !native_closed(cartesian(loop.pieces.front().front()),
                                            cartesian(loop.pieces.back().back()))) {
                ignored.push_back({{"source_path", path}, {"reason", "open_boundary_not_closed"}});
                return;
            }
            loops.push_back(std::move(loop));
        } catch (const std::exception &e) {
            error(path, e.what());
        }
    }
    void stroke(const Piece &p, std::vector<Point2> &out, double &bound, unsigned depth = 0) {
        require(depth <= 60, "trim subdivision depth");
        double sign = 0, deviation = 0;
        bool finite_hull = true;
        std::vector<Point2> polygon;
        for (const auto &h : p) {
            if (h[3] == 0) {
                finite_hull = false;
                break;
            }
            const double s = h[3] > 0 ? 1 : -1;
            if (sign && s != sign) {
                finite_hull = false;
                break;
            }
            sign = s;
            const auto q = cartesian(h);
            polygon.push_back({q[0], q[1]});
        }
        if (finite_hull)
            for (const auto &q : polygon)
                deviation = std::max(deviation, distance(q, polygon.front(), polygon.back()));
        if (finite_hull && deviation <= tolerance / 8) {
            require(segments < limit, "trim segment budget exhausted");
            ++segments;
            if (out.empty())
                out.push_back(polygon.front());
            out.push_back(polygon.back());
            bound = std::max(bound, deviation);
            return;
        }
        Piece work = p, left(p.size()), right(p.size());
        left[0] = work.front();
        right.back() = work.back();
        for (std::size_t r = 1; r < p.size(); ++r) {
            for (std::size_t i = 0; i < p.size() - r; ++i)
                for (unsigned k = 0; k < 4; ++k)
                    work[i][k] = 0.5 * work[i][k] + 0.5 * work[i + 1][k];
            left[r] = work.front();
            right[p.size() - r - 1] = work[p.size() - r - 1];
        }
        stroke(left, out, bound, depth + 1);
        stroke(right, out, bound, depth + 1);
    }
};
} // namespace
BsplineTrim BsplineSurface::trim(double tolerance, unsigned max_segments) const {
    require(std::isfinite(tolerance) && tolerance > 0,
            "trim tolerance must be finite and positive");
    require(max_segments > 0, "trim segment limit must be positive");
    BsplineTrim result;
    result.tolerance_ = tolerance;
    result.outer_active_ = outer_boundary_active();
    Builder build{tolerance, max_segments};
    build.collect(boundaries_, "");
    for (const auto &loop : build.loops) {
        try {
            double scale = 1;
            for (const auto &piece : loop.pieces)
                for (const auto &h : piece) {
                    const auto p = cartesian(h);
                    scale = std::max({scale, std::abs(p[0]), std::abs(p[1])});
                }
            const double roundoff = 64 * std::numeric_limits<double>::epsilon() * scale;
            require(roundoff < tolerance / 8, "trim tolerance below coordinate precision");
            std::vector<Point2> polygon;
            double bound = roundoff;
            for (std::size_t i = 0; i < loop.pieces.size(); ++i) {
                const auto a = cartesian(loop.pieces[i].front());
                const auto b = cartesian(
                    loop.pieces[(i + loop.pieces.size() - 1) % loop.pieces.size()].back());
                require(std::hypot(a[0] - b[0], a[1] - b[1]) <= roundoff,
                        "trim boundary has a gap or discontinuity");
                build.stroke(loop.pieces[i], polygon, bound);
            }
            polygon.back() = polygon.front();
            result.loops_.push_back(std::move(polygon));
            result.errors_.push_back(bound + roundoff);
            build.sources.push_back(
                {{"source_path", loop.path},
                 {"source_boundary_type", loop.source_type},
                 {"effective_boundary_type", loop.source_type == 1 ? 2 : loop.source_type},
                 {"polyline_index", result.loops_.size() - 1},
                 {"deviation_bound", bound + roundoff}});
        } catch (const std::exception &e) {
            build.error(loop.path, e.what());
        }
    }
    result.complete_ = build.complete;
    result.report_ = {{"status", build.complete ? "complete" : "incomplete"},
                      {"coordinate_space", "source_uv"},
                      {"outer_boundary_active", result.outer_active_},
                      {"fill_rule", "parity"},
                      {"boundary_tolerance", tolerance},
                      {"segments", build.segments},
                      {"loops", build.sources},
                      {"ignored", build.ignored},
                      {"errors", build.errors},
                      {"representation", "derived_polylines"}};
    return result;
}
TrimLocation BsplineTrim::classify(Point2 uv) const {
    require(std::isfinite(uv[0]) && std::isfinite(uv[1]) && uv[0] >= 0 && uv[0] <= 1 &&
                uv[1] >= 0 && uv[1] <= 1,
            "trim query must be inside the normalized UV square");
    if (!complete_)
        return TrimLocation::Indeterminate;
    // The native point-in-bounds routine returns true when there are no effective boundaries.
    if (loops_.empty())
        return TrimLocation::Inside;
    bool parity = false, uncertain = false, boundary = false;
    for (std::size_t i = 0; i < loops_.size(); ++i) {
        const auto &loop = loops_[i];
        bool inside = false;
        for (std::size_t j = 1; j < loop.size(); ++j) {
            const auto a = loop[j - 1], b = loop[j];
            const double d = distance(uv, a, b);
            if (d + errors_[i] <= tolerance_)
                boundary = true;
            else if (d <= tolerance_ + errors_[i])
                uncertain = true;
            if ((a[0] < uv[0] && uv[0] <= b[0]) || (b[0] < uv[0] && uv[0] <= a[0])) {
                const long double y =
                    a[1] + (static_cast<long double>(uv[0]) - a[0]) * (b[1] - a[1]) / (b[0] - a[0]);
                if (uv[1] > y)
                    inside = !inside;
            }
        }
        parity = parity != inside;
    }
    if (boundary)
        return TrimLocation::BoundaryBand;
    if (uncertain)
        return TrimLocation::Indeterminate;
    return (outer_active_ != parity) ? TrimLocation::Inside : TrimLocation::Outside;
}
} // namespace p3d
