#include "internal.hpp"
#include <map>

namespace p3d {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native curve frame: non-finite arithmetic");
    return x;
}
Point3 difference(const Point3 &b, const Point3 &a) {
    return {finite(b[0] - a[0]), finite(b[1] - a[1]), finite(b[2] - a[2])};
}
double dot(const Point3 &a, const Point3 &b) {
    return finite(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
}
double length(const Point3 &a) {
    return std::sqrt(dot(a, a));
}
Point3 cross(const Point3 &a, const Point3 &b) {
    return {finite(a[1] * b[2] - a[2] * b[1]), finite(a[2] * b[0] - a[0] * b[2]),
            finite(a[0] * b[1] - a[1] * b[0])};
}
void scale(Point3 &a, double s) {
    for (auto &x : a)
        x = finite(x * s);
}
// GeVec3d, deliberately different from the GePoint3d normalization used by
// the B-spline Frenet frame. Zero magnitude becomes the positive X axis.
double normalize(Point3 &a) {
    const auto n = length(a);
    if (n > 0)
        scale(a, finite(1 / n));
    else
        a = {1, 0, 0};
    return n;
}
using Columns = std::array<Point3, 3>;
const Columns identity{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
Columns two_vectors(const Point3 &a, const Point3 &b) {
    return {a, b, cross(a, b)};
}
bool triad(Point3 tangent, Columns &out, bool normalized) {
    double n = length(tangent);
    const double threshold = n / 64;
    const bool ok = n != 0;
    if (!ok) {
        tangent[2] = 1;
        n = 1;
    }
    const Point3 reference = std::abs(tangent[0]) < threshold && std::abs(tangent[1]) < threshold
                                 ? Point3{0, 1, 0}
                                 : Point3{0, 0, 1};
    auto first = cross(reference, tangent), second = cross(tangent, first);
    normalize(first);
    normalize(second);
    normalize(tangent);
    out = {tangent, first, second};
    if (!normalized)
        for (auto &v : out)
            scale(v, n);
    return ok;
}
bool square_normalize(Columns &out, unsigned primary, unsigned secondary) {
    auto work = out;
    const unsigned third = 3 - primary - secondary;
    const auto primary_length = normalize(work[primary]);
    work[third] = cross(work[primary], work[secondary]);
    if (normalize(work[third]) == 0)
        return false; // Retains the original matrix.
    work[secondary] = cross(work[third], work[primary]);
    const auto secondary_length = normalize(work[secondary]);
    if (secondary_length == 0) {
        if (primary_length == 0)
            out = identity;
        else {
            Columns t;
            triad(work[primary], t, true);
            work[primary] = t[0];
            work[secondary] = t[1];
            work[third] = t[2];
            out = work;
        }
        return false;
    }
    if (primary == (secondary + 1) % 3)
        scale(work[third], -1);
    out = work;
    return true;
}
bool parallel(const Point3 &a, const Point3 &b) {
    const auto c = cross(a, b);
    return dot(c, c) <= finite((dot(a, a) * 1e-24) * dot(b, b));
}
bool usable_pair(const Point3 &a, const Point3 &b, Columns &out) {
    if (length(b) < length(a) * 1e-6 || std::atan2(length(cross(b, a)), std::abs(dot(b, a))) < 1e-8)
        return false;
    out = two_vectors(a, b);
    return square_normalize(out, 0, 1);
}
struct Evaluation {
    Point3 point{}, first{}, second{}, third{};
};
std::optional<Point3> significant(const Evaluation &e) {
    const double a = length(e.first), b = length(e.second), c = length(e.third);
    const auto sum = finite(a + b + c);
    if (sum == 0)
        return {};
    const auto threshold = sum * 1e-15;
    if (a >= threshold)
        return e.first;
    if (b >= threshold)
        return e.second;
    if (c >= threshold)
        return e.third;
    return {};
}
Point3 point(const Json &j, const std::string &prefix) {
    Point3 result{};
    const char *keys[] = {"X", "Y", "Z"};
    for (unsigned i = 0; i < 3; ++i) {
        require(j.at(prefix + keys[i]).is_number(), "native curve frame: expected point number");
        result[i] = finite(j.at(prefix + keys[i]).get<double>());
    }
    return result;
}
double number(const Json &j, const char *key) {
    require(j.at(key).is_number(), "native curve frame: expected number");
    return finite(j.at(key).get<double>());
}
std::string type(const Json &j) {
    return j.at("_type").get<std::string>();
}
std::vector<Point3> points(const Json &j) {
    const auto &flat = j.at("points");
    if (flat.is_null())
        return {};
    require(flat.is_array() && flat.size() % 3 == 0, "native curve frame: expected XYZ triplets");
    std::vector<Point3> out(flat.size() / 3);
    for (std::size_t i = 0; i < flat.size(); ++i) {
        require(flat[i].is_number(), "native curve frame: expected point number");
        out[i / 3][i % 3] = finite(flat[i].get<double>());
    }
    return out;
}
Evaluation segment(const Point3 &a, const Point3 &b, double fraction, double tangent_scale = 1,
                   bool endpoint_stable = false) {
    Evaluation e;
    e.first = difference(b, a);
    for (unsigned i = 0; i < 3; ++i)
        e.point[i] = endpoint_stable && fraction > .5 ? finite(b[i] + (fraction - 1) * e.first[i])
                                                      : finite(a[i] + fraction * e.first[i]);
    scale(e.first, tangent_scale);
    return e;
}
// fractionToPoint's blending kernel is not computeDerivatives: it clamps even
// closed curves, uses the knot-only pole shift, and substitutes W=1 when W=0.
Evaluation bspline_endpoint(const BsplineCurve &c, double fraction) {
    require(c.order() <= 26, "native curve frame: B-spline order exceeds 26");
    const auto &k = c.knots();
    const auto domain = c.knot_domain();
    const double u =
        std::clamp(finite((1 - fraction) * domain[0] + fraction * domain[1]), domain[0], domain[1]);
    const auto end = u == domain[1] ? std::lower_bound(k.begin(), k.end(), u)
                                    : std::upper_bound(k.begin(), k.end(), u);
    require(end != k.begin() && end != k.end(), "native curve frame: B-spline endpoint span");
    const auto span = std::size_t(end - k.begin() - 1), degree = std::size_t(c.order() - 1);
    require(span >= degree && span + degree < k.size(), "native curve frame: B-spline span extent");
    std::array<double, 26> basis{}, derivative{}, left{}, right{};
    basis[0] = 1;
    for (std::size_t j = 1; j <= degree; ++j) {
        left[j] = finite(u - k[span + 1 - j]);
        right[j] = finite(k[span + j] - u);
        double saved = 0, saved_derivative = 0;
        for (std::size_t r = 0; r < j; ++r) {
            const auto denominator = finite(right[r + 1] + left[j - r]);
            auto value = basis[r], d = derivative[r];
            if (denominator != 0) {
                value = finite(value / denominator);
                d = finite(d / denominator);
            }
            basis[r] = finite(saved + right[r + 1] * value);
            saved = finite(left[j - r] * value);
            derivative[r] = finite(saved_derivative + right[r + 1] * d - value);
            saved_derivative = finite(left[j - r] * d + value);
        }
        basis[j] = saved;
        derivative[j] = saved_derivative;
    }
    Point3 h{}, dh{};
    double w = 0, dw = 0;
    for (std::size_t j = 0; j <= degree; ++j) {
        auto index = std::int64_t(span - degree + j) + c.periodic_pole_shift();
        if (c.closed()) {
            index %= std::int64_t(c.poles().size());
            if (index < 0)
                index += c.poles().size();
        }
        require(index >= 0 && std::uint64_t(index) < c.poles().size(),
                "native curve frame: pole index");
        const auto i = std::size_t(index);
        for (unsigned a = 0; a < 3; ++a) {
            h[a] = finite(h[a] + basis[j] * c.poles()[i][a]);
            dh[a] = finite(dh[a] + derivative[j] * c.poles()[i][a]);
        }
        if (c.rational()) {
            w = finite(w + basis[j] * c.weights()[i]);
            dw = finite(dw + derivative[j] * c.weights()[i]);
        }
    }
    Evaluation e;
    if (!c.rational() || w == 0)
        w = 1;
    for (unsigned a = 0; a < 3; ++a) {
        e.point[a] = finite(h[a] / w);
        e.first[a] = finite((dh[a] - dw * e.point[a]) / w);
    }
    scale(e.first, domain[1] - domain[0]);
    return e;
}
bool almost_equal(const Point3 &a, const Point3 &b) {
    const auto d = difference(b, a);
    const auto threshold =
        finite((((((a[0] * a[0] + a[1] * a[1]) + a[2] * a[2]) + b[0] * b[0]) + b[1] * b[1]) +
                b[2] * b[2] + 1) *
               1e-20);
    return dot(d, d) < threshold;
}
std::optional<Columns> polyline_frame(const std::vector<Point3> &p) {
    if (p.size() < 2)
        return {};
    std::size_t b = 1;
    if (almost_equal(p[1], p[0])) {
        // The native forward search increments the supplied index before
        // testing. P[1] has already failed the backward comparison.
        for (b = 2; b < p.size() && almost_equal(p[0], p[b]); ++b) {
        }
        if (b == p.size())
            return {};
    }
    for (std::size_t i = b + 1; i < p.size(); ++i) {
        auto x = difference(p[b], p[0]), y = difference(p[i], p[0]);
        const auto nx = normalize(x), ny = normalize(y);
        if (!(nx > ny * 1e-12 && ny > nx * 1e-12))
            continue;
        auto z = cross(x, y);
        if (!(normalize(z) > 1e-12))
            continue;
        y = cross(z, x);
        normalize(y);
        return Columns{x, y, z};
    }
    Columns out;
    triad(difference(p[b], p[0]), out, true);
    return out;
}
struct Frame {
    Columns basis = identity;
    Point3 origin{};
    std::string method;
    std::vector<std::string> paths;
    std::optional<bool> normalization_success = std::nullopt;
};
class Query {
    std::map<const Json *, BsplineCurve> splines_;

  public:
    std::string active_path;
    const BsplineCurve &bspline(const Json &j) {
        const auto found = splines_.find(&j);
        if (found != splines_.end())
            return found->second;
        const auto t = type(j);
        auto c = t == "BsplineCurve"         ? BsplineCurve::from_bgfb(j)
                 : t == "AkimaCurve"         ? AkimaCurve::from_bgfb(j).bspline()
                 : t == "InterpolationCurve" ? InterpolationCurve::from_bgfb(j).bspline()
                                             : TransitionSpiral::from_bgfb(j).native_fit().curve;
        return splines_.emplace(&j, std::move(c)).first->second;
    }
    static bool spline_type(const std::string &t) {
        return t == "BsplineCurve" || t == "AkimaCurve" || t == "InterpolationCurve" ||
               t == "TransitionSpiral";
    }
    std::optional<Evaluation> evaluate(const Json &j, const std::string &path, double fraction,
                                       bool derivatives) {
        active_path = path;
        const auto t = type(j);
        if (t == "CurveVector")
            return {};
        if (t == "LineSegment") {
            const auto &s = j.at("segment");
            return segment(point(s, "point0"), point(s, "point1"), fraction, 1, true);
        }
        if (t == "LineString" || t == "PointString") {
            const auto p = points(j);
            if (p.empty())
                return {};
            if (p.size() == 1) {
                Evaluation e;
                e.point = p[0];
                return e;
            }
            // This callback is needed at 0 and 1 only; preserve the native
            // fractional-segment arithmetic at the upper endpoint as well.
            const double step = 1. / double(p.size() - 1);
            const auto index = fraction == 1 ? p.size() - 2 : 0;
            const auto local = (fraction - double(index) * step) / step;
            return segment(p[index], p[index + 1], local, double(p.size() - 1));
        }
        if (t == "EllipticArc") {
            const auto &a = j.at("arc");
            const auto center = point(a, "center"), v0 = point(a, "vector0"),
                       v90 = point(a, "vector90");
            const auto sweep = number(a, "sweepRadians"),
                       angle = finite(number(a, "startRadians") + sweep * fraction);
            const double cs = std::cos(angle), sn = std::sin(angle);
            Evaluation e;
            for (unsigned i = 0; i < 3; ++i) {
                e.point[i] = finite((cs * v0[i] + sn * v90[i]) + center[i]);
                e.first[i] = finite((-sn * v0[i] + cs * v90[i]) * sweep);
                if (derivatives)
                    e.second[i] = finite(-(cs * v0[i] + sn * v90[i]) * (sweep * sweep));
            }
            // The primitive's third-derivative callback explicitly writes zero.
            return e;
        }
        require(spline_type(t), "native curve frame: unsupported primitive " + t);
        const auto &c = bspline(j);
        if (!derivatives)
            return bspline_endpoint(c, fraction);
        try {
            const auto d = c.native_derivatives_at(fraction, 3);
            return Evaluation{d[0], d[1], d[2], d[3]};
        } catch (const std::runtime_error &e) {
            if (std::string(e.what()) == "B-spline native derivatives: knot tolerance failure")
                return {};
            throw;
        }
    }
    std::optional<Frame> primitive_frame(const Json &j, const std::string &path) {
        active_path = path;
        const auto t = type(j);
        if (t == "CurveVector")
            return {};
        if (spline_type(t)) {
            Json f;
            try {
                f = bspline(j).native_frame_at(0);
            } catch (const std::runtime_error &e) {
                if (std::string(e.what()) == "B-spline native derivatives: knot tolerance failure")
                    return {};
                throw;
            }
            Frame out;
            for (unsigned i = 0; i < 3; ++i) {
                out.origin[i] = f["frame"][i][3];
                for (unsigned c = 0; c < 3; ++c)
                    out.basis[c][i] = f["frame"][i][c];
            }
            out.method = "primitive_" + f.at("method").get<std::string>();
            out.paths = {path};
            return out;
        }
        if (t == "LineString" || t == "PointString") {
            const auto p = points(j);
            const auto b = polyline_frame(p);
            if (!b)
                return {};
            return Frame{*b, p.front(), "primitive_polyline_frame", {path}};
        }
        const auto e = evaluate(j, path, 0, true);
        if (!e)
            return {};
        Frame out;
        out.origin = e->point;
        out.paths = {path};
        if (t == "LineSegment") {
            if (!triad(e->first, out.basis, true))
                out.basis = identity;
            out.method = "primitive_segment_frame";
        } else {
            out.basis = two_vectors(e->first, e->second);
            out.normalization_success = square_normalize(out.basis, 0, 1);
            out.method = "primitive_derivative_frame";
        }
        return out;
    }
    static const Json *child(const Json &entry) {
        if (entry.is_null())
            return nullptr;
        const auto &g = entry.at("geometry");
        return g.is_null() ? nullptr : &g;
    }
    std::optional<Frame> array(const Json &j, const std::string &path, int preference,
                               unsigned depth) {
        active_path = path;
        require(depth <= 80, "native curve frame: nesting limit exceeded");
        require(type(j) == "CurveVector", "native curve frame: expected CurveVector table");
        const auto &entries = j.at("curves");
        if (entries.is_null())
            return {};
        require(entries.is_array(), "native curve frame: expected curves array");
        const auto child_path = [&](std::size_t i) {
            return path + "/curves/" + std::to_string(i) + "/geometry";
        };
        if (preference == 1 || preference == 2) {
            bool have = false;
            Point3 p{}, a{}, b{};
            std::string first_path, last_path;
            for (std::size_t i = 0; i < entries.size(); ++i) {
                active_path = child_path(i);
                const auto c = child(entries[i]);
                if (!c)
                    continue;
                auto start = evaluate(*c, active_path, 0, false);
                if (!start)
                    continue;
                auto end = evaluate(*c, active_path, 1, false);
                if (!end)
                    continue;
                normalize(start->first);
                normalize(end->first);
                if (!have) {
                    p = start->point;
                    a = start->first;
                    first_path = active_path;
                    have = true;
                }
                b = end->first;
                last_path = active_path;
            }
            if (have) {
                const bool collinear = parallel(a, b);
                if (!collinear || preference == 1) {
                    Frame out;
                    out.origin = p;
                    if (collinear)
                        triad(a, out.basis, false);
                    else
                        out.basis = two_vectors(a, b);
                    out.normalization_success = square_normalize(out.basis, 0, 2);
                    out.method = collinear ? "endpoint_axis_frame" : "endpoint_tangents_frame";
                    out.paths = {first_path, last_path};
                    return out;
                }
            }
        }
        for (std::size_t i = 0; i < entries.size(); ++i) {
            active_path = child_path(i);
            const auto c = child(entries[i]);
            if (!c)
                continue;
            if (type(*c) == "CurveVector") {
                auto f = array(*c, child_path(i), 0, depth + 1);
                if (f)
                    return f;
                continue;
            }
            const auto e = evaluate(*c, child_path(i), 0, true);
            if (!e)
                continue;
            Columns basis;
            if (usable_pair(e->first, e->second, basis))
                return Frame{basis, e->point, "local_derivatives_frame", {child_path(i)}, true};
            const auto a = significant(*e);
            if (!a)
                continue;
            for (std::size_t k = i + 1; k < entries.size(); ++k) {
                active_path = child_path(k);
                const auto next = child(entries[k]);
                require(next, "native curve frame: null later child has no defined native "
                              "derivative callback");
                const auto d = evaluate(*next, child_path(k), 0, true);
                if (!d)
                    continue;
                const auto b = significant(*d);
                if (!b)
                    continue;
                if (usable_pair(*a, difference(d->point, e->point), basis))
                    return Frame{basis,
                                 e->point,
                                 "later_origin_frame",
                                 {child_path(i), child_path(k)},
                                 true};
                if (usable_pair(*a, *b, basis))
                    return Frame{basis,
                                 e->point,
                                 "later_derivatives_frame",
                                 {child_path(i), child_path(k)},
                                 true};
            }
        }
        for (std::size_t i = 0; i < entries.size(); ++i) {
            active_path = child_path(i);
            const auto c = child(entries[i]);
            if (!c)
                continue;
            auto f = primitive_frame(*c, child_path(i));
            if (f)
                return f;
        }
        return {};
    }
};
} // namespace
Json native_curve_frame(const Json &curve_vector, int search_preference) {
    Query query;
    Json result{{"profile", "native_curve_array_frame"}, {"search_preference", search_preference}};
    try {
        const auto f = query.array(curve_vector, "", search_preference, 0);
        if (!f) {
            result["status"] = "native_failure";
            return result;
        }
        Matrix4 matrix{};
        for (unsigned row = 0; row < 3; ++row) {
            for (unsigned col = 0; col < 3; ++col)
                matrix[row][col] = finite(f->basis[col][row]);
            matrix[row][3] = finite(f->origin[row]);
        }
        matrix[3][3] = 1;
        const auto determinant = dot(f->basis[0], cross(f->basis[1], f->basis[2]));
        result.update({{"status", "computed"},
                       {"frame", matrix},
                       {"method", f->method},
                       {"source_paths", f->paths},
                       {"basis_status", determinant == 0 ? "degenerate" : "nondegenerate"}});
        if (f->normalization_success)
            result["square_normalization_success"] = *f->normalization_success;
    } catch (const std::exception &e) {
        result.update({{"status", "not_evaluated"},
                       {"reason", e.what()},
                       {"source_path", query.active_path}});
    }
    return result;
}
} // namespace p3d
