#include "guided.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace p3d {
namespace {
constexpr double pi = 3.1415926535897932384626433832795;
using HPoint = std::array<double, 4>;
using Piece = std::vector<HPoint>;
Point3 add(Point3 a, Point3 b) {
    for (unsigned k = 0; k < 3; ++k)
        a[k] += b[k];
    return a;
}
Point3 sub(Point3 a, Point3 b) {
    for (unsigned k = 0; k < 3; ++k)
        a[k] -= b[k];
    return a;
}
Point3 mul(Point3 a, double s) {
    for (auto &v : a)
        v *= s;
    return a;
}
double norm(Point3 a) {
    return std::hypot(a[0], a[1], a[2]);
}
bool curve_array_closed(Point3 a, Point3 b) {
    // Native curve-array closure uses a strict, coordinate-scaled squared distance.
    // Keep this separate from fitting tolerances and the source boundary type.
    auto d = sub(a, b);
    double distance2 = d[1] * d[1] + d[0] * d[0] + d[2] * d[2];
    double scale2 =
        a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + b[0] * b[0] + b[1] * b[1] + b[2] * b[2] + 1.;
    return std::isfinite(distance2) && std::isfinite(scale2) &&
           distance2 < scale2 * 1.0000000000000001e-20;
}
double max_component(Point3 p) {
    return std::max({std::abs(p[0]), std::abs(p[1]), std::abs(p[2])});
}
bool contiguous_curves(Point3 a, Point3 b) {
    const double distance = max_component(sub(a, b));
    const double extent = std::max(max_component(a), max_component(b));
    return distance < 1e-8 || distance < extent * 1e-8 + 1e-8;
}
bool coons_corner_aligned(Point3 a, Point3 b) {
    return max_component(sub(a, b)) <= 1e-5;
}
HPoint mix(HPoint a, HPoint b, double t) {
    for (unsigned k = 0; k < 4; ++k)
        a[k] = a[k] * (1 - t) + b[k] * t;
    return a;
}
HPoint homogeneous(Point3 p, double w = 1) {
    return {p[0] * w, p[1] * w, p[2] * w, w};
}
Point3 cartesian(HPoint p) {
    require(std::isfinite(p[3]) && p[3] > 0, "invalid guided surface weight");
    Point3 v{p[0] / p[3], p[1] / p[3], p[2] / p[3]};
    for (auto x : v)
        require(std::isfinite(x), "nonfinite guided surface point");
    return v;
}
std::pair<Piece, Piece> split(Piece work, double t) {
    Piece left{work.front()}, right{work.back()};
    while (work.size() > 1) {
        for (std::size_t i = 0; i + 1 < work.size(); ++i)
            work[i] = mix(work[i], work[i + 1], t);
        work.pop_back();
        left.push_back(work.front());
        right.push_back(work.back());
    }
    std::reverse(right.begin(), right.end());
    return {left, right};
}
struct Curve {
    unsigned degree = 1;
    std::vector<double> breaks;
    std::vector<Piece> pieces;
    std::size_t count() const {
        return pieces.size() * degree + 1;
    }
    HPoint pole(std::size_t i) const {
        return i == count() - 1 ? pieces.back().back() : pieces[i / degree][i % degree];
    }
    void elevate() {
        require(degree < 3, "unsupported guided boundary degree");
        for (auto &piece : pieces) {
            Piece out{piece.front()};
            for (unsigned i = 1; i <= degree; ++i)
                out.push_back(mix(piece[i], piece[i - 1], double(i) / (degree + 1)));
            out.push_back(piece.back());
            piece = std::move(out);
        }
        ++degree;
    }
    void refine(double t) {
        auto found = std::lower_bound(breaks.begin(), breaks.end(), t);
        if (found != breaks.end() && *found == t)
            return;
        require(found != breaks.begin() && found != breaks.end(), "guided boundary knot range");
        auto i = std::size_t(found - breaks.begin()) - 1;
        auto parts = split(pieces[i], (t - breaks[i]) / (breaks[i + 1] - breaks[i]));
        pieces[i] = std::move(parts.first);
        pieces.insert(pieces.begin() + i + 1, std::move(parts.second));
        breaks.insert(found, t);
    }
};
void unique(std::vector<double> &v) {
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
}
Curve source_curve(const GuidedBoundary &input, unsigned budget, bool length_weighted = false) {
    Curve out;
    if (!input.parts.empty()) {
        require(input.parts.size() <= budget, "guided composite boundary budget");
        auto length = [](const Curve &c) {
            double result = 0;
            for (std::size_t i = 1; i < c.count(); ++i)
                result += norm(sub(cartesian(c.pole(i)), cartesian(c.pole(i - 1))));
            return result;
        };
        for (const auto &part : input.parts) {
            require(part.parts.empty(), "nested composite guide requires native curve-array rules");
            auto next = source_curve(part, budget);
            if (out.pieces.empty()) {
                out = std::move(next);
                continue;
            }
            while (out.degree < next.degree)
                out.elevate();
            while (next.degree < out.degree)
                next.elevate();
            require(
                contiguous_curves(cartesian(out.pole(out.count() - 1)), cartesian(next.pole(0))),
                "discontinuous composite guide is not supported");
            double cut = .5;
            if (length_weighted) {
                double a = length(out), b = length(next);
                require(std::isfinite(a + b) && a > 0 && b > 0, "composite guide polygon length");
                cut = a / (a + b);
            }
            require(cut > 0 && cut < 1, "composite guide parameter precision exhausted");
            for (auto &t : out.breaks)
                t *= cut;
            for (std::size_t i = 1; i < next.breaks.size(); ++i)
                out.breaks.push_back(cut + (1 - cut) * next.breaks[i]);
            // A native contiguous join retains the preceding curve's last pole/weight,
            // omitting the following curve's first pole. Do this after length weighting.
            next.pieces.front().front() = out.pieces.back().back();
            out.pieces.insert(out.pieces.end(), next.pieces.begin(), next.pieces.end());
            require(out.count() <= budget, "guided composite control budget");
        }
        return out;
    }
    if (input.ellipse) {
        require(std::isfinite(input.start) && std::isfinite(input.sweep) &&
                    std::abs(input.sweep) > 0 && std::abs(input.sweep) <= 2 * pi,
                "invalid or multi-turn guided ellipse");
        unsigned spans = std::abs(input.sweep) <= 2 * pi / 3   ? 1
                         : std::abs(input.sweep) <= 4 * pi / 3 ? 2
                                                               : 3;
        out.degree = 2;
        auto at = [&](double a) {
            return add(input.center,
                       add(mul(input.axis_x, std::cos(a)), mul(input.axis_y, std::sin(a))));
        };
        for (unsigned i = 0; i < spans; ++i) {
            double a = input.start + input.sweep * i / spans,
                   b = input.start + input.sweep * (i + 1) / spans, w = std::cos((b - a) / 2);
            auto middle = add(input.center, mul(sub(at((a + b) / 2), input.center), 1 / w));
            out.pieces.push_back({homogeneous(at(a)), homogeneous(middle, w), homogeneous(at(b))});
            out.breaks.push_back(double(i) / spans);
        }
        out.breaks.push_back(1);
        // Native periodic conics open at parameter zero by removing exterior knots.
        // The seven poles and the source angular seam stay in place.
        if (std::abs(input.sweep) == 2 * pi)
            out.pieces.back().back() = out.pieces.front().front();
    } else {
        std::vector<Point3> points;
        for (auto p : input.points) {
            cartesian(homogeneous(p));
            if (points.empty() || norm(sub(p, points.back())) > 0)
                points.push_back(p);
        }
        require(points.size() >= 2 && points.size() <= budget, "guided polyline boundary size");
        out.breaks.push_back(0);
        for (std::size_t i = 1; i < points.size(); ++i) {
            out.breaks.push_back(out.breaks.back() + norm(sub(points[i], points[i - 1])));
            out.pieces.push_back({homogeneous(points[i - 1]), homogeneous(points[i])});
        }
        double length = out.breaks.back();
        require(std::isfinite(length) && length > 0, "guided polyline length");
        for (auto &t : out.breaks)
            t /= length;
    }
    for (std::size_t i = 1; i < out.breaks.size(); ++i)
        require(out.breaks[i] > out.breaks[i - 1], "guided boundary parameter precision exhausted");
    for (auto &p : out.pieces)
        for (auto h : p)
            cartesian(h);
    return out;
}
void compatible(std::vector<Curve *> curves, unsigned budget) {
    unsigned degree = 1;
    std::vector<double> knots;
    for (auto c : curves) {
        degree = std::max(degree, c->degree);
        knots.insert(knots.end(), c->breaks.begin(), c->breaks.end());
    }
    unique(knots);
    require(knots.size() <= budget / degree, "guided compatible boundary budget");
    for (auto c : curves) {
        while (c->degree < degree)
            c->elevate();
        for (auto k : knots)
            c->refine(k);
    }
}
double blend(std::size_t i, std::size_t n) {
    if (i < n / 2)
        return 0;
    if (n % 2 && i == n / 2)
        return .5;
    return 1;
}
struct Patch {
    Curve bottom, top, left, right;
    std::vector<HPoint> poles; // u-major, v varies fastest
    unsigned nu = 0, nv = 0;
    Patch(Curve a, Curve b, Curve c, Curve d, unsigned budget)
        : bottom(std::move(a)), top(std::move(b)), left(std::move(c)), right(std::move(d)) {
        // The native three-pole rule runs before bottom/top knot compatibility.
        if (bottom.count() == 3) {
            bottom.elevate();
            top.elevate();
        }
        if (left.count() == 3) {
            left.elevate();
            right.elevate();
        }
        compatible({&bottom, &top}, budget);
        nu = unsigned(bottom.count());
        nv = unsigned(left.count());
        require(nv <= budget && nu <= budget / nv, "guided control net budget");
        auto p00 = cartesian(bottom.pole(0)), p10 = cartesian(bottom.pole(nu - 1));
        auto p01 = cartesian(top.pole(0)), p11 = cartesian(top.pole(nu - 1));
        require(coons_corner_aligned(p00, cartesian(left.pole(0))) &&
                    coons_corner_aligned(p10, cartesian(right.pole(0))) &&
                    coons_corner_aligned(p01, cartesian(left.pole(nv - 1))) &&
                    coons_corner_aligned(p11, cartesian(right.pole(nv - 1))),
                "guided boundary corner mismatch");
        for (unsigned i = 0; i < nu; ++i)
            for (unsigned j = 0; j < nv; ++j) {
                double u = blend(i, nu), v = blend(j, nv);
                auto a0 = bottom.pole(i), a1 = top.pole(i), b0 = left.pole(j), b1 = right.pole(j);
                auto p = add(add(mul(cartesian(a0), 1 - v), mul(cartesian(a1), v)),
                             add(mul(cartesian(b0), 1 - u), mul(cartesian(b1), u)));
                auto corner = add(add(mul(p00, (1 - u) * (1 - v)), mul(p10, u * (1 - v))),
                                  add(mul(p01, (1 - u) * v), mul(p11, u * v)));
                double w = (a0[3] * (1 - v) + a1[3] * v) * (b0[3] * (1 - u) + b1[3] * u);
                auto h = homogeneous(sub(p, corner), w);
                cartesian(h);
                poles.push_back(h);
            }
    }
    std::vector<HPoint> cell(unsigned i, unsigned j) const {
        std::vector<HPoint> out;
        for (unsigned u = 0; u <= bottom.degree; ++u)
            for (unsigned v = 0; v <= left.degree; ++v)
                out.push_back(poles[(i * bottom.degree + u) * nv + j * left.degree + v]);
        return out;
    }
    Point3 at(double u, double v) const {
        auto locate = [](const std::vector<double> &k, double t) {
            auto i = t >= 1 ? k.size() - 2
                            : std::size_t(std::upper_bound(k.begin(), k.end(), t) - k.begin()) - 1;
            return std::make_pair(unsigned(i), (t - k[i]) / (k[i + 1] - k[i]));
        };
        auto a = locate(bottom.breaks, u), b = locate(left.breaks, v);
        Piece rows;
        for (unsigned i = 0; i <= bottom.degree; ++i) {
            Piece row;
            for (unsigned j = 0; j <= left.degree; ++j)
                row.push_back(
                    poles[(a.first * bottom.degree + i) * nv + b.first * left.degree + j]);
            rows.push_back(split(row, b.second).first.back());
        }
        return cartesian(split(rows, a.second).first.back());
    }
};
// Bound rational derivatives over one Bezier rectangle using positive-weight convex hulls.
// Taylor interpolation on either triangle is bounded by Muu/8 + Muv/4 + Mvv/8.
double error_bound(std::vector<HPoint> cp, unsigned p, unsigned q) {
    auto center = cartesian(cp.front());
    double wmin = std::numeric_limits<double>::infinity();
    for (auto &h : cp) {
        for (unsigned k = 0; k < 3; ++k)
            h[k] -= center[k] * h[3];
        wmin = std::min(wmin, h[3]);
    }
    auto bounds = [](const std::vector<HPoint> &v) {
        Point2 b{};
        for (auto h : v) {
            b[0] = std::max(b[0], norm({h[0], h[1], h[2]}));
            b[1] = std::max(b[1], std::abs(h[3]));
        }
        return b;
    };
    auto derivative = [](const std::vector<HPoint> &a, unsigned p, unsigned q, bool u) {
        std::vector<HPoint> out;
        if ((u ? p : q) == 0)
            return out;
        for (unsigned i = 0; i <= (u ? p - 1 : p); ++i)
            for (unsigned j = 0; j <= (u ? q : q - 1); ++j) {
                auto x = a[i * (q + 1) + j], y = a[(i + (u ? 1 : 0)) * (q + 1) + j + (u ? 0 : 1)];
                for (unsigned k = 0; k < 4; ++k)
                    y[k] = (y[k] - x[k]) * (u ? p : q);
                out.push_back(y);
            }
        return out;
    };
    auto u = derivative(cp, p, q, true), v = derivative(cp, p, q, false);
    auto a = bounds(cp), bu = bounds(u), bv = bounds(v);
    auto uu = bounds(derivative(u, p - 1, q, true)), vv = bounds(derivative(v, p, q - 1, false)),
         uv = bounds(derivative(u, p - 1, q, false));
    double c = a[0] / wmin, cu = (bu[0] + c * bu[1]) / wmin, cv = (bv[0] + c * bv[1]) / wmin;
    double muu = (uu[0] + 2 * cu * bu[1] + c * uu[1]) / wmin;
    double mvv = (vv[0] + 2 * cv * bv[1] + c * vv[1]) / wmin;
    double muv = (uv[0] + cu * bv[1] + cv * bu[1] + c * uv[1]) / wmin;
    auto error = (muu + mvv) / 8 + muv / 4;
    require(std::isfinite(error), "guided surface error bound overflow");
    return error;
}
std::vector<double> grid(const std::vector<double> &breaks, unsigned n) {
    std::vector<double> out;
    for (std::size_t k = 1; k < breaks.size(); ++k)
        for (unsigned i = 0; i < n; ++i)
            out.push_back(breaks[k - 1] + (breaks[k] - breaks[k - 1]) * double(i) / n);
    for (unsigned i = 0; i <= n; ++i)
        out.push_back(double(i) / n);
    unique(out);
    return out;
}
} // namespace
GuidedMesh guided_surface(const std::vector<GuidedBoundary> &bottom,
                          const std::vector<GuidedBoundary> &top,
                          const std::vector<GuidedBoundary> &guides, const Tessellation &policy,
                          bool closed) {
    auto count = bottom.size();
    require(count && top.size() == count && guides.size() == count + (closed ? 0 : 1),
            "guided boundary count");
    require(count <= policy.max_segments, "guided patch budget");
    if (policy.chord_tolerance)
        require(std::isfinite(*policy.chord_tolerance) && *policy.chord_tolerance > 0,
                "guided surface tolerance");
    std::vector<Curve> rails;
    auto part_count = [](const GuidedBoundary &g) {
        return std::max(std::size_t(1), g.parts.size());
    };
    const bool length_weighted = std::any_of(guides.begin(), guides.end(), [&](const auto &g) {
        return part_count(g) != part_count(guides.front());
    });
    for (const auto &g : guides)
        rails.push_back(source_curve(g, policy.max_segments, length_weighted));
    std::vector<Curve *> refs;
    for (auto &c : rails)
        refs.push_back(&c);
    compatible(refs, policy.max_segments);
    std::vector<Patch> patches;
    unsigned divisions = std::max(1u, policy.full_circle_segments / 4);
    double bound = 0;
    for (std::size_t i = 0; i < count; ++i) {
        patches.emplace_back(source_curve(bottom[i], policy.max_segments),
                             source_curve(top[i], policy.max_segments), rails[i],
                             rails[(i + 1) % rails.size()], policy.max_segments);
        auto &p = patches.back();
        if (policy.chord_tolerance)
            for (unsigned u = 0; u < p.bottom.pieces.size(); ++u)
                for (unsigned v = 0; v < p.left.pieces.size(); ++v)
                    bound =
                        std::max(bound, error_bound(p.cell(u, v), p.bottom.degree, p.left.degree));
    }
    if (closed)
        for (bool upper : {false, true}) {
            const auto &first = upper ? patches.front().top : patches.front().bottom;
            const auto &last = upper ? patches.back().top : patches.back().bottom;
            require(curve_array_closed(cartesian(first.pole(0)),
                                       cartesian(last.pole(last.count() - 1))),
                    "closed guided profile endpoints do not coincide");
        }
    if (policy.chord_tolerance) {
        double required = std::ceil(std::sqrt(bound / (*policy.chord_tolerance)));
        require(std::isfinite(required) && required <= policy.max_segments,
                "guided surface tolerance exceeds budget");
        divisions = std::max(divisions, unsigned(required));
    }
    require(divisions <= policy.max_segments, "guided surface subdivision budget");
    // A single linear rail span needs only its end rows in the default mesh.
    // With a requested tolerance retain the full rational/mixed derivative bound.
    const unsigned v_divisions =
        !policy.chord_tolerance &&
                std::all_of(patches.begin(), patches.end(), [](const auto &p) { return p.nv == 2; })
            ? 1
            : divisions;
    std::vector<double> vs;
    std::vector<std::vector<double>> us;
    std::size_t columns = 1;
    Json counts = Json::array();
    for (auto &p : patches) {
        require(p.bottom.pieces.size() <= policy.max_segments / divisions &&
                    p.left.pieces.size() <= policy.max_segments / divisions,
                "guided grid budget");
        auto u = grid(p.bottom.breaks, divisions), v = grid(p.left.breaks, v_divisions);
        columns += u.size() - 1;
        us.push_back(std::move(u));
        vs.insert(vs.end(), v.begin(), v.end());
        require(vs.size() <= policy.max_segments * std::size_t(4), "guided shared grid budget");
        unique(vs);
        counts.push_back({{"u_degree", p.bottom.degree},
                          {"v_degree", p.left.degree},
                          {"u_poles", p.nu},
                          {"v_poles", p.nv}});
    }
    require(vs.size() <= policy.max_segments && columns <= policy.max_segments / vs.size(),
            "guided surface vertex budget");
    GuidedMesh out;
    for (unsigned end = 0; end < 2; ++end)
        out.cap_boundaries_closed[end] =
            curve_array_closed(patches.front().at(0, end), patches.back().at(1, end));
    for (auto v : vs) {
        std::vector<Point3> ring;
        for (std::size_t i = 0; i < count; ++i)
            for (std::size_t u = 0; u + 1 < us[i].size(); ++u)
                ring.push_back(patches[i].at(us[i][u], v));
        ring.push_back(closed ? ring.front() : patches.back().at(1, v));
        out.rings.push_back(std::move(ring));
    }
    out.note = {{"method", "native_control_net_coons"},
                {"profile_closed", closed},
                {"control_blend", "half_zero_half_one_odd_middle_half"},
                {"arc_max_span_degrees", 120},
                {"composite_guide_parameterization", length_weighted
                                                         ? "sequential_control_polygon_length"
                                                         : "sequential_equal_intervals"},
                {"periodic_conic_opening", "source_seam_parameter_zero"},
                {"patches", counts},
                {"native_fit_tolerance_behavior", "not_fully_reproduced"},
                {"polyline_parameterization", "normalized_chord_length"}};
    if (policy.chord_tolerance)
        out.note["surface_to_mesh_error_bound"] = bound / double(divisions) / divisions;
    return out;
}
} // namespace p3d
