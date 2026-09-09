#include "loft_curve.hpp"
namespace p3d {
namespace {
using loft_detail::cartesian;
using loft_detail::Curve;
using loft_detail::H;
double number(const Json &v) {
    require(v.is_number(), "loft coordinate must be numeric");
    const double x = v.get<double>();
    require(std::isfinite(x), "nonfinite loft coordinate");
    return x;
}
Point3 point(const Json &v, const std::string &prefix) {
    return {number(v.at(prefix + "X")), number(v.at(prefix + "Y")), number(v.at(prefix + "Z"))};
}
H homogeneous(Point3 p, double w = 1) {
    return {p[0] * w, p[1] * w, p[2] * w, w};
}
const Json &members(const Json &v) {
    require(v.at("_type") == "CurveVector" && v.at("curves").is_array() && !v.at("curves").empty(),
            "loft requires nonempty source curve arrays");
    return v.at("curves");
}
int boundary_type(const Json &v) {
    const auto &t = v.at("type");
    require(t.is_number_integer() && t >= 1 && t <= 5, "loft source boundary type");
    return t.get<int>();
}
bool closed(Point3 a, Point3 b) {
    double d2 = 0, scale2 = 1;
    for (unsigned k = 0; k < 3; ++k) {
        d2 += (a[k] - b[k]) * (a[k] - b[k]);
        scale2 += a[k] * a[k] + b[k] * b[k];
    }
    return std::isfinite(d2) && std::isfinite(scale2) && d2 < scale2 * 1.0000000000000001e-20;
}
struct Builder {
    unsigned limit;
    Curve primitive(const Json &v, const std::string &path) {
        const auto type = v.at("_type").get<std::string>();
        Curve c;
        if (type == "BsplineCurve")
            c = Curve::from_bspline(BsplineCurve::from_bgfb(v), limit);
        else if (type == "AkimaCurve" || type == "InterpolationCurve" || type == "TransitionSpiral")
            throw std::runtime_error("native loft conversion returns no curve for " + type +
                                     " at " + path);
        else if (type == "LineSegment" || type == "LineString") {
            std::vector<Point3> points;
            if (type == "LineSegment") {
                const auto &s = v.at("segment");
                points = {point(s, "point0"), point(s, "point1")};
            } else {
                const auto &a = v.at("points");
                require(a.is_array() && a.size() % 3 == 0 && a.size() / 3 <= limit,
                        "loft line-string XYZ/control budget");
                for (std::size_t i = 0; i < a.size(); i += 3)
                    points.push_back({number(a[i]), number(a[i + 1]), number(a[i + 2])});
            }
            c.knots.push_back(0);
            for (auto p : points) {
                double length = 0;
                if (!c.poles.empty()) {
                    auto previous = cartesian(c.poles.back());
                    length = std::hypot(p[0] - previous[0], p[1] - previous[1], p[2] - previous[2]);
                    if (length == 0)
                        continue;
                }
                c.poles.push_back(homogeneous(p));
                c.knots.push_back(c.knots.back() + length);
            }
            require(c.poles.size() >= 2 && c.knots.back() > 0 && std::isfinite(c.knots.back()),
                    "degenerate loft line string");
            const double length = c.knots.back();
            c.knots.push_back(length);
            for (auto &t : c.knots)
                t /= length;
        } else if (type == "EllipticArc") {
            const auto &a = v.at("arc");
            const auto center = point(a, "center"), x = point(a, "vector0"),
                       y = point(a, "vector90");
            const double start = number(a.at("startRadians")), sweep = number(a.at("sweepRadians"));
            constexpr double pi = 3.1415926535897932384626433832795;
            require(sweep != 0 && std::abs(sweep) <= 2 * pi, "degenerate or multi-turn loft arc");
            const unsigned n = std::abs(sweep) <= 2 * pi / 3   ? 1
                               : std::abs(sweep) <= 4 * pi / 3 ? 2
                                                               : 3;
            c.degree = 2;
            c.rational = true;
            c.knots.assign(3, 0);
            auto at = [&](double t, double w) {
                H h{};
                for (unsigned k = 0; k < 3; ++k)
                    h[k] = center[k] * w + x[k] * std::cos(t) + y[k] * std::sin(t);
                h[3] = w;
                return h;
            };
            c.poles.push_back(at(start, 1));
            for (unsigned i = 0; i < n; ++i) {
                const double t0 = start + sweep * i / n, t1 = start + sweep * (i + 1) / n;
                c.poles.push_back(at((t0 + t1) / 2, std::cos((t1 - t0) / 2)));
                c.poles.push_back(at(t1, 1));
                c.knots.insert(c.knots.end(), i + 1 == n ? 3 : 2, double(i + 1) / n);
            }
            if (std::abs(sweep) == 2 * pi)
                c.poles.back() = c.poles.front();
        } else
            throw std::runtime_error("unsupported loft source primitive: " + type);
        c.check(limit);
        return c;
    }
    Curve guide(const Json &v, bool weighted, const std::string &path) {
        require(boundary_type(v) <= 3, "region or nested loft guide is not supported");
        const auto &array = members(v);
        require(array.size() <= limit, "loft guide member budget");
        auto c = primitive(array[0].at("geometry"), path + "/curves/0/geometry");
        const auto first = cartesian(c.poles.front());
        auto last = cartesian(c.poles.back());
        for (std::size_t i = 1; i < array.size(); ++i) {
            auto next = primitive(array[i].at("geometry"),
                                  path + "/curves/" + std::to_string(i) + "/geometry");
            last = cartesian(next.poles.back());
            c = loft_detail::append(std::move(c), std::move(next), weighted, limit);
        }
        require(!weighted || !closed(first, last),
                "native close/reopen of length-weighted closed guides is not yet supported");
        return c;
    }
};
double blend(std::size_t i, std::size_t n) {
    return i < n / 2 ? 0 : n % 2 && i == n / 2 ? .5 : 1;
}
BsplineSurface patch(Curve bottom, Curve top, Curve left, Curve right, unsigned limit) {
    const auto p00 = cartesian(bottom.poles.front()), p10 = cartesian(bottom.poles.back()),
               p01 = cartesian(top.poles.front()), p11 = cartesian(top.poles.back());
    auto corner = [](Point3 a, Point3 b) {
        for (unsigned k = 0; k < 3; ++k)
            require(std::abs(a[k] - b[k]) <= 1e-5, "loft source corner mismatch");
    };
    corner(p00, cartesian(left.poles.front()));
    corner(p10, cartesian(right.poles.front()));
    corner(p01, cartesian(left.poles.back()));
    corner(p11, cartesian(right.poles.back()));
    // The three-pole rule precedes bottom/top compatibility in native Coons.
    if (left.poles.size() == 3) {
        left.elevate(left.degree + 1, limit);
        right.elevate(right.degree + 1, limit);
    }
    if (bottom.poles.size() == 3) {
        bottom.elevate(bottom.degree + 1, limit);
        top.elevate(top.degree + 1, limit);
    }
    loft_detail::compatible({&bottom, &top}, limit);
    require(left.knots == right.knots && left.degree == right.degree, "loft guide compatibility");
    const auto nu = bottom.poles.size(), nv = left.poles.size();
    require(nv <= limit && nu <= limit / nv, "loft surface control budget");
    Json xyz = Json::array(), weights = Json::array();
    const bool rational = bottom.rational || top.rational || left.rational || right.rational;
    for (std::size_t j = 0; j < nv; ++j)
        for (std::size_t i = 0; i < nu; ++i) {
            const double u = blend(i, nu), v = blend(j, nv);
            const auto a0 = cartesian(bottom.poles[i]), a1 = cartesian(top.poles[i]),
                       b0 = cartesian(left.poles[j]), b1 = cartesian(right.poles[j]);
            const double w = (bottom.poles[i][3] * (1 - v) + top.poles[i][3] * v) *
                             (left.poles[j][3] * (1 - u) + right.poles[j][3] * u);
            H h{};
            h[3] = w;
            for (unsigned k = 0; k < 3; ++k) {
                const double corners = (1 - u) * (1 - v) * p00[k] + u * (1 - v) * p10[k] +
                                       (1 - u) * v * p01[k] + u * v * p11[k];
                h[k] = (a0[k] * (1 - v) + a1[k] * v + b0[k] * (1 - u) + b1[k] * u - corners) * w;
            }
            cartesian(h);
            for (unsigned k = 0; k < 3; ++k)
                xyz.push_back(h[k]);
            if (rational)
                weights.push_back(w);
        }
    return BsplineSurface::from_bgfb({{"_type", "BsplineSurface"},
                                      {"orderU", bottom.degree + 1},
                                      {"orderV", left.degree + 1},
                                      {"closedU", false},
                                      {"closedV", false},
                                      {"numPolesU", nu},
                                      {"numPolesV", nv},
                                      {"knotsU", bottom.knots},
                                      {"knotsV", left.knots},
                                      {"poles", xyz},
                                      {"weights", rational ? weights : Json(nullptr)},
                                      {"numRulesU", 0},
                                      {"numRulesV", 0},
                                      {"boundaries", nullptr},
                                      {"holeOrigin", 0}});
}
} // namespace
SectionLoft SectionLoft::from_bgfb(const Json &table, unsigned max_control_points) {
    require(table.at("_type") == "P3DSectionLoft", "expected BGFB P3DSectionLoft table");
    require(max_control_points >= 4, "loft control budget must be at least four");
    require(table.at("capped").is_boolean(), "loft cap flag");
    const auto &bottom = table.at("section0"), &top = table.at("section1"),
               &groups = table.at("guide_groups");
    require(groups.is_array() && !groups.empty() && groups.size() <= max_control_points,
            "loft guide groups");
    require(boundary_type(bottom) == boundary_type(top), "loft section boundary types must match");
    std::vector<std::pair<const Json *, const Json *>> loops;
    const bool parity = boundary_type(bottom) == 4;
    if (parity) {
        const auto &a = members(bottom), &b = members(top);
        require(a.size() == b.size() && a.size() == groups.size(),
                "loft parity loop correspondence");
        for (std::size_t i = 0; i < a.size(); ++i) {
            const auto &x = a[i].at("geometry"), &y = b[i].at("geometry");
            require(boundary_type(x) == (i ? 3 : 2) && boundary_type(y) == (i ? 3 : 2),
                    "loft parity outer/inner source order");
            loops.emplace_back(&x, &y);
        }
    } else {
        require(boundary_type(bottom) <= 3 && groups.size() == 1,
                "unsupported loft section region");
        loops.emplace_back(&bottom, &top);
    }
    Builder builder{max_control_points};
    SectionLoft out;
    out.source_ = table;
    std::size_t total = 0;
    Json loop_notes = Json::array();
    for (std::size_t loop = 0; loop < loops.size(); ++loop) {
        const auto &a = members(*loops[loop].first), &b = members(*loops[loop].second),
                   &g = groups[loop];
        const bool is_closed = boundary_type(*loops[loop].first) != 1;
        require(a.size() == b.size() && a.size() <= max_control_points && g.is_array() &&
                    g.size() == a.size() + (is_closed ? 0 : 1),
                "loft primitive/guide correspondence");
        require(g.size() >= 2 && g.size() <= 5000, "loft requires two to 5000 source guides");
        std::vector<Curve> lower, upper, guides;
        for (std::size_t i = 0; i < a.size(); ++i) {
            const auto suffix = (parity ? "/curves/" + std::to_string(loop) + "/geometry" : "") +
                                std::string("/curves/") + std::to_string(i) + "/geometry";
            lower.push_back(builder.primitive(a[i].at("geometry"), "/section0" + suffix));
            upper.push_back(builder.primitive(b[i].at("geometry"), "/section1" + suffix));
        }
        if (is_closed) {
            require(closed(cartesian(lower.front().poles.front()),
                           cartesian(lower.back().poles.back())) &&
                        closed(cartesian(upper.front().poles.front()),
                               cartesian(upper.back().poles.back())),
                    "loft closed section endpoints do not coincide");
        }
        bool equal_counts = true;
        const auto count = members(g[0]).size();
        for (const auto &v : g)
            equal_counts &= members(v).size() == count;
        for (std::size_t i = 0; i < g.size(); ++i)
            guides.push_back(
                builder.guide(g[i], !equal_counts,
                              "/guide_groups/" + std::to_string(loop) + "/" + std::to_string(i)));
        std::vector<Curve *> pointers;
        for (auto &c : guides)
            pointers.push_back(&c);
        loft_detail::compatible(pointers, max_control_points);
        for (std::size_t i = 0; i < a.size(); ++i) {
            auto surface = patch(std::move(lower[i]), std::move(upper[i]), guides[i],
                                 guides[(i + 1) % guides.size()], max_control_points);
            const auto count = surface.poles().size();
            require(count <= max_control_points - total, "loft total surface control budget");
            total += count;
            out.sides_.push_back({std::move(surface), loop, i});
        }
        loop_notes.push_back({{"loop_index", loop},
                              {"boundary_type", boundary_type(*loops[loop].first)},
                              {"side_count", a.size()},
                              {"guide_count", g.size()},
                              {"guide_parameterization", equal_counts ? "sequential_equal_domains"
                                                                      : "control_polygon_length"}});
    }
    out.report_ = {
        {"status", "valid"},
        {"representation", "derived_bspline_side_surfaces"},
        {"side_count", out.sides_.size()},
        {"control_point_count", total},
        {"loops", loop_notes},
        {"caps_requested", table.at("capped")},
        {"cap_status", table.at("capped").get<bool>() ? "not_reconstructed" : "not_requested"}};
    return out;
}
} // namespace p3d
