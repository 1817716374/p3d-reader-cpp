#include "p3d/pcurve.hpp"
#include "loft_curve.hpp"
#include "native_pcurve_points.hpp"

namespace p3d {
namespace {
double number(const Json &j) {
    require(j.is_number(), "native boundary coordinate is not numeric");
    const double v = j.get<double>();
    require(std::isfinite(v), "native boundary coordinate is not finite");
    return v;
}
Point3 point(const Json &j, const std::string &prefix) {
    return {number(j.at(prefix + "X")), number(j.at(prefix + "Y")), number(j.at(prefix + "Z"))};
}
Json curve_table(const BsplineCurve &c) {
    std::vector<double> poles;
    for (const auto &p : c.poles())
        poles.insert(poles.end(), p.begin(), p.end());
    return {{"_type", "BsplineCurve"},
            {"order", c.order()},
            {"closed", c.closed()},
            {"poles", poles},
            {"weights", c.rational() ? Json(c.weights()) : Json(nullptr)},
            {"knots", c.knots()}};
}
BsplineCurve polyline(const Json &poles) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", 2},
                                    {"closed", false},
                                    {"poles", poles},
                                    {"weights", nullptr},
                                    {"knots", nullptr}});
}
double magnitude(const Point3 &p) {
    const double length = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    require(std::isfinite(length), "native boundary axis length overflow");
    return length;
}
bool closed_points(const Point3 &a, const Point3 &b) {
    double distance = 0, scale = 1;
    for (unsigned k = 0; k < 3; ++k) {
        distance += (a[k] - b[k]) * (a[k] - b[k]);
        scale += a[k] * a[k] + b[k] * b[k];
    }
    require(std::isfinite(distance) && std::isfinite(scale), "native boundary closure overflow");
    return distance < scale * 1e-20;
}
BsplineCurve ellipse(const Json &value) {
    const auto &a = value.at("arc");
    const auto center = point(a, "center"), x = point(a, "vector0"), y = point(a, "vector90");
    const double lx = magnitude(x), ly = magnitude(y);
    constexpr double pi = 3.141592653589793;
    auto angle = [&](double t) {
        const double s = lx * std::sin(t), c = ly * std::cos(t);
        if (std::abs(s) < 1e-5 || std::abs(c) < 1e-5)
            return t;
        const double adjusted = std::atan2(s, c);
        if (std::abs(t) < pi)
            return adjusted;
        return adjusted + std::copysign(std::ceil(std::abs(t) / pi) * pi, t);
    };
    const double start = angle(number(a.at("startRadians")));
    const double sweep = angle(number(a.at("sweepRadians")));
    require(std::isfinite(start) && std::isfinite(sweep) && std::isfinite(start + sweep),
            "native ellipse angle conversion overflow");
    const unsigned spans = std::abs(sweep) <= 2.0943951023931953   ? 1
                           : std::abs(sweep) <= 4.1887902047863905 ? 2
                                                                   : 3;
    const bool closed = std::abs(sweep) > 6.283185307178586;
    const double w = std::cos(std::abs(sweep) * .5 / spans), twice = 2 * w, blend = 2 * (1 + w);
    require(w != 0, "native ellipse control weight is zero");
    const double half_step = spans == 3 ? sweep / 6 : spans == 2 ? sweep * .25 : sweep * .5;
    std::vector<Point2> circle(2 * spans + 1);
    for (unsigned i = 0; i <= spans; ++i) {
        const double t = i == spans ? start + sweep : start + double(2 * i) * half_step;
        circle[2 * i] = {std::cos(t), std::sin(t)};
    }
    for (unsigned i = 0; i < spans; ++i) {
        const double t = start + double(2 * i + 1) * half_step;
        const Point2 mid{std::cos(t), std::sin(t)};
        for (unsigned k = 0; k < 2; ++k)
            circle[2 * i + 1][k] =
                (blend * mid[k] - (circle[2 * i][k] + circle[2 * i + 2][k])) / twice;
    }
    std::vector<double> weights, poles;
    for (unsigned i = 0; i < circle.size(); ++i) {
        const double weight = i % 2 ? w : 1;
        weights.push_back(weight);
        // setEllipticArc first weights the unit-circle controls; setbyGeEllipse3d
        // deweights those controls, applies the original basis, then reweights.
        const double u = (circle[i][0] * weight) / weight, v = (circle[i][1] * weight) / weight;
        for (unsigned k = 0; k < 3; ++k)
            poles.push_back(((center[k] + x[k] * u) + y[k] * v) * weight);
    }
    std::vector<double> inside;
    if (spans == 2)
        inside = {.5, .5};
    if (spans == 3)
        inside = closed ? std::vector<double>{0, 1. / 3, 1. / 3, 2. / 3, 2. / 3, 1}
                        : std::vector<double>{1. / 3, 1. / 3, 2. / 3, 2. / 3};
    std::vector<double> knots(weights.size() + (closed ? 5 : 3), 0);
    std::copy(inside.begin(), inside.end(), knots.begin() + 3);
    for (unsigned i = 0; i < 3; ++i) {
        if (closed) {
            knots[i] = knots[i + inside.size() + 1] - 1;
            knots[3 + inside.size() + i] = knots[2 + i] + 1;
        } else
            knots[3 + inside.size() + i] = 1;
    }
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", 3},
                                    {"closed", closed},
                                    {"poles", poles},
                                    {"weights", weights},
                                    {"knots", knots}});
}
struct Boundaries {
    const BsplineSurface &surface;
    const PCurveBoundaryStrokeOptions &options;
    unsigned visits = 0, controls = 0, members = 0, opened_controls = 0;
    std::string active_path;
    Json sources = Json::array(), ignored = Json::array();
    std::map<const Json *, std::optional<BsplineCurve>> cache;
    std::vector<std::vector<BsplineCurve>> loops;
    Boundaries(const BsplineSurface &s, const PCurveBoundaryStrokeOptions &o)
        : surface(s), options(o) {}
    void visit(unsigned depth) {
        require(depth <= options.max_tree_depth, "native boundary tree depth budget exhausted");
        require(visits < options.max_tree_visits, "native boundary tree visit budget exhausted");
        ++visits;
    }
    const std::optional<BsplineCurve> &convert(const Json &v) {
        const auto found = cache.find(&v);
        if (found != cache.end())
            return found->second;
        const auto type = v.at("_type").get<std::string>();
        if (type == "CurveVector" || type == "PointString")
            return cache.emplace(&v, std::nullopt).first->second;
        const auto *poles = v.contains("poles")    ? &v.at("poles")
                            : v.contains("points") ? &v.at("points")
                                                   : nullptr;
        if (poles && poles->is_array())
            require(poles->size() / 3 <= options.max_controls - controls,
                    "native boundary source control budget exhausted");
        std::optional<BsplineCurve> c;
        if (type == "BsplineCurve")
            c = BsplineCurve::from_bgfb(v);
        else if (type == "AkimaCurve")
            c = AkimaCurve::from_bgfb(v).bspline();
        else if (type == "InterpolationCurve")
            c = InterpolationCurve::from_bgfb(v).bspline();
        else if (type == "TransitionSpiral")
            c = TransitionSpiral::from_bgfb(v).native_fit(options.max_integration_intervals).curve;
        else if (type == "EllipticArc")
            c = ellipse(v);
        else if (type == "LineSegment") {
            const auto &s = v.at("segment");
            const auto a = point(s, "point0"), b = point(s, "point1");
            c = polyline({a[0], a[1], a[2], b[0], b[1], b[2]});
        } else if (type == "LineString") {
            const auto &p = v.at("points");
            require(p.is_array() && p.size() % 3 == 0, "native boundary line-string XYZ triplets");
            if (p.size() >= 6)
                c = polyline(p);
        } else if (type != "CurveVector" && type != "PointString")
            throw std::runtime_error("unsupported native boundary primitive: " + type);
        if (c) {
            require(c->poles().size() <= options.max_controls - controls,
                    "native boundary converted control budget exhausted");
            controls += unsigned(c->poles().size());
        }
        return cache.emplace(&v, std::move(c)).first->second;
    }
    bool endpoints(const Json &v, Point3 &first, Point3 &last, unsigned depth) {
        visit(depth);
        if (v.is_null())
            return false;
        const auto type = v.at("_type").get<std::string>();
        if (type == "CurveVector") {
            bool found = false;
            const auto &entries = v.at("curves");
            require(entries.is_null() || entries.is_array(), "native boundary endpoint array");
            for (const auto &entry : entries) {
                Point3 a{}, b{};
                if (endpoints(entry.at("geometry"), a, b, depth + 1)) {
                    if (!found)
                        first = a;
                    last = b;
                    found = true;
                }
            }
            return found;
        }
        if (type == "LineString" || type == "PointString") {
            const auto &p = v.at("points");
            require(p.is_array() && p.size() % 3 == 0, "native boundary endpoint XYZ triplets");
            if (p.empty())
                return false;
            for (unsigned k = 0; k < 3; ++k) {
                first[k] = number(p[k]);
                last[k] = number(p[p.size() - 3 + k]);
            }
            return true;
        }
        if (type == "EllipticArc") {
            const auto &a = v.at("arc");
            const auto c = point(a, "center"), x = point(a, "vector0"), y = point(a, "vector90");
            const double start = number(a.at("startRadians")),
                         end = start + number(a.at("sweepRadians"));
            require(std::isfinite(end), "native boundary ellipse endpoint angle overflow");
            for (unsigned k = 0; k < 3; ++k) {
                first[k] = (c[k] + x[k] * std::cos(start)) + y[k] * std::sin(start);
                last[k] = (c[k] + x[k] * std::cos(end)) + y[k] * std::sin(end);
            }
            return true;
        }
        const auto &c = convert(v);
        if (!c)
            return false;
        first = detail::pcurve_point(*c, 0).point;
        last = detail::pcurve_point(*c, 1).point;
        return true;
    }
    void collect(const Json &v, const std::string &path, unsigned depth = 0) {
        active_path = path;
        visit(depth);
        if (v.is_null())
            return;
        require(v.at("_type") == "CurveVector", "native boundary requires CurveVector");
        const auto &t = v.at("type");
        require(t.is_number_integer() && t >= 0 && t <= 5, "unknown native boundary type");
        const auto type = t.get<unsigned>();
        const auto &entries = v.at("curves");
        require(entries.is_null() || entries.is_array(), "native boundary member array");
        if (type == 4 || type == 5) {
            for (std::size_t i = 0; i < entries.size(); ++i) {
                const auto p = path + "/curves/" + std::to_string(i) + "/geometry";
                const auto &child = entries[i].at("geometry");
                if (child.is_object() && child.value("_type", std::string()) == "CurveVector")
                    collect(child, p, depth + 1);
                else {
                    visit(depth + 1);
                    ignored.push_back(
                        {{"source_path", p}, {"reason", "region_member_is_not_curve_array"}});
                }
            }
            return;
        }
        if (type == 0) {
            ignored.push_back({{"source_path", path}, {"reason", "boundary_type_none"}});
            return;
        }
        if (type == 1) {
            Point3 a{}, b{};
            if (!endpoints(v, a, b, depth) || !closed_points(a, b)) {
                ignored.push_back({{"source_path", path}, {"reason", "open_boundary_not_closed"}});
                return;
            }
        }
        require(loops.size() < options.sampling.max_loops,
                "native boundary loop count budget exhausted");
        std::vector<BsplineCurve> loop;
        Json provenance = Json::array();
        for (std::size_t i = 0; i < entries.size(); ++i) {
            active_path = path + "/curves/" + std::to_string(i) + "/geometry";
            visit(depth + 1);
            require(members < options.sampling.max_curves,
                    "native boundary member count budget exhausted");
            ++members;
            const auto &value = entries[i].at("geometry");
            if (value.is_null()) {
                ignored.push_back({{"source_path", active_path}, {"reason", "null_member"}});
                continue;
            }
            const auto &source = convert(value);
            if (!source) {
                ignored.push_back({{"source_path", active_path},
                                   {"reason", "native_trim_conversion_unavailable"},
                                   {"source_type", value.at("_type")}});
                continue;
            }
            Json opening = nullptr;
            auto table =
                source->closed()
                    ? loft_detail::open_periodic_boundary(*source, options.max_controls, &opening)
                          .table()
                    : curve_table(*source);
            const auto count = table.at("poles").size() / 3;
            require(count <= options.max_controls - opened_controls,
                    "native boundary opened control budget exhausted");
            opened_controls += unsigned(count);
            const std::array<Point2, 2> domains{surface.u().knot_domain(),
                                                surface.v().knot_domain()};
            for (unsigned k = 0; k < 2; ++k) {
                if (domains[k] == Point2{0, 1})
                    continue;
                const double scale = 1 / (domains[k][1] - domains[k][0]),
                             shift = -domains[k][0] * scale;
                for (std::size_t j = 0; j < count; ++j) {
                    const double weight =
                        table.at("weights").is_null() ? 1 : number(table["weights"][j]);
                    table["poles"][3 * j + k] =
                        number(table["poles"][3 * j + k]) * scale + shift * weight;
                }
            }
            loop.push_back(BsplineCurve::from_bgfb(table));
            provenance.push_back({{"source_path", active_path},
                                  {"source_type", value.at("_type")},
                                  {"source_curve_knot_domain", source->knot_domain()},
                                  {"source_curve_closed", source->closed()},
                                  {"prepared_curve_knot_domain", loop.back().knot_domain()},
                                  {"prepared_poles", count},
                                  {"opening", opening}});
        }
        if (loop.empty()) {
            ignored.push_back({{"source_path", path}, {"reason", "empty_converted_boundary"}});
            return;
        }
        sources.push_back({{"source_path", path},
                           {"source_boundary_type", type},
                           {"effective_boundary_type", type == 1 ? 2u : type},
                           {"members", provenance}});
        loops.push_back(std::move(loop));
    }
};
} // namespace
PCurveLoopStrokes sample_native_surface_boundaries(const BsplineSurface &surface,
                                                   const PCurveBoundaryStrokeOptions &options) {
    require(options.max_controls && options.max_tree_visits && options.max_tree_depth &&
                options.max_integration_intervals,
            "native boundary preparation budgets must be positive");
    require(std::isfinite(options.sampling.uv_tolerance) &&
                std::isfinite(options.sampling.spatial_tolerance) && options.sampling.max_points &&
                options.sampling.max_evaluations && options.sampling.max_loops &&
                options.sampling.max_curves,
            "native boundary sampling options are invalid");
    Boundaries builder{surface, options};
    PCurveLoopStrokes result;
    try {
        builder.collect(surface.boundaries(), "");
        result = sample_native_pcurve_loops(surface, builder.loops, options.sampling);
    } catch (const std::exception &e) {
        result.loops.clear();
        result.report = {{"status", "incomplete"},
                         {"reason", e.what()},
                         {"failed_source_path", builder.active_path},
                         {"sample_count", 0},
                         {"sampled_tolerances_met", false},
                         {"continuous_error_bound", nullptr}};
    }
    result.report["algorithm"] = "native_surface_boundary_restroke";
    result.report["source_region_conversion"] = "surface_boundary_tree";
    result.report["parameter_coordinates"] = "surface_fractions";
    result.report["implicit_closure"] = "not_performed";
    result.report["source_knot_domain"] = {surface.u().knot_domain(), surface.v().knot_domain()};
    result.report["outer_boundary_active"] = surface.outer_boundary_active();
    result.report["boundary_sources"] = std::move(builder.sources);
    result.report["ignored"] = std::move(builder.ignored);
    result.report["tree_visits"] = builder.visits;
    result.report["converted_controls"] = builder.controls;
    result.report["opened_controls"] = builder.opened_controls;
    return result;
}
} // namespace p3d
