// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Source area visitor adapted from cv_properties.cpp (see THIRD_PARTY.md).
// Changes: P3D dispatch/arithmetic, immutable JSON sources and bounded work.
#include "native_curve_area.hpp"
#include "native_curve_conversion.hpp"
#include "native_pcurve_points.hpp"

namespace p3d::curve_detail {
namespace {
double finite(double v) {
    require(std::isfinite(v), "native source area nonfinite arithmetic");
    return v;
}
double number(const Json &v) {
    require(v.is_number(), "native source area numeric layout");
    return finite(v.get<double>());
}
Point3 point(const Json &v, const std::string &prefix) {
    return {number(v.at(prefix + "X")), number(v.at(prefix + "Y")), number(v.at(prefix + "Z"))};
}
Point3 subtract(Point3 a, Point3 b) {
    for (unsigned i = 0; i < 3; ++i)
        a[i] = finite(a[i] - b[i]);
    return a;
}
Point3 cross(Point3 a, Point3 b) {
    Point3 out;
    for (unsigned i = 0; i < 3; ++i)
        out[i] = finite(a[(i + 1) % 3] * b[(i + 2) % 3] - a[(i + 2) % 3] * b[(i + 1) % 3]);
    return out;
}
double dot(Point3 a, Point3 b) {
    return finite((a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]);
}
Point3 identity(Point3 p, bool vector = false) {
    // Native point/vector transform preserves the disconnected-coordinate sentinel.
    for (double x : p)
        if (x == std::numeric_limits<double>::max())
            return p;
    Point3 out;
    for (unsigned i = 0; i < 3; ++i) {
        out[i] = finite((p[1] * (i == 1 ? 1. : 0.) + p[0] * (i == 0 ? 1. : 0.)) +
                        p[2] * (i == 2 ? 1. : 0.));
        if (!vector)
            out[i] += 0.;
    }
    return out;
}
void moment(BsplineArea &out, Point3 centroid, Point3 normal, double scale) {
    for (unsigned r = 0; r < 3; ++r) {
        out.normal_sum[r] = finite(out.normal_sum[r] + normal[r] * scale);
        const double a = finite(scale * centroid[r]);
        for (unsigned c = 0; c < 3; ++c)
            out.centroid_tensor[r][c] = finite(out.centroid_tensor[r][c] + a * normal[c]);
    }
}
void segment(BsplineArea &out, Point3 a, Point3 b) {
    a = subtract(a, out.reference);
    b = subtract(b, out.reference);
    Point3 centroid;
    for (unsigned i = 0; i < 3; ++i)
        centroid[i] = finite(a[i] * (1. / 3) + b[i] * (1. / 3));
    moment(out, centroid, cross(a, b), .5);
}
struct Visitor {
    swept_detail::TubeBudget &budget;
    void charge(std::size_t n) {
        BezierWork{budget.work, budget.max_work}.charge(n);
    }
    void enter(unsigned depth) {
        require(depth <= 256, "native source area nesting limit exceeded");
        charge(1);
    }
    const Json &members(const Json &v) {
        require(v.is_object() && v.value("_type", std::string()) == "CurveVector" &&
                    v.contains("type") && v.at("type").is_number_integer(),
                "native source area CurveVector layout");
        const auto &m = v.at("curves");
        require(m.is_array(), "native source area member layout");
        return m;
    }
    BsplineCurve spline(const Json &v) {
        const auto &p = v.at("poles");
        require(p.is_array() && p.size() % 3 == 0 && p.size() / 3 <= budget.max_control_points,
                "native source area B-spline control budget");
        charge(p.size());
        auto c = BsplineCurve::from_bgfb(v);
        require(c.order() <= 26, "native source area B-spline order");
        return c;
    }
    struct Reference {
        Point3 point;
        bool weight_fallback = false;
    };
    std::optional<Reference> start(const Json &v, unsigned depth) {
        enter(depth);
        if (v.is_null())
            return {};
        const auto type = v.at("_type").get<std::string>();
        if (type == "CurveVector") {
            for (const auto &m : members(v))
                if (auto p = start(m.at("geometry"), depth + 1))
                    return p;
            return {};
        }
        if (type == "BsplineCurve") {
            const auto c = spline(v);
            charge(c.knots().size() + 8 * std::size_t(c.order()) * c.order());
            const auto p = detail::pcurve_point(c, 0);
            return Reference{p.point, p.zero_weight_fallback};
        }
        charge(32);
        if (type == "LineSegment")
            return Reference{point(v.at("segment"), "point0")};
        if (type == "LineString") {
            const auto &p = v.at("points");
            require(p.is_array() && p.size() % 3 == 0,
                    "native source area reference polyline layout");
            if (p.empty())
                return {};
            return Reference{{number(p[0]), number(p[1]), number(p[2])}};
        }
        require(type == "EllipticArc", "native source area reference primitive unsupported");
        const auto &a = v.at("arc");
        auto p = point(a, "center");
        const auto x = point(a, "vector0"), y = point(a, "vector90");
        const double angle = number(a.at("startRadians"));
        for (unsigned i = 0; i < 3; ++i)
            p[i] = finite((p[i] + x[i] * std::cos(angle)) + y[i] * std::sin(angle));
        return Reference{p};
    }
    std::optional<std::array<Point3, 2>> ends(const Json &v, unsigned depth = 0) {
        enter(depth);
        if (v.is_null())
            return {};
        const auto type = v.at("_type").get<std::string>();
        if (type == "CurveVector") {
            std::optional<std::array<Point3, 2>> out;
            for (const auto &m : members(v)) {
                auto e = ends(m.at("geometry"), depth + 1);
                if (e) {
                    if (!out)
                        out = e;
                    else
                        (*out)[1] = (*e)[1];
                }
            }
            return out;
        }
        if (type == "BsplineCurve") {
            const auto c = spline(v);
            charge(c.knots().size() + 16 * std::size_t(c.order()) * c.order());
            return primitive_endpoints(v, &c);
        }
        require(type == "LineSegment" || type == "LineString" || type == "EllipticArc",
                "native source area endpoint primitive unsupported");
        charge(32);
        return primitive_endpoints(v);
    }
    // Basic-region nested groups recurse through the same visitor and reference;
    // they do not independently resolve their own boundary types or normal.
    void visit(const Json &v, BsplineArea &out, unsigned depth) {
        enter(depth);
        const auto type = v.at("_type").get<std::string>();
        if (type == "CurveVector") {
            for (const auto &m : members(v))
                visit(m.at("geometry"), out, depth + 1);
        } else if (type == "BsplineCurve") {
            accumulate_native_bspline_area(spline(v), out, {budget.work, budget.max_work});
        } else if (type == "LineSegment") {
            charge(64);
            const auto &s = v.at("segment");
            segment(out, identity(point(s, "point0")), identity(point(s, "point1")));
        } else if (type == "LineString") {
            const auto &p = v.at("points");
            require(p.is_array() && p.size() % 3 == 0 && p.size() / 3 <= budget.max_control_points,
                    "native source area polyline layout/control budget");
            auto at = [&](std::size_t i) {
                // Unlike the newer upstream visitor, this native polyline
                // branch sends the stored points directly to AddSegment.
                return Point3{number(p[3 * i]), number(p[3 * i + 1]), number(p[3 * i + 2])};
            };
            for (std::size_t i = 1; i < p.size() / 3; ++i) {
                charge(64);
                segment(out, at(i - 1), at(i));
            }
        } else if (type == "EllipticArc") {
            charge(256);
            const auto &a = v.at("arc");
            const auto center = identity(point(a, "center"));
            const auto x = identity(point(a, "vector0"), true);
            const auto y = identity(point(a, "vector90"), true);
            const double start = number(a.at("startRadians")), sweep = number(a.at("sweepRadians"));
            const double end = finite(start + sweep), alpha = sweep * .5;
            Point3 first, last, centroid;
            const double radius = finite((std::sin(alpha) + std::sin(alpha)) / (3. * alpha));
            const double middle = finite(alpha + start);
            for (unsigned i = 0; i < 3; ++i) {
                first[i] = finite((center[i] + x[i] * std::cos(start)) + y[i] * std::sin(start));
                last[i] = finite((center[i] + x[i] * std::cos(end)) + y[i] * std::sin(end));
                centroid[i] = finite((center[i] + x[i] * (std::cos(middle) * radius)) +
                                     y[i] * (std::sin(middle) * radius));
            }
            segment(out, first, center);
            segment(out, center, last);
            moment(out, subtract(centroid, out.reference), cross(x, y), alpha);
        } else
            require(false, "native source area visitor primitive unsupported");
    }
    CurveVectorArea area(const Json &v, unsigned depth = 0) {
        enter(depth);
        const auto &m = members(v);
        CurveVectorArea out;
        out.report = {{"boundary_type", v.at("type")}, {"children", Json::array()}};
        const auto reference = start(v, depth);
        if (!reference) {
            out.report["failure"] = "no_source_reference_point";
            return out;
        }
        const auto &type = v.at("type");
        if (type == 2 || type == 3) {
            out.value.reference = reference->point;
            out.value.reference_weight_fallback = reference->weight_fallback;
            for (const auto &p : m)
                visit(p.at("geometry"), out.value, depth + 1);
        } else if (type == 4 || type == 5) {
            std::vector<BsplineArea> children;
            std::size_t largest = 0;
            double max_area = 0;
            for (const auto &p : m) {
                const auto &child = p.at("geometry");
                if (child.value("_type", std::string()) != "CurveVector") {
                    out.report["failure"] = "region_member_is_not_curve_vector";
                    return out;
                }
                auto a = area(child, depth + 1);
                if (!a.value.valid) {
                    out.report["failure"] = "child_area_failed";
                    out.report["failure_member"] = children.size();
                    return out;
                }
                if (std::abs(a.value.area) > max_area) {
                    largest = children.size();
                    max_area = std::abs(a.value.area);
                }
                children.push_back(a.value);
            }
            require(!children.empty(), "native region area missing reference child");
            const auto positive = children[largest].normal;
            out.report["largest_member"] = largest;
            for (std::size_t i = 0; i < children.size(); ++i) {
                charge(64);
                auto &a = children[i];
                const double direction = dot(a.normal, positive);
                const bool reverse = i != largest && (type == 4 ? direction > 0 : direction < 0);
                if (reverse)
                    for (auto &x : a.normal)
                        x = -x;
                moment(out.value, a.centroid, a.normal, a.area);
                out.report["children"].push_back({{"source_member", i}, {"reversed", reverse}});
            }
        } else {
            out.report["failure"] = "boundary_type_has_no_area";
            return out;
        }
        finish_native_curve_area(out.value);
        if (!out.value.valid)
            out.report["failure"] = "zero_accumulated_area";
        return out;
    }
};
} // namespace
CurveVectorArea native_curve_vector_area(const Json &v, swept_detail::TubeBudget &b) {
    auto out = Visitor{b}.area(v);
    out.report["native_result"] = out.value.valid;
    out.report["work_used"] = b.work;
    return out;
}
std::optional<std::array<Point3, 2>> native_source_curve_endpoints(const Json &v,
                                                                   swept_detail::TubeBudget &b) {
    return Visitor{b}.ends(v);
}
Json native_facet_orientation_flags(const Json &source, Point3 tangent,
                                    swept_detail::TubeBudget &b) {
    for (double x : tangent)
        finite(x);
    Visitor visitor{b};
    const auto &members = visitor.members(source);
    const bool parity = source.at("type") == 4;
    Json flags = Json::array(), locations = Json::array(), rings = Json::array();
    bool ok = true;
    const auto count = parity ? members.size() : 1;
    for (std::size_t i = 0; i < count; ++i) {
        const auto &ring = parity ? members[i].at("geometry") : source;
        const auto area = visitor.area(ring);
        Json report{{"source_ring", i}, {"area_valid", area.value.valid}};
        if (!area.value.valid) {
            const auto e = visitor.ends(ring);
            const bool closed = e && endpoint_pair_closed((*e)[0], (*e)[1]);
            report["source_endpoint_closed"] = closed;
            if (closed) {
                ok = false;
                report["flag_index"] = nullptr;
                rings.push_back(std::move(report));
                break;
            }
            if (parity) {
                report["flag_index"] = nullptr;
                rings.push_back(std::move(report));
                continue;
            }
            flags.push_back(false);
        } else {
            const double direction = dot(tangent, area.value.normal);
            const bool positive = direction > 1e-14;
            flags.push_back(parity && i > 0 ? positive : !positive);
            report["normal"] = area.value.normal;
            report["dot"] = direction;
        }
        report["flag_index"] = flags.size() - 1;
        locations.push_back(i);
        rings.push_back(std::move(report));
    }
    return {{"scope", "native_source_facet_orientation_flags"},
            {"native_result", ok},
            {"flags", std::move(flags)},
            {"source_rings", std::move(locations)},
            {"rings", std::move(rings)},
            {"work_used", b.work}};
}
} // namespace p3d::curve_detail
