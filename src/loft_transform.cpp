#include <p3d/solid.hpp>
#include "internal.hpp"

namespace p3d {
namespace {
double number(const Json &v) {
    require(v.is_number(), "loft transform requires numeric coordinates");
    const auto x = v.get<double>();
    require(std::isfinite(x), "loft transform nonfinite coordinate or weight");
    return x;
}
bool angle_in_sweep(double angle, double start, double sweep) {
    constexpr double tau = 6.283185307179586;
    const double delta = angle - start, end = start + sweep;
    require(std::isfinite(delta) && std::isfinite(end), "loft arc angular range overflow");
    // Native angle normalization preserves exact one-period boundary cases.
    // Reducing every delta with fmod would change those inclusive tests.
    double adjusted = angle;
    if (sweep < 0) {
        if (delta < -tau)
            adjusted = start - std::fmod(-delta, tau);
        else if (delta > 0)
            adjusted = delta <= tau ? angle - tau : std::fmod(delta, tau) + (start - tau);
    } else {
        if (delta > tau)
            adjusted = start + std::fmod(delta, tau);
        else if (delta < 0)
            adjusted = delta >= -tau ? angle + tau : (start + tau) - std::fmod(-delta, tau);
    }
    require(std::isfinite(adjusted), "loft arc angular normalization overflow");
    return sweep < 0 ? adjusted <= start && adjusted >= end : adjusted >= start && adjusted <= end;
}
struct Placement {
    const Matrix4 &matrix;
    const LoftSourceTransformOptions &options;
    std::size_t points = 0, nodes = 0, skipped = 0;
    bool bspline_identity = true;
    std::string path;
    Json replacements = Json::array();

    Placement(const Matrix4 &m, const LoftSourceTransformOptions &o) : matrix(m), options(o) {
        require(o.max_depth <= 80, "loft transform depth ceiling exceeded");
        for (const auto &row : m)
            for (double x : row)
                require(std::isfinite(x), "loft transform nonfinite matrix");
        require(m[3] == std::array<double, 4>{0, 0, 0, 1}, "loft transform must be affine");
        // GeBsplineCurve::transformCurve invokes native GeTransform::isIdentity:
        // the linear test is inclusive, while translation uses strict bounds.
        for (unsigned r = 0; r < 3; ++r) {
            bspline_identity &= m[r][3] > -1e-10 && m[r][3] < 1e-10;
            for (unsigned c = 0; c < 3; ++c)
                bspline_identity &= std::abs(m[r][c] - (r == c ? 1. : 0.)) <= 1e-12;
        }
    }
    Point3 point(Point3 p, double weight) {
        require(points < options.max_points, "loft transform point budget exceeded");
        ++points;
        Point3 out{};
        for (unsigned r = 0; r < 3; ++r) {
            out[r] = ((matrix[r][0] * p[0] + matrix[r][1] * p[1]) + matrix[r][2] * p[2]) +
                     matrix[r][3] * weight;
            require(std::isfinite(out[r]), "loft transformed coordinate overflow");
        }
        return out;
    }
    Point3 named(Json &j, const char *prefix, double w) {
        const std::string key(prefix);
        const auto p =
            point({number(j.at(key + "X")), number(j.at(key + "Y")), number(j.at(key + "Z"))}, w);
        for (unsigned a = 0; a < 3; ++a)
            j[key + "XYZ"[a]] = p[a];
        return p;
    }
    void packed(Json &j, const Json *weights, bool skip) {
        require(j.is_array() && j.size() % 3 == 0, "loft transform XYZ count");
        const auto count = j.size() / 3;
        require(count <= options.max_points - points, "loft transform point budget exceeded");
        require(!weights || weights->is_null() || weights->is_array(),
                "loft transform weights must be an array or null");
        const bool rational = weights && !weights->is_null() && !weights->empty();
        require(!rational || (weights->is_array() && weights->size() == count),
                "loft transform pole/weight count mismatch");
        for (std::size_t i = 0; i < count; ++i) {
            const Point3 p{number(j[3 * i]), number(j[3 * i + 1]), number(j[3 * i + 2])};
            const double w = rational ? number((*weights)[i]) : 1;
            if (skip) {
                ++points;
                continue;
            }
            const auto q = point(p, w);
            for (unsigned a = 0; a < 3; ++a)
                j[3 * i + a] = q[a];
        }
    }
    void curve(Json &j, unsigned depth, const std::string &current) {
        path = current;
        require(depth <= options.max_depth, "loft transform nesting budget exceeded");
        require(nodes < options.max_curve_nodes, "loft transform curve budget exceeded");
        ++nodes;
        const auto type = j.at("_type").get<std::string>();
        if (type == "CurveVector") {
            auto &members = j.at("curves");
            require(members.is_array(), "loft transform requires curve array");
            for (std::size_t i = 0; i < members.size(); ++i) {
                const bool arc = members[i].at("geometry").value("_type", "") == "EllipticArc";
                curve(members[i].at("geometry"), depth + 1,
                      current + "/curves/" + std::to_string(i) + "/geometry");
                if (arc && members[i].at("geometry").at("_type") == "LineSegment")
                    members[i]["geometryType"] = 1;
            }
        } else if (type == "LineSegment") {
            named(j.at("segment"), "point0", 1);
            named(j.at("segment"), "point1", 1);
        } else if (type == "LineString")
            packed(j.at("points"), nullptr, false);
        else if (type == "BsplineCurve") {
            const auto it = j.find("weights");
            packed(j.at("poles"), it == j.end() ? nullptr : &*it, bspline_identity);
            skipped += bspline_identity;
        } else if (type == "EllipticArc") {
            auto &arc = j.at("arc");
            const auto center = named(arc, "center", 1);
            const auto u = named(arc, "vector0", 0), v = named(arc, "vector90", 0);
            const double start = number(arc.at("startRadians")),
                         sweep = number(arc.at("sweepRadians"));
            auto length = [](const Point3 &p) {
                return std::sqrt((p[0] * p[0] + p[1] * p[1]) + p[2] * p[2]);
            };
            const double a = length(u), b = length(v);
            require(std::isfinite(a) && std::isfinite(b), "loft arc axis length overflow");
            if (a < 1e-5 || b < 1e-5) {
                // First-axis collapse has priority, including when both collapse.
                const bool use_sine = a < 1e-5;
                const auto axis = use_sine ? v : u;
                const double end = start + sweep;
                require(std::isfinite(end), "loft arc angular range overflow");
                const double first = use_sine ? std::sin(start) : std::cos(start),
                             last = use_sine ? std::sin(end) : std::cos(end);
                double lo = std::min(first, last), hi = std::max(first, last);
                constexpr double pi = 3.141592653589793;
                if (angle_in_sweep(use_sine ? pi / 2 : 0., start, sweep))
                    hi = 1;
                if (angle_in_sweep(use_sine ? 3 * pi / 2 : pi, start, sweep))
                    lo = -1;
                Json segment = {{"_type", "DSegment3d"}};
                for (unsigned k = 0; k < 3; ++k) {
                    const double p = center[k] + axis[k] * lo, q = center[k] + axis[k] * hi;
                    require(std::isfinite(p) && std::isfinite(q),
                            "loft collapsed arc endpoint overflow");
                    segment[std::string("point0") + "XYZ"[k]] = p;
                    segment[std::string("point1") + "XYZ"[k]] = q;
                }
                replacements.push_back({{"source_path", current},
                                        {"transformed_arc", j},
                                        {"retained_axis", use_sine ? "vector90" : "vector0"},
                                        {"scalar_interval", {lo, hi}},
                                        {"endpoint_order", "minimum_then_maximum"}});
                j = {{"_type", "LineSegment"}, {"segment", std::move(segment)}};
            }
        } else
            throw std::runtime_error("loft transform unsupported source curve: " + type);
    }
};
} // namespace
CurveSolidTransformResult transform_bgfb_curve_solid(const Json &table, const Matrix4 &matrix,
                                                     const CurveSolidTransformOptions &options) {
    LoftSourceTransformResult out;
    try {
        const auto type = table.at("_type").get<std::string>();
        require(type == "P3DSectionLoft" || type == "DgnExtrusion" || type == "DgnRuledSweep",
                "curve solid transform source type");
        Placement placement(matrix, options);
        Json transformed = table;
        try {
            if (type == "DgnExtrusion") {
                auto &v = transformed.at("extrusionVector");
                placement.path = "/extrusionVector";
                const auto p =
                    placement.point({number(v.at("x")), number(v.at("y")), number(v.at("z"))}, 0);
                for (unsigned i = 0; i < 3; ++i)
                    v[std::string(1, "xyz"[i])] = p[i];
                require(transformed.at("baseCurve").at("_type") == "CurveVector",
                        "extrusion base curve array required");
                placement.curve(transformed.at("baseCurve"), 0, "/baseCurve");
            } else if (type == "DgnRuledSweep") {
                auto &curves = transformed.at("curves");
                require(curves.is_array(), "ruled section array required");
                for (std::size_t i = 0; i < curves.size(); ++i) {
                    require(curves[i].at("_type") == "CurveVector",
                            "ruled section curve array required");
                    placement.curve(curves[i], 0, "/curves/" + std::to_string(i));
                }
            } else {
                require(transformed.at("section0").at("_type") == "CurveVector" &&
                            transformed.at("section1").at("_type") == "CurveVector",
                        "loft transform sections must be curve arrays");
                placement.curve(transformed.at("section0"), 0, "/section0");
                placement.curve(transformed.at("section1"), 0, "/section1");
                auto &groups = transformed.at("guide_groups");
                require(groups.is_array(), "loft transform guide groups");
                for (std::size_t i = 0; i < groups.size(); ++i) {
                    require(groups[i].is_array(), "loft transform guide group");
                    for (std::size_t j = 0; j < groups[i].size(); ++j) {
                        require(groups[i][j].at("_type") == "CurveVector",
                                "loft transform guide must be a curve array");
                        placement.curve(groups[i][j], 0,
                                        "/guide_groups/" + std::to_string(i) + "/" +
                                            std::to_string(j));
                    }
                }
            }
        } catch (...) {
            out.report["source_path"] = placement.path;
            throw;
        }
        out.transformed = std::move(transformed);
        out.report = {{"scope", type == "P3DSectionLoft" ? "source_sections_and_guides"
                                                         : "source_curves_and_parameters"},
                      {"curve_nodes", placement.nodes},
                      {"point_count", placement.points},
                      {"bspline_near_identity", placement.bspline_identity},
                      {"bspline_skipped", placement.skipped},
                      {"arc_replacements", std::move(placement.replacements)},
                      {"surface_mapping", "reconstruct_from_transformed_source"},
                      {"mesh_status", "not_evaluated"}};
        out.status = "transformed";
    } catch (const std::exception &e) {
        out.transformed = nullptr;
        out.report["reason"] = e.what();
    }
    return out;
}
LoftSourceTransformResult transform_bgfb_section_loft(const Json &table, const Matrix4 &matrix,
                                                      const LoftSourceTransformOptions &options) {
    if (table.is_object() && table.value("_type", Json()) == "P3DSectionLoft")
        return transform_bgfb_curve_solid(table, matrix, options);
    LoftSourceTransformResult out;
    out.report["reason"] = "loft transform source type";
    return out;
}
} // namespace p3d
