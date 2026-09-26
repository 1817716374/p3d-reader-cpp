#include "native_tube_reference.hpp"
#include "native_tube_path_source.hpp"
#include "native_curve_area.hpp"
#include "native_curve_conversion.hpp"

namespace p3d::swept_detail {
namespace {
Json line(Point3, Point3);
struct Clone {
    TubeBudget &budget;
    std::size_t controls = 0;
    Json members = Json::array(), skipped = Json::array();
    bool split_polylines = false;
    Json empty_polylines = Json::array();
    void charge(std::size_t n) {
        curve_detail::BezierWork{budget.work, budget.max_work}.charge(n);
    }
    void enter(unsigned depth) {
        require(depth <= 256, "native reference profile nesting limit exceeded");
        charge(1);
    }
    void add_controls(std::size_t n) {
        require(controls <= budget.max_control_points && n <= budget.max_control_points - controls,
                "native reference profile control budget exceeded");
        controls += n;
    }
    // Copy opaque source descriptors too, but account for every node and byte
    // before copying it. Descriptors are retained data, not new native objects.
    Json copy(const Json &v, unsigned depth = 0) {
        enter(depth);
        if (v.is_object()) {
            Json out = Json::object();
            for (auto i = v.begin(); i != v.end(); ++i) {
                charge(i.key().size());
                out[i.key()] = copy(i.value(), depth + 1);
            }
            return out;
        }
        if (v.is_array()) {
            Json out = Json::array();
            for (const auto &item : v)
                out.push_back(copy(item, depth + 1));
            return out;
        }
        if (v.is_string())
            charge(v.get_ref<const std::string &>().size());
        if (v.is_binary())
            charge(v.get_binary().size());
        return v;
    }
    Json geometry(const Json &v, const std::string &source, const std::string &working,
                  unsigned depth = 0) {
        enter(depth);
        require(v.is_object(), "native reference profile geometry layout");
        const auto type = v.at("_type").get<std::string>();
        if (type != "CurveVector") {
            if (type == "LineSegment")
                add_controls(2);
            else if (type == "EllipticArc")
                add_controls(3);
            else {
                require(type == "LineString" || type == "BsplineCurve",
                        "native reference profile primitive unsupported");
                const auto &points = v.at(type == "LineString" ? "points" : "poles");
                require(points.is_array() && points.size() % 3 == 0,
                        "native reference profile control layout");
                add_controls(points.size() / 3);
            }
            return copy(v);
        }
        const auto &input = v.at("curves");
        require(input.is_array(), "native reference profile members layout");
        Json out = Json::object();
        for (auto i = v.begin(); i != v.end(); ++i)
            if (i.key() != "curves") {
                charge(i.key().size());
                out[i.key()] = copy(i.value());
            }
        out["curves"] = Json::array();
        for (std::size_t i = 0; i < input.size(); ++i) {
            charge(1);
            const auto &member = input[i];
            const auto &g = member.at("geometry");
            const auto from = source + ".curves[" + std::to_string(i) + "].geometry";
            if (g.is_null()) {
                // Native CurveArray::clone omits source nulls before endpoint
                // queries. This compaction applies only to the working copy.
                skipped.push_back(from);
                continue;
            }
            const auto kind = g.at("_type").get<std::string>();
            if (split_polylines && kind == "LineString") {
                const auto &p = g.at("points");
                require(p.is_array() && p.size() % 3 == 0,
                        "native path source polyline control layout");
                const auto n = p.size() / 3;
                if (n < 2)
                    empty_polylines.push_back(from);
                auto point = [&](std::size_t j) {
                    Point3 result;
                    for (unsigned k = 0; k < 3; ++k) {
                        require(p[3 * j + k].is_number(), "native path source coordinate layout");
                        result[k] = p[3 * j + k].get<double>();
                        require(std::isfinite(result[k]),
                                "native path source nonfinite coordinate");
                    }
                    return result;
                };
                for (std::size_t j = 1; j < n; ++j) {
                    charge(32);
                    add_controls(2);
                    const auto to =
                        working + ".curves[" + std::to_string(out["curves"].size()) + "].geometry";
                    members.push_back({{"source_path", from},
                                       {"working_path", to},
                                       {"source_segment", j - 1},
                                       {"descriptor_policy", "new_primitive"}});
                    out["curves"].push_back({{"geometry", line(point(j - 1), point(j))}});
                }
                continue;
            }
            const auto to =
                working + ".curves[" + std::to_string(out["curves"].size()) + "].geometry";
            Json item = Json::object();
            const bool new_wrapper = split_polylines && kind == "CurveVector";
            if (!new_wrapper)
                for (auto field = member.begin(); field != member.end(); ++field)
                    if (field.key() != "geometry") {
                        charge(field.key().size());
                        item[field.key()] = copy(field.value());
                    }
            Json location{{"source_path", from}, {"working_path", to}};
            if (split_polylines)
                location["descriptor_policy"] = new_wrapper ? "new_primitive" : "cloned";
            members.push_back(std::move(location));
            item["geometry"] = geometry(g, from, to, depth + 1);
            out["curves"].push_back(std::move(item));
        }
        return out;
    }
};
std::uint32_t boundary_type(const Json &v) {
    require(v.is_object() && v.value("_type", std::string()) == "CurveVector" &&
                v.contains("type") && v.at("type").is_number_integer(),
            "native reference profile CurveVector layout");
    const auto &type = v.at("type");
    require((type.is_number_unsigned() || type.get<std::int64_t>() >= 0) &&
                type.get<std::uint64_t>() <= std::numeric_limits<std::uint32_t>::max(),
            "native reference profile boundary type range");
    return type.get<std::uint32_t>();
}
Json line(Point3 a, Point3 b) {
    Json segment = Json::object();
    for (unsigned i = 0; i < 3; ++i) {
        segment[std::string("point0") + "XYZ"[i]] = a[i];
        segment[std::string("point1") + "XYZ"[i]] = b[i];
    }
    return {{"_type", "LineSegment"}, {"segment", std::move(segment)}};
}
} // namespace
TubeReferenceProfile prepare_tube_reference_profile(const Json &source, TubeBudget &budget) {
    Clone clone{budget};
    clone.charge(1);
    const auto root_type = boundary_type(source);
    const Json *selected = &source;
    std::string path = "profile";
    if (root_type == 4 || root_type == 5) {
        const auto &m = source.at("curves");
        require(m.is_array() && !m.empty(), "native reference profile missing first child");
        selected = &m[0].at("geometry");
        path += ".curves[0].geometry";
    }
    const auto type = boundary_type(*selected);
    TubeReferenceProfile out;
    out.profile = clone.geometry(*selected, path, "reference_profile");
    out.report = {{"scope", "native_facet_reference_profile"},
                  {"source_path", path},
                  {"source_boundary_type", type},
                  {"boundary_promoted", type <= 1},
                  {"closing_member", nullptr},
                  {"members", std::move(clone.members)},
                  {"skipped_null_sources", std::move(clone.skipped)}};
    if (type <= 1) {
        const auto ends = curve_detail::native_source_curve_endpoints(out.profile, budget);
        const auto points = ends.value_or(std::array<Point3, 2>{});
        out.report["working_endpoints_found"] = ends.has_value();
        out.report["working_endpoints"] = points;
        if (!curve_detail::endpoint_pair_closed(points[0], points[1])) {
            clone.add_controls(2);
            clone.charge(32);
            out.report["closing_member"] = out.profile["curves"].size();
            out.profile["curves"].push_back({{"geometry", line(points[1], points[0])}});
        }
        out.profile["type"] = 2;
    }
    const auto area = curve_detail::native_curve_vector_area(out.profile, budget);
    out.area_valid = area.value.valid;
    out.report["area"] = area.report;
    if (out.area_valid) {
        out.reference = area.value.centroid;
        out.plane_origin = area.value.centroid;
        out.plane_normal = area.value.normal;
        out.report["reference_source"] = "working_profile_area";
    } else {
        const auto ends = curve_detail::native_source_curve_endpoints(source, budget);
        const auto points = ends.value_or(std::array<Point3, 2>{});
        clone.charge(16);
        double largest = 0;
        for (unsigned i = 0; i < 3; ++i) {
            out.reference[i] = points[1][i] + points[0][i];
            require(std::isfinite(out.reference[i]),
                    "native reference profile endpoint sum overflow");
            largest = std::max(largest, std::abs(out.reference[i]));
        }
        // Native point division returns the unchanged numerator when the
        // divisor is small relative to it. Do not replace with an ideal midpoint.
        const bool guarded = largest * 1e-12 >= 2.;
        if (!guarded)
            for (auto &x : out.reference)
                x *= .5;
        out.report["reference_source"] = "original_profile_endpoints";
        out.report["fallback_endpoints_found"] = ends.has_value();
        out.report["fallback_endpoints"] = points;
        out.report["fallback_division_guarded"] = guarded;
    }
    out.report["work_used"] = budget.work;
    out.report["working_control_points"] = clone.controls;
    return out;
}
TubePathSource prepare_tube_path_source(const Json &source, TubeBudget &budget) {
    const auto type = boundary_type(source);
    TubePathSource out;
    const auto endpoints = curve_detail::native_source_curve_endpoints(source, budget);
    out.endpoints_found = endpoints.has_value();
    if (endpoints)
        out.source_endpoints = *endpoints;
    Clone clone{budget};
    clone.split_polylines = true;
    out.path = clone.geometry(source, "path", "working_path");
    out.report = {{"scope", "native_facet_path_source"},
                  {"source_boundary_type", type},
                  {"source_endpoints_found", out.endpoints_found},
                  {"source_endpoints", out.source_endpoints},
                  {"members", std::move(clone.members)},
                  {"skipped_null_sources", std::move(clone.skipped)},
                  {"polylines_without_segments", std::move(clone.empty_polylines)},
                  {"working_control_points", clone.controls},
                  {"work_used", budget.work}};
    return out;
}
TubeFacetSources prepare_tube_facet_sources(const Json &profile, const Json &path,
                                            TubeBudget &budget) {
    require(!profile.is_null() && !path.is_null(), "native facet preparation null source");
    auto reference = prepare_tube_reference_profile(profile, budget);
    auto working_path = prepare_tube_path_source(path, budget);
    return {std::move(reference), std::move(working_path)};
}
} // namespace p3d::swept_detail
