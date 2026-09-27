#include <p3d/swept_body.hpp>
#include "native_curve_conversion.hpp"
#include "native_bezier.hpp"

namespace p3d {
namespace {
Json variant(Json geometry) {
    return {{"_type", "VariantGeometry"}, {"geometry", std::move(geometry)}};
}
Json region(unsigned type, Json curves = Json::array()) {
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", std::move(curves)}};
}
Json curve_table(const BsplineCurve &c) {
    Json poles = Json::array();
    for (const auto &p : c.poles())
        for (double x : p)
            poles.push_back(x);
    return {{"_type", "BsplineCurve"},
            {"order", c.order()},
            {"closed", c.closed()},
            {"poles", std::move(poles)},
            {"weights", c.rational() ? Json(c.weights()) : Json()},
            {"knots", c.knots()}};
}
} // namespace
SweptBodyUVBoundaryResult
extract_swept_body_uv_boundaries(const SweptBodySurface &source,
                                 const SweptBodyUVBoundaryOptions &options) {
    SweptBodyUVBoundaryResult out;
    std::size_t used = 0, controls = 0, curves = 0;
    curve_detail::BezierWork work{used, options.max_work};
    auto account = [&](std::size_t points) {
        require(points <= options.max_control_points - controls,
                "swept UV boundary control budget");
        require(curves < options.max_curves, "swept UV boundary curve budget");
        controls += points;
        ++curves;
        work.charge(points);
        work.charge(1);
    };
    try {
        require(source.geometry.value("_type", "") == "BsplineSurface",
                "swept UV boundary surface type");
        require(source.geometry.at("boundaries").is_null(),
                "swept UV boundary cannot merge BGFB tree");
        const auto &origin = source.geometry.at("holeOrigin");
        require(origin.is_number_integer(), "swept UV boundary holeOrigin must be integer");
        require(!options.prefer_parameter_curves || source.boundary_curves.empty() ||
                    source.boundary_curves.size() == source.boundary_points.size(),
                "swept UV boundary pcurve record mismatch");
        work.charge(source.boundary_points.size());
        Json children = Json::array(), records = Json::array();
        const bool outer = options.include_outer && origin == 0;
        if (outer) {
            account(5);
            children.push_back(variant(region(
                2, Json::array({variant(
                       {{"_type", "LineString"},
                        {"points",
                         {0., 0., 0., 1., 0., 0., 1., 1., 0., 0., 1., 0., 0., 0., 0.}}})}))));
        }
        for (std::size_t i = 0; i < source.boundary_points.size(); ++i) {
            const bool pcurve = options.prefer_parameter_curves &&
                                !source.boundary_curves.empty() &&
                                !source.boundary_curves[i].empty();
            Json members = Json::array();
            Json record{{"runtime_record", i},
                        {"source", pcurve ? "parameter_curves" : "uv_points"}};
            if (pcurve) {
                for (const auto &curve : source.boundary_curves[i]) {
                    account(curve.poles().size());
                    work.charge(curve.knots().size());
                    members.push_back(variant(curve_table(curve)));
                }
                record["closure"] = "unchanged";
            } else {
                const auto &points = source.boundary_points[i];
                account(points.size());
                Json flat = Json::array();
                for (const auto &p : points) {
                    require(std::isfinite(p[0]) && std::isfinite(p[1]),
                            "swept UV boundary nonfinite point");
                    flat.push_back(p[0]);
                    flat.push_back(p[1]);
                    flat.push_back(0.);
                }
                record["closure"] = "unchanged";
                if (points.size() > 1) {
                    const Point3 a{points.front()[0], points.front()[1], 0},
                        z{points.back()[0], points.back()[1], 0};
                    if (curve_detail::endpoint_pair_closed(a, z)) {
                        for (unsigned j = 0; j < 3; ++j)
                            flat[flat.size() - 3 + j] = a[j];
                        record["closure"] = "last_replaced_by_first";
                    } else {
                        require(controls < options.max_control_points,
                                "swept UV boundary closing control budget");
                        ++controls;
                        work.charge(1);
                        for (double x : a)
                            flat.push_back(x);
                        record["closure"] = "first_appended";
                    }
                }
                members.push_back(variant({{"_type", "LineString"}, {"points", std::move(flat)}}));
            }
            record["curve_count"] = members.size();
            children.push_back(variant(region(2, std::move(members))));
            records.push_back(std::move(record));
        }
        out.region = region(4, std::move(children));
        out.report = {{"outer_included", outer},
                      {"records", std::move(records)},
                      {"control_points", controls},
                      {"curve_count", curves},
                      {"orientation_adjusted", false}};
        out.status = "extracted"; // Native creates an empty type-4 array as well.
    } catch (const std::bad_alloc &) {
        throw;
    } catch (const std::exception &e) {
        out.region = nullptr;
        out.report["reason"] = e.what();
    }
    out.report["work_used"] = used;
    return out;
}
BsplineSurfaceMesh mesh_swept_body_surface(const SweptBodySurface &source,
                                           const BsplineMeshOptions &options) {
    try {
        const auto surface = BsplineSurface::from_bgfb(source.geometry);
        auto mesh = surface.mesh_runtime_boundaries(source.boundary_points, options);
        mesh.report["working_location"] = source.working_location;
        mesh.report["source_member"] = source.source_member;
        mesh.report["boundary_source"] = "runtime_uv_points";
        return mesh;
    } catch (const std::bad_alloc &) {
        throw;
    } catch (const std::exception &e) {
        BsplineSurfaceMesh mesh;
        mesh.report = {{"status", "incomplete"}, {"reason", e.what()}};
        return mesh;
    }
}
} // namespace p3d
