#include "loft_curve.hpp"
#include "native_pcurve_points.hpp"
#include "native_surface_iso.hpp"

namespace p3d {
namespace {
Json region(unsigned type, Json curves = Json::array()) {
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", std::move(curves)}};
}
Json variant(Json geometry) {
    return {{"_type", "VariantGeometry"}, {"geometry", std::move(geometry)}};
}
loft_detail::Curve endpoint_curve(const BsplineSurface &surface, bool top, unsigned limit) {
    const auto &u = surface.u(), &v = surface.v();
    require(!u.closed() && !v.closed() && u.knot_domain() == std::array<double, 2>{0, 1} &&
                v.knot_domain() == std::array<double, 2>{0, 1},
            "loft cap requires prepared open normalized side surfaces");
    for (unsigned i = 0; i < v.order(); ++i)
        require(v.knots()[i] == 0 && v.knots()[v.knots().size() - 1 - i] == 1,
                "loft cap endpoint requires a clamped side surface");
    loft_detail::Curve curve;
    curve.degree = u.order() - 1;
    curve.rational = surface.rational();
    curve.knots = u.knots();
    require(u.pole_count() <= limit, "loft cap control budget");
    std::size_t work = 0;
    const auto iso = detail::native_iso_v_curve(
        surface, top ? 1. : 0., {work, std::numeric_limits<std::size_t>::max()}, limit);
    for (std::size_t i = 0; i < u.pole_count(); ++i) {
        const auto &p = iso.curve.poles()[i];
        const double weight = curve.rational ? iso.curve.weights()[i] : 1;
        curve.poles.push_back({p[0], p[1], p[2], weight});
    }
    curve.check(limit, false);
    return curve;
}
bool closed(Point3 a, Point3 b) {
    double distance2 = 0, scale2 = 1;
    for (unsigned k = 0; k < 3; ++k) {
        distance2 += (a[k] - b[k]) * (a[k] - b[k]);
        scale2 += a[k] * a[k] + b[k] * b[k];
    }
    require(std::isfinite(distance2) && std::isfinite(scale2),
            "loft cap closure comparison outside finite range");
    return distance2 < scale2 * 1.0000000000000001e-20;
}
void reverse(loft_detail::Curve &curve) {
    std::reverse(curve.poles.begin(), curve.poles.end());
    std::reverse(curve.knots.begin(), curve.knots.end());
    // Native reversal normalizes the reversed knot array. Prepared side
    // isocurves are clamped to [0,1], so its denominator is exactly -1.
    for (auto &k : curve.knots)
        k = (k - 1) / -1;
}
} // namespace

LoftCapRegions SectionLoft::cap_regions(unsigned max_control_points) const {
    require(max_control_points > 0, "loft cap control budget must be positive");
    LoftCapRegions out;
    out.report = {{"status", "not_requested"},
                  {"representation", "derived_loft_cap_regions"},
                  {"mesh_status", "not_evaluated"},
                  {"planarity_status", "not_evaluated"}};
    if (!source_.at("capped").get<bool>())
        return out;
    out.report["status"] = "incomplete";
    try {
        require(!sides_.empty(), "loft cap requires side surfaces");
        std::vector<std::vector<std::size_t>> groups;
        for (std::size_t i = 0; i < sides_.size(); ++i) {
            const auto &side = sides_[i];
            if (side.loop_index == groups.size())
                groups.emplace_back();
            require(!groups.empty() && side.loop_index == groups.size() - 1 &&
                        side.primitive_index == groups.back().size(),
                    "loft cap side ordering");
            groups.back().push_back(i);
        }
        std::array<Json, 2> caps{region(4), region(4)};
        Json links = Json::array();
        std::size_t total = 0, zero_controls = 0, endpoint_fallbacks = 0;
        for (std::size_t loop = 0; loop < groups.size(); ++loop) {
            const auto &indices = groups[loop];
            for (unsigned end = 0; end < 2; ++end) {
                std::vector<loft_detail::Curve> curves;
                for (auto index : indices) {
                    auto curve =
                        endpoint_curve(sides_[index].surface, end != 0, max_control_points);
                    require(curve.poles.size() <= max_control_points - total,
                            "loft cap total control budget");
                    total += curve.poles.size();
                    for (const auto &h : curve.poles)
                        zero_controls += curve.rational && h[3] == 0;
                    curves.push_back(std::move(curve));
                }
                Point3 first{}, last{};
                for (std::size_t i = 0; i < curves.size(); ++i) {
                    const auto boundary = BsplineCurve::from_bgfb(curves[i].table());
                    const auto a = detail::pcurve_point(boundary, 0);
                    const auto b = detail::pcurve_point(boundary, 1);
                    if (i == 0)
                        first = a.point;
                    last = b.point;
                    endpoint_fallbacks += a.zero_weight_fallback + b.zero_weight_fallback;
                }
                out.report["zero_weight_control_count"] = zero_controls;
                out.report["endpoint_zero_weight_fallbacks"] = endpoint_fallbacks;
                if (!closed(first, last)) {
                    out.report["status"] = "native_failure";
                    out.report["reason"] = "loft cap boundary endpoints do not coincide";
                    out.report["failed_loop"] = loop;
                    out.report["failed_end"] = end == 0 ? "bottom" : "top";
                    return out;
                }
                Json ring = region(loop ? 3 : 2);
                for (std::size_t position = 0; position < curves.size(); ++position) {
                    const auto original = end ? position : curves.size() - 1 - position;
                    auto &curve = curves[original];
                    if (end == 0)
                        reverse(curve);
                    ring["curves"].push_back(variant(curve.table()));
                    const auto path =
                        (groups.size() == 1 ? std::string()
                                            : "/curves/" + std::to_string(loop) + "/geometry") +
                        "/curves/" + std::to_string(position) + "/geometry";
                    links.push_back({{"cap", end == 0 ? "bottom" : "top"},
                                     {"region_path", path},
                                     {"side_index", indices[original]},
                                     {"loop_index", loop},
                                     {"primitive_index", original},
                                     {"surface_v", end},
                                     {"u_reversed", end == 0}});
                }
                if (groups.size() == 1)
                    caps[end] = std::move(ring);
                else
                    caps[end]["curves"].push_back(variant(std::move(ring)));
            }
        }
        out.bottom = std::move(caps[0]);
        out.top = std::move(caps[1]);
        out.report["status"] = "complete";
        out.report["loop_count"] = groups.size();
        out.report["control_point_count"] = total;
        out.report["boundary_curves"] = std::move(links);
    } catch (const std::exception &e) {
        out.bottom = nullptr;
        out.top = nullptr;
        out.report["status"] = "incomplete";
        out.report["reason"] = e.what();
    }
    return out;
}

LoftFaceIndexSet SectionLoft::face_indices(unsigned max_cap_control_points) const {
    const auto native = native_faces(max_cap_control_points);
    LoftFaceIndexSet out;
    out.report = {{"status", native.report.at("status")},
                  {"representation", "native_loft_face_indices"},
                  {"index_scope", "solid"},
                  {"cap_status", native.report.at("cap_status")},
                  {"material_part_mapping", "not_established"}};
    const bool capped = source_.at("capped").get<bool>();
    if (native.report.at("status") != "complete") {
        out.report["face_geometry_failure"] = native.report;
        return out;
    }
    out.report["side_adjustments"] = native.report.at("side_adjustments");
    // getCappedPatches retains one patch per source primitive, including its
    // linear-V knot cleanup. getFaceIndices uses one counter across all loops.
    // Do not use getBsplineSurface's joined loop surfaces as the face count.
    if (sides_.size() > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
        out.report["status"] = "incomplete";
        out.report["reason"] = "loft native face counter range";
        return out;
    }
    out.indices.reserve(sides_.size() + (capped ? 2 : 0));
    if (capped) {
        out.indices.push_back({-1, 0, 0});
        out.indices.push_back({-1, 1, 0});
    }
    for (std::size_t i = 0; i < sides_.size(); ++i)
        out.indices.push_back({0, static_cast<std::int64_t>(i), 0});
    out.report["status"] = "complete";
    out.report["side_count"] = sides_.size();
    out.report["cap_count"] = capped ? 2 : 0;
    return out;
}
} // namespace p3d
