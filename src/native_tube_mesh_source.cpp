#include "native_tube_mesh_source.hpp"
#include "native_curve_area.hpp"
#include "native_curve_conversion.hpp"
#include "native_curve_planarity.hpp"
namespace p3d::swept_detail {
namespace {
std::optional<std::array<Point3, 2>> endpoints(const Json &g, TubeBudget &b, unsigned depth) {
    const curve_detail::BezierWork work{b.work, b.max_work};
    work.charge(1);
    require(depth < 80, "native sweep endpoint nesting budget");
    if (g.is_null())
        return {};
    if (g.value("_type", "") != "CurveVector")
        return curve_detail::native_source_curve_endpoints(g, b);
    const auto &members = g.at("curves");
    require(members.is_null() || members.is_array(), "native sweep endpoint member array");
    require(members.size() <= b.max_control_points, "native sweep endpoint member budget");
    std::optional<std::array<Point3, 2>> out;
    for (const auto &m : members) {
        work.charge(1);
        if (m.is_null())
            continue;
        auto pair = endpoints(m.at("geometry"), b, depth + 1);
        if (pair) {
            if (!out)
                out = pair;
            else
                (*out)[1] = (*pair)[1];
        }
    }
    return out;
}
Json closure(const Json &g, TubeBudget &b) {
    const auto pair = endpoints(g, b, 0);
    const bool closed = pair && curve_detail::endpoint_pair_closed((*pair)[0], (*pair)[1]);
    return {{"endpoints", pair ? Json(*pair) : Json()}, {"closed", closed}};
}
} // namespace
TubeMeshSourceConditions tube_mesh_source_conditions(const Json &profile, const Json &path,
                                                     bool capped, TubeBudget &b) {
    require(profile.is_object() && profile.value("_type", "") == "CurveVector" &&
                path.is_object() && path.value("_type", "") == "CurveVector",
            "native sweep requires nonnull source profile and path arrays");
    TubeMeshSourceConditions out;
    const auto planar = curve_detail::native_curve_vector_planarity(path, b.max_control_points,
                                                                    {b.work, b.max_work});
    out.report = {{"scope", "native_sweep_mesh_source_conditions"},
                  {"path_planarity", planar},
                  {"source_capped", capped},
                  {"path_closure_evaluated", false}};
    require(profile.at("type").is_number_integer(), "native sweep profile boundary type");
    if (profile.at("type") == 4) {
        const auto &rings = profile.at("curves");
        require(rings.is_null() || rings.is_array(), "native sweep parity member array");
        require(rings.size() <= b.max_control_points, "native sweep parity ring budget");
        out.profile_closed = true;
        out.report["profile_rings"] = Json::array();
        for (std::size_t i = 0; i < rings.size(); ++i) {
            curve_detail::BezierWork{b.work, b.max_work}.charge(1);
            require(!rings[i].is_null(), "native sweep null parity child query");
            const auto &ring = rings[i].at("geometry");
            require(ring.is_object() && ring.value("_type", "") == "CurveVector",
                    "native sweep parity child is not a curve array");
            auto r = closure(ring, b);
            r["source_ring"] = i;
            const bool closed = r.at("closed").get<bool>();
            out.report["profile_rings"].push_back(std::move(r));
            if (!closed) {
                out.profile_closed = false;
                out.report["accepted"] = false;
                out.report["failure"] = "open_parity_profile_child";
                return out;
            }
        }
    } else {
        auto r = closure(profile, b);
        out.profile_closed = r.at("closed").get<bool>();
        out.report["profile_closure"] = std::move(r);
    }
    if (planar.at("planar").get<bool>()) {
        auto r = closure(path, b);
        out.path_closed = r.at("closed").get<bool>();
        out.report["path_closure"] = std::move(r);
        out.report["path_closure_evaluated"] = true;
    }
    out.cap_eligible = out.profile_closed && capped && !out.path_closed;
    out.accepted = true;
    out.report["accepted"] = true;
    out.report["profile_closed"] = out.profile_closed;
    out.report["mesh_path_closed"] = out.path_closed;
    out.report["cap_eligible"] = out.cap_eligible;
    return out;
}
} // namespace p3d::swept_detail
