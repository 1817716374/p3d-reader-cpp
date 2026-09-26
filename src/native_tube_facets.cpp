#include "native_tube_facets.hpp"
#include "native_curve_conversion.hpp"

namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    require(b.work <= b.max_work && n <= b.max_work - b.work,
            "native swept facet preparation work budget exceeded");
    b.work += n;
}
Json segment(const Json &points, std::size_t index) {
    Json detail = Json::object();
    for (unsigned end = 0; end < 2; ++end)
        for (unsigned axis = 0; axis < 3; ++axis) {
            const auto &v = points[3 * (index + end) + axis];
            require(v.is_number(), "native facet polyline numeric layout");
            detail[std::string("point") + char('0' + end) + "XYZ"[axis]] = v;
        }
    return {{"_type", "LineSegment"}, {"segment", std::move(detail)}};
}
struct Partition {
    TubeBudget &b;
    Json skipped = Json::array();
    std::string failure_path, failure_reason;
    std::size_t derived = 0, retained = 0;
    bool fail(const std::string &path, const char *reason) {
        failure_path = path;
        failure_reason = reason;
        return false;
    }
    bool group(const Json &value, const std::string &path, std::vector<TubeFacetMember> &out) {
        charge(b, 1);
        if (value.is_null())
            return fail(path, "null_curve_vector");
        require(value.is_object() && value.value("_type", std::string()) == "CurveVector",
                "native facet group layout");
        if (value.at("type") == 4)
            return fail(path, "nested_parity_region");
        const auto &members = value.at("curves");
        require(members.is_array(), "native facet group member layout");
        for (std::size_t i = 0; i < members.size(); ++i) {
            charge(b, 1);
            const auto &g = members[i].at("geometry");
            const auto source_path = path + "/curves/" + std::to_string(i) + "/geometry";
            require(g.is_object() && g.contains("_type") && g.at("_type").is_string(),
                    "native facet primitive pointer/type unavailable");
            const auto type = g.at("_type").get<std::string>();
            if (type == "LineString") {
                const auto &p = g.at("points");
                require(p.is_array() && p.size() % 3 == 0 && p.size() / 3 <= b.max_control_points,
                        "native facet polyline layout/control budget");
                if (p.size() < 6)
                    return fail(source_path, "polyline_requires_two_points");
                for (std::size_t j = 0; j + 1 < p.size() / 3; ++j) {
                    charge(b, 8);
                    out.push_back({1, i, j, source_path, &g, segment(p, j)});
                    ++derived;
                }
            } else if (type == "LineSegment" || type == "EllipticArc" || type == "BsplineCurve") {
                unsigned boundary = 1;
                if (type != "LineSegment") {
                    std::optional<BsplineCurve> curve;
                    if (type == "BsplineCurve") {
                        auto copy = convert_tube_primitive(g, b);
                        require(copy.curve.order() <= 26, "native facet endpoint query order");
                        charge(b, 16 * std::size_t(copy.curve.order()) * copy.curve.order());
                        curve = std::move(copy.curve);
                    } else
                        charge(b, 32);
                    const auto ends =
                        curve_detail::primitive_endpoints(g, curve ? &*curve : nullptr);
                    if (ends && curve_detail::endpoint_pair_closed((*ends)[0], (*ends)[1]))
                        boundary = 2;
                }
                out.push_back({boundary, i, {}, source_path, &g, {}});
                ++retained;
            } else
                skipped.push_back({{"source_path", source_path}, {"source_type", type}});
        }
        return true;
    }
};
} // namespace
TubeFacetProfile partition_tube_facet_profile(std::shared_ptr<const Json> source, TubeBudget &b) {
    TubeFacetProfile out{std::move(source), {}, {}};
    Partition p{b, Json::array(), {}, {}, 0, 0};
    bool ok = false;
    std::size_t discarded = 0;
    if (!out.source || out.source->is_null())
        p.fail("/profile", "null_profile");
    else {
        const auto &value = *out.source;
        require(value.is_object() && value.value("_type", std::string()) == "CurveVector",
                "native facet profile requires CurveVector");
        charge(b, 1);
        if (value.at("type") == 4) {
            const auto &members = value.at("curves");
            require(members.is_array(), "native facet parity member layout");
            ok = true;
            for (std::size_t i = 0; i < members.size(); ++i) {
                const auto &g = members[i].at("geometry");
                const auto path = "/profile/curves/" + std::to_string(i) + "/geometry";
                std::vector<TubeFacetMember> group;
                if (!g.is_object() || g.value("_type", std::string()) != "CurveVector")
                    ok = p.fail(path, "missing_child_curve_vector");
                else
                    ok = p.group(g, path, group);
                if (!ok) {
                    discarded = group.size();
                    break;
                }
                out.groups.push_back(std::move(group));
            }
        } else {
            std::vector<TubeFacetMember> group;
            ok = p.group(value, "/profile", group);
            if (ok)
                out.groups.push_back(std::move(group));
            else
                discarded = group.size();
        }
    }
    out.report = {
        {"scope", "native_swept_facet_profile_partition"},
        {"native_result", ok},
        {"completed_groups", out.groups.size()},
        {"discarded_temporary_members", discarded},
        {"created_source_wrappers", p.retained},
        {"created_polyline_segments", p.derived},
        {"skipped_primitives", std::move(p.skipped)},
        {"failure_path", p.failure_path.empty() ? Json(nullptr) : Json(p.failure_path)},
        {"failure_reason", p.failure_reason.empty() ? Json(nullptr) : Json(p.failure_reason)},
        {"work_used", b.work},
        {"face_indices", nullptr}};
    return out;
}
Json enumerate_tube_facet_indices(const TubeFacetGroups &groups, std::size_t cap_count,
                                  bool generation_succeeded, TubeBudget &b) {
    Json indices = Json::array(), locations = Json::array();
    if (generation_succeeded) {
        // Native emits at most two cap indices, even if a caller supplies more
        // cap objects. Side numbering is one global signed-32-bit counter.
        for (std::size_t i = 0; i < std::min(std::size_t(2), cap_count); ++i) {
            charge(b, 1);
            indices.push_back({-1, std::int64_t(i), 0});
        }
        std::int64_t index = 0;
        for (std::size_t g = 0; g < groups.size(); ++g) {
            charge(b, 1);
            for (std::size_t m = 0; m < groups[g].size(); ++m) {
                charge(b, 1);
                for (std::size_t k = 0; k < groups[g][m].size(); ++k) {
                    charge(b, 1);
                    require(index < INT32_MAX, "native swept face index counter overflow");
                    indices.push_back({0, index, 0});
                    locations.push_back(
                        {{"face_index", {0, index, 0}}, {"group", g}, {"member", m}, {"patch", k}});
                    ++index;
                }
            }
        }
    }
    return {{"scope", "native_swept_facet_indices"},
            {"generation_succeeded", generation_succeeded},
            {"face_indices", std::move(indices)},
            {"side_locations", std::move(locations)},
            {"work_used", b.work}};
}
} // namespace p3d::swept_detail
