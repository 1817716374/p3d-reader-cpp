#include "native_tube_facet_caps.hpp"
#include "native_surface_iso.hpp"
#include "native_curve_conversion.hpp"
#include "native_curve_segment.hpp"
#include "native_pcurve_points.hpp"
namespace p3d::swept_detail {
namespace {
curve_detail::BezierWork work(TubeBudget &b) {
    return {b.work, b.max_work};
}
Json region(unsigned type) {
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", Json::array()}};
}
Json variant(Json geometry) {
    return {{"_type", "VariantGeometry"}, {"geometry", std::move(geometry)}};
}
Json table(const BsplineCurve &c, TubeBudget &b) {
    require(c.poles().size() <= b.max_control_points, "native facet cap control budget");
    work(b).charge(c.poles().size());
    work(b).charge(c.knots().size());
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
detail::NativeIsoCurve iso(const TubeFacetSurface *s, double v, TubeBudget &b) {
    require(s, "native facet cap cannot dereference null surface");
    const auto surface = BsplineSurface::from_bgfb(s->geometry);
    return detail::native_iso_v_curve(surface, v, work(b), b.max_control_points);
}
bool closed(const std::vector<BsplineCurve> &curves, TubeBudget &b) {
    if (curves.empty())
        return false;
    // 162170 asks every primitive for both ends but compares only the FIRST
    // successful start with the LAST successful end. No interior gap checks.
    Point3 a{}, z{};
    for (std::size_t i = 0; i < curves.size(); ++i) {
        work(b).charge(2 * std::size_t(curves[i].order()) * curves[i].order());
        const auto first = detail::pcurve_point(curves[i], 0).point;
        const auto last = detail::pcurve_point(curves[i], 1).point;
        if (!i)
            a = first;
        z = last;
    }
    return curve_detail::endpoint_pair_closed(a, z);
}
} // namespace
TubeCaps tube_facet_cap_regions(const std::vector<TubeFacetCapPair> &groups, TubeBudget &b) {
    TubeCaps out;
    out.report = {{"scope", "native_grouped_facet_cap_regions"},
                  {"native_result", false},
                  {"completed_rings", 0},
                  {"start_reversed", false},
                  {"rings", Json::array()}};
    auto fail = [&](const char *reason) {
        out.report["reason"] = reason;
        out.report["work_used"] = b.work;
        return std::move(out);
    };
    require(groups.size() <= b.max_control_points, "native facet cap group budget");
    work(b).charge(groups.size());
    if (groups.empty())
        return fail("no_endpoint_surface_groups");
    if (groups.size() > 1)
        out.start = out.end = region(4);
    std::vector<std::vector<BsplineCurve>> starts;
    for (std::size_t g = 0; g < groups.size(); ++g) {
        const auto &pair = groups[g];
        if (pair.first.size() != pair.last.size())
            return fail("endpoint_member_counts_differ");
        require(pair.first.size() <= b.max_control_points, "native facet cap member budget");
        const unsigned type = g ? 3 : 2;
        Json first = region(type), last = region(type);
        // The single-ring caller publishes the region objects BEFORE extraction;
        // multi-ring callers append a child only after both closure checks pass.
        if (groups.size() == 1) {
            out.start = first;
            out.end = last;
        }
        auto &first_region = groups.size() == 1 ? out.start : first;
        auto &last_region = groups.size() == 1 ? out.end : last;
        std::vector<BsplineCurve> first_curves, last_curves;
        Json ring_report{{"group", g}, {"boundary_type", type}, {"members", Json::array()}};
        for (std::size_t m = 0; m < pair.first.size(); ++m) {
            auto a = iso(pair.first[m], 0, b);
            auto z = iso(pair.last[m], 1, b);
            first_region["curves"].push_back(variant(table(a.curve, b)));
            last_region["curves"].push_back(variant(table(z.curve, b)));
            first_curves.push_back(std::move(a.curve));
            last_curves.push_back(std::move(z.curve));
            ring_report["members"].push_back(
                {{"member", m},
                 {"start_zero_weight_fallbacks", a.zero_weight_fallbacks},
                 {"end_zero_weight_fallbacks", z.zero_weight_fallbacks}});
        }
        const bool first_closed = closed(first_curves, b);
        const bool last_closed = first_closed && closed(last_curves, b);
        ring_report["start_endpoint_closed"] = first_closed;
        ring_report["end_endpoint_closed"] = first_closed ? Json(last_closed) : Json();
        out.report["rings"].push_back(std::move(ring_report));
        if (!first_closed || !last_closed)
            return fail("cap_ring_source_endpoints_not_closed");
        if (groups.size() > 1) {
            out.start["curves"].push_back(variant(std::move(first)));
            out.end["curves"].push_back(variant(std::move(last)));
        }
        starts.push_back(std::move(first_curves));
        out.report["completed_rings"] = g + 1;
    }
    require(b.max_control_points <= UINT32_MAX, "native facet cap reversal control limit");
    for (std::size_t g = 0; g < starts.size(); ++g) {
        // 177ff0 keeps parity child order, reverses each type-2/3 member list,
        // then invokes primitive reversal in the new member order.
        std::reverse(starts[g].begin(), starts[g].end());
        auto &r = groups.size() == 1 ? out.start : out.start["curves"][g]["geometry"];
        r["curves"] = Json::array();
        Json reversals = Json::array();
        for (auto &c : starts[g]) {
            Json report;
            const bool ok = curve_detail::reverse_native_working_curve(
                c, static_cast<unsigned>(b.max_control_points), work(b), report);
            report["curve_operation_result"] = ok;
            // Native B-spline primitive wrapper ignores the curve-level bool.
            reversals.push_back(std::move(report));
            r["curves"].push_back(variant(table(c, b)));
        }
        out.report["rings"][g]["reversal"] = std::move(reversals);
    }
    out.report["start_reversed"] = true;
    if (out.start["curves"].empty() || out.end["curves"].empty()) {
        out.start = out.end = nullptr;
        return fail("empty_final_cap_regions");
    }
    out.report["native_result"] = true;
    out.report["reason"] = "completed";
    out.report["work_used"] = b.work;
    return out;
}
CappedTubeFacets cap_tube_facet_assembly(TubeFacetGroupAssembly assembly, bool capped,
                                         TubeBudget &b) {
    CappedTubeFacets out;
    out.sides = std::move(assembly);
    out.success = out.sides.success;
    out.report = {{"scope", "native_grouped_facets_and_caps"},
                  {"source_capped", capped},
                  {"source_endpoint_closed", nullptr},
                  {"caps_requested", false},
                  {"selected_surfaces", Json::array()}};
    if (out.success && capped) {
        require(out.sides.generation.preparation.placement.has_value(),
                "native facet caps lack source path preparation");
        const auto &source = out.sides.generation.preparation.placement->branches.path.sources.path;
        const bool source_closed = source.endpoints_found &&
                                   curve_detail::endpoint_pair_closed(source.source_endpoints[0],
                                                                      source.source_endpoints[1]);
        out.report["source_endpoint_closed"] = source_closed;
        if (!source_closed) {
            out.report["caps_requested"] = true;
            std::vector<TubeFacetCapPair> groups;
            for (std::size_t g = 0; g < out.sides.groups.size(); ++g) {
                TubeFacetCapPair pair;
                Json selected = Json::array();
                for (std::size_t m = 0; m < out.sides.groups[g].size(); ++m) {
                    work(b).charge(1);
                    const auto &indices = out.sides.groups[g][m];
                    require(!indices.empty(), "native facet caps cannot read an empty member");
                    const auto &surfaces = out.sides.generation.groups.at(g).at(m).surfaces;
                    pair.first.push_back(&surfaces.at(indices.front()));
                    pair.last.push_back(&surfaces.at(indices.back()));
                    selected.push_back(
                        {{"member", m}, {"first", indices.front()}, {"last", indices.back()}});
                }
                out.report["selected_surfaces"].push_back(std::move(selected));
                groups.push_back(std::move(pair));
            }
            out.attempted = tube_facet_cap_regions(groups, b);
            out.success = out.attempted.report.at("native_result");
            if (out.success) {
                out.caps.push_back(out.attempted.start);
                out.caps.push_back(out.attempted.end);
            }
        }
    }
    out.face_indices =
        enumerate_tube_facet_reference_indices(out.sides.groups, out.caps.size(), out.success, b);
    out.report["native_result"] = out.success;
    out.report["work_used"] = b.work;
    return out;
}
CappedTubeFacets prepare_swept_tube_facets_with_caps(const Json &profile, const Json &path,
                                                     bool capped, TubeBudget &b) {
    return cap_tube_facet_assembly(
        assemble_tube_facet_groups(generate_tube_facet_groups(profile, path, b), b), capped, b);
}
} // namespace p3d::swept_detail
