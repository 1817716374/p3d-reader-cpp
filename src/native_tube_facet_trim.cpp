#include "native_tube_facet_trim.hpp"
#include "native_curve_planarity.hpp"
namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
Json path_group(const BsplineCurve &c, TubeBudget &b) {
    require(c.poles().size() <= b.max_control_points, "native facet trim path control budget");
    charge(b, c.knots().size());
    for (unsigned i = 0; i < 8; ++i)
        charge(b, c.poles().size());
    Json poles = Json::array();
    for (const auto &p : c.poles())
        for (double x : p)
            poles.push_back(x);
    const Json source{
        {"_type", "BsplineCurve"}, {"order", c.order()},
        {"closed", c.closed()},    {"poles", std::move(poles)},
        {"knots", c.knots()},      {"weights", c.rational() ? Json(c.weights()) : Json()}};
    return {{"_type", "CurveVector"},
            {"type", 1},
            {"curves", Json::array({{{"_type", "VariantGeometry"}, {"geometry", source}}})}};
}
bool class_one(int value) {
    return value == 1 || value == -1;
}
} // namespace
TubeFacetTrimResult trim_tube_facet_chain(const TubeFacetComposition &chain,
                                          const BsplineCurve &path, const BsplineCurve &profile,
                                          TubeBudget &b) {
    require(chain.prepared && chain.report.value("seams_applied", false) &&
                chain.nodes.size() == chain.seams.size(),
            "native facet trimming requires completed seam processing");
    TubeFacetTrimResult out;
    out.report = {{"scope", "native_facet_chain_trimming"},
                  {"native_result", false},
                  {"path_planarity", nullptr},
                  {"visits", Json::array()},
                  {"writes", Json::array()},
                  {"failed_samples", 0},
                  {"boundary_storage", "separate_native_uv_polylines"}};
    if (chain.nodes.empty()) {
        out.report["reason"] = "empty_chain";
        return out;
    }
    require(chain.nodes.size() <= b.max_control_points, "native facet trim node budget");
    std::size_t controls = 0;
    for (const auto &node : chain.nodes) {
        const auto s = BsplineSurface::from_bgfb(node.surface);
        require(s.boundaries().is_null(), "native facet trim input must be initially untrimmed");
        require(s.poles().size() <= b.max_control_points - controls,
                "native facet trim cumulative surface control budget");
        controls += s.poles().size();
        for (unsigned i = 0; i < 8; ++i)
            charge(b, s.poles().size());
        charge(b, s.u().knots().size());
        charge(b, s.v().knots().size());
        out.surfaces.push_back(node.surface);
    }
    out.boundaries.resize(chain.nodes.size());
    const auto planarity = curve_detail::native_curve_vector_planarity(
        path_group(path, b), b.max_control_points, {b.work, b.max_work});
    const bool planar = planarity.at("planar").get<bool>();
    out.report["path_planarity"] = planarity;
    TubeFacetBoundarySegments carried, current, saved_head;
    // f7b80 leaves this buffer unchanged on two empty inputs; retain it across
    // all native write sites, including the delayed head write.
    std::vector<Point3> polygon;
    std::size_t failed = 0, output_points = 0;
    auto copy_segments = [&](const TubeFacetBoundarySegments &source) {
        for (const auto &s : source)
            charge(b, s.size());
        return source;
    };
    auto write = [&](std::size_t index, const TubeFacetBoundarySegments &first,
                     TubeFacetBoundarySegments &second, const char *reason) {
        auto combination = compose_tube_facet_boundary(polygon, first, second, b);
        auto &state = out.boundaries[index];
        std::size_t old = 0;
        for (const auto &p : state.allocated)
            old += p.size();
        // realloc/append uses the current active count. Previously inactive
        // trailing records do not become input to the next append.
        state.allocated.resize(state.active_count);
        const bool appended = append_tube_facet_uv_boundary(state.allocated, polygon, b);
        require(!state.allocated.empty(),
                "native facet trim would dereference absent first boundary");
        std::size_t now = 0;
        for (const auto &p : state.allocated)
            now += p.size();
        require(old <= output_points && now <= b.max_control_points - (output_points - old),
                "native facet trim cumulative boundary point budget");
        output_points = output_points - old + now;
        // f81c0 clears the FIRST pcurve, sets holeOrigin=1, then numBounds=1.
        // A second append is therefore not the selected active boundary.
        state.active_count = 1;
        out.surfaces[index]["holeOrigin"] = 1;
        out.report["writes"].push_back({{"node", index},
                                        {"reason", reason},
                                        {"combination", std::move(combination)},
                                        {"appended", appended},
                                        {"allocated_boundaries", state.allocated.size()},
                                        {"active_count", 1},
                                        {"first_pcurve_cleared", true}});
    };
    for (std::size_t i = 0; i < chain.nodes.size(); ++i) {
        charge(b, 1);
        const auto &seam = chain.seams[i];
        Json visit{{"node", i}, {"classifier", seam.classifier}};
        if (class_one(seam.classifier)) {
            const bool wrap = i + 1 == chain.nodes.size();
            const auto next = wrap ? 0 : i + 1;
            require(seam.plane && seam.plane->alive,
                    "native facet trim missing or invalidated seam plane");
            auto previous = copy_segments(carried);
            // Earlier writes change only boundary state and holeOrigin, never
            // the isocurve geometry, and the native sampler ignores trimming.
            auto samples =
                sample_tube_facet_uv_seam(BsplineSurface::from_bgfb(out.surfaces[i]),
                                          BsplineSurface::from_bgfb(out.surfaces[next]), profile,
                                          seam.plane->value, wrap ? planar : true, b);
            current = std::move(samples.first);
            carried = std::move(samples.second);
            visit["next"] = next;
            visit["require_same_point"] = wrap ? planar : true;
            visit["sampling"] = std::move(samples.report);
            if (!samples.success) {
                ++failed;
                visit["action"] = "sample_failed_continue";
            } else if (!wrap && i == 0 && class_one(chain.seams.back().classifier)) {
                saved_head = copy_segments(current);
                visit["action"] = "head_boundary_deferred";
            } else {
                write(i, previous, current, wrap ? "tail_wrap" : "adjacent_seam");
                if (wrap)
                    write(0, carried, saved_head, "delayed_head");
                visit["action"] = wrap ? "tail_and_head_written" : "boundary_written";
            }
        } else if (!carried.empty()) {
            current.clear();
            write(i, carried, current, "previous_seam_only");
            carried.clear();
            visit["action"] = "previous_seam_consumed";
        } else {
            visit["action"] = "no_boundary_needed";
        }
        out.report["visits"].push_back(std::move(visit));
    }
    out.success = true;
    out.report["native_result"] = true;
    out.report["failed_samples"] = failed;
    out.report["all_samples_succeeded"] = failed == 0;
    out.report["work_used"] = b.work;
    return out;
}
TubeFacetTrimResult finalize_tube_facet_boundaries(const TubeFacetComposition &chain,
                                                   const BsplineCurve &profile, TubeBudget &b) {
    const auto path = prepare_tube_facet_trim_path(chain, b);
    auto out = trim_tube_facet_chain(chain, path.curve, profile, b);
    out.report["path_preparation"] = path.report;
    return out;
}
} // namespace p3d::swept_detail
