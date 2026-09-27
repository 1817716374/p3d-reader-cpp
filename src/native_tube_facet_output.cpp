#include "native_tube_facet_output.hpp"
namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
} // namespace
TubeFacetSurface copy_tube_facet_surface(const Json &table, const TubeFacetUvBoundaryState &state,
                                         TubeBudget &b) {
    require(!table.contains("boundaries") || table.at("boundaries").is_null(),
            "native generated facet copy does not accept a source trim tree");
    require(state.active_count <= state.allocated.size(),
            "native facet copy active boundary count exceeds storage");
    const auto surface = BsplineSurface::from_bgfb(table);
    require(surface.poles().size() <= b.max_control_points,
            "native facet copy control point budget");
    for (unsigned pass = 0; pass < 8; ++pass)
        charge(b, surface.poles().size());
    charge(b, surface.u().knots().size());
    charge(b, surface.v().knots().size());
    require(state.active_count <= b.max_control_points, "native facet copy boundary count budget");
    std::size_t points = 0;
    for (std::size_t i = 0; i < state.active_count; ++i) {
        require(state.allocated[i].size() <= b.max_control_points - points,
                "native facet copy cumulative UV point budget");
        points += state.allocated[i].size();
        charge(b, state.allocated[i].size());
        charge(b, state.allocated[i].size());
    }
    TubeFacetSurface out{table, {}};
    // Copy uses the original knot buffer verbatim if present. Only a missing
    // buffer takes the native default-knot generator, without normalizing.
    if (surface.u().source_knots().empty())
        out.geometry["knotsU"] = surface.u().knots();
    if (surface.v().source_knots().empty())
        out.geometry["knotsV"] = surface.v().knots();
    out.boundaries.assign(state.allocated.begin(), state.allocated.begin() + state.active_count);
    return out;
}
TubeFacetSurfaceOutput
append_tube_facet_surfaces(std::vector<TubeFacetSurface> &surfaces, std::vector<bool> *flags,
                           const BsplineCurve *prefix_path, const BsplineCurve *prefix_section,
                           const BsplineCurve *suffix_path, const BsplineCurve *suffix_section,
                           TubeBudget &b) {
    TubeFacetSurfaceOutput out;
    out.report = {{"scope", "native_facet_surface_output"},
                  {"native_return_value", nullptr},
                  {"status", "not_evaluated"},
                  {"initial_surface_count", surfaces.size()},
                  {"flags_requested", flags != nullptr},
                  {"appended_surface_count", 0}};
    out.generation = generate_tube_facet_boundaries(prefix_path, prefix_section, suffix_path,
                                                    suffix_section, false, b);
    out.report["generation_status"] = out.generation.report.at("status");
    if (out.generation.status == TubeFacetSeamStatus::pending_general) {
        out.report["status"] = "pending_general";
        out.report["work_used"] = b.work;
        return out;
    }
    const auto &chain = out.generation.composition;
    const auto &trimmed = out.generation.trimmed;
    require(chain.nodes.size() == trimmed.surfaces.size() &&
                chain.nodes.size() == trimmed.boundaries.size() &&
                chain.nodes.size() == chain.seams.size(),
            "native facet output inconsistent completed chain");
    const auto initial = surfaces.size();
    try {
        for (std::size_t i = 0; i < chain.nodes.size(); ++i) {
            require(surfaces.size() < b.max_control_points, "native facet output surface budget");
            auto copy = copy_tube_facet_surface(trimmed.surfaces[i], trimmed.boundaries[i], b);
            if (flags)
                flags->push_back(static_cast<std::uint32_t>(chain.seams[i].classifier) <= 1u);
            surfaces.push_back(std::move(copy));
        }
    } catch (...) {
        // Native failed surface copying clears ALL output entries, including
        // entries that predate this call. Library resource/unsupported errors
        // remain exceptions, rather than being disguised as native false.
        surfaces.clear();
        if (flags)
            flags->clear();
        throw;
    }
    out.report["native_return_value"] = true;
    out.report["status"] = "complete";
    out.report["appended_surface_count"] = surfaces.size() - initial;
    out.report["work_used"] = b.work;
    return out;
}
} // namespace p3d::swept_detail
