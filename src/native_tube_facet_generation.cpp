#include "native_tube_facet_generation.hpp"
namespace p3d::swept_detail {
TubeFacetGeneration generate_tube_facet_boundaries(const BsplineCurve *prefix_path,
                                                   const BsplineCurve *prefix_section,
                                                   const BsplineCurve *suffix_path,
                                                   const BsplineCurve *suffix_section, bool rigid,
                                                   TubeBudget &b) {
    TubeFacetGeneration out;
    out.report = {{"scope", "native_facet_generation"},
                  {"status", "native_failure"},
                  {"native_return_code", 1},
                  {"trim_profile_source", nullptr},
                  {"all_trim_samples_succeeded", nullptr}};
    out.composition = prepare_tube_facet_composition(prefix_path, prefix_section, suffix_path,
                                                     suffix_section, rigid, b);
    if (!out.composition.prepared) {
        out.report["failed_stage"] = "branch_generation_or_composition";
        out.report["work_used"] = b.work;
        return out;
    }
    const auto seams = process_tube_facet_seams(out.composition, b);
    if (seams.status != TubeFacetSeamStatus::complete) {
        out.status = seams.status;
        out.report["failed_stage"] = "seam_processing";
        if (seams.status == TubeFacetSeamStatus::pending_general) {
            out.report["status"] = "pending_general";
            out.report["native_return_code"] = nullptr;
        }
        out.report["work_used"] = b.work;
        return out;
    }
    const auto profile_index = prefix_section ? 1u : 3u;
    const auto &profile = out.composition.working_sources[profile_index];
    require(bool(profile), "native facet generation lost its selected working section");
    out.report["trim_profile_source"] = profile_index;
    out.trimmed = finalize_tube_facet_boundaries(out.composition, *profile, b);
    if (!out.trimmed.success) {
        // Native f95b0 deletes the chain when finalization returns false. Source
        // work survives; it is not rolled back to the original input controls.
        out.composition.nodes.clear();
        out.composition.seams.clear();
        out.composition.prepared = false;
        out.composition.report["status"] = "native_failure";
        out.report["failed_stage"] = "boundary_processing";
    } else {
        out.status = TubeFacetSeamStatus::complete;
        out.composition.report["status"] = "boundaries_processed";
        out.report["status"] = "complete";
        out.report["native_return_code"] = 0;
        out.report["all_trim_samples_succeeded"] = out.trimmed.report.at("all_samples_succeeded");
    }
    out.report["work_used"] = b.work;
    return out;
}
} // namespace p3d::swept_detail
