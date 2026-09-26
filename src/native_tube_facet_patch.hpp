#pragma once
#include "native_tube_refinement.hpp"
namespace p3d::swept_detail {
struct TubeFacetPatch {
    std::optional<Json> surface;
    Matrix3 final_frame{};
    bool success = false;
    // ce4a0's private refined path; it does not modify the callback's path copy.
    std::vector<Point3> working_trace_poles;
    Json report;
};
// ce4a0: incoming frame has already been advanced by the callback. Separate
// from the whole-surface tensor patch; preserves adaptive curvature correction.
TubeFacetPatch generate_tube_facet_patch(const BsplineCurve &section,
                                         const BsplineCurve &normalized_bezier, Matrix3 frame,
                                         bool rigid, TubeBudget &);
// cf5a0: first frame query on a private Bezier, advance shared frame, then
// generate the facet with a second private working copy of that Bezier.
TubeFacetPatch tube_facet_patch(const BsplineCurve &section, const BsplineCurve &normalized_bezier,
                                Matrix3 frame, bool rigid, TubeBudget &);
} // namespace p3d::swept_detail
