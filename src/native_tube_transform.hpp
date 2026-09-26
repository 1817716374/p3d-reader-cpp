#pragma once
#include "native_tube_facets.hpp"
namespace p3d::swept_detail {
using TubeCurveViews = std::vector<std::shared_ptr<const BsplineCurve>>;
struct TubeCurveTransform {
    TubeCurveViews curves;
    Json report;
};
// Native list transform, simulated on immutable inputs. clone=true makes one
// independent curve per occurrence. Otherwise only existing pointer aliases
// share working state, including repeated transforms and partial failure.
// Null transform and near-identity transform skip control arithmetic.
TubeCurveTransform transform_tube_curve_list(const TubeCurveViews &, const Matrix4 *transform,
                                             bool clone, TubeBudget &);
struct TubeFacetBranches {
    TubeCurveViews original, transformed;
    Json report;
};
// Connects source member conversion/opening to the native transform branch.
// preserve_original selects cloning; otherwise both branches refer to the
// transformed working object, as in the native shared-list assignment.
TubeFacetBranches prepare_tube_facet_branches(const TubeFacetMember &, const Matrix4 *,
                                              bool preserve_original, TubeBudget &);
} // namespace p3d::swept_detail
