#pragma once
#include "native_tube_path_placement.hpp"
#include "native_tube_transform.hpp"
namespace p3d::swept_detail {
struct TubeFacetSectionBranches {
    // Original is the native conversion list after any shared in-place writes.
    TubeCurveViews original, prefix, suffix;
    bool success = false;
    Json report;
};
// A null matrix means that path branch is absent. Copies only the prefix when
// both paths exist. Reverses only the selected list's first curve, propagating
// that mutation to existing aliases. Inputs remain immutable.
TubeFacetSectionBranches place_tube_facet_sections(const TubeCurveViews &, const Matrix4 *prefix,
                                                   const Matrix4 *suffix, bool ring_flag,
                                                   TubeBudget &);
struct TubeFacetPreparation {
    std::optional<TubeFacetPathPlacement> placement;
    TubeFacetProfile profile;
    Json orientation;
    bool success = false;
    Json report;
};
// Source validation, path placement, source orientation, then profile partition
// in native order. Curves are prepared per member, not eagerly for every ring:
// later face generation can mutate shared path state between members.
TubeFacetPreparation prepare_tube_facet_inputs(const Json &profile, const Json &path, TubeBudget &);
TubeFacetSectionBranches prepare_tube_facet_sections(const TubeFacetPreparation &,
                                                     std::size_t group, std::size_t member,
                                                     TubeBudget &);
} // namespace p3d::swept_detail
