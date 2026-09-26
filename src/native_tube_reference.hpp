#pragma once
#include "native_tube.hpp"
namespace p3d::swept_detail {
struct TubeReferenceProfile {
    Json profile; // Independent native working copy, not a replacement source.
    Point3 reference{};
    Point3 plane_origin{}, plane_normal{0, 0, 1};
    bool area_valid = false;
    Json report;
};
// Reference-profile portion of native face-patch path preparation. Types 4/5
// select only their first direct child; types 0/1 of the selected copy become
// type 2, with an end-to-start line when needed. Area failure uses endpoints of
// the original root and native guarded division, leaving the default plane.
// This is not the path splitting, placement or complete swept-body entry point.
TubeReferenceProfile prepare_tube_reference_profile(const Json &, TubeBudget &);
} // namespace p3d::swept_detail
