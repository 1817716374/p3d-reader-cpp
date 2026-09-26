#pragma once
#include "native_tube_reference.hpp"
namespace p3d::swept_detail {
struct TubePathSource {
    Json path;
    std::array<Point3, 2> source_endpoints{};
    bool endpoints_found = false;
    Json report;
};
// Native path work copy: split every LineString into its adjacent segments,
// recursively retaining child groups. New segments and child wrappers receive
// no inherited primitive descriptors; other supported primitives are cloned.
// Endpoints are queried on the original root before this conversion.
TubePathSource prepare_tube_path_source(const Json &, TubeBudget &);
struct TubeFacetSources {
    TubeReferenceProfile reference;
    TubePathSource path;
};
// Initial source preparation in native order. Closest-point/plane-intersection
// selection, path splitting and transforms are separate subsequent operations.
TubeFacetSources prepare_tube_facet_sources(const Json &profile, const Json &path, TubeBudget &);
} // namespace p3d::swept_detail
