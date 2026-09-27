#pragma once
#include "native_polyface_face_data.hpp"
namespace p3d::swept_detail {
struct NativePolygonConvexity {
    bool convex = false;
    Point3 unit_normal{};
    double positive_turn_sum = 0, negative_turn_sum = 0;
    Json report;
};
// Native 3D cross-product test. Exact trailing copies of the first point are
// ignored; this is not a planarity, self-intersection or polygon-validity test.
NativePolygonConvexity native_polygon_convexity(const std::vector<Point3> &, TubeBudget &);
struct NativePolyfaceFacetQueries {
    std::size_t max_facet_size = 0;
    bool has_convex_facets = true, complete = false;
    Json report;
};
// Original outer-builder max-face and convex-facet queries, on prepared
// indexed-query data. Point-only visitors deliberately ignore other channels.
NativePolyfaceFacetQueries query_native_polyface_facets(const NativePolyfaceFaceDataState &,
                                                        TubeBudget &);
} // namespace p3d::swept_detail
