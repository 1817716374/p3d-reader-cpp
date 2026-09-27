#pragma once
#include "native_vu_graph.hpp"
namespace p3d::swept_detail {
struct NativeVuCentroid {
    bool succeeded = false;
    Point2 point{};
    // Native reported area applies a second half-factor after success.
    double reported_area = 0;
    std::size_t positive_count = 0, nonpositive_count = 0;
};
NativeVuCentroid native_vu_face_centroid(const NativeVuGraph &, std::size_t, TubeBudget &);
// Exact vertical sweep, including the native conditional centroid fallback.
// Does not select a minimum or classify exterior faces on the caller's behalf.
Json triangulate_native_vu_face(NativeVuGraph &, std::size_t start, TubeBudget &);
// Native edge-limit-three route after exterior classification. Other polygon
// edge limits have a separate coalescing path and are not represented here.
// Does not perform edge flipping or produce final source/attribute indices.
Json triangulate_native_vu_interiors(NativeVuGraph &, TubeBudget &);
} // namespace p3d::swept_detail
