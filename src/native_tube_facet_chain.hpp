#pragma once
#include "native_tube_facet_patch.hpp"
namespace p3d::swept_detail {
struct TubeFacetSeam {
    // Native classifier is initially -2; 2 means no seam plane is needed.
    int classifier = -2;
    std::optional<Point3> incoming, outgoing;
    std::optional<std::array<Point3, 2>> plane; // origin, unnormalized normal
};
// Updates the plane from an existing tangent pair without normalizing or
// repeating the parallel check. A degenerate normal clears only the pair;
// the native routine retains any previous plane. Missing pairs are a no-op.
void update_native_tube_facet_plane(TubeFacetSeam &, const Point3 &point);
TubeFacetSeam native_tube_facet_seam(Point3 incoming, Point3 outgoing, const Point3 &point);
struct TubeFacetNode {
    Json surface;
    TubeFacetSeam end_seam;
};
struct TubeFacetChain {
    std::vector<TubeFacetNode> nodes;
    std::vector<Point3> working_source_poles;
    Matrix3 final_frame{};
    bool success = false;
    Json report;
};
// f8a90 with the native cf5a0 callback. Builds ordered independent patches and
// end-seam descriptors; does not perform f8130/f81c0 miter/plane clipping.
TubeFacetChain build_tube_facet_chain(const BsplineCurve &section, const BsplineCurve &path,
                                      bool rigid, TubeBudget &);
} // namespace p3d::swept_detail
