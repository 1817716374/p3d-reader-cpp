#pragma once
#include "native_tube.hpp"
namespace p3d::swept_detail {
struct TubeCaps {
    Json start = nullptr, end = nullptr, report;
};
// Native cap-region helper over already oriented side surfaces. On a native
// failure the report is false and may retain partially built, unreversed caps.
// This does not triangulate, prove planarity, or supply native face indices.
TubeCaps tube_cap_regions(const std::vector<Json> &surfaces, TubeBudget &);
struct CappedTube {
    TubeSurfaces sides;
    std::vector<Json> caps;
    Json report;
};
// Native outer caller: caps depend on source endpoint closure, not the closed
// flag of the converted working trace. Failed caps are not appended; generated
// side surfaces remain available. No complete solid/mesh guarantee is implied.
CappedTube prepare_swept_tube_with_caps(const Json &profile, const Json &path, bool capped,
                                        TubeBudget &);
} // namespace p3d::swept_detail
