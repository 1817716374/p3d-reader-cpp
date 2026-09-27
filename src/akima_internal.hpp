#pragma once
#include "internal.hpp"

namespace p3d::akima_detail {
// A finite, bounded input may still fail the native post-filter count guard.
// Keep that confirmed rejection separate from numerical/preparation exceptions.
struct Fit {
    std::optional<BsplineCurve> curve;
    std::vector<Point3> source;
    std::vector<std::size_t> retained_indices;
    Json report;
};
Fit fit(const Json &table);
} // namespace p3d::akima_detail
