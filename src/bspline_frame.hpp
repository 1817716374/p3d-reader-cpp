#pragma once
#include "internal.hpp"

namespace p3d {
struct NativeBsplineFrame {
    Json report;
    // Weighted XYZ after the derivative tolerance query and, when used, the
    // polygon fallback's second unweight/reweight pass. Source is immutable.
    std::vector<Point3> working_poles;
};
NativeBsplineFrame native_bspline_frame_working(const BsplineCurve &, double fraction);
} // namespace p3d
