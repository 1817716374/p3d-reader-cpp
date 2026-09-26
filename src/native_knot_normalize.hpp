// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from imodel-native bsputil.cpp bspknot_normalizeKnotVector.
// Changes: native P3D division order, threshold and bounded vector storage.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#pragma once
#include "internal.hpp"
namespace p3d::curve_detail {
inline bool normalize_native_knots(std::vector<double> &knots, std::size_t count, unsigned order,
                                   bool closed) {
    require(order >= 2 && count >= order && count <= INT32_MAX &&
                knots.size() == count + (closed ? 2 * std::size_t(order) - 1 : order),
            "native knot normalization layout");
    auto finite = [](double x) {
        require(std::isfinite(x), "native knot normalization nonfinite arithmetic");
        return x;
    };
    const double low = knots[order - 1];
    const double range = finite(knots[closed ? count + order - 1 : count] - low);
    if (std::abs(range) < 1e-10)
        return false;
    for (auto &k : knots)
        k = finite((k - low) / range);
    if (closed) {
        knots[count + order - 1] = 1;
        // Preserve sequential writes: the final lower extension reads the
        // upper endpoint already rewritten by an earlier loop iteration.
        for (unsigned i = 0; i < order; ++i) {
            knots[i] = finite(knots[count + i] - 1);
            knots[count + order - 1 + i] = finite(knots[order - 1 + i] + 1);
        }
    } else {
        std::fill(knots.begin() + count, knots.end(), 1.);
    }
    return true;
}
} // namespace p3d::curve_detail
