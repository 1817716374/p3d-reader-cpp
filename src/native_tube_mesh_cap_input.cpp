// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from CurveVector.cpp, IPolyfaceConstruction_Add.cpp and
// PolyfaceAddTriangulation.cpp, preserving original P3D cap input rules.
// See THIRD_PARTY.md.
#include "native_tube_mesh_cap_input.hpp"
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
// Original AlmostEqual also operates on generated disconnects. Keep its IEEE
// comparisons: disconnect vs finite is false, identical disconnects are true.
// Unlike ordinary endpoint queries, intermediate infinite squares are needed.
bool near(const Point3 &a, const Point3 &b) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    const double distance = (dy * dy + dx * dx) + dz * dz;
    double scale = a[0] * a[0] + a[1] * a[1];
    scale += a[2] * a[2];
    scale += b[0] * b[0];
    scale += b[1] * b[1];
    scale += b[2] * b[2];
    scale += 1;
    return scale * 0x1.79ca10c924224p-67 > distance;
}
Point3 interpolate(const Point3 &a, const Point3 &b, double t) {
    Point3 p{};
    for (unsigned k = 0; k < 3; ++k) {
        const double delta = finite(b[k] - a[k]);
        p[k] = t <= 0.5 ? finite(finite(delta * t) + a[k]) : finite(finite(delta * (t - 1)) + b[k]);
    }
    return p;
}
std::size_t segment_count(const Point3 &a, const Point3 &b, double maximum) {
    if (!(maximum > 0))
        return 1;
    const double dx = finite(b[0] - a[0]), dy = finite(b[1] - a[1]), dz = finite(b[2] - a[2]);
    const double length = std::sqrt(finite(finite(dy * dy + dx * dx) + dz * dz));
    if (!(length > maximum))
        return 1;
    const double count = length / maximum + 0x1.fffeb074a771dp-1;
    // Native truncates to signed int32, sign-extends, then clamps the unsigned
    // result to 100. Conversion overflow therefore also selects 100.
    if (!(count < 2147483648.0))
        return 100;
    return std::min(std::size_t(100), static_cast<std::size_t>(count));
}
} // namespace
NativeTubeMeshCapInput
prepare_native_tube_mesh_cap_input(const std::vector<std::vector<Point3>> &input, bool reverse,
                                   double max_edge_length, TubeBudget &budget) {
    finite(max_edge_length);
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    std::size_t storage = 0;
    auto grow = [&](std::size_t n) {
        require(n <= budget.max_control_points - storage, "native cap input storage budget");
        storage += n;
        work.charge(n);
    };
    grow(input.size());
    for (const auto &ring : input) {
        grow(ring.size());
        for (const auto &p : ring)
            for (double x : p)
                finite(x);
    }
    NativeTubeMeshCapInput out;
    out.report = {{"scope", "native_polyline_mesh_cap_input"},
                  {"source_ring_count", input.size()},
                  {"reverse_each_ring", reverse},
                  {"enforce_xy_orientation", false},
                  {"resolved_max_edge_length", max_edge_length},
                  {"mesh_generated", false},
                  {"rings", Json::array()}};
    auto finish = [&](const char *reason) {
        out.report["reason"] = reason;
        out.report["preparation_succeeded"] = out.preparation_succeeded;
        out.report["region_type"] = out.region_type;
        out.report["stroked_point_count"] = out.stroked_points.size();
        out.report["compressed_point_count"] = out.points.size();
        return std::move(out);
    };
    if (input.empty())
        return finish("no_rings");
    out.region_type = input.size() == 1 ? 2 : 4;
    const double disconnected = std::numeric_limits<double>::max();
    for (std::size_t r = 0; r < input.size(); ++r) {
        grow(2);
        auto ring = input[r];
        if (reverse)
            std::reverse(ring.begin(), ring.end());
        bool snapped = false, appended = false;
        if (ring.size() > 1) {
            work.charge(32);
            if (near(ring.front(), ring.back())) {
                ring.back() = ring.front();
                snapped = true;
            } else {
                grow(1);
                ring.push_back(ring.front());
                appended = true;
            }
        }
        const unsigned type = r ? 3 : 2;
        out.ring_types.push_back(type);
        std::size_t samples = 0;
        Json counts = Json::array();
        for (std::size_t i = 1; i < ring.size(); ++i) {
            work.charge(24);
            const auto count = segment_count(ring[i - 1], ring[i], max_edge_length);
            grow(1);
            counts.push_back(count);
            const double step = 1.0 / static_cast<double>(count);
            for (std::size_t j = 0; j <= count; ++j) {
                work.charge(50);
                const auto p = interpolate(ring[i - 1], ring[i],
                                           j == count ? 1.0 : static_cast<double>(j) * step);
                ++samples;
                if (out.stroked_points.empty() || !near(p, out.stroked_points.back())) {
                    grow(1);
                    out.stroked_points.push_back(p);
                }
            }
        }
        // Recursive child regions append a disconnect after EVERY child,
        // including empty children and the last one. Single rings do not.
        if (input.size() > 1) {
            grow(1);
            out.stroked_points.push_back({disconnected, disconnected, disconnected});
        }
        out.report["rings"].push_back({{"source_index", r},
                                       {"type", type},
                                       {"closure_snapped", snapped},
                                       {"closure_appended", appended},
                                       {"segment_counts", std::move(counts)},
                                       {"samples_evaluated", samples}});
        out.rings.push_back(std::move(ring));
    }
    // Original addTriangulation tests the uncompressed count. It does not
    // strip trailing disconnects or repeat the count check after compression.
    if (out.stroked_points.size() < 3)
        return finish("too_few_stroked_points");
    grow(1);
    out.points.push_back(out.stroked_points.front());
    for (std::size_t i = 1; i < out.stroked_points.size(); ++i) {
        work.charge(32);
        if (!near(out.stroked_points[i], out.points.back())) {
            grow(1);
            out.points.push_back(out.stroked_points[i]);
        }
    }
    while (out.points.size() > 1) {
        work.charge(32);
        if (!near(out.points.back(), out.points.front()))
            break;
        out.points.pop_back();
        --storage;
    }
    require(out.points.size() <= INT32_MAX, "native cap polygon signed point count");
    // Source selects lower-left/unit axes, unlike the face triangulation path.
    out.projection = prepare_native_polygon_projection(out.points, budget, 1);
    out.preparation_succeeded = out.projection.frame_succeeded;
    out.report["projection"] = out.projection.report;
    return finish(out.preparation_succeeded ? "prepared_for_triangulation" : "frame_failed");
}
} // namespace p3d::swept_detail
