#include "native_tube_mesh_index_rules.hpp"
// Quad index selection also follows Bentley imodel-native Polyface.cpp.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// P3D arithmetic order, low-count branch and edge visibility are preserved.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_bezier_support.hpp"
#include <set>
namespace p3d::swept_detail {
NativeFacetIndexPlan native_facet_index_plan(const std::vector<Point3> &points,
                                             TubeBudget &budget) {
    using curve_detail::bezier_support::finite;
    require(!points.empty() && points.size() <= budget.max_control_points &&
                points.size() <= INT32_MAX,
            "native facet point extent");
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(points.size());
    for (const auto &p : points)
        for (double x : p)
            finite(x);
    NativeFacetIndexPlan out;
    if (points.size() <= 3) {
        out.route = NativeFacetIndexPlan::Route::passthrough;
        work.charge(points.size() + 1);
        require(points.size() + 1 <= budget.max_control_points, "native facet index output budget");
        for (std::size_t i = 0; i < points.size(); ++i)
            out.indices.push_back(std::int32_t(i + 1));
        out.indices.push_back(0);
    } else if (points.size() == 4) {
        require(budget.max_control_points >= 8, "native quad index output budget");
        work.charge(64);
        auto cross = [&](std::size_t i, std::size_t j, std::size_t k) {
            Point3 a{}, b{};
            for (std::size_t q = 0; q < 3; ++q) {
                a[q] = finite(points[j][q] - points[i][q]);
                b[q] = finite(points[k][q] - points[i][q]);
            }
            return Point3{finite(finite(a[1] * b[2]) - finite(a[2] * b[1])),
                          finite(finite(a[2] * b[0]) - finite(a[0] * b[2])),
                          finite(finite(a[0] * b[1]) - finite(a[1] * b[0]))};
        };
        auto dot = [&](Point3 a, Point3 b) {
            return finite(finite(finite(a[1] * b[1]) + finite(a[0] * b[0])) + finite(a[2] * b[2]));
        };
        const auto d0 = dot(cross(0, 1, 2), cross(0, 2, 3));
        const auto d1 = dot(cross(1, 2, 3), cross(1, 3, 0));
        out.route = NativeFacetIndexPlan::Route::quad;
        out.indices = d0 > d1 ? std::vector<std::int32_t>{1, 2, -3, 0, -1, 3, 4, 0}
                              : std::vector<std::int32_t>{1, -2, 4, 0, -4, 2, 3, 0};
    }
    if (points.size() > 4) {
        require(points.size() < budget.max_control_points && points.size() < INT32_MAX,
                "native facet visitor closure budget");
        work.charge(points.size() + 1);
        auto wrapped = points;
        wrapped.push_back(points.front()); // Native visitor wrap count is one.
        out.projection = prepare_native_polygon_projection(wrapped, budget);
    }
    // Larger faces require the original projected-loop triangulator. Never
    // replace that branch with an arbitrary triangle fan or claim completion.
    return out;
}
TubeMeshVisibleIndices apply_tube_mesh_edge_visibility(const std::vector<std::int32_t> &input,
                                                       const TubeMeshVisibilityLayout &layout,
                                                       TubeBudget &budget) {
    require(input.size() <= budget.max_control_points && input.size() <= INT32_MAX &&
                layout.coordinate_count <= budget.max_control_points &&
                layout.coordinate_count <= INT32_MAX && layout.u_count >= 2 &&
                layout.v_count >= 2 && layout.u_count <= INT32_MAX && layout.v_count <= INT32_MAX,
            "native visibility input extent");
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(input.size());
    for (auto x : input)
        require(x != INT32_MIN, "native visibility signed magnitude overflow");
    using Edge = std::pair<std::int32_t, std::int32_t>;
    struct Compare {
        const curve_detail::BezierWork &work;
        bool operator()(Edge a, Edge b) const {
            work.charge(1);
            return a < b;
        }
    };
    std::set<Edge, Compare> visible(Compare{work});
    auto insert = [&](std::int64_t a, std::int64_t b) {
        work.charge(1);
        require(a >= INT32_MIN && a <= INT32_MAX && b >= INT32_MIN && b <= INT32_MAX,
                "native visibility edge integer overflow");
        require(visible.size() < budget.max_control_points, "native visibility edge set budget");
        if (a > b)
            std::swap(a, b);
        visible.emplace(std::int32_t(a), std::int32_t(b));
    };
    if (!layout.trimmed) {
        auto column = [&](std::int64_t first) {
            for (std::size_t j = 1; j < layout.v_count; ++j) {
                const auto next = first + std::int64_t(layout.u_count);
                insert(first, next);
                first = next;
            }
        };
        if (layout.first_column_visible)
            column(1);
        if (layout.last_column_visible)
            column(std::int64_t(layout.u_count));
    } else {
        if (layout.first_column_visible)
            for (std::int64_t k = 1; k < layout.first_column_count; ++k)
                insert(k, k + 1);
        if (layout.last_column_visible) {
            // Actual coordinate count, including any native extra corner.
            const auto offset = std::int64_t(layout.coordinate_count) - layout.last_column_count;
            require(offset >= INT32_MIN && offset <= INT32_MAX,
                    "native visibility last column offset overflow");
            for (std::int64_t k = 1; k < layout.last_column_count; ++k)
                insert(offset + k, offset + k + 1);
        }
    }
    TubeMeshVisibleIndices out;
    out.indices = input;
    std::int32_t first = 0;
    bool start = true;
    std::size_t changed = 0;
    for (std::size_t i = 0; i + 1 < out.indices.size(); ++i) {
        work.charge(1);
        auto &x = out.indices[i];
        if (start) {
            first = x;
            start = false;
        }
        if (!x) {
            start = true;
            continue;
        }
        const auto a = std::abs(x);
        const auto b = std::abs(out.indices[i + 1] ? out.indices[i + 1] : first);
        const auto key = std::minmax(a, b);
        const bool show = visible.find({key.first, key.second}) != visible.end();
        if (show != (x > 0)) {
            x = -x;
            ++changed;
        }
    }
    out.report = {{"scope", "native_swept_point_edge_visibility"},
                  {"selected_edges", visible.size()},
                  {"changed_signs", changed},
                  {"attribute_indices_changed", false},
                  {"work_used", budget.work}};
    return out;
}
} // namespace p3d::swept_detail
