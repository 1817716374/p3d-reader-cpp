#include "native_builder_coordinates.hpp"
// Coordinate comparison and map insertion adapted from Bentley imodel-native
// PolyfaceCoordinateMap.cpp and PolyfaceAdd.cpp.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Native tree traversal, normal tolerance and floating-point sequence differ
// from newer upstream implementations. See THIRD_PARTY.md.
#include "mesh_registry.hpp"
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"

namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
// Tolerance equivalence is non-transitive. Do not pass this comparison to an
// STL ordered container: its strict weak ordering requirement is not met.
// The explicit red/black tree preserves the native lower-bound search path.
class CoordinateMap {
    struct Node {
        Point3 key{};
        std::size_t children[2]{}, parent = 0;
        bool red = true;
    };
    std::vector<Node> nodes{{}};
    std::size_t root = 0, comparisons = 0, rotations = 0;
    double absolute, relative;
    curve_detail::BezierWork work;

    bool less(const Point3 &a, const Point3 &b) {
        work.charge(25);
        ++comparisons;
        double tol = absolute;
        if (relative != 0) {
            double sum = finite(std::abs(a[0]) + std::abs(b[0]));
            sum = finite(sum + std::abs(a[1]));
            sum = finite(sum + std::abs(b[1]));
            sum = finite(sum + std::abs(a[2]));
            sum = finite(sum + std::abs(b[2]));
            tol = finite(tol + finite(relative * sum));
        }
        for (int k = 2; k >= 0; --k) {
            const double delta = finite(b[k] - a[k]);
            if (delta > tol)
                return false;
            if (delta < -tol)
                return true;
        }
        return false;
    }
    void rotate(std::size_t x, unsigned side) {
        work.charge(1);
        ++rotations;
        const auto y = nodes[x].children[1 - side];
        nodes[x].children[1 - side] = nodes[y].children[side];
        if (nodes[y].children[side])
            nodes[nodes[y].children[side]].parent = x;
        const auto p = nodes[x].parent;
        nodes[y].parent = p;
        if (!p)
            root = y;
        else
            nodes[p].children[nodes[p].children[1] == x] = y;
        nodes[y].children[side] = x;
        nodes[x].parent = y;
    }

  public:
    CoordinateMap(double abs, double rel, curve_detail::BezierWork w)
        : absolute(finite(abs)), relative(finite(rel)), work(w) {
        require(abs >= 0 && rel >= 0, "native builder coordinate map negative tolerance");
        nodes.front().red = false;
    }
    std::size_t insert(const Point3 &key) {
        work.charge(1);
        for (double x : key)
            finite(x);
        std::size_t parent = 0, candidate = 0, at = root;
        unsigned side = 0;
        while (at) {
            parent = at;
            side = less(nodes[at].key, key) ? 1 : 0;
            if (!side)
                candidate = at;
            at = nodes[at].children[side];
        }
        if (candidate && !less(key, nodes[candidate].key))
            return candidate - 1;
        const auto id = nodes.size();
        nodes.push_back(Node{key, {0, 0}, parent, true});
        if (!parent)
            root = id;
        else
            nodes[parent].children[side] = id;
        at = id;
        while (nodes[nodes[at].parent].red) {
            work.charge(1);
            parent = nodes[at].parent;
            auto grandparent = nodes[parent].parent;
            side = nodes[grandparent].children[1] == parent;
            const auto uncle = nodes[grandparent].children[1 - side];
            if (nodes[uncle].red) {
                nodes[parent].red = nodes[uncle].red = false;
                nodes[grandparent].red = true;
                at = grandparent;
            } else {
                if (at == nodes[parent].children[1 - side]) {
                    at = parent;
                    rotate(at, side);
                    parent = nodes[at].parent;
                    grandparent = nodes[parent].parent;
                }
                nodes[parent].red = false;
                nodes[grandparent].red = true;
                rotate(grandparent, 1 - side);
            }
        }
        nodes[root].red = false;
        return id - 1;
    }
    Json report() const {
        return {{"absolute_tolerance", absolute},
                {"relative_tolerance", relative},
                {"coordinates", nodes.size() - 1},
                {"comparisons", comparisons},
                {"rotations", rotations}};
    }
};
} // namespace

NativeBuilderCoordinates
map_native_builder_coordinates(const std::vector<NativeBuilderCoordinateBatch> &batches,
                               const NativeBuilderCoordinateOptions &options, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    require(batches.size() <= budget.max_control_points, "native builder batch storage budget");
    std::size_t input_count = 0;
    for (const auto &batch : batches) {
        work.charge(1);
        for (auto n : {batch.points.size(), batch.parameters.size(), batch.normals.size()}) {
            require(n <= budget.max_control_points - input_count,
                    "native builder coordinate storage budget");
            input_count += n;
        }
    }
    CoordinateMap points(options.point_absolute_tolerance, options.point_relative_tolerance, work),
        params(options.parameter_absolute_tolerance, options.parameter_relative_tolerance, work),
        normals(1e-14, 1e-6, work);
    NativeBuilderCoordinates out;
    out.batches.reserve(batches.size());
    for (const auto &batch : batches) {
        finite(batch.parameter_scope);
        NativeBuilderCoordinateRemap remap;
        remap.points.reserve(batch.points.size());
        remap.parameters.reserve(batch.parameters.size());
        remap.normals.reserve(batch.normals.size());
        for (const auto &point : batch.points) {
            const auto index = points.insert(point);
            if (index == out.points.size())
                out.points.push_back(point);
            remap.points.push_back(index);
        }
        for (const auto &uv : batch.parameters) {
            const auto index = params.insert({uv[0], uv[1], batch.parameter_scope});
            if (index == out.parameters.size())
                out.parameters.push_back(uv);
            remap.parameters.push_back(index);
        }
        for (auto normal : batch.normals) {
            work.charge(12);
            for (auto &x : normal) {
                finite(x);
                if (batch.reverse_normals)
                    x = -x;
            }
            if (batch.normalize_normals)
                normal = mesh_detail::unit(normal);
            const auto index = normals.insert(normal);
            if (index == out.normals.size())
                out.normals.push_back(normal);
            remap.normals.push_back(index);
        }
        out.batches.push_back(std::move(remap));
    }
    out.report = {{"operation", "native_builder_coordinate_insertion"},
                  {"input_coordinates", input_count},
                  {"batches", batches.size()},
                  {"points", points.report()},
                  {"parameters", params.report()},
                  {"normals", normals.report()},
                  {"facet_assembly_performed", false}};
    return out;
}
} // namespace p3d::swept_detail
