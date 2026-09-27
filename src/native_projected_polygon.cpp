// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from vupoly.cpp with original P3D projection/coordinate-output rules.
// See THIRD_PARTY.md.
#include "native_projected_polygon.hpp"
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
#include "native_vu_cluster.hpp"
#include "native_vu_regularize.hpp"
#include "native_vu_exterior.hpp"
#include "native_vu_triangulate.hpp"
#include "native_vu_flip.hpp"
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
void transform(std::vector<Point3> &points, const Matrix4 &matrix, curve_detail::BezierWork work) {
    for (const auto &row : std::array<unsigned, 3>{0, 1, 2})
        for (unsigned k = 0; k < 4; ++k)
            finite(matrix[row][k]);
    for (auto &p : points) {
        work.charge(25);
        // This wrapper tests only X and Y; do not import the frame helper's
        // three-component disconnect test or force local Z to zero.
        const double marker = std::numeric_limits<double>::max();
        if (p[0] == marker || p[1] == marker)
            continue;
        const auto source = p;
        for (unsigned i = 0; i < 3; ++i) {
            const auto &r = matrix[i];
            p[i] = finite(finite(finite(finite(source[1] * r[1]) + finite(source[0] * r[0])) +
                                 finite(source[2] * r[2])) +
                          r[3]);
        }
    }
}
} // namespace
NativeProjectedPolygon triangulate_native_projected_polygon(const std::vector<Point3> &points,
                                                            const Matrix4 &local_to_world,
                                                            const Matrix4 &world_to_local,
                                                            double xy_tolerance,
                                                            TubeBudget &budget) {
    require(!points.empty() && points.size() <= budget.max_control_points &&
                points.size() <= INT32_MAX,
            "native projected polygon point extent");
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(points.size());
    for (const auto &p : points)
        for (double x : p)
            finite(x);
    auto local = points;
    transform(local, world_to_local, work);
    NativeProjectedPolygon out;
    out.input_graph = build_native_vu_polygon_input(local, xy_tolerance, budget);
    auto &input = out.input_graph;
    prepare_native_vu_merge(input, budget, 1e-14, 1e-9);
    input.report["regularization"] = regularize_native_vu_graph(input.graph, budget);
    const auto cursor =
        input.report["regularization"]["candidate_array_read_index"].get<std::size_t>();
    input.report["exterior_classification"] = mark_native_vu_exterior(input.graph, cursor, budget);
    input.report["interior_triangulation"] = triangulate_native_vu_interiors(input.graph, budget);
    input.report["edge_adjustment"] = flip_native_vu_triangles(input.graph, budget);
    auto output = collect_native_vu_mesh_indices(input.graph, local, budget);
    out.native_succeeded = output.faces.succeeded;
    out.complete =
        out.native_succeeded && output.faces.report["all_emitted_faces_triangular"] == true;
    input.report["coordinate_index_output"] = output.faces.report;
    input.report["triangulated"] = out.complete;
    out.points = std::move(output.points);
    out.indices = std::move(output.faces.indices);
    // Original wrapper only maps back after the inner operation succeeds.
    if (out.native_succeeded)
        transform(out.points, local_to_world, work);
    out.report = {{"scope", "native_projected_polygon_coordinate_output"},
                  {"native_succeeded", out.native_succeeded},
                  {"complete", out.complete},
                  {"output_coordinate_space", out.native_succeeded ? "world" : "local"},
                  {"source_point_count", points.size()},
                  {"output_point_count", out.points.size()},
                  {"max_edges_per_face", 3},
                  {"index_output", std::move(output.faces.report)}};
    return out;
}
} // namespace p3d::swept_detail
