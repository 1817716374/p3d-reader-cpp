#include "native_tube_mesh_edges.hpp"
// Grid-index construction follows Bentley imodel-native BlockedVector.cpp.
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Original P3D correspondence and bounded state replace runtime containers.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
namespace p3d::swept_detail {
TubeMeshEdgeState make_tube_mesh_edge_state(std::size_t patches, std::size_t strips,
                                            bool profile_closed, bool path_closed, TubeBudget &b) {
    require(patches && strips && patches <= INT32_MAX && strips <= INT32_MAX &&
                patches <= b.max_control_points && strips <= b.max_control_points,
            "native mesh edge state dimensions");
    const curve_detail::BezierWork work{b.work, b.max_work};
    work.charge(strips);
    TubeMeshEdgeState s;
    s.patch_count = patches;
    s.strip_count = strips;
    s.profile_closed = profile_closed;
    s.path_closed = path_closed;
    s.rows.resize(strips);
    if (profile_closed) {
        work.charge(patches);
        s.profile_seam.resize(patches);
    }
    if (path_closed) {
        work.charge(strips);
        s.path_seam.resize(strips);
    }
    return s;
}
TubeMeshRegularMesh connect_tube_mesh_regular_vertices(const TubeMeshRegularVertices &grid,
                                                       TubeMeshEdgeState &original,
                                                       const TubeMeshEdgeOptions &opt,
                                                       TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    const auto pi = original.next_patch, si = original.next_strip;
    require(original.patch_count && original.strip_count && original.patch_count <= INT32_MAX &&
                original.strip_count <= INT32_MAX && pi < original.patch_count &&
                si < original.strip_count && original.rows.size() == original.strip_count &&
                original.profile_seam.size() ==
                    (original.profile_closed ? original.patch_count : 0) &&
                original.path_seam.size() == (original.path_closed ? original.strip_count : 0),
            "native mesh edge state position or shape");
    require(grid.u_count >= 2 && grid.v_count >= 2 && grid.u_count <= INT32_MAX &&
                grid.v_count <= INT32_MAX &&
                grid.u_count <= budget.max_control_points / grid.v_count &&
                grid.vertices.size() == grid.u_count * grid.v_count,
            "native mesh edge grid dimensions");
    std::size_t stored = 0;
    auto count = [&](std::size_t n) {
        require(n <= budget.max_control_points - stored,
                "native mesh edge cumulative storage budget");
        stored += n;
    };
    count(original.rows.size());
    count(original.profile_seam.size());
    count(original.path_seam.size());
    for (const auto &v : original.rows)
        count(v.size());
    for (const auto &v : original.profile_seam)
        count(v.size());
    for (const auto &v : original.path_seam)
        count(v.size());
    count(original.column.size());
    count(original.start_points.size());
    count(original.end_points.size());
    work.charge(original.rows.size());
    work.charge(original.profile_seam.size());
    work.charge(original.path_seam.size());
    work.charge(original.column.size());
    work.charge(original.rows[si].size());
    auto &state = original;
    // Only the two native predecessor snapshots are copied. Other vectors
    // are append-only for this call, so their old sizes suffice for rollback;
    // do not copy the whole accumulated group once for every strip.
    struct Rollback {
        TubeMeshEdgeState &s;
        std::size_t pi, si, profile_size, path_size, start_size, end_size;
        std::vector<Point3> column, row;
        bool committed = false;
        ~Rollback() {
            if (committed)
                return;
            s.column.swap(column);
            s.rows[si].swap(row);
            if (s.profile_closed)
                s.profile_seam[pi].resize(profile_size);
            if (s.path_closed)
                s.path_seam[si].resize(path_size);
            s.start_points.resize(start_size);
            s.end_points.resize(end_size);
        }
    } rollback{state,
               pi,
               si,
               state.profile_closed ? state.profile_seam[pi].size() : 0,
               state.path_closed ? state.path_seam[si].size() : 0,
               state.start_points.size(),
               state.end_points.size(),
               state.column,
               state.rows[si]};
    const auto &old_column = rollback.column, &old_row = rollback.row;
    auto get = [](const std::vector<Point3> &v, std::size_t i) -> Point3 {
        require(i < v.size(), "native mesh edge correspondence index");
        for (double x : v[i])
            require(std::isfinite(x), "native mesh nonfinite cached point");
        return v[i];
    };
    auto last = [&](const std::vector<Point3> &v) -> Point3 {
        require(!v.empty(), "native mesh edge correspondence empty endpoint");
        return get(v, v.size() - 1);
    };
    auto append = [&](std::vector<Point3> &v, Point3 p) {
        work.charge(1);
        count(1);
        v.push_back(p);
    };
    auto clear = [&](std::vector<Point3> &v) {
        stored -= v.size();
        v.clear();
    };
    const bool pc = state.profile_closed, vc = state.path_closed;
    const bool last_strip = si + 1 == state.strip_count, last_patch = pi + 1 == state.patch_count;
    // Native seam vectors receive a neighbour endpoint before old row/column
    // snapshots are made. Copies, clear and seeds are ordered, not deduped.
    if (pc && si == 0 && pi > 0)
        append(state.profile_seam[pi], last(state.profile_seam[pi - 1]));
    if (vc && pi == 0 && si > 0)
        append(state.path_seam[si], last(state.path_seam[si - 1]));
    clear(state.column);
    clear(state.rows[si]);
    if (pi > 0)
        append(state.column, last(old_row));
    if (si > 0)
        append(state.rows[si], last(state.rows[si - 1]));
    TubeMeshRegularMesh out;
    auto emit = [&](Point3 p) {
        work.charge(1);
        require(out.points.size() < budget.max_control_points,
                "native mesh coordinate output budget");
        out.points.push_back(p);
    };
    auto cap_start = [&](Point3 p) {
        if (opt.collect_start)
            append(state.start_points, p);
    };
    auto cap_end = [&](Point3 p) {
        if (opt.collect_end)
            append(state.end_points, p);
    };
    std::size_t replacements = 0;
    for (std::size_t j = 0; j < grid.v_count; ++j)
        for (std::size_t i = 0; i < grid.u_count; ++i) {
            work.charge(32);
            const auto &vertex = grid.vertices[j * grid.u_count + i];
            const auto p = vertex.point;
            for (double x : p)
                require(std::isfinite(x), "native mesh nonfinite source point");
            bool inserted = false;
            const bool first_u = i == 0, last_u = i + 1 == grid.u_count, first_v = j == 0,
                       last_v = j + 1 == grid.v_count;
            if (pc) {
                if (si == 0 && first_u && (pi == 0 || !first_v))
                    append(state.profile_seam[pi], p);
                if (last_strip && last_u) {
                    auto q = get(state.profile_seam[pi], j);
                    emit(q);
                    ++replacements;
                    inserted = true;
                    if (first_v)
                        cap_start(get(state.profile_seam[pi], 0));
                    if (last_v)
                        cap_end(q);
                }
            }
            if (si > 0 && first_u) {
                if (!(vc && last_patch && last_v)) {
                    emit(get(old_column, j));
                    ++replacements;
                }
                if (first_v)
                    cap_start(get(old_column, 0));
                if (last_v)
                    cap_end(get(old_column, j));
                inserted = true;
            }
            if (last_u && (pi == 0 || !first_v)) {
                if (vc && last_patch && last_v)
                    append(state.column, last(state.path_seam[si]));
                else
                    append(state.column, p);
            }
            if (vc) {
                if (pi == 0 && first_v && (si == 0 || !first_u))
                    append(state.path_seam[si], p);
                if (last_patch && last_v) {
                    emit(get(state.path_seam[si], i));
                    ++replacements;
                    inserted = true;
                }
            }
            if (pi > 0 && (si == 0 || !first_u) && !(pc && last_strip && last_u) && first_v) {
                auto q = get(old_row, i);
                emit(q);
                ++replacements;
                inserted = true;
                cap_start(q);
            }
            if (last_v && (si == 0 || !first_u)) {
                if (pc && last_strip && last_u)
                    append(state.rows[si], get(state.rows[0], 0));
                else
                    append(state.rows[si], p);
            }
            if (!inserted) {
                emit(p);
                if (first_v)
                    cap_start(p);
                if (last_v)
                    cap_end(p);
            }
            if (opt.normals) {
                for (double x : vertex.normal)
                    require(std::isfinite(x), "native mesh nonfinite normal");
                out.normals.push_back(vertex.normal);
            }
            if (opt.parameters) {
                for (double x : vertex.parameter)
                    require(std::isfinite(x), "native mesh nonfinite parameter");
                out.parameters.push_back(vertex.parameter);
            }
        }
    // 24e9e0 / 28ef20: integer division counts COMPLETE coordinate rows.
    // At the double-closed final corner both native seam branches can append;
    // keep the extra original point instead of forcing one point per sample.
    const auto rows = out.points.size() / grid.u_count;
    require(rows >= 2 && out.points.size() <= INT32_MAX, "native triangle grid coordinate extent");
    const auto cells = (rows - 1) * (grid.u_count - 1);
    require(cells <= budget.max_control_points / 8, "native triangle grid index budget");
    work.charge(cells * 8);
    out.point_indices.reserve(cells * 8);
    for (std::size_t j = 1; j < rows; ++j)
        for (std::size_t i = 1; i < grid.u_count; ++i) {
            const auto a = std::int32_t((j - 1) * grid.u_count + i), b = a + 1,
                       c = std::int32_t(j * grid.u_count + i), d = c + 1;
            // Keep the conversion helper's original order, distinct from the
            // direct implicit-grid visitor's cyclic ordering of the second face.
            for (auto n : {a, b, c, 0, b, d, c, 0})
                out.point_indices.push_back(n);
        }
    if (opt.normals)
        out.normal_indices = out.point_indices;
    if (opt.parameters)
        out.parameter_indices = out.point_indices;
    out.report = {{"scope", "native_swept_regular_strip_mesh"},
                  {"patch", pi},
                  {"strip", si},
                  {"input_vertices", grid.vertices.size()},
                  {"coordinates", out.points.size()},
                  {"complete_rows", rows},
                  {"unused_tail_coordinates", out.points.size() % grid.u_count},
                  {"correspondence_writes", replacements},
                  {"triangles", cells * 2},
                  {"coordinate_combination_applied", false},
                  {"caps_generated", false}};
    if (++state.next_strip == state.strip_count) {
        state.next_strip = 0;
        ++state.next_patch;
    }
    rollback.committed = true;
    return out;
}
} // namespace p3d::swept_detail
