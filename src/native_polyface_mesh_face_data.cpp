// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from Polyface.cpp and FacetFaceData.cpp. Original P3D ranges,
// statistics and typed visitor positioning. See THIRD_PARTY.md.
#include "native_polyface_mesh_face_data.hpp"
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
namespace p3d::swept_detail {
namespace {
using curve_detail::bezier_support::finite;
using Work = curve_detail::BezierWork;
template <std::size_t N> std::array<std::array<double, N>, 2> null_range() {
    std::array<std::array<double, N>, 2> out;
    out[0].fill(std::numeric_limits<double>::max());
    out[1].fill(-std::numeric_limits<double>::max());
    return out;
}
bool is_null(const std::array<Point2, 2> &range) {
    // GeRange2d::isNull checks magnitude, not reversed min/max bounds.
    for (const auto &end : range)
        for (double x : end)
            if (std::abs(finite(x)) >= 1e100)
                return true;
    return false;
}
template <std::size_t N>
void extend(std::array<std::array<double, N>, 2> &range, const std::array<double, N> &point,
            Work work) {
    work.charge(N * 3);
    for (double x : point)
        finite(x);
    if constexpr (N == 3) {
        // Original 3D range extension ignores a disconnected point if any
        // coordinate is +DBL_MAX. The 2D overload has no such filter.
        for (double x : point)
            if (x == std::numeric_limits<double>::max())
                return;
    }
    for (std::size_t k = 0; k < N; ++k) {
        if (range[0][k] > point[k])
            range[0][k] = point[k];
        if (point[k] > range[1][k])
            range[1][k] = point[k];
    }
}
struct Diagnostics {
    std::size_t visited = 0, invalid_points = 0, truncated_attributes = 0, distance_missing = 0;
    bool complete = true;
    Json visits = Json::array();
    void add(NativePolyfaceVisit &&v) {
        visited += v.read_indices.size();
        complete = complete && v.complete;
        if (v.report.contains("invalid_point_facets"))
            invalid_points += v.report["invalid_point_facets"].get<std::size_t>();
        if (v.report.contains("truncated_attribute_facets"))
            for (const auto &n : v.report["truncated_attribute_facets"])
                truncated_attributes += n.get<std::size_t>();
        if (v.report.contains("incomplete_attribute_reads"))
            truncated_attributes += v.report["incomplete_attribute_reads"].get<std::size_t>();
        visits.push_back(std::move(v.report));
    }
};
struct DistanceResult {
    std::size_t samples = 0;
    bool assigned = false;
};
DistanceResult distance_range(NativeBuilderFaceData &data, const NativePolyfaceMesh &mesh,
                              std::size_t first, std::size_t end, TubeBudget &budget,
                              Diagnostics &diagnostics) {
    Work work{budget.work, budget.max_work};
    DistanceResult result;
    Point2 sum{}, squares{};
    bool began = false, missing = false;
    auto v = consume_native_polyface_from(
        mesh, budget, static_cast<std::uint32_t>(first),
        [&](std::size_t read, const NativePolyfaceVisitorFacet &f,
            const NativePolyfaceVisitedData &d) {
            if (began && read >= end)
                return false;
            began = true;
            if (f.points.empty() || d.parameters.empty()) {
                missing = true;
                ++diagnostics.distance_missing;
                return false;
            }
            require(d.parameters.size() >= f.points.size(),
                    "native face distance reads incomplete UV visitor");
            for (std::size_t k = 2; k < f.points.size(); ++k) {
                work.charge(55);
                Point2 uv0{}, uv1{};
                Point3 delta0{}, delta1{}, du{}, dv{};
                for (unsigned i = 0; i < 2; ++i) {
                    uv0[i] = finite(d.parameters[k - 2][i] - d.parameters[k - 1][i]);
                    uv1[i] = finite(d.parameters[k - 1][i] - d.parameters[k][i]);
                }
                for (unsigned i = 0; i < 3; ++i) {
                    delta0[i] = finite(f.points[k - 2][i] - f.points[k - 1][i]);
                    delta1[i] = finite(f.points[k - 1][i] - f.points[k][i]);
                }
                const double cross =
                    std::abs(finite(finite(uv0[0] * uv1[1]) - finite(uv1[0] * uv0[1])));
                if (cross == 0)
                    continue;
                for (unsigned i = 0; i < 3; ++i) {
                    du[i] = finite(finite(delta0[i] * uv1[1]) + finite(delta1[i] * -uv0[1]));
                    dv[i] = finite(finite(delta1[i] * uv0[0]) + finite(delta0[i] * -uv1[0]));
                }
                auto magnitude = [&](const Point3 &p) {
                    const double xy = finite(finite(p[0] * p[0]) + finite(p[1] * p[1]));
                    return std::sqrt(finite(xy + finite(p[2] * p[2])));
                };
                const Point2 scale{finite(magnitude(du) / cross), finite(magnitude(dv) / cross)};
                for (unsigned i = 0; i < 2; ++i) {
                    sum[i] = finite(sum[i] + scale[i]);
                    squares[i] = finite(squares[i] + finite(scale[i] * scale[i]));
                }
                ++result.samples;
                require(result.samples <= std::numeric_limits<std::int32_t>::max(),
                        "native face distance sample counter overflow");
            }
            return true;
        },
        face_channel);
    diagnostics.add(std::move(v));
    if (missing || !began)
        return result;
    if (!result.samples)
        return result;
    const double count = static_cast<double>(result.samples), inverse = 1 / count;
    std::array<Point2, 2> range{};
    for (unsigned i = 0; i < 2; ++i) {
        const double mean = finite(sum[i] * inverse);
        const double variance = finite(finite(squares[i] / count) - finite(mean * mean));
        const double scale = finite(mean + std::sqrt(std::abs(variance)));
        range[1][i] =
            finite(scale * finite(data.parameter_range[1][i] - data.parameter_range[0][i]));
    }
    data.parameter_distance_range = range;
    result.assigned = true;
    return result;
}
struct Context {
    NativePolyfaceMesh state;
    Work work;
    TubeBudget &budget;
    std::size_t storage = 0;
    Diagnostics diagnostics;
    Json calls = Json::array();
    Context(const NativePolyfaceMesh &input, TubeBudget &b) : work{b.work, b.max_work}, budget(b) {
        std::size_t total = 0;
        auto count = [&](std::size_t n) {
            require(n <= b.max_control_points - total, "native face-data input storage budget");
            total += n;
            work.charge(n);
        };
        const auto &m = input.data;
        count(m.coordinates.points.size());
        count(m.coordinates.normals.size());
        count(m.coordinates.parameters.size());
        for (const auto &v : m.indices.indices)
            count(v.size());
        count(m.face_data.size());
        for (auto n :
             {input.double_colors.size(), input.float_colors.size(), input.integer_colors.size(),
              input.color_table.size(), input.face_uv_points.size(), input.face_material_ids.size(),
              input.face_smooth_groups.size(), input.illumination_name.size()})
            count(n);
        count(m.edge_chains.size());
        for (const auto &e : m.edge_chains) {
            count(e.point_indices.size());
            count(e.topology_ids.size());
        }
        require(m.indices.indices[point_channel].size() < std::numeric_limits<std::uint32_t>::max(),
                "native face-data index count exceeds visitor cursor");
        storage = total;
        state = input;
    }
    bool set(const std::optional<NativeBuilderFaceData> &provided, std::size_t end) {
        work.charge(1);
        auto &m = state.data;
        auto &face_indices = m.indices.indices[face_channel];
        const auto &point_indices = m.indices.indices[point_channel];
        Json report{
            {"first_index", face_indices.size()}, {"requested_end", end}, {"appended", false}};
        if (!m.indices.active[face_channel] || face_indices.size() >= point_indices.size()) {
            report["reason"] = "inactive_or_no_remaining_indices";
            calls.push_back(std::move(report));
            return false;
        }
        if (!end)
            end = point_indices.size();
        require(end <= point_indices.size(), "native face-data end index exceeds source");
        NativeBuilderFaceData data = provided ? *provided : native_null_face_data();
        const bool extend_uv = is_null(data.parameter_range) && !m.coordinates.parameters.empty();
        data.point_range = null_range<3>();
        data.normal_range = null_range<3>();
        bool began = false;
        auto visited = consume_native_polyface_from(
            state, budget, static_cast<std::uint32_t>(face_indices.size()),
            [&](std::size_t read, const NativePolyfaceVisitorFacet &f,
                const NativePolyfaceVisitedData &d) {
                if (began && read >= end)
                    return false;
                began = true;
                require(d.normals.empty() || d.normals.size() >= f.points.size(),
                        "native face-data reads incomplete normal visitor");
                require(!extend_uv || d.parameters.empty() ||
                            d.parameters.size() >= f.points.size(),
                        "native face-data reads incomplete UV visitor");
                for (std::size_t i = 0; i < f.points.size(); ++i) {
                    extend(data.point_range, f.points[i], work);
                    if (!d.normals.empty())
                        extend(data.normal_range, d.normals[i], work);
                    if (extend_uv && !d.parameters.empty())
                        extend(data.parameter_range, d.parameters[i], work);
                }
                return true;
            },
            face_channel);
        diagnostics.add(std::move(visited));
        if (!began) {
            report["reason"] = "visitor_start_failed";
            calls.push_back(std::move(report));
            return false;
        }
        DistanceResult distance;
        const bool compute_distance = state.pool_active[native_parameter_pool] &&
                                      !m.coordinates.parameters.empty() &&
                                      is_null(data.parameter_distance_range);
        if (compute_distance)
            distance = distance_range(data, state, face_indices.size(), end, budget, diagnostics);
        require(m.face_data.size() < budget.max_control_points &&
                    m.face_data.size() <
                        static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()),
                "native face-data record budget");
        const auto growth = 1 + (end > face_indices.size() ? end - face_indices.size() : 0);
        require(growth <= budget.max_control_points - storage,
                "native face-data output storage budget");
        storage += growth;
        m.face_data.push_back(data);
        require(end <= budget.max_control_points, "native face-data index output budget");
        for (std::size_t i = face_indices.size(); i < end; ++i) {
            work.charge(1);
            face_indices.push_back(
                point_indices[i] == 0 ? 0 : static_cast<std::int32_t>(m.face_data.size()));
        }
        report["appended"] = true;
        report["end_index"] = end;
        report["parameter_range_extended"] = extend_uv;
        report["distance_requested"] = compute_distance;
        report["distance_samples"] = distance.samples;
        report["distance_range_assigned"] = distance.assigned;
        calls.push_back(std::move(report));
        return true;
    }
    NativePolyfaceMeshFaceData finish(const char *operation, bool native_build) {
        NativePolyfaceMeshFaceData out;
        const auto &indices = state.data.indices.indices;
        const bool covered = indices[face_channel].size() == indices[point_channel].size();
        bool references_valid = covered;
        if (covered)
            for (std::size_t i = 0; i < indices[face_channel].size(); ++i) {
                work.charge(1);
                const auto index = indices[face_channel][i];
                if (indices[point_channel][i] == 0 ? index != 0
                                                   : index <= 0 || static_cast<std::size_t>(index) >
                                                                       state.data.face_data.size())
                    references_valid = false;
            }
        const bool raw_without_storage = state.mesh_style != 1 &&
                                         !state.data.coordinates.points.empty() &&
                                         indices[point_channel].empty();
        out.complete = covered && references_valid && diagnostics.complete &&
                       !diagnostics.distance_missing && !raw_without_storage;
        out.report = {{"operation", operation},
                      {"complete", out.complete},
                      {"calls", std::move(calls)},
                      {"all_point_indices_assigned", covered},
                      {"visited_facets", diagnostics.visited},
                      {"face_references_valid", references_valid},
                      {"invalid_point_facets", diagnostics.invalid_points},
                      {"truncated_attribute_visits", diagnostics.truncated_attributes},
                      {"missing_distance_data", diagnostics.distance_missing},
                      {"visitor_wrap_count", 0},
                      {"raw_source_has_no_face_index_storage", raw_without_storage},
                      {"visits", std::move(diagnostics.visits)}};
        if (native_build)
            out.report["native_succeeded"] = true;
        out.output = std::move(state);
        return out;
    }
};
} // namespace

NativePolyfaceMeshFaceData
set_native_polyface_mesh_face_data(const NativePolyfaceMesh &input,
                                   const std::optional<NativeBuilderFaceData> &provided,
                                   std::size_t end, TubeBudget &budget) {
    Context context(input, budget);
    context.set(provided, end);
    return context.finish("native_typed_set_new_face", false);
}
NativePolyfaceMeshFaceData build_native_polyface_mesh_face_data(const NativePolyfaceMesh &input,
                                                                TubeBudget &budget) {
    Context context(input, budget);
    auto &mesh = context.state.data;
    mesh.indices.active[face_channel] = true;
    if (!mesh.coordinates.parameters.empty()) {
        context.set(std::nullopt, mesh.indices.indices[point_channel].size());
        for (auto &face : mesh.face_data) {
            context.work.charge(1);
            face.parameter_range = {Point2{0, 0}, Point2{1, 1}};
        }
    } else {
        auto v = consume_native_polyface_from(
            context.state, budget, 0,
            [&](std::size_t read, const NativePolyfaceVisitorFacet &,
                const NativePolyfaceVisitedData &d) {
                context.set(native_null_face_data(),
                            read + d.edge_count + (mesh.num_per_face == 0 ? 1 : 0));
                return true;
            },
            face_channel);
        context.diagnostics.add(std::move(v));
    }
    // This native version leaves FaceData activity unchanged and FaceIndex
    // active even for empty data. Do not import newer upstream flag cleanup.
    return context.finish("native_typed_build_per_face_face_data", true);
}
} // namespace p3d::swept_detail
