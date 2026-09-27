// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from Polyface.cpp, FacetFaceData.cpp, PolyfaceVisitor.cpp and
// polyfaceAddNormals.cpp.
// Original P3D ranges, activity flags, visitor bounds and operation order.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_polyface_face_data.hpp"
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
};
struct Facet {
    std::size_t read = 0, next = 0, count = 0;
    std::vector<Point3> points, normals;
    std::vector<Point2> parameters;
};
class Visitor {
    const NativeBuilderPolyface &mesh;
    Work work;
    Diagnostics &diagnostics;
    template <std::size_t N>
    std::vector<std::array<double, N>> collect(const std::vector<std::array<double, N>> &pool,
                                               const std::vector<std::int32_t> &indices,
                                               std::size_t first, std::size_t count) {
        std::vector<std::array<double, N>> out;
        if (indices.empty())
            return out;
        out.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            work.charge(1);
            require(first + i < indices.size(),
                    "native face-data visitor index read outside source");
            const auto signed_index = static_cast<std::int64_t>(indices[first + i]);
            if (signed_index == 0)
                break;
            const auto index =
                static_cast<std::uint64_t>(signed_index < 0 ? -signed_index : signed_index) - 1;
            if (index >= pool.size())
                break;
            out.push_back(pool[static_cast<std::size_t>(index)]);
        }
        return out;
    }

  public:
    Visitor(const NativeBuilderPolyface &m, Work w, Diagnostics &d)
        : mesh(m), work(w), diagnostics(d) {}
    bool read(std::size_t first, Facet &out) {
        work.charge(1);
        const auto &indices = mesh.indices.indices;
        const auto &point_indices = indices[point_channel];
        if (first >= point_indices.size())
            return false;
        std::size_t count = 0, next = 0;
        if (mesh.num_per_face > 1) {
            while (count < mesh.num_per_face && first + count < point_indices.size() &&
                   point_indices[first + count]) {
                work.charge(1);
                ++count;
            }
            next = first + mesh.num_per_face;
        } else {
            while (first < point_indices.size() && point_indices[first] == 0) {
                work.charge(1);
                ++first;
            }
            while (first + count < point_indices.size() && point_indices[first + count]) {
                work.charge(1);
                ++count;
            }
            next = first + count + 1;
        }
        require(next <= std::numeric_limits<std::uint32_t>::max(),
                "native face-data visitor cursor overflow");
        if (count == 0)
            return false;
        out = Facet{};
        out.read = first;
        out.next = next;
        out.count = count;
        const auto &coordinates = mesh.coordinates;
        out.points = collect(coordinates.points, point_indices, first, count);
        if (out.points.size() < count) {
            ++diagnostics.invalid_points;
            return false;
        }
        auto channel = [&](const auto &pool, std::size_t which) {
            const auto &src = indices[which].empty() && pool.size() == coordinates.points.size()
                                  ? point_indices
                                  : indices[which];
            auto values = collect(pool, src, first, count);
            if (!src.empty() && values.size() < count)
                ++diagnostics.truncated_attributes;
            return values;
        };
        out.normals = channel(coordinates.normals, normal_channel);
        out.parameters = channel(coordinates.parameters, parameter_channel);
        // Face-data consumers keep the constructor's wrap count of zero.
        // Attach(..., true) requests all data; it does not request wrapping.
        ++diagnostics.visited;
        return true;
    }
};
struct DistanceResult {
    std::size_t samples = 0;
    bool assigned = false;
};
DistanceResult distance_range(NativeBuilderFaceData &data, const NativeBuilderPolyface &mesh,
                              std::size_t first, std::size_t end, Work work,
                              Diagnostics &diagnostics) {
    Visitor visitor(mesh, work, diagnostics);
    Facet face;
    DistanceResult result;
    if (!visitor.read(first, face))
        return result;
    Point2 sum{}, squares{};
    do {
        if (face.points.empty() || face.parameters.empty()) {
            ++diagnostics.distance_missing;
            return result; // Do not publish sums collected from earlier facets.
        }
        require(face.parameters.size() >= face.count,
                "native face distance reads incomplete UV visitor");
        for (std::size_t k = 2; k < face.count; ++k) {
            work.charge(55);
            Point2 uv0{}, uv1{};
            Point3 delta0{}, delta1{}, du{}, dv{};
            for (unsigned i = 0; i < 2; ++i) {
                uv0[i] = finite(face.parameters[k - 2][i] - face.parameters[k - 1][i]);
                uv1[i] = finite(face.parameters[k - 1][i] - face.parameters[k][i]);
            }
            for (unsigned i = 0; i < 3; ++i) {
                delta0[i] = finite(face.points[k - 2][i] - face.points[k - 1][i]);
                delta1[i] = finite(face.points[k - 1][i] - face.points[k][i]);
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
    } while (visitor.read(face.next, face) && face.read < end);
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
    NativePolyfaceFaceDataState state;
    Work work;
    TubeBudget &budget;
    std::size_t storage = 0;
    Diagnostics diagnostics;
    Json calls = Json::array();
    Context(const NativePolyfaceFaceDataState &input, TubeBudget &b)
        : work{b.work, b.max_work}, budget(b) {
        std::size_t total = 0;
        auto count = [&](std::size_t n) {
            require(n <= b.max_control_points - total, "native face-data input storage budget");
            total += n;
            work.charge(n);
        };
        const auto &m = input.mesh;
        count(m.coordinates.points.size());
        count(m.coordinates.normals.size());
        count(m.coordinates.parameters.size());
        for (const auto &v : m.indices.indices)
            count(v.size());
        count(m.face_data.size());
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
        auto &m = state.mesh;
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
        Visitor visitor(m, work, diagnostics);
        Facet face;
        if (!visitor.read(face_indices.size(), face)) {
            report["reason"] = "visitor_start_failed";
            calls.push_back(std::move(report));
            return false;
        }
        do {
            require(face.normals.empty() || face.normals.size() >= face.count,
                    "native face-data reads incomplete normal visitor");
            require(!extend_uv || face.parameters.empty() || face.parameters.size() >= face.count,
                    "native face-data reads incomplete UV visitor");
            for (std::size_t i = 0; i < face.count; ++i) {
                extend(data.point_range, face.points[i], work);
                if (!face.normals.empty())
                    extend(data.normal_range, face.normals[i], work);
                if (extend_uv && !face.parameters.empty())
                    extend(data.parameter_range, face.parameters[i], work);
            }
        } while (visitor.read(face.next, face) && face.read < end);
        DistanceResult distance;
        const bool compute_distance = state.parameter_pool_active &&
                                      !m.coordinates.parameters.empty() &&
                                      is_null(data.parameter_distance_range);
        if (compute_distance)
            distance = distance_range(data, m, face_indices.size(), end, work, diagnostics);
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
    NativePolyfaceFaceDataResult finish(const char *operation, bool native_build) {
        NativePolyfaceFaceDataResult out;
        const auto &indices = state.mesh.indices.indices;
        const bool covered = indices[face_channel].size() == indices[point_channel].size();
        bool references_valid = covered;
        if (covered)
            for (std::size_t i = 0; i < indices[face_channel].size(); ++i) {
                work.charge(1);
                const auto index = indices[face_channel][i];
                if (indices[point_channel][i] == 0 ? index != 0
                                                   : index <= 0 || static_cast<std::size_t>(index) >
                                                                       state.mesh.face_data.size())
                    references_valid = false;
            }
        out.complete = covered && references_valid && diagnostics.invalid_points == 0 &&
                       diagnostics.truncated_attributes == 0 && diagnostics.distance_missing == 0;
        out.report = {{"operation", operation},
                      {"complete", out.complete},
                      {"calls", std::move(calls)},
                      {"all_point_indices_assigned", covered},
                      {"visited_facets", diagnostics.visited},
                      {"face_references_valid", references_valid},
                      {"invalid_point_facets", diagnostics.invalid_points},
                      {"truncated_attribute_visits", diagnostics.truncated_attributes},
                      {"missing_distance_data", diagnostics.distance_missing},
                      {"visitor_wrap_count", 0}};
        if (native_build)
            out.report["native_succeeded"] = true;
        out.output = std::move(state);
        return out;
    }
};
} // namespace

NativeBuilderFaceData native_null_face_data() {
    NativeBuilderFaceData data;
    data.parameter_distance_range = null_range<2>();
    data.parameter_range = null_range<2>();
    data.point_range = null_range<3>();
    data.normal_range = null_range<3>();
    return data;
}
NativePolyfaceFaceDataResult
set_native_polyface_face_data(const NativePolyfaceFaceDataState &input,
                              const std::optional<NativeBuilderFaceData> &provided, std::size_t end,
                              TubeBudget &budget) {
    Context context(input, budget);
    context.set(provided, end);
    return context.finish("native_set_new_face", false);
}
NativePolyfaceFaceDataResult
build_native_polyface_face_data(const NativePolyfaceFaceDataState &input, TubeBudget &budget) {
    Context context(input, budget);
    auto &mesh = context.state.mesh;
    mesh.indices.active[face_channel] = true;
    if (!mesh.coordinates.parameters.empty()) {
        context.set(std::nullopt, mesh.indices.indices[point_channel].size());
        for (auto &face : mesh.face_data) {
            context.work.charge(1);
            face.parameter_range = {Point2{0, 0}, Point2{1, 1}};
        }
    } else {
        Visitor visitor(mesh, context.work, context.diagnostics);
        Facet face;
        std::size_t next = 0;
        while (visitor.read(next, face)) {
            context.set(native_null_face_data(),
                        face.read + face.count + (mesh.num_per_face == 0 ? 1 : 0));
            next = face.next;
        }
    }
    // This native version leaves FaceData activity unchanged and FaceIndex
    // active even for empty data. Do not import newer upstream flag cleanup.
    return context.finish("native_build_per_face_face_data", true);
}
} // namespace p3d::swept_detail
