// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from Polyface.cpp, FacetFaceData.cpp, PolyfaceVisitor.cpp and
// polyfaceAddNormals.cpp, PolyfaceEdgeChain.cpp, pf_halfEdgeArray.h and MTGGraph.cpp.
// Original P3D ranges, activity flags, visitor bounds and operation order.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_polyface_face_data.hpp"
#include "native_polyface_attributes.hpp"
#include "native_polyface_connectivity.hpp"
#include "native_polyface_edge_chains.hpp"
#include "native_attribute_sort.hpp"
#include "native_polygon_projection.hpp"
#include "native_bezier.hpp"
#include "native_bezier_support.hpp"
#include <map>

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
    std::size_t ignored_truncation_channel;
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
    Visitor(const NativeBuilderPolyface &m, Work w, Diagnostics &d,
            std::size_t ignored = polyface_channel_count)
        : mesh(m), work(w), diagnostics(d), ignored_truncation_channel(ignored) {}
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
            if (!src.empty() && values.size() < count && which != ignored_truncation_channel)
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
namespace {
NativePolyfaceAttributes build_attributes(const NativePolyfaceFaceDataState &input, bool parameters,
                                          int selector, TubeBudget &budget) {
    Context context(input, budget);
    auto &state = context.state;
    auto &mesh = state.mesh;
    auto &coordinates = mesh.coordinates;
    const auto &points = mesh.indices.indices[point_channel];
    const auto channel = parameters ? parameter_channel : normal_channel;
    auto &indices = mesh.indices.indices[channel];
    NativePolyfaceAttributes out;
    out.report = {{"operation", parameters ? "native_build_per_face_parameters"
                                           : "native_build_per_face_normals"},
                  {"coordinate_selector", parameters ? selector : 0},
                  {"visitor_wrap_count", 0}};
    if (points.empty()) {
        out.report["reason"] = "empty_point_index_array";
        out.report["native_succeeded"] = false;
        out.report["complete"] = false;
        out.output = std::move(state);
        return out;
    }
    context.storage -= indices.size();
    context.storage -= parameters ? coordinates.parameters.size() : coordinates.normals.size();
    auto grow = [&](std::size_t n) {
        require(n <= budget.max_control_points - context.storage,
                "native attribute output storage budget");
        context.storage += n;
        context.work.charge(n);
    };
    grow(points.size());
    indices.assign(points.size(), 0);
    mesh.indices.active[channel] = true;
    if (parameters) {
        coordinates.parameters.clear();
        state.parameter_pool_active = true;
        for (auto &f : mesh.face_data) {
            context.work.charge(1);
            f.parameter_range = null_range<2>();
        }
    } else {
        coordinates.normals.clear();
        state.normal_pool_active = true;
    }
    Visitor visitor(mesh, context.work, context.diagnostics, channel);
    Facet face;
    std::size_t next = 0, failed_frames = 0, range_updates = 0, range_skipped = 0;
    Json frames = Json::array();
    while (visitor.read(next, face)) {
        next = face.next;
        auto frame = prepare_native_polygon_projection(face.points, budget,
                                                       parameters ? selector : 0, parameters);
        frames.push_back({{"read_index", face.read}, {"frame", std::move(frame.report)}});
        if (!frame.frame_succeeded)
            ++failed_frames;
        if (parameters) {
            for (std::size_t i = 0; i < face.count; ++i) {
                grow(1);
                require(coordinates.parameters.size() < static_cast<std::size_t>(INT32_MAX),
                        "native parameter index overflow");
                const Point2 uv = frame.frame_succeeded
                                      ? Point2{frame.points[i][0], frame.points[i][1]}
                                      : Point2{};
                coordinates.parameters.push_back(uv);
                const std::size_t read = face.read + i;
                indices[read] = static_cast<std::int32_t>(coordinates.parameters.size());
                const auto &face_indices = mesh.indices.indices[face_channel];
                if (mesh.indices.active[face_channel] && read < face_indices.size()) {
                    // Native 252407 reads the signed stored value directly,
                    // with no one-based adjustment, then uses an unsigned bound.
                    const auto key = face_indices[read];
                    if (key >= 0 && static_cast<std::size_t>(key) < mesh.face_data.size()) {
                        extend(mesh.face_data[static_cast<std::size_t>(key)].parameter_range, uv,
                               context.work);
                        ++range_updates;
                    } else
                        ++range_skipped;
                }
            }
        } else if (frame.frame_succeeded) {
            grow(1);
            require(coordinates.normals.size() < static_cast<std::size_t>(INT32_MAX),
                    "native normal index overflow");
            coordinates.normals.push_back({frame.local_to_world[0][2], frame.local_to_world[1][2],
                                           frame.local_to_world[2][2]});
            for (std::size_t i = 0; i < face.count; ++i) {
                context.work.charge(1);
                indices[face.read + i] = static_cast<std::int32_t>(coordinates.normals.size());
            }
        }
    }
    std::size_t missing = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        context.work.charge(1);
        if (points[i] != 0 && indices[i] == 0)
            ++missing;
    }
    out.native_succeeded = true;
    out.complete = missing == 0 && failed_frames == 0 && range_skipped == 0 &&
                   context.diagnostics.invalid_points == 0 &&
                   context.diagnostics.truncated_attributes == 0;
    out.report["native_succeeded"] = out.native_succeeded;
    out.report["complete"] = out.complete;
    out.report["frames"] = std::move(frames);
    out.report["failed_frames"] = failed_frames;
    out.report["unassigned_nonzero_indices"] = missing;
    out.report["invalid_point_facets"] = context.diagnostics.invalid_points;
    out.report["truncated_attribute_visits"] = context.diagnostics.truncated_attributes;
    if (parameters) {
        out.report["face_range_indexing"] = "native_direct_stored_index";
        out.report["face_range_updates"] = range_updates;
        out.report["face_range_updates_skipped"] = range_skipped;
    }
    out.output = std::move(state);
    return out;
}
} // namespace
NativePolyfaceAttributes build_native_polyface_normals(const NativePolyfaceFaceDataState &input,
                                                       TubeBudget &budget) {
    return build_attributes(input, false, 0, budget);
}
NativePolyfaceAttributes build_native_polyface_parameters(const NativePolyfaceFaceDataState &input,
                                                          int selector, TubeBudget &budget) {
    return build_attributes(input, true, selector, budget);
}
NativePolyfaceConnectivity
build_native_polyface_connectivity(const NativePolyfaceFaceDataState &input,
                                   bool ignore_degeneracies, TubeBudget &budget) {
    Context context(input, budget);
    auto &mesh = context.state.mesh;
    const auto &indices = mesh.indices.indices[point_channel];
    require(mesh.coordinates.points.size() <= static_cast<std::size_t>(INT32_MAX),
            "native connectivity vertex label overflow");
    NativePolyfaceConnectivity out;
    auto &edges = out.half_edges;
    auto &nodes = out.nodes;
    auto grow = [&](std::size_t count) {
        require(count <= budget.max_control_points - context.storage,
                "native connectivity storage budget");
        context.storage += count;
        context.work.charge(count);
    };
    auto vertex = [&](std::size_t read) {
        const auto v = static_cast<std::int64_t>(indices[read]);
        return static_cast<std::size_t>(v < 0 ? -v : v);
    };
    Visitor visitor(mesh, context.work, context.diagnostics);
    Facet face;
    std::size_t next = 0, filtered_corners = 0, skipped_faces = 0, visited_corners = 0;
    while (visitor.read(next, face)) {
        next = face.next;
        visited_corners += face.count;
        require(face.count <= budget.max_control_points - context.storage,
                "native connectivity temporary corner budget");
        std::vector<std::size_t> pack;
        pack.reserve(face.count);
        std::size_t count = face.count;
        if (ignore_degeneracies) {
            while (count > 1 && vertex(face.read + count - 1) == vertex(face.read)) {
                context.work.charge(1);
                --count;
            }
            pack.push_back(0);
            for (std::size_t k = 1; k < count; ++k) {
                context.work.charge(1);
                if (vertex(face.read + k) != vertex(face.read + pack.back()))
                    pack.push_back(k);
            }
            filtered_corners += face.count - pack.size();
            if (pack.size() == 2) {
                filtered_corners += 2;
                ++skipped_faces;
                continue;
            }
        } else {
            for (std::size_t k = 0; k < count; ++k) {
                context.work.charge(1);
                pack.push_back(k);
            }
        }
        grow(pack.size());
        for (std::size_t k = 0; k < pack.size(); ++k) {
            context.work.charge(1);
            const auto a = face.read + pack[k], z = face.read + pack[(k + 1) % pack.size()];
            edges.push_back({vertex(a), vertex(z), a, z, 0, indices[a] > 0});
        }
    }
    // Sort whole half-edge identities with the original MSVC exchange order.
    // Stable sorting equal keys would change pairing and subsequent node IDs.
    grow(edges.size());
    std::vector<std::size_t> order(edges.size());
    for (std::size_t i = 0; i < order.size(); ++i)
        order[i] = i;
    auto key = [](const NativePolyfaceHalfEdge &e) {
        return std::pair{std::min(e.vertex0, e.vertex1), std::max(e.vertex0, e.vertex1)};
    };
    auto less = [&](std::size_t a, std::size_t z) {
        context.work.charge(1);
        return key(edges[a]) < key(edges[z]);
    };
    NativeAttributeSort sorter(order, less);
    sorter.sort(0, order.size(), order.size());
    grow(edges.size());
    std::vector<NativePolyfaceHalfEdge> sorted;
    sorted.reserve(edges.size());
    for (auto i : order)
        sorted.push_back(edges[i]);
    edges = std::move(sorted);
    std::size_t boundary_edges = 0, paired_edges = 0, special_groups = 0, same_direction_pairs = 0;
    if (!edges.empty()) {
        std::size_t max_read = 0;
        for (const auto &e : edges) {
            context.work.charge(1);
            max_read = std::max({max_read, e.read_index, e.successor_read_index});
        }
        require(max_read < static_cast<std::size_t>(INT32_MAX),
                "native connectivity read label overflow");
        grow(max_read + 1);
        out.read_to_half_edge.assign(max_read + 1, SIZE_MAX);
        for (std::size_t i = 0; i < edges.size(); ++i)
            out.read_to_half_edge[edges[i].read_index] = i;
        auto make_edge = [&](std::size_t a, std::size_t z, bool primary_a, bool primary_z,
                             std::int32_t read_a, std::int32_t read_z, bool boundary) {
            grow(2);
            require(nodes.size() <= static_cast<std::size_t>(INT32_MAX) - 2,
                    "native connectivity node label overflow");
            const auto n = nodes.size();
            nodes.push_back({n, n + 1, primary_a ? native_mtg_primary : 0,
                             static_cast<std::int32_t>(a - 1), read_a});
            nodes.push_back({n + 1, n, primary_z ? native_mtg_primary : 0,
                             static_cast<std::int32_t>(z - 1), read_z});
            if (boundary) {
                nodes[n].mask |= native_mtg_boundary;
                nodes[n + 1].mask |= native_mtg_boundary | native_mtg_exterior;
                if (a == z)
                    nodes[n + 1].mask |= native_mtg_polar_loop;
                ++boundary_edges;
            }
            return n;
        };
        auto pair = [&](std::size_t a, std::size_t z) {
            auto &x = edges[a];
            auto &y = edges[z];
            const auto n = make_edge(x.vertex0, y.vertex0, x.visible, y.visible,
                                     static_cast<std::int32_t>(x.read_index),
                                     static_cast<std::int32_t>(y.read_index), false);
            x.node = n;
            y.node = n + 1;
            ++paired_edges;
            if (x.vertex0 != y.vertex1 || x.vertex1 != y.vertex0)
                ++same_direction_pairs;
        };
        auto successor = [&](std::size_t i) {
            context.work.charge(1);
            return out.read_to_half_edge[edges[i].successor_read_index];
        };
        for (std::size_t first = 0; first < edges.size();) {
            std::size_t last = first + 1;
            while (last < edges.size() && key(edges[last]) == key(edges[first])) {
                context.work.charge(1);
                ++last;
            }
            bool coupled = false;
            if (last - first == 2) {
                pair(first, first + 1);
                coupled = true;
            } else if (last - first == 4) {
                auto partner = [&](std::size_t a, std::size_t excluded) {
                    for (std::size_t i = first; i < last; ++i) {
                        context.work.charge(1);
                        if (i != a && i != excluded && edges[i].vertex0 == edges[a].vertex1 &&
                            edges[i].vertex1 == edges[a].vertex0)
                            return i;
                    }
                    return SIZE_MAX;
                };
                for (std::size_t a = first; a < last && !coupled; ++a) {
                    const auto z = successor(a);
                    if (z == SIZE_MAX)
                        continue;
                    const auto c = successor(z);
                    if (c == SIZE_MAX)
                        continue;
                    const auto d = successor(c);
                    if (edges[z].vertex0 == edges[z].vertex1 && c >= first && c < last && d == a) {
                        const auto pa = partner(a, c), pc = partner(c, a);
                        if (pa != SIZE_MAX && pc != SIZE_MAX && pa != pc) {
                            pair(a, pa);
                            pair(c, pc);
                            coupled = true;
                            ++special_groups;
                        }
                    }
                }
            }
            if (!coupled)
                for (std::size_t i = first; i < last; ++i) {
                    auto &e = edges[i];
                    e.node = make_edge(e.vertex0, e.vertex1, true, true,
                                       static_cast<std::int32_t>(e.read_index), -1, true);
                }
            first = last;
        }
        auto twist = [&](std::size_t a, std::size_t z) {
            context.work.charge(8);
            // V-F-V reaches the face predecessor in the original MTG node chain.
            const auto pa = nodes[nodes[nodes[a].vertex_successor].face_successor].vertex_successor;
            const auto pz = nodes[nodes[nodes[z].vertex_successor].face_successor].vertex_successor;
            std::swap(nodes[a].vertex_successor, nodes[z].vertex_successor);
            std::swap(nodes[pa].face_successor, nodes[pz].face_successor);
        };
        for (std::size_t i = 0; i < edges.size(); ++i) {
            const auto j = successor(i);
            require(j < edges.size(), "native connectivity missing face successor");
            twist(nodes[edges[i].node].face_successor, edges[j].node);
        }
        out.native_succeeded = true;
    }
    std::size_t label_mismatches = 0, nonzero = 0;
    for (auto i : indices) {
        context.work.charge(1);
        if (i)
            ++nonzero;
    }
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        context.work.charge(1);
        const auto &n = nodes[i];
        require(n.face_successor < nodes.size() && n.vertex_successor < nodes.size(),
                "native connectivity broken successor");
        require(nodes[n.face_successor].vertex_successor == (i ^ 1),
                "native connectivity broken edge mate");
        if (n.vertex_index != nodes[n.vertex_successor].vertex_index)
            ++label_mismatches;
    }
    out.complete = out.native_succeeded && visited_corners == nonzero && label_mismatches == 0 &&
                   context.diagnostics.invalid_points == 0 &&
                   context.diagnostics.truncated_attributes == 0;
    out.report = {{"operation", "native_polyface_to_mtg_connectivity"},
                  {"native_succeeded", out.native_succeeded},
                  {"complete", out.complete},
                  {"ignore_degeneracies", ignore_degeneracies},
                  {"filtered_corners", filtered_corners},
                  {"skipped_degenerate_faces", skipped_faces},
                  {"paired_edges", paired_edges},
                  {"boundary_edges", boundary_edges},
                  {"four_edge_degenerate_couplings", special_groups},
                  {"same_direction_pairs", same_direction_pairs},
                  {"vertex_label_mismatches", label_mismatches},
                  {"unvisited_nonzero_indices", nonzero - visited_corners},
                  {"invalid_point_facets", context.diagnostics.invalid_points},
                  {"truncated_attribute_visits", context.diagnostics.truncated_attributes},
                  {"vertex_label_tag", -1},
                  {"read_index_label_tag", -10004}};
    out.points = std::move(mesh.coordinates.points);
    return out;
}
NativePolyfaceEdgeChains build_native_polyface_edge_chains(const NativePolyfaceFaceDataState &input,
                                                           std::size_t draw_method_index,
                                                           TubeBudget &budget) {
    Context context(input, budget);
    auto &mesh = context.state.mesh;
    NativePolyfaceEdgeChains out;
    Json report{{"operation", "native_build_edge_chains"},
                {"draw_method_index", draw_method_index},
                {"draw_method_used", false},
                {"visitor_wrap_count", 1}};
    auto finish = [&](bool native, bool complete, const char *reason) {
        out.native_status = native ? 0 : 1;
        out.native_succeeded = native;
        out.complete = complete;
        report["native_status"] = out.native_status;
        report["native_succeeded"] = native;
        report["complete"] = complete;
        report["reason"] = reason;
        out.output = std::move(context.state);
        out.report = std::move(report);
        return std::move(out);
    };
    if (!mesh.edge_chains.empty())
        return finish(false, false, "edge_chains_already_exist");
    const auto &indices = mesh.indices.indices[point_channel];
    using Key = std::pair<std::size_t, std::size_t>;
    struct Faces {
        std::size_t first, last, occurrences;
    };
    auto less = [&](const Key &a, const Key &b) {
        context.work.charge(1);
        return a < b;
    };
    std::map<Key, Faces, decltype(less)> edge_faces(less);
    auto grow = [&](std::size_t count) {
        require(count <= budget.max_control_points - context.storage,
                "native edge-chain storage budget");
        context.storage += count;
        context.work.charge(count);
    };
    auto vertex = [&](std::size_t read) {
        context.work.charge(1);
        const auto n = static_cast<std::int64_t>(indices[read]);
        const auto index = static_cast<std::uint64_t>(n < 0 ? -n : n) - 1;
        require(index < static_cast<std::uint64_t>(INT32_MAX),
                "native edge-chain point index overflow");
        return static_cast<std::size_t>(index);
    };
    auto key = [](std::size_t a, std::size_t b) { return a < b ? Key{a, b} : Key{b, a}; };
    Diagnostics passes[2];
    std::size_t visited[2]{}, qualified[2]{}, skipped[2]{};
    std::size_t visible = 0, unique = 0, shared = 0, over_two = 0, boundary = 0;
    {
        Visitor visitor(mesh, context.work, passes[0]);
        Facet face;
        std::size_t next = 0, facet_index = 0;
        while (visitor.read(next, face)) {
            next = face.next;
            visited[0] += face.count;
            if (face.count < 3) {
                ++skipped[0];
                continue;
            }
            ++qualified[0];
            ++facet_index;
            for (std::size_t i = 0; i < face.count; ++i) {
                context.work.charge(1);
                if (indices[face.read + i] <= 0)
                    continue;
                ++visible;
                const auto edge =
                    key(vertex(face.read + i), vertex(face.read + (i + 1) % face.count));
                const auto found = edge_faces.find(edge);
                if (found == edge_faces.end()) {
                    grow(1);
                    edge_faces.emplace(edge, Faces{facet_index, facet_index, 1});
                    ++unique;
                } else {
                    auto &f = found->second;
                    ++f.occurrences;
                    if (facet_index < f.first) {
                        f.last = f.first;
                        f.first = facet_index;
                    } else
                        f.last = facet_index;
                }
            }
            // Preserved original increment at both ends: qualified faces are 1,3,5,...
            ++facet_index;
        }
    }
    Json emitted = Json::array();
    {
        Visitor visitor(mesh, context.work, passes[1]);
        Facet face;
        std::size_t next = 0;
        while (visitor.read(next, face)) {
            next = face.next;
            visited[1] += face.count;
            if (face.count < 3) {
                ++skipped[1];
                continue;
            }
            ++qualified[1];
            for (std::size_t i = 0; i < face.count; ++i) {
                const auto a = vertex(face.read + i), b = vertex(face.read + (i + 1) % face.count);
                const auto found = edge_faces.find(key(a, b));
                if (found == edge_faces.end())
                    continue;
                const auto &f = found->second;
                // The second pass deliberately ignores this half-edge's visibility.
                NativeBuilderEdgeChain chain;
                if (f.first == f.last) {
                    chain.topology_type = 24;
                    chain.topology_ids = {static_cast<std::uint32_t>(a),
                                          static_cast<std::uint32_t>(b)};
                    ++boundary;
                } else {
                    chain.topology_type = 4;
                    chain.topology_ids = {static_cast<std::uint32_t>(f.first),
                                          static_cast<std::uint32_t>(f.last)};
                    ++shared;
                }
                chain.point_indices = {static_cast<std::int32_t>(a + 1),
                                       static_cast<std::int32_t>(b + 1)};
                if (f.occurrences > 2)
                    ++over_two;
                grow(5);
                mesh.edge_chains.push_back(std::move(chain));
                emitted.push_back({{"read_index", face.read + i},
                                   {"visible_occurrences", f.occurrences},
                                   {"first_visible_facet", f.first},
                                   {"last_visible_facet", f.last},
                                   {"emitted_from_hidden_half_edge", indices[face.read + i] < 0}});
                edge_faces.erase(found);
                --context.storage;
            }
        }
    }
    std::size_t nonzero = 0;
    for (auto i : indices) {
        context.work.charge(1);
        if (i)
            ++nonzero;
    }
    bool complete = edge_faces.empty();
    Json pass_reports = Json::array();
    for (unsigned p = 0; p < 2; ++p) {
        const auto &d = passes[p];
        complete = complete && visited[p] == nonzero && d.invalid_points == 0 &&
                   d.truncated_attributes == 0;
        pass_reports.push_back({{"visited_facets", d.visited},
                                {"qualified_facets", qualified[p]},
                                {"skipped_short_facets", skipped[p]},
                                {"unvisited_nonzero_indices", nonzero - visited[p]},
                                {"invalid_point_facets", d.invalid_points},
                                {"truncated_attribute_visits", d.truncated_attributes}});
    }
    report["passes"] = std::move(pass_reports);
    report["emitted"] = std::move(emitted);
    report["visible_half_edge_occurrences"] = visible;
    report["registered_edges"] = unique;
    report["unemitted_edges"] = edge_faces.size();
    report["shared_edge_records"] = shared;
    report["vertex_edge_records"] = boundary;
    report["edges_with_over_two_visible_occurrences"] = over_two;
    return finish(true, complete, "completed");
}
} // namespace p3d::swept_detail
