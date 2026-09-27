#include "native_builder_polyface.hpp"
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
            // Finite source keys can include native DBL_MAX disconnects.
            // The original comparison retains IEEE overflow in its tolerance
            // and differences. Rejecting these intermediate infinities blocks
            // multi-ring cap insertion; dropping the marker changes tree order.
            double sum = std::abs(a[0]) + std::abs(b[0]);
            sum += std::abs(a[1]);
            sum += std::abs(b[1]);
            sum += std::abs(a[2]);
            sum += std::abs(b[2]);
            tol += relative * sum;
        }
        for (int k = 2; k >= 0; --k) {
            const double delta = b[k] - a[k];
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
void append_coordinates(NativeBuilderCoordinates &out, CoordinateMap &points, CoordinateMap &params,
                        CoordinateMap &normals, const NativeBuilderCoordinateBatch &batch,
                        bool ensure_default_parameter, curve_detail::BezierWork work) {
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
    if (ensure_default_parameter && batch.parameters.empty() && out.parameters.empty()) {
        const auto index = params.insert({0, 0, batch.parameter_scope});
        out.parameters.push_back({0, 0});
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
    for (const auto &batch : batches)
        append_coordinates(out, points, params, normals, batch, false, work);
    out.report = {{"operation", "native_builder_coordinate_insertion"},
                  {"input_coordinates", input_count},
                  {"batches", batches.size()},
                  {"points", points.report()},
                  {"parameters", params.report()},
                  {"normals", normals.report()},
                  {"facet_assembly_performed", false}};
    return out;
}

NativeBuilderPolyfaceOutput
assemble_native_builder_polyfaces(const std::vector<NativeBuilderPolyface> &sources,
                                  const NativeBuilderPolyfaceOptions &options, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    require(sources.size() <= budget.max_control_points, "native builder source count budget");
    std::size_t input_count = 0, output_count = 0;
    auto input = [&](std::size_t n) {
        require(n <= budget.max_control_points - input_count,
                "native builder input storage budget");
        input_count += n;
        work.charge(n);
    };
    auto output = [&](std::size_t n) {
        require(n <= budget.max_control_points - output_count,
                "native builder output storage budget");
        output_count += n;
        work.charge(n);
    };
    for (const auto &source : sources) {
        input(source.coordinates.points.size());
        input(source.coordinates.parameters.size());
        input(source.coordinates.normals.size());
        for (const auto &indices : source.indices.indices)
            input(indices.size());
        input(source.face_data.size());
        input(source.edge_chains.size());
        for (const auto &chain : source.edge_chains) {
            input(chain.topology_ids.size());
            input(chain.point_indices.size());
        }
    }
    CoordinateMap points(options.coordinates.point_absolute_tolerance,
                         options.coordinates.point_relative_tolerance, work),
        params(options.coordinates.parameter_absolute_tolerance,
               options.coordinates.parameter_relative_tolerance, work),
        normals(1e-14, 1e-6, work);
    NativeBuilderPolyfaceOutput out;
    const bool initial_parameters =
        options.attributes_enabled_at_construction && options.parameters_required;
    const bool initial_normals =
        options.attributes_enabled_at_construction && options.normals_required;
    out.indices.active = {true, initial_parameters, initial_normals, false, initial_parameters};
    out.parameter_pool_active = initial_parameters;
    out.normal_pool_active = initial_normals;
    out.native_succeeded = true;
    std::size_t rejected = 0, discarded_face_indices = 0, ignored_colors = 0;
    std::size_t suppressed_attributes = 0, missing_edge_keys = 0;
    std::size_t unrepresented_attributes = 0;
    std::array<std::size_t, polyface_channel_count> missing_keys{};
    Json reports = Json::array();
    auto append_index = [&](std::size_t channel, std::int32_t value) {
        output(1);
        out.indices.active[channel] = true;
        out.indices.indices[channel].push_back(value);
    };
    auto mapped = [&](const std::vector<std::size_t> &map, std::int32_t value, bool absolute,
                      std::size_t &missing) {
        // The native map uses operator[]: unknown keys acquire mapped value 0.
        // Compute in 64 bits to avoid C++ abs(INT_MIN) and subtraction overflow.
        const auto wide = static_cast<std::int64_t>(value);
        const auto key = (absolute && wide < 0 ? -wide : wide) - 1;
        std::size_t target = 0;
        if (key < 0 || static_cast<std::uint64_t>(key) >= map.size())
            ++missing;
        else
            target = map[static_cast<std::size_t>(key)];
        require(target < static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()),
                "native builder signed index capacity");
        return static_cast<std::int32_t>(target + 1);
    };
    for (std::size_t source_index = 0; source_index < sources.size(); ++source_index) {
        work.charge(1);
        const auto &source = sources[source_index];
        Json report{{"source", source_index}};
        if (out.num_per_face != source.num_per_face) {
            if (out.coordinates.points.empty() && out.indices.indices[point_channel].empty())
                out.num_per_face = source.num_per_face;
            else {
                ++rejected;
                out.native_succeeded = false;
                report["native_succeeded"] = false;
                report["reason"] = "incompatible_num_per_face";
                reports.push_back(std::move(report));
                continue;
            }
        }
        out.two_sided =
            out.coordinates.points.empty() ? source.two_sided : out.two_sided || source.two_sided;
        const auto &src = source.indices.indices;
        const auto &point_indices = src[point_channel];
        const auto *normal_indices =
            out.indices.active[normal_channel] && !src[normal_channel].empty()
                ? &src[normal_channel]
                : nullptr;
        const auto *parameter_indices =
            out.indices.active[parameter_channel] && !src[parameter_channel].empty()
                ? &src[parameter_channel]
                : nullptr;
        const auto *face_indices = !src[face_channel].empty() ? &src[face_channel] : nullptr;
        bool normal_fallback = false, parameter_fallback = false;
        if (!normal_indices &&
            source.coordinates.points.size() == source.coordinates.normals.size() &&
            !point_indices.empty()) {
            normal_indices = &point_indices;
            normal_fallback = true;
        }
        if (!parameter_indices &&
            source.coordinates.points.size() == source.coordinates.parameters.size() &&
            !point_indices.empty()) {
            parameter_indices = &point_indices;
            parameter_fallback = true;
        }
        if (face_indices && face_indices->size() != point_indices.size()) {
            discarded_face_indices += face_indices->size();
            report["discarded_face_indices"] = face_indices->size();
            face_indices = nullptr;
        }
        auto suppressed = [&](std::size_t channel, const std::vector<std::int32_t> *selected) {
            const auto &original = src[channel];
            if (original.empty() || selected == &original)
                return;
            suppressed_attributes += original.size();
            // A disabled initial channel may use the original point-index
            // fallback. An equivalent independent index array loses no data;
            // report its bypass, but do not call the resulting values missing.
            bool equivalent = selected && selected->size() == original.size();
            if (equivalent)
                for (std::size_t i = 0; i < original.size(); ++i) {
                    work.charge(1);
                    const auto a = std::int64_t(original[i]), b = std::int64_t((*selected)[i]);
                    if ((a < 0 ? -a : a) != (b < 0 ? -b : b)) {
                        equivalent = false;
                        break;
                    }
                }
            if (!equivalent)
                unrepresented_attributes += original.size();
        };
        suppressed(normal_channel, normal_indices);
        suppressed(parameter_channel, parameter_indices);
        ignored_colors += src[color_channel].size();
        report["normal_index_source"] = normal_fallback  ? "point_fallback"
                                        : normal_indices ? "explicit"
                                                         : "absent";
        report["parameter_index_source"] = parameter_fallback  ? "point_fallback"
                                           : parameter_indices ? "explicit"
                                                               : "absent";
        const bool default_parameter = out.parameter_pool_active &&
                                       source.coordinates.parameters.empty() &&
                                       out.coordinates.parameters.empty();
        const auto old_coordinate_count = out.coordinates.points.size() +
                                          out.coordinates.normals.size() +
                                          out.coordinates.parameters.size();
        append_coordinates(out.coordinates, points, params, normals, source.coordinates,
                           default_parameter, work);
        output(out.coordinates.points.size() + out.coordinates.normals.size() +
               out.coordinates.parameters.size() - old_coordinate_count);
        const auto &remap = out.coordinates.batches.back();
        out.parameter_pool_active = out.parameter_pool_active || !remap.parameters.empty();
        out.normal_pool_active = out.normal_pool_active || !remap.normals.empty();
        report["coordinate_batch"] = out.coordinates.batches.size() - 1;
        report["default_parameter_inserted"] = default_parameter;
        std::vector<std::size_t> face_remap;
        face_remap.reserve(source.face_data.size());
        output(source.face_data.size());
        for (const auto &data : source.face_data) {
            face_remap.push_back(out.face_data.size());
            out.face_data.push_back(data);
        }
        for (std::size_t i = 0; i < point_indices.size(); ++i) {
            work.charge(1);
            const auto point = point_indices[i];
            if (point == 0) {
                // Native stopActiveIndexVectors skips empty buffers. Only the
                // face channel suppresses a repeated terminator; the other
                // active channels append another zero even after a zero.
                for (std::size_t channel = 0; channel < polyface_channel_count; ++channel)
                    if (out.indices.active[channel] && !out.indices.indices[channel].empty() &&
                        (channel != face_channel || out.indices.indices[channel].back() != 0))
                        append_index(channel, 0);
                continue;
            }
            const auto pi = mapped(remap.points, point, true, missing_keys[point_channel]);
            append_index(point_channel, point < 0 ? -pi : pi);
            if (normal_indices) {
                require(i < normal_indices->size(),
                        "native builder normal index read outside source");
                append_index(normal_channel, mapped(remap.normals, (*normal_indices)[i], true,
                                                    missing_keys[normal_channel]));
            }
            if (parameter_indices) {
                require(i < parameter_indices->size(),
                        "native builder UV index read outside source");
                append_index(parameter_channel, mapped(remap.parameters, (*parameter_indices)[i],
                                                       true, missing_keys[parameter_channel]));
            } else if (out.parameter_pool_active)
                append_index(parameter_channel, 1);
            if (face_indices)
                append_index(face_channel, mapped(face_remap, (*face_indices)[i], false,
                                                  missing_keys[face_channel]));
        }
        output(source.edge_chains.size());
        for (const auto &chain : source.edge_chains) {
            output(chain.topology_ids.size());
            output(chain.point_indices.size());
            NativeBuilderEdgeChain copy{chain.topology_type, chain.topology_ids, {}};
            copy.point_indices.reserve(chain.point_indices.size());
            for (const auto index : chain.point_indices)
                copy.point_indices.push_back(mapped(remap.points, index, false, missing_edge_keys));
            out.edge_chains.push_back(std::move(copy));
        }
        report["native_succeeded"] = true;
        reports.push_back(std::move(report));
    }
    bool aligned = true, bounded = true;
    const auto point_index_count = out.indices.indices[point_channel].size();
    const std::array<std::size_t, polyface_channel_count> pool_sizes{
        out.coordinates.points.size(), out.coordinates.parameters.size(),
        out.coordinates.normals.size(), 0, out.face_data.size()};
    for (std::size_t channel = 0; channel < polyface_channel_count; ++channel) {
        if (!out.indices.active[channel])
            continue;
        const auto &indices = out.indices.indices[channel];
        aligned = aligned && indices.size() == point_index_count;
        for (const auto index : indices) {
            work.charge(1);
            const auto wide = static_cast<std::int64_t>(index);
            const auto magnitude = static_cast<std::uint64_t>(wide < 0 ? -wide : wide);
            bounded = bounded && magnitude <= pool_sizes[channel];
        }
    }
    bool all_keys_present = missing_edge_keys == 0;
    for (const auto count : missing_keys)
        all_keys_present = all_keys_present && count == 0;
    out.complete = out.native_succeeded && discarded_face_indices == 0 && ignored_colors == 0 &&
                   unrepresented_attributes == 0 && aligned && bounded && all_keys_present;
    out.coordinates.report = {{"operation", "native_builder_coordinate_insertion"},
                              {"batches", out.coordinates.batches.size()},
                              {"points", points.report()},
                              {"parameters", params.report()},
                              {"normals", normals.report()}};
    out.report = {{"operation", "native_add_polyface_matched"},
                  {"native_succeeded", out.native_succeeded},
                  {"complete", out.complete},
                  {"sources", std::move(reports)},
                  {"rejected_sources", rejected},
                  {"discarded_face_indices", discarded_face_indices},
                  {"ignored_color_indices", ignored_colors},
                  {"suppressed_attribute_indices", suppressed_attributes},
                  {"unrepresented_attribute_indices", unrepresented_attributes},
                  {"missing_remap_keys", missing_keys},
                  {"missing_edge_remap_keys", missing_edge_keys},
                  {"active_index_channels_aligned", aligned},
                  {"indices_in_range", bounded},
                  {"attribute_preparation_performed", false}};
    return out;
}
} // namespace p3d::swept_detail
