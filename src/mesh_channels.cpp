#include "internal.hpp"
namespace p3d {
namespace {
// This consumer uses zero delimiters even when the mesh declares a fixed width.
// Keep its view separate from the source polygons used for ordinary geometry.
struct FaceLayout {
    std::size_t faces = 0, corners = 0, trailing_corners = 0;
    bool matches = true, negative = false;
};
FaceLayout consumer_layout(const Json &indices, const Json &polygons) {
    FaceLayout result;
    std::size_t start = 0, source = 0;
    for (std::size_t end = 0; end < indices.size(); ++end) {
        if (indices[end] != 0)
            continue;
        if (end > start) {
            while (source < polygons.size() && polygons[source]["point_indices"].empty())
                ++source;
            if (source >= polygons.size() ||
                polygons[source]["point_indices"].size() != end - start)
                result.matches = false;
            for (std::size_t i = start; i < end; ++i) {
                result.negative |= indices[i].get<std::int64_t>() < 0;
                if (result.matches && indices[i] != polygons[source]["point_indices"][i - start])
                    result.matches = false;
            }
            ++source;
            ++result.faces;
            result.corners += end - start;
        }
        start = end + 1;
    }
    while (source < polygons.size() && polygons[source]["point_indices"].empty())
        ++source;
    result.trailing_corners = indices.size() - start;
    result.matches &= source == polygons.size() && result.trailing_corners == 0;
    return result;
}
Json native_mesh_routing(const Json &channels, const FaceLayout &layout, const Json &polygons) {
    const auto &materials = channels["face_material_ids"];
    const auto &groups = channels["face_smoothing_groups"];
    Json out = {{"profile", "bimbase_2025_triangulate2"},
                {"evaluation", "input_routing_only"},
                {"status", "mapped"},
                {"polygon_layout", "nonempty_zero_terminated"},
                {"polygon_count", layout.faces},
                {"corner_count", layout.corners},
                {"ignored_unterminated_corner_count", layout.trailing_corners},
                {"source_polygon_layout_matches", layout.matches},
                {"material_copy_count_source", "face_smoothing_groups_count"},
                {"requested_material_copy_count", groups.size()},
                {"polygons", Json::array()}};
    // Model only bounded source reads. The native routine does not check the
    // material list length before copying the smoothing-group count of IDs.
    if (channels["status"] != "decoded")
        out["status"] = "unavailable_extension";
    else if (materials.empty())
        out["status"] = "no_face_materials";
    else if (!layout.faces)
        out["status"] = "no_terminated_faces";
    else if (layout.negative)
        out["status"] = "unsafe_signed_point_lookup";
    else if (channels["face_uv_points"].size() != layout.corners)
        out["status"] = "face_uv_count_mismatch";
    else if (groups.size() > materials.size())
        out["status"] = "unsafe_material_copy";
    if (out["status"] != "mapped")
        return out;
    bool smoothing = false;
    if (groups.size() == layout.faces)
        for (const auto &group : groups)
            smoothing |= group.get<std::int32_t>() > 0;
    out["uv_source"] = "face_uv_points";
    out["explicit_normals_used"] = false;
    out["normal_mode"] = smoothing ? "smoothing_groups" : "flat_triangles";
    out["normal_rule"] = {{"group_comparison", "signed_integer_equality"},
                          {"zero_group", "flat_triangles"},
                          {"weighting", "unit_triangle_normals"},
                          {"normalize_sum", true},
                          {"position_comparison", "componentwise_tolerance"},
                          {"position_tolerance", 1e-7}};
    out["copied_material_count"] = groups.size();
    out["unused_source_material_count"] = materials.size() - groups.size();
    out["discarded_copied_material_count"] =
        groups.size() > layout.faces ? groups.size() - layout.faces : 0;
    out["zero_padded_material_count"] =
        layout.faces > groups.size() ? layout.faces - groups.size() : 0;
    // The consumer's polygons remain in their own order if they do not match
    // fixed-width source polygons; never guess a triangle correspondence.
    std::size_t source = 0;
    for (std::size_t face = 0; face < layout.faces; ++face) {
        Json source_polygon = nullptr;
        if (layout.matches) {
            while (polygons[source]["point_indices"].empty())
                ++source;
            source_polygon = source++;
        }
        const bool copied = face < groups.size();
        out["polygons"].push_back(
            {{"source_polygon", source_polygon},
             {"material_id", copied ? materials[face] : Json(std::uint64_t(0))},
             {"source_material_index", copied ? Json(face) : Json()},
             {"normal_group", smoothing && groups[face] != 0 ? groups[face] : Json()}});
    }
    return out;
}
} // namespace
Json decode_mesh_channels(const Bytes &b, const Json &arrays, const Json &polygons,
                          std::uint32_t num_per_face) {
    Json out = {{"status", "decoded"},
                {"reader_profile", "bimbase_2025_command25"},
                {"fields", Json::array()},
                {"float_colors", Json::array()},
                {"color_indices", Json::array()},
                {"illumination_name", nullptr},
                {"illumination_name_utf16_hex", ""},
                {"illumination_name_status", "absent"},
                {"face_uv_points", Json::array()},
                {"face_smoothing_groups", Json::array()},
                {"face_material_ids", Json::array()}};
    Reader r(b);
    const char *names[] = {"float_colors",   "color_indices",         "illumination_name",
                           "face_uv_points", "face_smoothing_groups", "face_material_ids"};
    const unsigned strides[] = {12, 4, 2, 16, 4, 8};
    bool invalid = false;
    for (unsigned field = 0; field < 6; ++field) {
        Json entry = {{"name", names[field]},
                      {"offset", r.p},
                      {"count", nullptr},
                      {"stride", strides[field]}};
        if (invalid) {
            entry["status"] = "not_read_after_invalid_field";
        } else if (field >= 3 && !r.left()) {
            entry["status"] = "omitted";
        } else if (r.left() < 4) {
            entry["status"] = "truncated_count";
            invalid = true;
        } else {
            const auto count = r.u32();
            entry["count"] = count;
            entry["payload_offset"] = r.p;
            if (count > r.left() / strides[field]) {
                entry["status"] = "truncated_payload";
                invalid = true;
            } else {
                entry["status"] = "decoded";
                if (field == 2) {
                    auto bytes = r.take(std::size_t(count) * 2);
                    out["illumination_name_utf16_hex"] = hex(bytes);
                    if (count) {
                        std::size_t end = 0;
                        while (end < bytes.size() && (bytes[end] || bytes[end + 1]))
                            end += 2;
                        out["illumination_name_status"] =
                            end < bytes.size() ? "decoded" : "unterminated";
                        try {
                            auto value = utf16(slice(bytes, 0, end));
                            Json(value).dump();
                            out["illumination_name"] = value;
                        } catch (const std::exception &) {
                            out["illumination_name_status"] = "invalid_unicode";
                        }
                    }
                } else {
                    auto &values = out[names[field]];
                    for (std::uint32_t i = 0; i < count; ++i) {
                        if (field == 0) {
                            Json rgb = Json::array();
                            for (unsigned c = 0; c < 3; ++c)
                                rgb.push_back(r.get<float>());
                            values.push_back(std::move(rgb));
                        } else if (field == 3)
                            values.push_back(r.doubles(2));
                        else if (field == 5)
                            values.push_back(r.u64());
                        else
                            values.push_back(r.i32());
                    }
                }
            }
        }
        out["fields"].push_back(std::move(entry));
    }
    out["consumed_bytes"] = r.p;
    out["remaining_hex"] = hex(r.take(r.left()));
    if (invalid)
        out["status"] = "invalid";
    else if (!out["remaining_hex"].get<std::string>().empty() ||
             out["illumination_name_status"] == "unterminated" ||
             out["illumination_name_status"] == "invalid_unicode")
        out["status"] = "partial";
    out["raw_hex"] = hex(b);
    Json bindings = {{"scope", "source_command_polygons"},
                     {"color_index_source", "param_index_array"},
                     {"serialized_color_indices_used_by_reader", false},
                     {"polygons", Json::array()}};
    std::size_t face_count = 0, corner_count = 0;
    for (const auto &poly : polygons) {
        const auto count = poly["point_indices"].size();
        if (count) {
            ++face_count;
            corner_count += count;
        }
    }
    auto complete = [&](unsigned i) { return out["fields"][i]["status"] == "decoded"; };
    const auto layout = consumer_layout(arrays[0], polygons);
    out["native_triangulation"] = native_mesh_routing(out, layout, polygons);
    auto state = [&](unsigned i, std::size_t expected) -> std::string {
        if (!complete(i))
            return out["fields"][i]["status"] == "omitted" ? "absent" : "unavailable";
        if (out[names[i]].empty())
            return "absent";
        if (num_per_face > 1 && !layout.matches)
            return "not_evaluated_for_fixed_width";
        return out[names[i]].size() == expected ? "mapped" : "count_mismatch";
    };
    bindings["face_uv_status"] = state(3, corner_count);
    bindings["smoothing_group_status"] = state(4, face_count);
    bindings["material_id_status"] = state(5, face_count);
    bindings["nonempty_polygon_count"] = face_count;
    bindings["polygon_corner_count"] = corner_count;
    // The shipped command reader skips the serialized color index array and
    // passes the parameter index pointer when a FloatRgb pool exists.
    const bool has_colors = complete(0) && !out["float_colors"].empty();
    bindings["color_status"] = has_colors ? (arrays[2].empty() ? "unindexed" : "mapped") : "absent";
    if (!(has_colors && !arrays[2].empty()) && bindings["face_uv_status"] != "mapped" &&
        bindings["smoothing_group_status"] != "mapped" &&
        bindings["material_id_status"] != "mapped") {
        out["bindings"] = std::move(bindings);
        return out;
    }
    std::size_t face = 0, corner = 0;
    for (const auto &poly : polygons) {
        Json binding = {{"color_indices", nullptr},
                        {"face_uv_point_indices", nullptr},
                        {"material_id", nullptr},
                        {"smoothing_group", nullptr}};
        const auto count = poly["point_indices"].size();
        if (count) {
            if (has_colors && !poly["uv_indices"].empty()) {
                Json indices = Json::array();
                bool valid = true;
                for (const auto &index : poly["uv_indices"]) {
                    const auto absolute = std::llabs(index.get<std::int64_t>());
                    if (!absolute || std::uint64_t(absolute) > out["float_colors"].size())
                        valid = false;
                    indices.push_back(absolute - 1);
                }
                if (valid)
                    binding["color_indices"] = std::move(indices);
                else
                    bindings["color_status"] = "invalid_reader_indices";
            }
            if (bindings["face_uv_status"] == "mapped") {
                Json indices = Json::array();
                for (std::size_t i = 0; i < count; ++i)
                    indices.push_back(corner + i);
                binding["face_uv_point_indices"] = std::move(indices);
            }
            if (bindings["material_id_status"] == "mapped")
                binding["material_id"] = out["face_material_ids"][face];
            if (bindings["smoothing_group_status"] == "mapped")
                binding["smoothing_group"] = out["face_smoothing_groups"][face];
            ++face;
            corner += count;
        }
        bindings["polygons"].push_back(std::move(binding));
    }
    out["bindings"] = std::move(bindings);
    return out;
}
bool has_mesh_channels(const Json &channels) {
    if (channels.value("status", "invalid") != "decoded")
        return true;
    for (const auto &field : channels["fields"])
        if (field["count"].is_number() && field["count"] != 0)
            return true;
    return false;
}
Json mesh_triangle_channels(const Json &channels,
                            const std::vector<std::optional<std::uint32_t>> &polygons,
                            const std::vector<Triangle> &corners) {
    require(polygons.size() == corners.size(), "mesh source corner count");
    Json triangles = Json::array();
    std::map<std::size_t, const Json *> native_polygons;
    if (channels.contains("native_triangulation"))
        for (const auto &poly : channels["native_triangulation"]["polygons"])
            if (!poly["source_polygon"].is_null())
                native_polygons.emplace(poly["source_polygon"].get<std::size_t>(), &poly);
    for (std::size_t i = 0; i < corners.size(); ++i) {
        require(polygons[i].has_value(), "mesh source polygon missing");
        const auto polygon = *polygons[i];
        Json binding = {{"color_indices", nullptr},
                        {"face_uv_point_indices", nullptr},
                        {"material_id", nullptr},
                        {"smoothing_group", nullptr}};
        if (!channels["bindings"]["polygons"].empty())
            binding = channels["bindings"]["polygons"].at(polygon);
        binding["source_polygon"] = polygon;
        binding["source_corners"] = corners[i];
        auto native = native_polygons.find(polygon);
        if (native != native_polygons.end())
            binding["native_triangulation"] = *native->second;
        for (const auto *name : {"color_indices", "face_uv_point_indices"}) {
            auto values = binding[name];
            if (!values.is_null()) {
                binding[name] = Json::array();
                for (auto corner : corners[i])
                    binding[name].push_back(values.at(corner));
            }
        }
        triangles.push_back(std::move(binding));
    }
    return {{"source", channels}, {"triangles", std::move(triangles)}};
}
void reverse_mesh_channel_corners(Json &channels) {
    for (auto &triangle : channels["triangles"])
        for (const auto *name : {"source_corners", "color_indices", "face_uv_point_indices"})
            if (!triangle[name].is_null())
                std::swap(triangle[name][1], triangle[name][2]);
}
} // namespace p3d
