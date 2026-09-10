#include "internal.hpp"
namespace p3d {
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
    auto state = [&](unsigned i, std::size_t expected) -> std::string {
        if (!complete(i))
            return out["fields"][i]["status"] == "omitted" ? "absent" : "unavailable";
        if (out[names[i]].empty())
            return "absent";
        if (num_per_face > 1)
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
