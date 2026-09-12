// Included inside material_semantics.cpp's anonymous namespace so numeric
// source parsing stays consistent with the other source-value views.
Json material_numeric_reads(const Json &a, const Json &mode, const Json &applicability, bool layer,
                            const Json &type, const Json &branch) {
    struct Field {
        const char *name;
        const char *mode_zero_name;
        char kind;       // d: double, i: signed integer, 2/3: vector, c: color
        unsigned owners; // 1: common Map, 2: single Map, 4: texture, 8: gamma, 16: tint
    };
    static const Field fields[] = {
        {"map_link", "map_link", 'i', 1},
        {"pattern_off", "pattern_off", 'i', 1},
        {"pattern_proj_offset", "pattern_proj_angles", '3', 1},
        {"pattern_proj_angles", "pattern_proj_scale", '3', 1},
        {"pattern_proj_scale", "pattern_u_flip", '3', 1},
        {"origin_uv_pro_matrix_on", "origin_uv_pro_matrix_xa", 'i', 1},
        {"origin_uv_pro_matrix_xa", "origin_uv_pro_matrix_ya", '3', 1},
        {"origin_uv_pro_matrix_ya", "origin_uv_pro_matrix_za", '3', 1},
        {"origin_uv_pro_matrix_za", nullptr, '3', 1},
        {"pattern_weight", "pattern_weight", 'd', 1},
        {"bump_map_scale", "bump_map_scale", 'd', 1},
        {"pattern_mapping", "pattern_mapping", 'i', 6},
        {"pattern_scalemode", "pattern_scalemode", 'i', 6},
        {"texture_filter_type", "low_value", 'i', 6},
        {"enable_antialiasing", "texture_filter_type", 'i', 2},
        {"pattern_angle", "pattern_angle", 'd', 6},
        {"pattern_scale", "pattern_scale", '2', 6},
        {"scale_z", "lxo_noise_value1", 'd', 6},
        {"image_gamma", "reflect_color", 'd', 6},
        {"pattern_opacity", "layer_blend", 'd', 6},
        {"low_value", "high_value", 'd', 4},
        {"high_value", "antialias_strength", 'd', 4},
        {"antialias_strength", "minimum_spot", 'd', 6},
        {"minimum_spot", "use_cell_colors", 'd', 4},
        {"pattern_offset", "pattern_offset", '2', 6},
        {"offset_z", "scale_z", 'd', 6},
        {"layer_gamma", "layer_color", 'd', 8},
        {"layer_color", "layer_brightness", 'c', 16},
    };
    Json params = Json::object();
    const bool texture = type == 1 || type == 2 || type == 3 || type == 4 || type == 7;
    for (const auto &field : fields) {
        if (layer ? !(field.owners & (texture          ? 4
                                      : type == 0xf00c ? 8
                                      : type == 0xf00d ? 16
                                                       : 0))
                  : !(field.owners & 3))
            continue;
        const bool same = field.mode_zero_name && std::string(field.name) == field.mode_zero_name;
        const char *key = mode.is_null() && !same ? nullptr
                          : mode == 0             ? field.mode_zero_name
                                                  : field.name;
        Json p = {{"source_keys", Json::array()}, {"status", "unresolved"}, {"value", nullptr}};
        if (key) {
            if (field.kind == '2' || field.kind == '3' || field.kind == 'c')
                p = vector(a, key, field.kind == '2' ? "xy" : field.kind == '3' ? "xyz" : "rgb");
            else
                p = number(a, key, field.kind == 'i', field.kind == 'i');
        }
        p["source_attribute_base"] = key ? Json(key) : Json();
        p["source_key_status"] = key              ? "identified"
                                 : mode.is_null() ? "unresolved_reader_mode"
                                                  : "unavailable_mode_zero_table_entry";
        p["reader_value"] = nullptr;
        p["normalization"] = "source_value";
        auto gate = applicability;
        if (!layer && !(field.owners & 1) && branch != "single_provider")
            gate = reader_applicability(branch.is_null() ? "unresolved" : "skipped",
                                        "requires_single_provider_branch");
        if (!layer && std::string(field.name) == "pattern_weight" && type != 1)
            gate = reader_applicability(type.is_null() ? "unresolved" : "skipped",
                                        "requires_pattern_map_type");
        if (!layer && std::string(field.name) == "bump_map_scale" && type != 2)
            gate = reader_applicability(type.is_null() ? "unresolved" : "skipped",
                                        "requires_bump_map_type");
        p["reader_applicability"] = gate;
        p["write_status"] = gate["status"] == "read"      ? p["status"] == "decoded" ? "written"
                                                            : material_xml_numeric_failed(p)
                                                                ? "not_written"
                                                                : "unresolved"
                            : gate["status"] == "skipped" ? "not_written"
                                                          : "unresolved";
        if (p["write_status"] == "written") {
            p["reader_value"] = p["value"];
            const std::string name = field.name;
            if (name == "pattern_off" || name == "origin_uv_pro_matrix_on" ||
                name == "enable_antialiasing") {
                p["reader_value"] = (p["value"] != 0) != (name == "pattern_off");
                p["normalization"] = name == "pattern_off" ? "zero_is_enabled" : "nonzero_is_true";
            } else if (name == "pattern_proj_scale") {
                for (auto &v : p["reader_value"])
                    if (v == 0)
                        v = 1.0;
                p["normalization"] = "zero_components_to_one";
            }
        }
        params[field.name] = std::move(p);
    }
    if (params.contains("pattern_offset")) {
        auto &xy = params["pattern_offset"], &z = params["offset_z"];
        z["depends_on"] = "pattern_offset";
        if (xy["write_status"] == "written") {
            if (material_xml_numeric_failed(z)) {
                z["reader_value"] = 0.0;
                z["write_status"] = "written";
                z["normalization"] = "failed_z_after_xy_uses_zero";
            }
            if (z["write_status"] == "written") {
                xy["reader_value"].push_back(z["reader_value"]);
                xy["normalization"] = "xy_then_z_write_three_components";
            } else {
                xy["reader_value"] = nullptr;
                xy["write_status"] = "unresolved";
            }
        } else {
            z["reader_value"] = nullptr;
            z["write_status"] = xy["write_status"];
            z["normalization"] = "requires_xy_write";
        }
    }
    return {{"mode", mode},
            {"parameters", params},
            {"scope",
             layer ? "source_layer_reader_if_invoked" : "source_map_reader_before_link_resolution"},
            {"value_policy", "confirmed_explicit_writes_without_constructor_state"}};
}
