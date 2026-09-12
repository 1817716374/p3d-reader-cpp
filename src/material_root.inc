// The parameter constructor runs before the XML root reader. These defaults
// are native object state, not a rendering conversion or file-format default.
Json material_root_input(const Json &attributes, const Json &mode) {
    struct Field {
        const char *name;
        const char *legacy_name;
        char kind; // c: RGB, d: double, i: int32, b: int32 converted to bool
        double initial;
    };
    static const Field fields[] = {
        {"color", "color", 'c', 1},
        {"specular_color", "specular_color", 'c', 1},
        {"transparent_color", "translucent_color", 'c', 1},
        {"translucent_color", "anisotropy", 'c', 1},
        {"glow_color", "glow_color_map", 'c', 1},
        {"reflect_color", "reflect_color_map", 'c', 1},
        {"exit_color", "back_face_culling", 'c', 0},
        {"ambient", "ambient", 'd', .5},
        {"finish", "finish", 'd', .05},
        {"reflect", "reflect", 'd', 0},
        {"transmit", "transmit", 'd', 0},
        {"diffuse", "diffuse", 'd', .5},
        {"specular", "specular", 'd', .05},
        {"refract", "refract", 'd', 1},
        {"thickness", "thickness", 'd', 0},
        {"glow", "glow", 'd', 0},
        {"translucency", "translucency", 'd', 0},
        {"reflection_rays", "displacement_distance", 'i', 64},
        {"refraction_rays", "shader_type", 'i', 64},
        {"subsurface_samples", "value_map_invert", 'i', 64},
        {"blur_reflections", "reflection_rays", 'b', 0},
        {"blur_refractions", "refraction_rays", 'b', 0},
        {"use_fur", "displacement_map", 'b', 0},
        {"cut_section_use_material", "cut_section_material_name", 'b', 0},
        {"back_face_culling", "enable_antialiasing", 'i', 0},
        {"displacement_distance", "linked_to_lxp", 'd', 0},
        {"dispersion", "clearcoat", 'd', 0},
        {"clearcoat", "blur_refractions", 'd', 0},
        {"anisotropy", "front_weighting", 'd', 0},
        {"front_weighting", "scattering_distance", 'd', 50},
        {"scattering_distance", "subsurface_samples", 'd', 0},
        {"reflect_fresnel", "dissolve", 'd', .05},
        {"dissolve", "absorption_distance", 'd', 0},
        {"absorption_distance", "refraction_roughness", 'd', 0},
        {"refraction_roughness", "glow_color", 'd', .05},
        {"diffuse_roughness", "rounded_edge_width", 'd', 0},
        {"rounded_edge_width", "shader_flag_visible_to_occlusion_rays", 'd', 0},
    };
    Json out = {{"reader_profile", "bimbase_2025_material_root_parameter_input"},
                {"scope", "constructor_and_root_numeric_attributes_before_version_conversion"},
                {"mode", mode},
                {"status", "resolved"},
                {"parameters", Json::object()}};
    auto flags = number(attributes, "Flags", true);
    flags["constructor_value"] = 0x182001u;
    flags["read_status"] = flags["status"] == "decoded"      ? "written"
                           : flags["status"] == "unresolved" ? "unresolved"
                                                             : "not_written";
    if (flags["read_status"] == "not_written")
        flags["value"] = flags["constructor_value"];
    if (flags["read_status"] == "unresolved")
        out["status"] = "partial";
    out["flags"] = std::move(flags);
    for (const auto &field : fields) {
        const bool same = std::string(field.name) == field.legacy_name;
        const char *key = mode.is_null() && !same ? nullptr
                          : mode == 0             ? field.legacy_name
                                                  : field.name;
        Json initial = field.kind == 'c'
                           ? Json::array({field.initial, field.initial, field.initial})
                       : field.kind == 'b' ? Json(bool(field.initial))
                       : field.kind == 'i' ? Json(std::int32_t(field.initial))
                                           : Json(field.initial);
        Json value = {{"constructor_value", initial},
                      {"value", nullptr},
                      {"source_key", key ? Json(key) : Json()},
                      {"read_status", "unresolved"}};
        if (key) {
            const bool integer = field.kind == 'i' || field.kind == 'b';
            auto read = field.kind == 'c' ? vector(attributes, key, "rgb")
                                          : number(attributes, key, integer, integer);
            value["source_read"] = read;
            if (material_xml_numeric_failed(read)) {
                value["value"] = initial;
                value["read_status"] = "not_written";
            } else if (read["status"] == "decoded") {
                value["value"] = field.kind == 'b' ? Json(read["value"] != 0) : read["value"];
                value["read_status"] = "written";
            }
        }
        if (value["read_status"] == "unresolved")
            out["status"] = "partial";
        out["parameters"][field.name] = std::move(value);
    }
    return out;
}
