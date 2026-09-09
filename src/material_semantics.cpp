#include "internal.hpp"
#include <locale>
#include <sstream>
namespace p3d {
namespace {
// Interpret only explicitly stored attributes. Native constructors and layered
// texture providers can supply further values; those are not file defaults.
Json number(const Json &a, const std::string &key, bool integer = false,
            bool signed_integer = false) {
    Json out = {{"source_keys", Json::array({key})}, {"status", "missing"}, {"value", nullptr}};
    if (!a.contains(key))
        return out;
    out["status"] = "invalid";
    if (!a[key].is_string())
        return out;
    const auto s = a[key].get<std::string>();
    if (integer) {
        auto start = s.find_first_not_of(" \t\r\n"), end = s.find_last_not_of(" \t\r\n");
        if (start == std::string::npos)
            return out;
        bool negative = signed_integer && s[start] == '-';
        if (s[start] == '+' || negative)
            ++start;
        if (start > end)
            return out;
        std::uint64_t value = 0;
        for (auto i = start; i <= end; ++i) {
            if (s[i] < '0' || s[i] > '9')
                return out;
            value = value * 10 + unsigned(s[i] - '0');
            if (value >
                (signed_integer ? std::uint64_t(INT32_MAX) + unsigned(negative) : UINT32_MAX))
                return out;
        }
        out["value"] = signed_integer ? Json(negative ? -std::int64_t(value) : std::int64_t(value))
                                      : Json(value);
    } else {
        std::istringstream input(s);
        input.imbue(std::locale::classic());
        double value;
        if (!(input >> value) || !std::isfinite(value))
            return out;
        input >> std::ws;
        if (!input.eof())
            return out;
        out["value"] = value;
    }
    out["status"] = "decoded";
    return out;
}
Json vector(const Json &a, const std::string &prefix, const char *axes) {
    Json components = Json::array(), keys = Json::array(), values = Json::array();
    bool invalid = false;
    unsigned present = 0;
    for (const char *axis = axes; *axis; ++axis) {
        auto key = prefix + "." + *axis;
        auto c = number(a, key);
        present += a.contains(key);
        invalid |= c["status"] == "invalid";
        keys.push_back(key);
        values.push_back(c["value"]);
        components.push_back(std::move(c));
    }
    const char *status = invalid                    ? "invalid"
                         : present == values.size() ? "decoded"
                         : present                  ? "partial"
                                                    : "missing";
    return {{"source_keys", keys},
            {"status", status},
            {"value", std::string(status) == "decoded" ? values : Json()},
            {"components", components}};
}
Json boolean(const Json &a, const std::string &key, bool invert = false, bool low_bit = false) {
    auto out = number(a, key, true, !low_bit);
    if (out["status"] == "decoded") {
        auto value = out["value"].get<std::int64_t>();
        out["source_value"] = value;
        out["value"] = (low_bit ? (value & 1) != 0 : value != 0) != invert;
    }
    return out;
}
} // namespace
Json material_parameter_semantics(const Json &a) {
    auto flags = number(a, "Flags", true);
    Json parameters = Json::object();
    std::uint32_t known = 0;
    auto add = [&](const char *name, const char *key, int bit, bool rgb = false) {
        auto item = rgb ? vector(a, key, "rgb") : number(a, key);
        if (bit >= 0) {
            known |= std::uint32_t(1) << bit;
            item["enabled"] = flags["status"] == "decoded"
                                  ? Json(bool(flags["value"].get<std::uint32_t>() & (1u << bit)))
                                  : Json();
            item["enabled_source"] = {
                {"attribute", "Flags"}, {"bit", bit}, {"status", flags["status"]}};
        } else {
            item["enabled"] = true;
            item["enabled_source"] = {{"status", "native_unconditional"}};
        }
        parameters[name] = std::move(item);
    };
    add("color", "color", 2, true);
    add("transparency", "transmit", 6);
    add("specular_color", "specular_color", 3, true);
    add("specular_factor", "specular", 9);
    add("glow_color", "glow_color", 21, true);
    add("glow_factor", "glow", -1);
    add("ambient_factor", "ambient", -1);
    add("diffuse_factor", "diffuse", 7);
    add("roughness_factor", "finish", 4);
    add("reflect_factor", "reflect", 5);
    add("refract_factor", "refract", 8);
    flags["identified_mask"] = known;
    flags["unassigned_bits"] =
        flags["status"] == "decoded" ? Json(flags["value"].get<std::uint32_t>() & ~known) : Json();
    return {{"flags", flags},
            {"parameters", parameters},
            {"value_policy", "explicit_source_values_without_defaults_or_renderer_conversion"}};
}
Json material_map_semantics(const Json &a) {
    auto type = number(a, "Type", true);
    type["role"] = nullptr;
    if (type["status"] == "decoded") {
        static const std::map<unsigned, const char *> roles = {
            {0, "none"},        {1, "pattern"},        {2, "bump"},          {31, "pbr_albedo"},
            {32, "pbr_normal"}, {33, "pbr_roughness"}, {34, "pbr_metallic"}, {35, "pbr_ao"}};
        auto it = roles.find(type["value"].get<unsigned>());
        type["role_status"] = it == roles.end() ? "unknown_value" : "identified";
        if (it != roles.end())
            type["role"] = it->second;
    } else
        type["role_status"] = type["status"];
    auto mode = number(a, "pattern_mapping", true, true);
    mode["name"] = nullptr;
    if (mode["status"] == "decoded") {
        static const std::map<int, const char *> modes = {{0, "parametric"}, {1, "elevation_drape"},
                                                          {2, "planar"},     {4, "cubic"},
                                                          {5, "spherical"},  {6, "cylindrical"}};
        auto it = modes.find(mode["value"].get<int>());
        mode["name_status"] = it == modes.end() ? "unknown_value" : "identified";
        if (it != modes.end())
            mode["name"] = it->second;
    } else
        mode["name_status"] = mode["status"];
    auto unit = number(a, "pattern_scalemode", true, true);
    unit["sdk_size_mode"] =
        unit["status"] == "decoded" ? Json(unit["value"] == 0 ? "relative" : "absolute") : Json();
    auto rows = Json::array();
    Json matrix = Json::array();
    bool complete = true;
    for (auto axis : {"xa", "ya", "za"}) {
        auto row = vector(a, std::string("origin_uv_pro_matrix_") + axis, "xyz");
        complete &= row["status"] == "decoded";
        matrix.push_back(row["value"]);
        rows.push_back(std::move(row));
    }
    Json out = {{"type", type},
                {"map_link", number(a, "map_link", true, true)},
                {"enabled", boolean(a, "pattern_off", true)},
                {"mapping_mode", mode},
                {"mapping_unit", unit},
                {"uv_scale", vector(a, "pattern_scale", "xy")},
                {"uv_offset", vector(a, "pattern_offset", "xy")},
                {"rotation_degrees", number(a, "pattern_angle")},
                {"scale_z", number(a, "scale_z")},
                {"offset_z", number(a, "offset_z")},
                {"projection_frame",
                 {{"offset", vector(a, "pattern_proj_offset", "xyz")},
                  {"angles", vector(a, "pattern_proj_angles", "xyz")},
                  {"scale", vector(a, "pattern_proj_scale", "xyz")}}},
                {"projection",
                 {{"enabled", boolean(a, "origin_uv_pro_matrix_on")},
                  {"source_rows", rows},
                  {"matrix_rows", complete ? matrix : Json()}}},
                {"value_policy", "explicit_source_values_without_defaults_or_uv_evaluation"}};
    // PatternFlags bit zero is the image-alpha selector. BumpFlags belongs
    // to a different map setting; the native PatternFlags reader skips PBR codes.
    if (type["value"] == 1) {
        out["use_image_alpha_channel"] = boolean(a, "PatternFlags", false, true);
        out["pattern_weight"] = number(a, "pattern_weight");
    } else if (type["value"] == 2)
        out["bump_factor"] = number(a, "bump_map_scale");
    return out;
}
} // namespace p3d
