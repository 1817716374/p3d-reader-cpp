#include "internal.hpp"
#include <codecvt>
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
// This is the native lexical split, not a filesystem or resource lookup. The
// original helper uses four 260-wchar buffers even when only one part is wanted.
Json texture_path_parts(const Json &source) {
    Json out = {{"source_value", source},
                {"status", "invalid"},
                {"logical_prefix", nullptr},
                {"stem", nullptr},
                {"extension", nullptr}};
    if (!source.is_string())
        return out;
    try {
        std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> codec;
        const auto bytes = source.get<std::string>();
        auto path = codec.from_bytes(bytes);
        if (codec.converted() != bytes.size())
            return out;
        if (path.find(u'\0') != std::u16string::npos)
            return out;
        auto colon = path.find(u':'), slash = path.find_first_of(u"/\\");
        out["logical_prefix"] = "";
        if (colon != std::u16string::npos && colon > 1 &&
            path.find(u':', colon + 1) == std::u16string::npos &&
            (slash == std::u16string::npos || colon < slash)) {
            out["logical_prefix"] = codec.to_bytes(path.substr(0, colon));
            path.erase(0, colon + 1);
        }
        if (path.size() > 1 && path[1] == u':')
            path.erase(0, 2);
        slash = path.find_last_of(u"/\\");
        auto basename = slash == std::u16string::npos ? path : path.substr(slash + 1);
        const auto dot = basename.find_last_of(u'.');
        auto stem = basename.substr(0, dot);
        auto ext = dot == std::u16string::npos ? std::u16string() : basename.substr(dot);
        const bool overflow = (slash != std::u16string::npos && slash + 1 >= 260) ||
                              stem.size() >= 260 || ext.size() >= 260;
        out["status"] = overflow ? "native_buffer_limit_clears_parts" : "decoded";
        out["stem"] = overflow ? "" : codec.to_bytes(stem);
        out["extension"] = overflow || ext.empty() ? "" : codec.to_bytes(ext.substr(1));
    } catch (const std::range_error &) {
        // Invalid UTF-8 does not select a different reader by accident.
    }
    return out;
}
int texture_name_equal(const Json &value, const char *expected) {
    if (!value.is_string())
        return -1;
    auto s = value.get<std::string>();
    for (auto &c : s) {
        if (static_cast<unsigned char>(c) >= 128)
            return -1; // Native _wcsicmp depends on the process locale here.
        if (c >= 'A' && c <= 'Z')
            c += 'a' - 'A';
    }
    return s == expected;
}
Json texture_provider_dispatch(const Json &provider, const Json &path) {
    Json out = {{"status", "unresolved"},
                {"branch", nullptr},
                {"provider_type_after_dispatch", provider},
                {"preset", nullptr}};
    const auto pma = texture_name_equal(path["extension"], "pma");
    if (pma < 0) {
        out["reason"] = "unresolved_reference_extension";
        return out;
    }
    if (pma) {
        struct Preset {
            const char *name;
            unsigned type;
        };
        static const Preset presets[] = {
            {"boards", 11}, {"brick", 12},    {"checker", 14}, {"checkr3d", 15}, {"bwnoise", 13},
            {"clouds", 16}, {"colnoise", 17}, {"flame", 18},   {"fog", 19},      {"marble", 20},
            {"rgb", 21},    {"sand", 22},     {"stone", 23},   {"turbulnc", 24}, {"turf", 25},
            {"water", 26},  {"waves", 27},    {"wood", 28},    {"wood01", 29},   {"grad1d", 31}};
        for (const auto &preset : presets) {
            const auto match = texture_name_equal(path["stem"], preset.name);
            if (match < 0) {
                out["reason"] = "locale_dependent_preset_name";
                return out;
            }
            if (!match)
                continue;
            out["status"] = "identified";
            out["branch"] = "pma_preset";
            out["preset"] = {{"name", preset.name},
                             {"procedure_type", preset.type},
                             {"parameter_status", "not_decoded"}};
            out["provider_type_after_dispatch"] = preset.type == 31 ? 3 : 4;
            return out;
        }
        out["branch"] = "pma_user_data";
    } else if (provider.is_null()) {
        out["reason"] = "unresolved_provider_type";
        return out;
    } else
        out["branch"] = provider == 3 || provider == 4 ? "M633" : provider == 7 ? "M541" : "none";
    out["status"] = "identified";
    return out;
}
Json reader_applicability(const char *status, const char *reason) {
    return {{"status", status},
            {"reason", reason},
            {"scope", "source_map_reader_before_link_resolution"}};
}
Json package_applicability(const Json &dispatch, const char *package) {
    if (dispatch["status"] != "identified")
        return reader_applicability("unresolved", "unresolved_provider_dispatch");
    return dispatch["branch"] == package
               ? reader_applicability("read", "selected_provider_branch")
               : reader_applicability("skipped", "different_provider_branch");
}
Json additional_texture_references(const Json &a) {
    Json out = {{"source_keys", {"M556"}},
                {"source_value", a.value("M556", Json())},
                {"status", "missing"},
                {"entries", Json::array()},
                {"order", "append_in_source_order"},
                {"resolution_status", "not_evaluated"}};
    if (!a.contains("M556"))
        return out;
    out["status"] = "invalid";
    if (!a["M556"].is_string())
        return out;
    const auto source = a["M556"].get<std::string>();
    if (source.find('\0') != std::string::npos)
        return out;
    out["status"] = "decoded";
    std::size_t pos = 0;
    bool quoted = false;
    while (pos < source.size()) {
        // Native token boundaries skip spaces and tabs, not commas or newlines.
        while (pos < source.size() && (source[pos] == ' ' || source[pos] == '\t'))
            ++pos;
        if (pos == source.size())
            break;
        const auto start = pos;
        std::string value;
        while (true) {
            std::size_t slashes = 0;
            while (pos < source.size() && source[pos] == '\\') {
                ++slashes;
                ++pos;
            }
            bool copy = true;
            if (pos < source.size() && source[pos] == '"') {
                if ((slashes & 1) == 0) {
                    copy = false;
                    if (quoted && pos + 1 < source.size() && source[pos + 1] == '"') {
                        ++pos;
                        copy = true;
                    }
                    // The native parser toggles even after a doubled quote.
                    // This is not CSV or JSON escaping.
                    quoted = !quoted;
                }
                slashes /= 2;
            }
            value.append(slashes, '\\');
            if (pos == source.size() ||
                (!quoted && (source[pos] == ' ' || source[pos] == '\t' || source[pos] == ',')))
                break;
            if (copy)
                value.push_back(source[pos]);
            ++pos;
        }
        out["entries"].push_back({{"append_index", out["entries"].size()},
                                  {"source_byte_offset", start},
                                  {"source_byte_count", pos - start},
                                  {"source_fragment", source.substr(start, pos - start)},
                                  {"value", value},
                                  {"unterminated_quote", quoted}});
        if (pos < source.size())
            ++pos;
    }
    return out;
}
struct ProcedureFields {
    const char *scalars;
    const char *integers;
    const char *colors;
};
#include "material_procedure_fields.inc"
Json procedure_parameters(const Json &a, const ProcedureFields &fields) {
    Json out = Json::object();
    for (unsigned kind = 0; kind < 3; ++kind) {
        std::istringstream names(kind == 0   ? fields.scalars
                                 : kind == 1 ? fields.integers
                                             : fields.colors);
        std::string name;
        while (names >> name) {
            auto param = kind == 2 ? vector(a, name, "RGB") : number(a, name, kind == 1);
            param["source_type"] = kind == 0 ? "float64" : kind == 1 ? "uint32" : "rgb_float64";
            out[name] = std::move(param);
        }
    }
    return out;
}
Json unmapped_parameters(const Json &source, const Json &parameters) {
    auto out = source;
    for (const auto &param : parameters)
        for (const auto &key : param["source_keys"])
            out.erase(key.get<std::string>());
    return out;
}
Json procedure_channel(const Json &owner) {
    Json entries = Json::array();
    for (std::size_t i = 0; i < owner["children"].size(); ++i) {
        const auto &n = owner["children"][i];
        if (n["tag"] != "M635")
            continue;
        auto params = procedure_parameters(n["attributes"], {"", "M171 M173 M174", ""});
        params["M171"]["reader_applies"] = false; // Read into a temporary, then discarded.
        auto selector = params["M174"]["status"] == "missing" ? params["M173"] : params["M174"];
        Json points = Json::array();
        for (std::size_t j = 0; j < n["children"].size(); ++j) {
            const auto &child = n["children"][j];
            auto p = procedure_parameters(
                child["attributes"],
                {"M175 M176 M179 M180 M181 M214 M215 M216", "M177 M178 M211 M212 M213 M217", ""});
            auto &order = p["M211"];
            order["reader_value"] = order["status"] == "decoded"
                                        ? (order["value"] == 0 ? Json(1) : order["value"])
                                        : Json();
            points.push_back(
                {{"child_index", j},
                 {"source_tag", child["tag"]},
                 {"source_parameters", child["attributes"]},
                 {"parameters", p},
                 {"unmapped_source_parameters", unmapped_parameters(child["attributes"], p)}});
        }
        entries.push_back(
            {{"child_index", i},
             {"selection", entries.empty() ? "selected" : "later_matching_node"},
             {"source_parameters", n["attributes"]},
             {"parameters", params},
             {"reader_selector", selector},
             {"unmapped_source_parameters", unmapped_parameters(n["attributes"], params)},
             {"control_entries", points}});
    }
    return {{"entries", entries}, {"selection_rule", "first_exact_M635_child"}};
}
} // namespace
Json material_replicator_nodes(const Json &owner) {
    Json entries = Json::array();
    for (std::size_t i = 0; i < owner["children"].size(); ++i) {
        const auto &node = owner["children"][i];
        if (node["tag"] != "M541")
            continue;
        const auto &a = node["attributes"];
        auto params = procedure_parameters(
            a, {"M542 M543 M544 M545 M546 M547 M550 M551 M552 M553", "M548 M549 M554 M555", ""});
        for (const auto key : {"M548", "M549"}) {
            auto &param = params[key];
            param["reader_value"] =
                param["status"] == "decoded" ? Json(param["value"] != 0) : Json();
        }
        entries.push_back({{"child_index", i},
                           {"selection", entries.empty() ? "selected" : "later_matching_node"},
                           {"source_parameters", a},
                           {"parameters", params},
                           {"unmapped_source_parameters", unmapped_parameters(a, params)}});
    }
    return {{"entries", entries},
            {"selection_rule", "first_exact_M541_child"},
            {"activation_status", "not_evaluated"},
            {"evaluation_status", "not_evaluated"},
            {"value_policy", "explicit_source_values_without_constructor_defaults"}};
}
Json material_reader_paths(const Json &tree, Json &maps) {
    auto version = number(tree["attributes"], "material_version", true, true);
    const Json mode = version["status"] == "missing" ? Json(0) : version["value"];
    Json profile = {{"scope", "native_material_xml_file_reader"},
                    {"version", version},
                    {"mode", mode},
                    {"mode_source", version["status"] == "missing"   ? "missing_uses_zero"
                                    : version["status"] == "decoded" ? "source_attribute"
                                                                     : "invalid_source"},
                    {"provider_type_source_key", mode.is_null() ? Json()
                                                 : mode == 0    ? Json("layer_state")
                                                                : Json("layer")}};
    for (auto &map : maps) {
        const auto &a = map["source_parameters"];
        auto path = texture_path_parts(a.value("Filename", Json("")));
        path["source_keys"] = {"Filename"};
        path["source_status"] = a.contains("Filename") ? "present" : "missing";
        Json reader = {{"scope", "source_map_reader_before_link_resolution"},
                       {"branch", nullptr},
                       {"status", "unresolved"},
                       {"source_filename", path}};
        auto layers = reader_applicability("unresolved", "unresolved_map_reader_branch");
        auto packages = layers;
        auto extras = layers;
        const auto &type = map["semantics"]["type"];
        if (!map["native_table_member"].get<bool>() || type["status"] == "missing" ||
            type["value"] == 0) {
            reader["status"] = "identified";
            reader["branch"] = "skipped";
            const auto reason = !map["native_table_member"].get<bool>() ? "outside_native_table"
                                : type["status"] == "missing"           ? "missing_map_type"
                                                                        : "zero_map_type";
            layers = packages = extras = reader_applicability("skipped", reason);
        } else if (type["status"] == "decoded" && path["status"] != "invalid") {
            const auto pma = texture_name_equal(path["extension"], "pma");
            const auto name = texture_name_equal(path["stem"], "layers");
            if (pma == 1 && name == 1) {
                reader["status"] = "identified";
                reader["branch"] = "layer_collection";
                layers = reader_applicability("read", "layers_pma_filename");
                packages = extras = reader_applicability("skipped", "layer_collection_branch");
            } else if (pma == 0 || name == 0) {
                reader["status"] = "identified";
                reader["branch"] = "single_provider";
                layers = reader_applicability("skipped", "single_provider_branch");
                extras = reader_applicability("read", "single_provider_branch");
                Json provider = {{"source_keys", Json::array()},
                                 {"status", "unresolved"},
                                 {"value", nullptr},
                                 {"reader_value", nullptr}};
                if (!mode.is_null()) {
                    provider = number(a, mode == 0 ? "layer_state" : "layer", true);
                    provider["reader_value"] = provider["value"];
                    provider["normalization"] = "source_value";
                    if (provider["status"] == "missing") {
                        provider["reader_value"] = 1;
                        provider["normalization"] = "native_constructor_type";
                    } else if (provider["value"] == 6) {
                        provider["reader_value"] = 4;
                        provider["normalization"] = "six_to_four";
                    } else if (provider["value"] == 0) {
                        provider["reader_value"] = a.contains("Filename") ? Json() : Json(1);
                        provider["normalization"] = a.contains("Filename")
                                                        ? "requires_first_resource_extension"
                                                        : "empty_resource_list_uses_image";
                    }
                }
                if (type["value"] == 30) {
                    provider["reader_value"] = 5;
                    provider["normalization"] = "map_type_thirty_override";
                }
                reader["single_provider_type_before_preset"] = provider;
                // Unlike the layered-reader argument, Map.Filename is passed
                // through a resource service before the procedural extension
                // test. Do not substitute the raw reference for that result.
                auto dispatch = a.contains("Filename")
                                    ? Json{{"status", "unresolved"},
                                           {"branch", nullptr},
                                           {"reason", "requires_first_resource_reference"},
                                           {"provider_type_after_dispatch", nullptr}}
                                    : texture_provider_dispatch(provider["reader_value"], path);
                reader["provider_dispatch"] = dispatch;
                map["procedures"]["reader_applicability"] = package_applicability(dispatch, "M633");
                map["replicators"]["reader_applicability"] =
                    package_applicability(dispatch, "M541");
            }
        }
        if (reader["branch"] != "single_provider") {
            map["procedures"]["reader_applicability"] = packages;
            map["replicators"]["reader_applicability"] = packages;
        }
        map["reader_path"] = std::move(reader);
        map["semantics"]["additional_texture_references"]["reader_applicability"] = extras;
        map["texture_layers"]["reader_applicability"] = layers;
        for (auto &layer : map["texture_layers"]["entries"]) {
            auto applicability = layers;
            const auto &sem = layer["semantics"];
            if (applicability["status"] == "read" && sem["type"]["reader_type"].is_null())
                applicability = sem["type"]["status"] == "ignored_missing_layer_marker"
                                    ? reader_applicability("skipped", "missing_layer_marker")
                                    : reader_applicability("unresolved", "unresolved_layer_type");
            layer["reader_applicability"] = applicability;
            for (const auto &pair : {std::pair<const char *, const char *>("procedures", "M633"),
                                     {"replicators", "M541"}})
                layer[pair.first]["reader_applicability"] =
                    applicability["status"] == "read"
                        ? package_applicability(sem["reader_path"], pair.second)
                        : applicability;
            if (layer["semantics"].contains("additional_texture_references"))
                layer["semantics"]["additional_texture_references"]["reader_applicability"] =
                    applicability;
        }
    }
    return profile;
}
Json material_procedure_nodes(const Json &owner) {
    Json entries = Json::array();
    for (std::size_t i = 0; i < owner["children"].size(); ++i) {
        const auto &node = owner["children"][i];
        if (node["tag"] != "M633")
            continue;
        const auto &a = node["attributes"];
        auto type = number(a, "M634", true);
        const auto value = type["status"] == "decoded" ? type["value"].get<std::uint32_t>() : 0;
        const bool known = value >= 1 && value <= 70;
        auto params = known ? procedure_parameters(a, procedure_fields[value]) : Json::object();
        auto unmapped = unmapped_parameters(a, params);
        unmapped.erase("M634");
        Json entry = {{"child_index", i},
                      {"selection", entries.empty() ? "selected" : "later_matching_node"},
                      {"source_parameters", a},
                      {"type", type},
                      {"reader_schema_status", known ? "identified" : "unavailable"},
                      {"parameters", params},
                      {"unmapped_source_parameters", unmapped}};
        if (value == 10 && known) {
            Json channels = Json::array();
            for (std::size_t j = 0; j < node["children"].size(); ++j) {
                const auto &child = node["children"][j];
                channels.push_back(
                    {{"child_index", j},
                     {"source_tag", child["tag"]},
                     {"selection", j < 5 ? "selected" : "outside_native_channel_limit"},
                     {"channel", procedure_channel(child)}});
            }
            entry["channels"] = channels;
        }
        entries.push_back(std::move(entry));
    }
    return {{"entries", entries},
            {"selection_rule", "first_exact_M633_child"},
            {"activation_status", "not_evaluated"},
            {"evaluation_status", "not_evaluated"},
            {"value_policy", "explicit_source_values_without_constructor_defaults"}};
}
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
                {"additional_texture_references", additional_texture_references(a)},
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
Json material_layer_semantics(const Json &a) {
    Json type = {{"source_keys", {"LayerType"}}, {"source_value", a.value("LayerType", Json())},
                 {"status", "missing"},          {"token", nullptr},
                 {"token_id", nullptr},          {"reader_type", nullptr},
                 {"reader_type_name", nullptr},  {"argument", nullptr}};
    Json out = {{"type", type},
                {"reader_path",
                 {{"status", "unresolved"},
                  {"branch", nullptr},
                  {"scope", "local_layer_reader_if_invoked"}}},
                {"value_policy", "explicit_source_values_without_defaults_or_compositing"}};
    if (!a.contains("LayerType"))
        return out;
    auto &t = out["type"];
    t["status"] = "invalid";
    if (!a["LayerType"].is_string())
        return out;
    const auto source = a["LayerType"].get<std::string>();
    if (source.find('\0') != std::string::npos)
        return out;
    const auto prefix = source.find("layer");
    if (prefix == std::string::npos) {
        t["status"] = "ignored_missing_layer_marker";
        return out;
    }
    // Native trimming uses iswspace in the process locale. ASCII whitespace is
    // stable; don't guess whether a non-ASCII boundary space was stripped.
    auto trim = [](std::string &s) {
        const auto start = s.find_first_not_of(" \t\r\n\v\f");
        s = start == std::string::npos
                ? std::string()
                : s.substr(start, s.find_last_not_of(" \t\r\n\v\f") - start + 1);
        static const char *spaces[] = {u8"\u0085", u8"\u00a0", u8"\u1680", u8"\u180e", u8"\u2000",
                                       u8"\u2001", u8"\u2002", u8"\u2003", u8"\u2004", u8"\u2005",
                                       u8"\u2006", u8"\u2007", u8"\u2008", u8"\u2009", u8"\u200a",
                                       u8"\u2028", u8"\u2029", u8"\u202f", u8"\u205f", u8"\u3000"};
        for (const auto space : spaces) {
            const std::string w(space);
            if (s.size() >= w.size() && (s.compare(0, w.size(), w) == 0 ||
                                         s.compare(s.size() - w.size(), w.size(), w) == 0))
                return false;
        }
        return true;
    };
    auto tail = source.substr(prefix + 5);
    if (!trim(tail)) {
        t["status"] = "locale_dependent_whitespace";
        return out;
    }
    // Only a literal space separates the token from its argument. A tab can
    // be trimmed at a token boundary but is not itself a token delimiter.
    const auto split = tail.find(' ');
    auto token = tail.substr(0, split);
    auto argument = split == std::string::npos ? std::string() : tail.substr(split + 1);
    if (!trim(token) || !trim(argument)) {
        t["status"] = "locale_dependent_whitespace";
        return out;
    }
    struct LayerType {
        const char *token;
        unsigned reader_type;
    };
    static const LayerType types[] = {{"IMAGE", 1},
                                      {"PROCEDURE", 2},
                                      {"GRADIENT", 3},
                                      {"NORMAL", 0xf000},
                                      {"ADD", 0xf001},
                                      {"SUBTRACT", 0xf002},
                                      {"ALPHA", 0xf003},
                                      {"DISSOLVE", 1},
                                      {"ATOP", 1},
                                      {"IN", 1},
                                      {"OUT", 1},
                                      {"GAMMA", 0xf00c},
                                      {"TINT", 0xf00d},
                                      {"BRIGHTNESS", 0xf00e},
                                      {"CONTRAST", 0xf00f},
                                      {"GROUP_START", 0xf012},
                                      {"GROUP_END", 0xf013},
                                      {"ALPHABACKGROUND_START", 0xf014},
                                      {"ALPHABACKGROUND_END", 0xf015},
                                      {"LXOPROCEDURE", 4},
                                      {"DIFFERENCE", 0xf016},
                                      {"NORMALMULTIPLY", 0xf017},
                                      {"DIVIDE", 0xf018},
                                      {"MULTIPLY", 0xf019},
                                      {"SCREEN", 0xf01a},
                                      {"OVERLAY", 0xf01b},
                                      {"SOFTLIGHT", 0xf01c},
                                      {"HARDLIGHT", 0xf01d},
                                      {"DARKEN", 0xf01e},
                                      {"LIGHTEN", 0xf01f},
                                      {"COLORDODGE", 0xf020},
                                      {"COLORBURN", 0xf021},
                                      {"8119LXOPROCEDURE", 4},
                                      {"CELL", 5},
                                      {"TEXTURE_REPLICATOR", 7}};
    unsigned code = 1;
    t["token"] = token;
    t["argument"] = argument;
    t["status"] = "unknown_token_image_fallback";
    for (std::size_t i = 0; i < sizeof(types) / sizeof(types[0]); ++i) {
        if (token != types[i].token)
            continue;
        code = types[i].reader_type;
        t["token_id"] = i;
        t["status"] = code == 1 && i != 0 ? "known_token_image_fallback" : "decoded";
        break;
    }
    t["reader_type"] = code;
    for (const auto &known : types)
        if (known.reader_type == code) {
            t["reader_type_name"] = known.token;
            break;
        }
    auto data = number(a, "LayerDataFlags", true);
    data["reader_value"] = nullptr;
    if (data["status"] == "decoded") {
        const auto v = data["value"].get<std::uint32_t>();
        data["reader_value"] = (v & 0xfffff1ffu) | ((v & 0x200u) << 2);
        data["discarded_source_bits"] = v & 0xc00u;
    }
    out["data_flags"] = std::move(data);
    const bool texture = code == 1 || code == 2 || code == 3 || code == 4 || code == 7;
    const bool group = code >= 0xf012 && code <= 0xf015;
    const bool read_flags = texture || code == 0xf000 || (code >= 0xf001 && code <= 0xf003) ||
                            code == 0xf00c || code == 0xf00d || group || code >= 0xf016;
    auto flags = number(a, "LayerFlags", true);
    flags["reader_applies"] = read_flags;
    flags["bits"] = Json::array();
    for (unsigned i = 0; i < 3; ++i)
        flags["bits"].push_back(
            {{"bit", i},
             {"value", flags["status"] == "decoded"
                           ? Json((flags["value"].get<std::uint32_t>() & (1u << i)) != 0)
                           : Json()}});
    flags["unassigned_bits"] =
        flags["status"] == "decoded" ? Json(flags["value"].get<std::uint32_t>() & ~7u) : Json();
    out["flags"] = std::move(flags);
    t["argument_role"] = texture ? "texture_reference" : group ? "group_name" : "not_read";
    if (texture) {
        auto path = texture_path_parts(argument);
        path["source_keys"] = {"LayerType"};
        auto dispatch = texture_provider_dispatch(code, path);
        dispatch["source_reference"] = std::move(path);
        out["reader_path"] = std::move(dispatch);
    } else
        out["reader_path"] = {{"status", "identified"},
                              {"branch", "non_texture_layer"},
                              {"provider_type_after_dispatch", code}};
    out["reader_path"]["scope"] = "local_layer_reader_if_invoked";
    Json params = Json::object();
    if (texture) {
        out["additional_texture_references"] = additional_texture_references(a);
        // These attributes are read for texture providers, not blend markers.
        for (const auto key : {"pattern_mapping", "pattern_scalemode", "texture_filter_type"})
            params[key] = number(a, key, true, true);
        for (const auto key :
             {"pattern_angle", "pattern_opacity", "image_gamma", "scale_z", "offset_z", "low_value",
              "high_value", "antialias_strength", "minimum_spot"})
            params[key] = number(a, key);
        params["pattern_scale"] = vector(a, "pattern_scale", "xy");
        params["pattern_offset"] = vector(a, "pattern_offset", "xy");
    } else if (code == 0xf00c)
        params["layer_gamma"] = number(a, "layer_gamma");
    else if (code == 0xf00d)
        params["layer_color"] = vector(a, "layer_color", "rgb");
    out["parameters"] = std::move(params);
    return out;
}
} // namespace p3d
