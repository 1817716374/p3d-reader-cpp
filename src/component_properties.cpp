#include "component_properties.hpp"
#include <cmath>

namespace p3d {
namespace {
constexpr std::uint64_t none_id = 0x4500450011720048ULL;
constexpr std::uint64_t bool_id = 0x1580142273520992ULL;
constexpr std::uint64_t double_id = 0x7175318778202422ULL;
constexpr std::uint64_t int_id = 0x4569571470222419ULL;
constexpr std::uint64_t uint_id = 0x2477702224190092ULL;
constexpr std::uint64_t string_id = 0x1316456973520092ULL;
constexpr std::uint64_t bytes_id = 0x0059665104550025ULL;
constexpr std::uint64_t vector_id = 0x3131099204415903ULL;
constexpr std::uint64_t map_id = 0x3131099213160368ULL;
constexpr std::uint64_t files_id = 0x0181635124290115ULL;
constexpr std::uint64_t function_id = 0x4827000104282422ULL;
constexpr std::uint64_t property_id = 0x0141762724222207ULL;
constexpr std::uint64_t noumenon_id = 0x0260975554222207ULL;
constexpr std::uint64_t data_key_id = 0x1395017306582671ULL;
constexpr std::uint64_t data_string_map_id = 0x4809735200591395ULL;
constexpr std::uint64_t data_uint_map_id = 0x1311011548825714ULL;
constexpr std::uint64_t material_id = 0x2624634702211655ULL;
constexpr std::uint64_t model_id = 0x0956146148825714ULL;
constexpr std::uint64_t entity_id = 0x1395755506582671ULL;
constexpr std::uint64_t color_id = 0x4767484556636631ULL;

std::int64_t signed_low32(std::int64_t value) {
    const auto low = std::uint32_t(value);
    return low <= INT32_MAX ? std::int64_t(low) : std::int64_t(low) - 0x100000000LL;
}

struct StackDecoder {
    const Bytes &source;
    std::size_t visits = 0;
    bool complete = true;

    void budget(unsigned depth) {
        require(depth <= 80, "component property nesting limit");
        require(++visits <= 1000000, "component property node limit");
    }
    Json text(std::size_t start, std::size_t size) const {
        const auto bytes = slice(source, start, size);
        Json out = {{"bytes_base64", base64(bytes)}, {"bytes", size}};
        if (std::all_of(bytes.begin(), bytes.end(), [](unsigned char c) { return c < 128; })) {
            out["text"] = std::string(bytes.begin(), bytes.end());
            out["encoding"] = "ascii";
        } else
            out["encoding"] = "unspecified_narrow_string";
        return out;
    }
    Json read(std::size_t begin, std::size_t &end, unsigned depth, std::uint64_t expected = 0) {
        budget(depth);
        require(end >= begin && end - begin >= 16, "component property frame is truncated");
        Reader header(source, end - 16);
        const auto size = header.u64(), id = header.u64();
        require(size <= end - begin - 16, "component property payload exceeds its frame");
        const auto start = end - 16 - std::size_t(size);
        Json out = {{"type_id", id}, {"offset", start}, {"bytes", size}, {"status", "decoded"}};
        const auto payload_end = end - 16;
        end = start;
        try {
            require(!expected || expected == id, "component property type mismatch");
            std::size_t cursor = payload_end;
            Reader raw(source, start);
            auto child = [&](std::uint64_t type = 0) {
                auto v = read(start, cursor, depth + 1, type);
                require(!type || v.at("status") == "decoded", "invalid typed component member");
                return v;
            };
            auto count = [&]() {
                auto v = child(uint_id);
                const auto n = v.at("value").get<std::uint64_t>();
                require(n <= (cursor - start) / 16 && n <= 1000000 - visits,
                        "component property count exceeds input or node budget");
                out["count_source_offset"] = v.at("offset");
                return n;
            };
            auto finish = [&]() {
                if (cursor > start) {
                    out["unread_prefix"] = {
                        {"offset", start},
                        {"bytes", cursor - start},
                        {"binary_base64", base64(slice(source, start, cursor - start))}};
                    out["status"] = "partial";
                    complete = false;
                }
            };
            if (id == none_id || id == bool_id) {
                require(size >= 1, "component byte value is truncated");
                out["kind"] = id == none_id ? "none" : "bool";
                out["storage_uint8"] = raw.u8();
                out["value"] = id == none_id ? Json(nullptr) : Json(source[start] != 0);
                cursor = start;
                if (size > 1)
                    out["ignored_suffix_bytes"] = size - 1;
            } else if (id == int_id || id == uint_id || id == double_id) {
                require(size >= 8, "component numeric value is truncated");
                if (id == int_id) {
                    out["kind"] = "int64";
                    out["value"] = raw.i64();
                } else if (id == uint_id) {
                    out["kind"] = "uint64";
                    out["value"] = raw.u64();
                } else {
                    out["kind"] = "double";
                    out["value"] = raw.f64();
                    out["bits_hex"] = hex(slice(source, start, 8));
                }
                cursor = start;
                if (size > 8)
                    out["ignored_suffix_bytes"] = size - 8;
            } else if (id == string_id || id == bytes_id) {
                out["kind"] = id == string_id ? "string" : "bytes";
                if (id == string_id)
                    out.update(text(start, size));
                else
                    out["bytes_base64"] = base64(slice(source, start, size));
                cursor = start;
            } else if (id == model_id || id == entity_id) {
                out["kind"] = id == model_id ? "model_id" : "entity_id";
                if (id == entity_id) {
                    out["entity_id_source"] = child(int_id);
                    out["entity_id"] =
                        std::uint64_t(out.at("entity_id_source").at("value").get<std::int64_t>());
                }
                out["model_id_source"] = child(int_id);
                out["model_id"] =
                    signed_low32(out.at("model_id_source").at("value").get<std::int64_t>());
                finish();
            } else if (id == color_id) {
                out["kind"] = "color";
                Json components = Json::array(), channels = Json::array();
                for (unsigned i = 0; i < 4; ++i)
                    components.push_back(child(double_id));
                std::reverse(components.begin(), components.end());
                bool converted = true;
                for (auto &component : components) {
                    // Read the binary double directly: JSON may serialize NaN as null.
                    Reader value(source, component.at("offset").get<std::size_t>());
                    const double scaled = value.f64() * 255.0;
                    const double truncated = std::trunc(scaled);
                    if (std::isfinite(truncated) && truncated >= -2147483648.0 &&
                        truncated <= 2147483647.0) {
                        const auto byte = std::uint8_t(std::int64_t(truncated));
                        channels.push_back(byte);
                        component["conversion_status"] = "converted";
                    } else {
                        channels.push_back(nullptr);
                        component["conversion_status"] = "invalid_int32_conversion";
                        converted = false;
                    }
                }
                out["components"] = std::move(components);
                out["component_order"] = "rgba";
                out["rgba_uint8"] = std::move(channels);
                out["conversion"] = "multiply_255_truncate_int32_low_byte";
                finish();
                if (!converted) {
                    // CVTTSD2SI invalid input depends on the native exception mask.
                    // Retain the source and do not invent a normal color channel.
                    out["status"] = "partial";
                    complete = false;
                }
            } else if (id == material_id) {
                out["kind"] = "material";
                out["fields"] = material(start, cursor, depth + 1);
                out["not_serialized_fields"] =
                    Json::array({"material_id", "display_name", "bump_map_file",
                                 "use_image_alpha_channel", "extended_data_json", "pbr_maps"});
                finish();
            } else if (id == data_key_id) {
                out["kind"] = "data_key";
                out["object_id_source"] = child(int_id);
                out["class_id_source"] = child(int_id);
                out["object_id"] = out.at("object_id_source").at("value");
                out["class_id"] = out.at("class_id_source").at("value");
                finish();
            } else if (id == data_string_map_id || id == data_uint_map_id) {
                out["kind"] =
                    id == data_string_map_id ? "data_key_string_map" : "data_key_uint64_map";
                const auto n = count();
                out["entries"] = Json::array();
                out["entry_order"] = "native_read_order";
                std::map<std::pair<std::int64_t, std::int64_t>, std::size_t> selected;
                for (std::uint64_t i = 0; i < n; ++i) {
                    auto key = child(data_key_id);
                    auto value = child(id == data_string_map_id ? string_id : uint_id);
                    selected[{key.at("class_id").get<std::int64_t>(),
                              key.at("object_id").get<std::int64_t>()}] = std::size_t(i);
                    out["entries"].push_back(
                        {{"key", std::move(key)}, {"value", std::move(value)}});
                }
                finish();
                if (out.at("status") == "decoded") {
                    Json indices = Json::array();
                    for (const auto &entry : selected)
                        indices.push_back(entry.second);
                    out["native_index"] = {{"selected_entry_indices", std::move(indices)},
                                           {"key_order", "signed_class_id_then_signed_object_id"},
                                           {"duplicate_rule", "last_in_native_read_order"}};
                }
            } else if (id == noumenon_id || id == vector_id || id == map_id) {
                out["kind"] = id == noumenon_id ? "noumenon" : id == vector_id ? "vector" : "map";
                const auto n = count();
                out["entries"] = Json::array();
                for (std::uint64_t i = 0; i < n; ++i) {
                    if (id == vector_id)
                        out["entries"].push_back(child());
                    else {
                        auto key = child(id == noumenon_id ? string_id : 0);
                        auto value = child(id == noumenon_id ? property_id : 0);
                        out["entries"].push_back(
                            {{"key", std::move(key)}, {"value", std::move(value)}});
                    }
                }
                out["entry_order"] = "native_read_order";
                if (id == map_id)
                    out["runtime_key_comparison"] = "not_evaluated";
                finish();
            } else if (id == property_id) {
                out["kind"] = "property";
                out["value"] = child();
                const auto n = count();
                out["attributes"] = Json::array();
                for (std::uint64_t i = 0; i < n; ++i) {
                    auto key = child(string_id);
                    auto value = child(property_id);
                    out["attributes"].push_back(
                        {{"key", std::move(key)}, {"value", std::move(value)}});
                }
                out["attribute_duplicate_rule"] = "read_into_existing_property";
                finish();
            } else if (id == function_id) {
                out["kind"] = "unified_function";
                out["module_name"] = child(string_id);
                out["operator_name"] = child(string_id);
                finish();
            } else if (id == files_id) {
                out["kind"] = "dependent_files";
                out["root"] = folder(start, cursor, depth + 1);
                finish();
            } else {
                out["kind"] = "unknown";
                out["status"] = "unsupported_type";
                out["payload_base64"] = base64(slice(source, start, size));
                complete = false;
            }
        } catch (const std::exception &e) {
            out["status"] = "invalid";
            out["decode_error"] = e.what();
            // The enclosing binary field retains the source. Do not duplicate a large
            // invalid payload at every level of a malformed recursive property.
            complete = false;
        }
        return out;
    }

    Json material(std::size_t begin, std::size_t &end, unsigned depth) {
        Json fields = Json::object();
        auto member = [&](std::uint64_t id) {
            auto v = read(begin, end, depth, id);
            require(v.at("status") == "decoded", "invalid registered material member");
            return v;
        };
        for (const auto *name :
             {"is_valid", "has_transparency", "has_specular_factor", "has_specular_color",
              "has_roughness_factor", "has_refract_factor", "has_reflect_factor", "has_map",
              "has_glow_factor", "has_glow_color", "has_diffuse_factor", "has_color",
              "has_ambient_factor"})
            fields[name] = member(bool_id);
        for (const auto *name : {"refract_factor", "reflect_factor", "roughness_factor",
                                 "diffuse_factor", "ambient_factor", "glow_factor"})
            fields[name] = member(double_id);
        auto vector = [&](const char *name, std::size_t count, bool reverse) {
            Json components = Json::array(), values = Json::array();
            for (std::size_t i = 0; i < count; ++i)
                components.push_back(member(double_id));
            if (reverse)
                std::reverse(components.begin(), components.end());
            for (const auto &component : components)
                values.push_back(component.at("value"));
            fields[name] = {{"value", std::move(values)},
                            {"components", std::move(components)},
                            {"component_order", count == 3 ? "rgb" : "xy"}};
        };
        vector("glow_color", 3, false);
        fields["specular_factor"] = member(double_id);
        vector("specular_color", 3, false);
        fields["transparency"] = member(double_id);
        vector("color", 3, false);
        fields["bump_factor"] = member(double_id);
        fields["w_rotation"] = member(double_id);
        fields["w_rotation"]["unit"] = "degrees";
        vector("uv_offset", 2, true);
        vector("uv_scale", 2, true);
        for (const auto *name : {"map_mode", "map_unit"}) {
            auto value = member(int_id);
            value["runtime_value_int32"] = signed_low32(value.at("value").get<std::int64_t>());
            fields[name] = std::move(value);
        }
        for (const auto *name : {"map_file", "name"}) {
            auto value = member(string_id);
            const auto offset = value.at("offset").get<std::size_t>();
            auto length = value.at("bytes").get<std::size_t>();
            auto stop = std::find(source.begin() + offset, source.begin() + offset + length, 0);
            length = std::size_t(stop - (source.begin() + offset));
            auto input = text(offset, length);
            input["encoding"] = "windows_ansi_code_page";
            input["termination"] = "first_nul";
            value["native_string_input"] = std::move(input);
            fields[name] = std::move(value);
        }
        return fields;
    }

    Json folder(std::size_t begin, std::size_t &end, unsigned depth) {
        budget(depth);
        auto typed = [&](std::uint64_t id) {
            auto v = read(begin, end, depth + 1, id);
            require(v.at("status") == "decoded", "invalid dependent file member");
            return v;
        };
        auto count = typed(uint_id);
        const auto n = count.at("value").get<std::uint64_t>();
        require(n <= (end - begin) / 32 && n <= 1000000 - visits,
                "dependent file count exceeds input or node budget");
        Json result = {{"kind", "folder"},
                       {"entries", Json::array()},
                       {"count_source_offset", count.at("offset")}};
        for (std::uint64_t i = 0; i < n; ++i) {
            auto name = typed(string_id);
            auto flag = typed(bool_id);
            auto value =
                flag.at("value").get<bool>() ? folder(begin, end, depth + 1) : typed(bytes_id);
            result["entries"].push_back({{"name", std::move(name)},
                                         {"folder_flag", std::move(flag)},
                                         {"value", std::move(value)}});
        }
        return result;
    }
};

Json source_input(const Json &root, const std::string &key, const std::string &kind) {
    if (!root.contains("entries") || root.at("status") == "invalid")
        return {{"status", "not_evaluated"}};
    const auto &entries = root.at("entries");
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const auto &entry = entries[i];
        if (entry.at("key").value("text", "") != key)
            continue;
        Json out = {{"property_index", i}, {"lookup", "first_exact_byte_key"}};
        const auto &property = entry.at("value");
        if (!property.contains("value")) {
            out["status"] = "not_evaluated";
            return out;
        }
        const auto &value = property.at("value");
        if (value.value("kind", "") != kind) {
            out["status"] = "wrong_value_type";
            return out;
        }
        if (value.at("status") != "decoded") {
            out["status"] = "not_evaluated";
            return out;
        }
        out["status"] = "found";
        out["value_offset"] = value.at("offset");
        out["bytes_base64"] = value.at("bytes_base64");
        out["bytes"] = value.at("bytes");
        return out;
    }
    return {{"status", "absent"}};
}
} // namespace

static Json decode_stack(const Bytes &bytes, std::uint64_t expected, const char *scope) {
    require(bytes.size() <= 64 * 1024 * 1024, "component property input limit");
    StackDecoder decoder{bytes};
    std::size_t end = bytes.size();
    Json out = {{"scope", scope}};
    try {
        out["root"] = decoder.read(0, end, 0, expected);
        out["status"] = decoder.complete ? "decoded" : "partial";
        if (out.at("root").at("status") == "invalid")
            out["status"] = "invalid";
        out["consumed_bytes"] = bytes.size() - end;
        if (end) {
            out["leading_bytes"] = {{"bytes", end},
                                    {"binary_base64", base64(slice(bytes, 0, end))}};
            if (out.at("status") == "decoded")
                out["status"] = "partial";
        }
    } catch (const std::exception &e) {
        out["status"] = "invalid";
        out["decode_error"] = e.what();
    }
    return out;
}

Json decode_data_unit(const Bytes &bytes) {
    return decode_stack(bytes, 0, "stored_data_unit");
}

Json decode_component_properties(const Bytes &bytes) {
    auto out = decode_stack(bytes, noumenon_id, "stored_component_properties");
    try {
        if (!out.contains("root"))
            return out;
        out["component_graphics_input"] =
            source_input(out.at("root"), "componentGraphics", "bytes");
        out["component_material_input"] =
            source_input(out.at("root"), "\x07_component_material", "string");
        auto &graphics = out["component_graphics_input"];
        if (graphics.at("status") == "found" && graphics.at("bytes") != 0) {
            try {
                graphics["graphics"] = decode_graphics_bytes(
                    slice(bytes, graphics.at("value_offset").get<std::size_t>(),
                          graphics.at("bytes").get<std::size_t>()));
            } catch (const std::exception &e) {
                graphics["decode_error"] = e.what();
            }
        }
        out["active_graphics_selection"] = "not_established";
        out["runtime_property_restoration"] = "not_evaluated";
    } catch (const std::exception &e) {
        out["status"] = "invalid";
        out["decode_error"] = e.what();
    }
    return out;
}
} // namespace p3d
