#include "internal.hpp"

namespace p3d {
namespace {
Json field(const Json &value) {
    return {{"constructor_value", value}, {"value", value}, {"read_status", "not_written"}};
}
Json default_layer() {
    return {{"origin", "native_constructor"}, {"source_layer_child_index", nullptr},
            {"data_flags", field(0x800u)},    {"enabled", field(true)},
            {"option_bit_1", field(false)},   {"option_bit_2", field(false)}};
}
bool failed(const Json &read) {
    return read.at("status") == "missing" ||
           read.value("conversion", Json::object()).value("status", Json()) ==
               "no_integer_assignment";
}
void flag_read(Json &target, const Json &read, int bit = -1, bool remap = false) {
    target["source_read"] = read;
    if (bit >= 0)
        target["source_bit"] = bit;
    if (read.at("status") == "decoded") {
        const auto v = read.at("value").get<std::uint32_t>();
        target["value"] = bit >= 0 ? Json(bool(v & (1u << bit)))
                          : remap  ? Json((v & 0xfffff1ffu) | ((v & 0x200u) << 2))
                                   : Json(v);
        target["read_status"] = "written";
    } else if (!failed(read)) {
        target["value"] = nullptr;
        target["read_status"] = "unresolved";
    }
}
void boolean_read(Json &target, const Json &read) {
    target["source_read"] = read;
    if (read.at("write_status") == "written") {
        target["value"] = read.at("reader_value");
        target["read_status"] = "written";
    } else if (read.at("write_status") != "not_written") {
        target["value"] = nullptr;
        target["read_status"] = "unresolved";
    }
}
const char *map_flag_key(std::uint32_t type) {
    static const char *keys[] = {nullptr,
                                 "PatternFlags",
                                 "BumpFlags",
                                 "SpecularFlags",
                                 "ReflectFlags",
                                 "TransparencyFlags",
                                 "TranslucencyFlags",
                                 "FinishFlags",
                                 "DiffuseFlags",
                                 "GlowAmountFlags",
                                 "ClearcoatAmountFlags",
                                 "AnisotropicDirectionFlags",
                                 nullptr,
                                 nullptr,
                                 nullptr,
                                 "DisplacementFlags",
                                 "NormalFlags",
                                 "M611",
                                 "M612",
                                 "M613",
                                 "M614",
                                 "M615",
                                 "M616",
                                 "M617",
                                 "M622",
                                 "M623",
                                 nullptr,
                                 nullptr,
                                 "M624",
                                 "M625"};
    return type < sizeof(keys) / sizeof(keys[0]) ? keys[type] : nullptr;
}
void update_status(Json &state) {
    if (state.at("status") != "resolved")
        return;
    for (const auto &layer : state.at("layers"))
        for (const auto key : {"data_flags", "enabled", "option_bit_1", "option_bit_2"})
            if (layer.at(key).at("value").is_null())
                state["status"] = "partial";
}
Json empty_state() {
    return {{"scope", "local_layer_sequence_and_flags_before_version_conversion"},
            {"status", "resolved"},
            {"object_state", "present"},
            {"layers", Json::array({default_layer()})},
            {"layer_count", 1}};
}
} // namespace

Json material_layer_input(const Json &map, std::size_t child_count) {
    auto out = empty_state();
    out["source_element_count"] = child_count;
    const auto branch = map.at("reader_path").at("branch");
    if (branch == "skipped") {
        out["object_state"] = "absent";
        out["layers"] = Json::array();
        out["layer_count"] = 0;
        return out;
    }
    if (branch == "single_provider") {
        auto &layer = out["layers"][0];
        layer["origin"] = "single_provider";
        const auto &attributes = map.at("source_parameters");
        const auto &reads = map.at("numeric_reader").at("parameters");
        boolean_read(layer["enabled"], reads.at("pattern_off"));
        flag_read(layer["data_flags"], material_xml_integer(attributes, "Flags", false));
        // This later read replaces only data-flags bit 11, even after Flags.
        const auto &aa = reads.at("enable_antialiasing");
        layer["data_flags"]["antialias_read"] = aa;
        if (aa.at("write_status") == "written") {
            auto &flags = layer["data_flags"]["value"];
            if (!flags.is_null())
                flags = (flags.get<std::uint32_t>() & ~0x800u) |
                        (aa.at("reader_value") == true ? 0x800u : 0);
            layer["data_flags"]["antialias_bit_written"] = aa.at("reader_value");
            layer["data_flags"]["read_status"] = flags.is_null() ? "unresolved" : "written";
        } else if (aa.at("write_status") != "not_written") {
            layer["data_flags"]["value"] = nullptr;
            layer["data_flags"]["read_status"] = "unresolved";
        }
        const auto type = map.at("semantics").at("type").at("value").get<std::uint32_t>();
        if (const auto key = map_flag_key(type))
            flag_read(layer[type == 1 ? "option_bit_1" : "option_bit_2"],
                      material_xml_integer(attributes, key, false), 0);
    } else if (branch == "layer_collection") {
        auto &layers = out["layers"];
        for (const auto &entry : map.at("texture_layers").at("entries")) {
            const auto gate = entry.at("reader_applicability").at("status");
            if (gate == "skipped")
                continue;
            if (gate != "read") {
                out["status"] = "partial";
                out["reason"] = "unresolved_layer_reader_acceptance";
                out["layer_count"] = nullptr;
                return out;
            }
            const auto index = entry.at("child_index").get<std::size_t>();
            auto &layer = layers.back();
            layer["origin"] = "xml_layer";
            layer["source_layer_child_index"] = index;
            const auto &sem = entry.at("semantics");
            flag_read(layer["data_flags"], sem.at("data_flags"), -1, true);
            if (sem.at("flags").at("reader_applies") == true) {
                flag_read(layer["enabled"], sem.at("flags"), 0);
                flag_read(layer["option_bit_1"], sem.at("flags"), 1);
                flag_read(layer["option_bit_2"], sem.at("flags"), 2);
            }
            // Success followed by any sibling creates another layer. That
            // layer survives even if all remaining siblings are ignored.
            if (index + 1 < child_count)
                layers.push_back(default_layer());
        }
        out["layer_count"] = layers.size();
    } else {
        out["status"] = "partial";
        out["reason"] = "unresolved_map_reader_branch";
        out["object_state"] = "unresolved";
        out["layer_count"] = nullptr;
        out["layers"] = Json::array();
    }
    update_status(out);
    return out;
}

Json material_layer_containers(const Json &maps, const Json &topology) {
    Json out = {{"scope", "local_layer_sequences_and_flags_after_version_conversion"},
                {"status", "resolved"},
                {"containers", Json::array()}};
    if (topology.at("status") != "resolved") {
        out["status"] = "partial";
        out["reason"] = "unresolved_version_conversion";
        return out;
    }
    auto &containers = out["containers"];
    for (const auto &object : topology.at("objects")) {
        Json state;
        if (object.at("origin") == "xml_map") {
            const auto &source = maps.at(object.at("source_map_index").get<std::size_t>());
            if (!source.contains("initial_layer_state")) {
                out["status"] = "partial";
                out["reason"] = "missing_initial_layer_state";
                out["containers"] = Json::array();
                return out;
            }
            state = source.at("initial_layer_state");
        } else if (object.at("local_layers").at("origin") == "native_layer_copy") {
            const auto id = object.at("local_layers").at("source_object_id").get<std::size_t>();
            state = containers.at(id);
            state["copied_from_object_id"] = id;
        } else
            state = empty_state();
        state["scope"] = "local_layer_sequence_and_flags_after_version_conversion";
        state["owner_object_id"] = object.at("object_id");
        state["source_map_index"] = object.at("source_map_index");
        if (state.at("status") != "resolved")
            out["status"] = "partial";
        containers.push_back(std::move(state));
    }
    for (const auto &update : topology.at("layer_flag_updates")) {
        auto &container =
            containers.at(update.at("layer_container_owner_object_id").get<std::size_t>());
        const auto mask = update.at("mask").get<std::uint32_t>();
        for (auto &layer : container["layers"]) {
            auto &flags = layer["data_flags"];
            flags["version_or_mask"] = mask;
            if (!flags["value"].is_null())
                flags["value"] = flags["value"].get<std::uint32_t>() | mask;
        }
    }
    return out;
}
} // namespace p3d
