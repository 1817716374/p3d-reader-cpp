#include "internal.hpp"

namespace p3d {
namespace {
Json converted_roots(const Json &input, const Json &mode, bool completed) {
    Json out = {{"status", "resolved"},
                {"parameters", Json::object()},
                {"operations", Json::array()},
                {"flags", input.at("flags").at("value")}};
    for (auto p = input.at("parameters").begin(); p != input.at("parameters").end(); ++p)
        out["parameters"][p.key()] = {{"value", p.value().at("value")},
                                      {"source_parameter", p.key()}};
    if (!completed || !mode.is_number_integer()) {
        out["status"] = "partial";
        out["flags"] = nullptr;
        for (auto &p : out["parameters"]) {
            p["value"] = nullptr;
            p["source_parameter"] = nullptr;
        }
        return out;
    }
    const auto version = mode.get<std::int64_t>();
    auto copy = [&](const char *to, const char *from) {
        out["parameters"][to] = out["parameters"][from];
        out["operations"].push_back(
            {{"operation", "copy_parameter"}, {"target", to}, {"source", from}});
    };
    auto flags = [&](std::uint32_t clear, std::uint32_t set) {
        if (!out["flags"].is_null())
            out["flags"] = (out["flags"].get<std::uint32_t>() & ~clear) | set;
        out["operations"].push_back(
            {{"operation", "change_flags"}, {"clear_mask", clear}, {"set_mask", set}});
    };
    if (version <= 2) {
        copy("transparent_color", "specular_color");
        copy("translucent_color", "color");
    }
    if (version <= 4) {
        flags(0x200000, 0x180000);
        copy("glow_color", "color");
        copy("refraction_roughness", "finish");
        copy("reflect_fresnel", "specular");
    }
    if (version <= 5) {
        copy("reflect_color", "specular_color");
        flags(0x400, 0);
    }
    if (version <= 6)
        copy("reflect_fresnel", "reflect");
    if (out["flags"].is_null())
        out["status"] = "partial";
    for (const auto &p : out["parameters"])
        if (p.at("value").is_null())
            out["status"] = "partial";
    return out;
}

// Identities refer to objects in this view, not file IDs or source Map indices.
// New maps are distinct objects even when they link to an existing map.
Json converted_maps(const Json &input, const Json &maps, const Json &bindings, const Json &mode) {
    Json out = {{"status", "resolved"},
                {"scope", "map_identity_links_and_layer_container_ownership"},
                {"objects", Json::array()},
                {"active_object_ids", Json::array()},
                {"operations", Json::array()},
                {"layer_flag_updates", Json::array()}};
    auto fail = [&](const char *reason) {
        out["status"] = "partial";
        out["reason"] = reason;
        // Objects and operations remain an executed prefix, not a final table.
        out["active_object_ids"] = Json::array();
    };
    if (!mode.is_number_integer() || bindings.at("status") != "resolved") {
        fail(!mode.is_number_integer() ? "unknown_version" : "unresolved_input_map_bindings");
        return out;
    }
    std::map<std::int64_t, std::size_t> active;
    auto &objects = out["objects"];
    for (const auto &entry : bindings.at("entries")) {
        if (entry.at("selection") != "active")
            continue;
        const auto index = entry.at("map_index").get<std::size_t>();
        const auto &read = maps.at(index).at("numeric_reader").at("parameters").at("pattern_off");
        Json enabled = read.at("write_status") == "written"       ? read.at("reader_value")
                       : read.at("write_status") == "not_written" ? Json(true)
                                                                  : Json();
        const auto type = entry.at("native_type_key").get<std::int64_t>();
        const auto id = objects.size();
        active[type] = id;
        objects.push_back(
            {{"object_id", id},
             {"native_type_key", type},
             {"source_map_index", index},
             {"origin", "xml_map"},
             {"active", true},
             {"enabled", enabled},
             {"link_type", entry.at("effective_link_type")},
             {"local_layers", {{"origin", "source_map_read"}, {"source_map_index", index}}}});
    }
    auto construct = [&](std::int64_t type) {
        const auto id = objects.size();
        Json previous;
        const auto old = active.find(type);
        if (old != active.end()) {
            previous = old->second;
            objects[old->second]["active"] = false;
            objects[old->second]["replaced_by_object_id"] = id;
        }
        objects.push_back({{"object_id", id},
                           {"native_type_key", type},
                           {"source_map_index", nullptr},
                           {"origin", "version_constructor"},
                           {"active", true},
                           {"enabled", true},
                           {"link_type", 0},
                           {"local_layers", {{"origin", "native_constructor"}}}});
        active[type] = id;
        out["operations"].push_back({{"operation", "construct_map"},
                                     {"object_id", id},
                                     {"type", type},
                                     {"replaced_object_id", previous}});
        return id;
    };
    auto link = [&](std::size_t id, std::int64_t target) {
        Json event = {{"operation", "set_link"}, {"object_id", id}, {"target_type", target}};
        const auto type = objects[id].at("native_type_key").get<std::int64_t>();
        if (target == type) {
            objects[id]["link_type"] = 0;
            event["result"] = "self_link_cleared";
        } else {
            std::set<std::size_t> seen;
            auto next = active.find(target);
            bool accepted = false;
            while (next != active.end() && next->second != id) {
                const auto current = next->second;
                if (!seen.insert(current).second) {
                    fail("cyclic_link_during_version_conversion");
                    event["result"] = "unresolved_cycle";
                    break;
                }
                const auto next_type = objects[current].at("link_type").get<std::int64_t>();
                if (!next_type) {
                    accepted = true;
                    break;
                }
                next = active.find(next_type);
            }
            if (accepted)
                objects[id]["link_type"] = target;
            if (!event.contains("result"))
                event["result"] = accepted ? "written" : "not_written";
        }
        out["operations"].push_back(std::move(event));
    };
    auto linked_copy = [&](std::int64_t type, std::int64_t target) {
        const auto source = active.at(target);
        const auto id = construct(type);
        link(id, target);
        objects[id]["enabled"] = objects[source]["enabled"];
        out["operations"].push_back(
            {{"operation", "copy_enabled"}, {"object_id", id}, {"source_object_id", source}});
    };
    const auto version = mode.get<std::int64_t>();
    if (version <= 2 && active.count(1)) {
        const auto enabled = objects[active.at(1)].at("enabled");
        if (enabled.is_null()) {
            fail("unknown_pattern_map_activation");
            return out;
        }
        if (enabled == true) {
            const auto id = construct(14);
            link(id, 1);
            if (out["status"] == "partial")
                return out;
            const auto &flags = input.at("flags").at("value");
            if (flags.is_null()) {
                fail("unknown_flags_for_legacy_map_creation");
                return out;
            }
            if (flags.get<std::uint32_t>() & 0x1000) {
                link(construct(12), 1);
                if (out["status"] == "partial")
                    return out;
                link(construct(13), 12);
                if (out["status"] == "partial")
                    return out;
            }
        }
    }
    if (version <= 3 && active.count(2)) {
        const auto &distance = input.at("parameters").at("displacement_distance").at("value");
        if (distance.is_null()) {
            fail("unknown_displacement_distance");
            return out;
        }
        if (distance != 0) {
            const auto source = active.at(2);
            const auto id = construct(15);
            const auto source_link = objects[source].at("link_type");
            objects[id]["link_type"] = source_link == 15 ? Json(0) : source_link;
            objects[id]["enabled"] = objects[source].at("enabled");
            objects[id]["copied_settings_from_object_id"] = source;
            objects[id]["settings_copy_scope"] =
                "link_enabled_weight_projection_frame_matrix_enable";
            // The map copy does not copy the explicit matrix axes.
            objects[id]["matrix_axes_status"] = "not_written_by_constructor_or_copy";
            objects[id]["local_layers"] = {{"origin", "native_layer_copy"},
                                           {"source_object_id", source}};
            out["operations"].push_back(
                {{"operation", "copy_map_state"}, {"object_id", id}, {"source_object_id", source}});
            objects[source]["active"] = false;
            objects[source]["removed_by_version_conversion"] = true;
            active.erase(2);
            out["operations"].push_back(
                {{"operation", "remove_map"}, {"object_id", source}, {"type", 2}});
        }
    }
    if (version <= 5) {
        if (active.count(12)) {
            linked_copy(4, 12);
            if (out["status"] == "partial")
                return out;
        }
        if (active.count(1)) {
            linked_copy(26, 1);
            if (out["status"] == "partial")
                return out;
        }
    }
    // The final pre-v8 walk accesses effective containers; a dangling link
    // falls back to the original local container without clearing that link.
    // Resolve every chain once. The sentinel denotes a missing terminal;
    // each such caller falls back to its own local container.
    std::vector<unsigned char> state(objects.size(), 0);
    std::vector<std::size_t> terminals(objects.size(), objects.size());
    for (const auto &entry : active) {
        auto current = entry.second;
        std::vector<std::size_t> trail;
        std::size_t terminal = objects.size();
        for (;;) {
            if (state[current] == 2) {
                terminal = terminals[current];
                break;
            }
            if (state[current] == 1) {
                fail("cyclic_effective_layer_container");
                return out;
            }
            state[current] = 1;
            trail.push_back(current);
            const auto target = objects[current].at("link_type").get<std::int64_t>();
            if (!target) {
                terminal = current;
                break;
            }
            auto next = active.find(target);
            if (next == active.end())
                break;
            current = next->second;
        }
        for (auto id : trail) {
            state[id] = 2;
            terminals[id] = terminal;
        }
    }
    std::set<std::size_t> flag_owners;
    for (const auto &entry : active) {
        const auto id = entry.second;
        const bool dangling = terminals[id] == objects.size();
        const auto current = dangling ? id : terminals[id];
        const auto owner = entry.first == 30 ? id : current;
        objects[id]["layer_container_owner_object_id"] = owner;
        objects[id]["layer_container_resolution"] = dangling      ? "dangling_uses_local"
                                                    : owner == id ? "local"
                                                                  : "native_shared";
        if (version < 8) {
            flag_owners.insert(owner);
            // Type 30 retains local layers; the getter copies linked mapping
            // into its first local layer as a separate operation.
            if (entry.first == 30 && current != id && !dangling)
                out["operations"].push_back({{"operation", "copy_first_layer_mapping"},
                                             {"object_id", id},
                                             {"source_object_id", current}});
        }
    }
    for (const auto owner : flag_owners)
        out["layer_flag_updates"].push_back({{"layer_container_owner_object_id", owner},
                                             {"operation", "or_each_layer_flags"},
                                             {"mask", 0x800}});
    for (const auto &entry : active)
        out["active_object_ids"].push_back(entry.second);
    return out;
}
} // namespace

Json material_version_conversion(const Json &input, const Json &maps, const Json &bindings,
                                 const Json &mode) {
    auto topology = converted_maps(input, maps, bindings, mode);
    auto parameters = converted_roots(input, mode, topology.at("status") == "resolved");
    const bool resolved =
        topology.at("status") == "resolved" && parameters.at("status") == "resolved";
    return {{"reader_profile", "bimbase_2025_material_version_conversion"},
            {"scope", "root_parameters_and_map_topology_before_provider_evaluation"},
            {"mode", mode},
            {"status", resolved ? "resolved" : "partial"},
            {"root_parameters", std::move(parameters)},
            {"map_topology", std::move(topology)}};
}
} // namespace p3d
