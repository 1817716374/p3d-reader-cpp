#include "internal.hpp"

namespace p3d {
namespace {
Json initial(const Json &value) {
    return {{"constructor_value", value}, {"value", value}, {"read_status", "not_written"}};
}
Json projection_constructor() {
    Json axes = Json::object();
    for (const auto axis : {"xa", "ya", "za"})
        axes[axis] = {{"value", nullptr}, {"state", "not_initialized_by_constructor"}};
    return {{"scope", "map_local_projection_and_weight_before_version_conversion"},
            {"status", "partial"},
            {"object_state", "present"},
            {"parameters",
             {{"pattern_proj_offset", initial(Json::array({0., 0., 0.}))},
              {"pattern_proj_angles", initial(Json::array({0., 0., 0.}))},
              {"pattern_proj_scale", initial(Json::array({1., 1., 1.}))},
              {"weight", initial(1.)},
              {"linked_option", initial(false)},
              {"origin_uv_pro_matrix_on", initial(false)}}},
            {"matrix",
             {{"status", "not_initialized_by_constructor"},
              {"axes", axes},
              {"storage_axis_order", {"xa", "ya", "za"}},
              {"storage_values", nullptr}}}};
}
bool missing_component(const Json &read) {
    if (!read.contains("components"))
        return false;
    for (const auto &c : read.at("components"))
        if (c.at("status") == "missing")
            return true;
    return false;
}
void apply(Json &target, const Json &read) {
    target["source_read"] = read;
    if (read.at("write_status") == "written") {
        target["value"] = read.at("reader_value");
        target["read_status"] = "written";
    } else if (read.at("write_status") != "not_written" && !missing_component(read)) {
        target["value"] = nullptr;
        target["read_status"] = "unresolved";
    }
}
void classify(Json &state) {
    state["status"] = state.at("matrix").at("status") == "decoded" ? "resolved" : "partial";
    for (const auto &p : state.at("parameters"))
        if (p.at("value").is_null())
            state["status"] = "partial";
}
} // namespace

Json material_projection_input(const Json &map) {
    auto out = projection_constructor();
    const auto branch = map.at("reader_path").at("branch");
    if (branch == "skipped") {
        out["object_state"] = "absent";
        out["status"] = "resolved";
        out["parameters"] = Json::object();
        out["matrix"] = nullptr;
        return out;
    }
    const auto &reads = map.at("numeric_reader").at("parameters");
    auto &p = out["parameters"];
    for (const auto key : {"pattern_proj_offset", "pattern_proj_angles", "pattern_proj_scale",
                           "origin_uv_pro_matrix_on"})
        apply(p[key], reads.at(key));
    const auto &type = map.at("semantics").at("type").at("value");
    if (type == 1)
        apply(p["weight"], reads.at("pattern_weight"));
    else if (type == 2)
        apply(p["weight"], reads.at("bump_map_scale"));
    else if (type.is_null()) {
        p["weight"]["value"] = nullptr;
        p["weight"]["read_status"] = "unresolved";
    }
    // Each vector getter writes its temporary only on complete success. The
    // matrix builder is then called unconditionally, regardless of its switch.
    auto &matrix = out["matrix"];
    matrix["status"] = "decoded";
    Json storage = Json::array();
    for (const auto axis : {"xa", "ya", "za"}) {
        const auto &read = reads.at(std::string("origin_uv_pro_matrix_") + axis);
        auto &value = matrix["axes"][axis];
        value["source_read"] = read;
        if (read.at("write_status") == "written") {
            value["state"] = "decoded";
            value["value"] = read.at("reader_value");
            for (const auto &v : value["value"])
                storage.push_back(v);
        } else {
            value["state"] = read.at("write_status") == "not_written" || missing_component(read)
                                 ? "indeterminate_native_temporary"
                                 : "unresolved_read";
            matrix["status"] = "partial";
        }
    }
    matrix["storage_values"] = matrix["status"] == "decoded" ? storage : Json();
    // +0x88 is a separate linked option; origin_uv_pro_matrix_on is +0xd8.
    // The native copy includes the former and leaves the latter at false.
    if (branch == "single_provider" && type != 1) {
        const auto &layer = map.at("initial_layer_state").at("layers").at(0);
        const auto &option = layer.at("option_bit_2");
        if (option.contains("source_read")) {
            const auto &read = option.at("source_read");
            const auto &link = reads.at("map_link");
            p["linked_option"]["source_read"] = read;
            p["linked_option"]["source_link_read"] = link;
            const bool failed = read.at("status") == "missing" ||
                                read.value("conversion", Json::object()).value("status", Json()) ==
                                    "no_integer_assignment";
            if (!failed) {
                const bool link_known = link.at("write_status") == "written" ||
                                        link.at("write_status") == "not_written";
                const bool no_link =
                    link.at("write_status") == "not_written" ||
                    (link.at("write_status") == "written" && link.at("value") == 0);
                if (link_known && no_link) {
                    // The branch uses the original link, before self-link clearing.
                } else if (link_known && read.at("status") == "decoded") {
                    p["linked_option"]["value"] = bool(read.at("value").get<std::uint32_t>() & 1);
                    p["linked_option"]["read_status"] = "written";
                } else {
                    p["linked_option"]["value"] = nullptr;
                    p["linked_option"]["read_status"] = "unresolved";
                }
            }
        }
    } else if (branch != "single_provider" && branch != "layer_collection") {
        p["linked_option"]["value"] = nullptr;
        p["linked_option"]["read_status"] = "unresolved";
    }
    classify(out);
    return out;
}

Json material_projection_states(const Json &maps, const Json &topology) {
    Json out = {{"scope", "map_local_projection_and_weight_after_version_conversion"},
                {"status", "resolved"},
                {"objects", Json::array()}};
    if (topology.at("status") != "resolved") {
        out["status"] = "partial";
        out["reason"] = "unresolved_version_conversion";
        return out;
    }
    for (const auto &object : topology.at("objects")) {
        auto state = projection_constructor();
        if (object.at("origin") == "xml_map") {
            const auto &source = maps.at(object.at("source_map_index").get<std::size_t>());
            if (!source.contains("initial_projection_state")) {
                out["status"] = "partial";
                out["reason"] = "missing_initial_projection_state";
                out["objects"] = Json::array();
                return out;
            }
            state = source.at("initial_projection_state");
        } else if (object.contains("copied_settings_from_object_id")) {
            const auto id = object.at("copied_settings_from_object_id").get<std::size_t>();
            const auto &source = out["objects"].at(id);
            for (const auto key : {"pattern_proj_offset", "pattern_proj_angles",
                                   "pattern_proj_scale", "weight", "linked_option"})
                state["parameters"][key] = source.at("parameters").at(key);
            state["copied_from_object_id"] = id;
        }
        state["scope"] = out["scope"];
        state["object_id"] = object.at("object_id");
        classify(state);
        if (state.at("status") != "resolved")
            out["status"] = "partial";
        out["objects"].push_back(std::move(state));
    }
    return out;
}
} // namespace p3d
