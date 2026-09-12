#include "internal.hpp"

namespace p3d {
Json material_replicator_input(const Json &package) {
    Json out = {{"scope", "M541_reader_output_from_empty_attachment"},
                {"status", "resolved"},
                {"object_state", "absent"},
                {"selected_child_index", nullptr},
                {"parameters", Json::object()},
                {"copy_policy",
                 {{"scope", "material_layer_copy_into_new_layer"},
                  {"replicator_object", "new_object"},
                  {"nested_settings", "new_object"},
                  {"parameter_values", "copy_all_fourteen"},
                  {"absent_source", "clear_destination_attachment"}}}};
    const auto gate = package.at("reader_applicability").at("status");
    if (gate == "skipped")
        return out;
    if (gate != "read") {
        out["status"] = "partial";
        out["object_state"] = "unresolved";
        return out;
    }
    const Json *selected = nullptr;
    for (const auto &entry : package.at("entries"))
        if (entry.at("selection") == "selected") {
            selected = &entry;
            break;
        }
    if (!selected)
        return out;
    out["object_state"] = "constructed";
    out["selected_child_index"] = selected->at("child_index");
    struct Field {
        const char *name;
        char kind;
        double initial;
    };
    static const Field fields[] = {
        {"M542", 'd', 25},    {"M543", 'd', 50},  {"M544", 'd', 80},  {"M545", 'd', 0},
        {"M546", 'd', 0},     {"M547", 'd', 0},   {"M548", 'b', 0},   {"M549", 'b', 0},
        {"M550", 'd', 2000},  {"M551", 'd', 500}, {"M552", 'd', 100}, {"M553", 'd', 100},
        {"M554", 'u', 40000}, {"M555", 'u', 100},
    };
    for (const auto &field : fields) {
        const auto &read = selected->at("parameters").at(field.name);
        const Json initial = field.kind == 'b'   ? Json(bool(field.initial))
                             : field.kind == 'u' ? Json(std::uint32_t(field.initial))
                                                 : Json(field.initial);
        Json value = {{"constructor_value", initial},
                      {"value", nullptr},
                      {"source_read", read},
                      {"read_status", "unresolved"}};
        if (read.at("status") == "decoded") {
            value["value"] = field.kind == 'b' ? Json(read.at("value") != 0) : read.at("value");
            value["read_status"] = "written";
        } else if (material_xml_numeric_failed(read)) {
            value["value"] = initial;
            value["read_status"] = "not_written";
        }
        if (value["read_status"] == "unresolved")
            out["status"] = "partial";
        out["parameters"][field.name] = std::move(value);
    }
    return out;
}

Json material_replicator_copies(const Json &map) {
    Json out = {{"scope", "M541_attachments_copied_with_local_layers"},
                {"status", "resolved"},
                {"entries", Json::array()}};
    auto add = [&](const Json &package, const Json &child_index) {
        const auto &state = package.at("initial_state");
        if (state.at("object_state") == "absent")
            return;
        if (state.at("status") != "resolved")
            out["status"] = "partial";
        out["entries"].push_back({{"source_layer_child_index", child_index},
                                  {"ownership", state.at("object_state") == "constructed"
                                                    ? Json("new_replicator_and_nested_settings")
                                                    : Json()},
                                  {"state", state}});
    };
    const auto branch = map.at("reader_path").at("branch");
    if (branch == "single_provider")
        add(map.at("replicators"), nullptr);
    else if (branch == "layer_collection") {
        for (const auto &layer : map.at("texture_layers").at("entries")) {
            const auto gate = layer.at("reader_applicability").at("status");
            if (gate == "skipped")
                continue;
            if (gate != "read") {
                out["status"] = "partial";
                continue;
            }
            add(layer.at("replicators"), layer.at("child_index"));
        }
    } else
        out["status"] = "partial";
    return out;
}
} // namespace p3d
