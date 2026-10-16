#include "scene_layers.hpp"
namespace p3d {
NativeSceneLayerCatalog::NativeSceneLayerCatalog(const Json &native, const Json &catalog)
    : data({{"scope", "saved_layers_before_draw_style_selection"},
            {"definitions", Json::array()},
            {"table_catalog", catalog}}) {
    std::map<std::size_t, std::size_t> definitions;
    for (std::size_t i = 0; i < native.size(); ++i) {
        const auto &record = native.at(i);
        if (!record.contains("layer_definition"))
            continue;
        definitions[i] = data["definitions"].size();
        data["definitions"].push_back({{"native_record_index", i},
                                       {"stream", record.at("stream")},
                                       {"offset", record.at("offset")},
                                       {"record_id", record.at("id")},
                                       {"layer", record.at("layer_definition")}});
    }
    const auto &source_tables = catalog.at("tables");
    for (std::size_t i = 0; i < source_tables.size(); ++i) {
        auto &table = tables[i];
        try {
            const auto &source = source_tables.at(i);
            require(source.at("membership_status") == "resolved", "layer_table_membership_unknown");
            for (const auto &index : source.at("member_record_indices")) {
                const auto ni = index.get<std::size_t>();
                const auto found = definitions.find(ni);
                require(found != definitions.end(), "layer_table_definition_missing");
                const auto &layer = data.at("definitions").at(found->second).at("layer");
                require(layer.value("status", "") == "decoded" && !layer.contains("decode_error"),
                        "layer_definition_decode_unknown");
                table.ids[layer.at("layer_id").get<std::uint32_t>()].push_back(found->second);
            }
            table.known = true;
        } catch (const std::exception &e) {
            table.reason = e.what();
        }
    }
    const auto &refs = catalog.at("model_references");
    for (std::size_t i = 0; i < refs.size(); ++i)
        models[refs.at(i).at("model_id").get<std::string>()].push_back(i);
}
Json NativeSceneLayerCatalog::bind(const Json &model, const Json &header) const {
    Json out = {{"status", "not_evaluated"},
                {"scope", "stored_entity_header_in_initial_physical_model_layer_table"},
                {"draw_style_selection", "not_evaluated"},
                {"material_loading", "not_evaluated"}};
    try {
        require(header.is_object() && header.value("status", "") == "decoded",
                "stored_graphic_header_unavailable");
        out["layer_id"] = header.at("layer_id");
        require(header.at("entity_getter_policy") == "stored_header",
                "entity_first_child_layer_query_required");
        const auto id = header.at("layer_id").get<std::uint32_t>();
        require(id != UINT32_MAX && id != UINT32_MAX - 1, "layer_style_context_required");
        const auto found = models.find(model.dump());
        require(found != models.end() && found->second.size() == 1,
                "physical_model_header_missing_or_ambiguous");
        const auto ri = found->second.front();
        const auto &selection =
            data.at("table_catalog").at("model_references").at(ri).at("initial_layer_cache");
        out["model_reference_index"] = ri;
        require(selection.at("status") == "resolved", "initial_model_layer_table_unresolved");
        const auto ti = selection.at("table_index").get<std::size_t>();
        out["table_index"] = ti;
        const auto table = tables.find(ti);
        require(table != tables.end() && table->second.known,
                table == tables.end() ? "layer_table_index_missing" : table->second.reason);
        const auto candidates = table->second.ids.find(id);
        if (candidates == table->second.ids.end()) {
            out["status"] = "missing";
            out["candidate_definition_indices"] = Json::array();
            return out;
        }
        const auto selected = candidates->second.back();
        out.update({{"status", "resolved"},
                    {"selection", "last_saved_layer_id"},
                    {"candidate_definition_indices", candidates->second},
                    {"layer_definition_index", selected},
                    {"native_record_index",
                     data.at("definitions").at(selected).at("native_record_index")}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
