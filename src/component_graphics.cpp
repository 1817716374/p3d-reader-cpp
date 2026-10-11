#include "internal.hpp"
#include <p3d/component_graphics.hpp>

namespace p3d {
Json resolve_stored_component_graphics(const Json &properties, const Json &instance) {
    Json out = {
        {"status", "not_evaluated"},
        {"scope", "stored_component_graphics_source_selection"},
        {"callback_assumption", "builtin_extract_component_graphics_and_get_graphics"},
        {"project_and_model_assumption", "valid_requested_model_and_active_project"},
        {"source_model_resolution_assumption", "graphics_factories_resolve_required_models"},
        {"editing_assumption", "input_is_not_current_edit_key"},
        {"property_state_assumption", "saved_noumenon_without_runtime_mutation"},
        {"source_association", "caller_supplied_same_component"},
        {"allocation_assumption", "successful"},
        {"service_return_assumption", "normal"},
        {"owning_element_part_mapping_status", "not_established"},
        {"transformed_geometry", "not_evaluated"},
        {"entries", Json::array()},
        {"stages", Json::array()}};
    try {
        require(properties.at("scope") == "stored_component_properties" &&
                    properties.at("status") == "decoded",
                "complete_component_properties_required");
        auto select = [&](const Json &sequence, const char *source, const char *path) {
            require(sequence.at("status") == "rebuilt", "source_sequence_not_established");
            require(sequence.at("entries").is_array(), "source_entries_must_be_array");
            out["stages"].push_back(
                {{"source", source}, {"entry_count", sequence.at("entries").size()}});
            if (sequence.at("entries").empty())
                return false;
            out["status"] = "selected";
            out["graphics_source"] = source;
            out["source_graphics_path"] = path;
            out["entries"] = sequence.at("entries");
            return true;
        };
        const auto &extraction = properties.at("parametric_extraction_input");
        if (extraction.value("branch", "") == "agent") {
            const auto &agent = properties.at("agent_graphics_input");
            require(agent.at("status") == "rebuilt", "agent_graphics_not_established");
            if (select(agent, "parametric_extraction", "properties.agent_graphics_input.graphics"))
                return out;
        } else {
            require(extraction.at("status") == "empty", "parametric_extraction_not_established");
            out["stages"].push_back({{"source", "parametric_extraction"}, {"entry_count", 0}});
        }
        const auto &saved = properties.at("component_graphics_fallback_input");
        require(saved.at("status") == "rebuilt", "saved_graphics_not_established");
        if (select(saved, "saved_component_graphics",
                   "properties.component_graphics_input.graphics"))
            return out;
        const auto mode = saved.at("destination_transform").get<std::string>();
        require(mode == "disabled" || mode == "identity_enabled",
                "cache_target_transform_not_established");
        out["cache_destination_transform"] = mode;
        const auto &cache =
            instance.at("cache_rebuild").at("with_project").at("destination_variants").at(mode);
        require(cache.at("status") == "rebuilt", "cache_sequence_not_established");
        if (select(cache, "instance_cache", "instance.instances"))
            return out;
        out["status"] = "default_handler_required";
        out["next_stage"] = "default_handler";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
