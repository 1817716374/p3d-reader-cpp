#include "internal.hpp"
#include "material_assignment_rule.hpp"
namespace p3d {
Json lookup_native_material_assignment_model_chain(
    const std::vector<NativeAssignmentModelScope> &scopes, const NativeAssignmentQuery &query,
    const NativeAssignmentModelChainOptions &options) {
    Json out = {{"status", "unresolved"},
                {"scope", "ordinary_model_ancestry_rules_before_layer_fallback"},
                {"model_context_selection", "caller_supplied_native_ancestry"},
                {"context_cache", "not_evaluated"},
                {"material_loading", "not_evaluated"},
                {"ancestry_complete", options.ancestry_complete},
                {"visited", Json::array()}};
    try {
        require(options.max_scopes > 0, "assignment_model_scope_budget_required");
        bool master = false;
        if (options.use_master_material)
            master = *options.use_master_material;
        else
            require(options.ancestry_complete && scopes.size() <= 1,
                    "master_material_setting_required");
        out["use_master_material"] = options.use_master_material ? Json(master) : Json();
        if (master)
            require(options.ancestry_complete, "complete_master_ancestry_required");
        const auto first = master && !scopes.empty() ? scopes.size() - 1 : 0;
        for (std::size_t i = first; i < scopes.size(); ++i) {
            require(out["visited"].size() < options.max_scopes,
                    "assignment_model_scope_budget_exceeded");
            const auto &scope = scopes[i];
            Json step = {
                {"scope_index", i}, {"source_key", scope.source_key}, {"status", "unresolved"}};
            out["visited"].push_back(step);
            require(scope.table_known, "assignment_model_table_unresolved");
            if (!scope.table) {
                out["visited"].back()["status"] = "table_absent";
                continue;
            }
            auto rule = lookup_native_material_assignment(*scope.table, query);
            out["visited"].back()["rule"] = rule;
            out["visited"].back()["status"] = rule.at("status");
            if (rule.at("status") == "matched") {
                out.update({{"status", "matched"},
                            {"scope_index", i},
                            {"source_key", scope.source_key},
                            {"rule", std::move(rule)}});
                return out;
            }
            require(rule.at("status") == "not_found", "assignment_model_rule_unresolved");
        }
        require(options.ancestry_complete, "remaining_native_ancestry_unresolved");
        out["status"] = "not_found";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json resolve_native_assignment_material_model_chain(
    const std::vector<NativeAssignmentModelScope> &scopes, const NativeAssignmentQuery &query,
    const NativeAssignmentModelChainOptions &options) {
    Json out = {{"status", "unresolved"},
                {"scope", "ordinary_model_ancestry_loading_before_layer_fallback"},
                {"draw_material_status", "not_evaluated"}};
    auto selection = lookup_native_material_assignment_model_chain(scopes, query, options);
    out["rule_selection"] = selection;
    if (selection.at("status") == "not_found") {
        out["status"] = "rule_not_found";
        return out;
    }
    if (selection.at("status") != "matched")
        return out;
    const auto index = selection.at("scope_index").get<std::size_t>();
    const auto &scope = scopes.at(index);
    auto loaded = detail::resolve_selected_assignment_rule(
        selection.at("rule"), scope.registered_catalog, scope.material);
    out["status"] = loaded.at("status");
    for (const auto *key : {"material_index", "native_diagnostic_code", "reason"})
        if (loaded.contains(key))
            out[key] = loaded.at(key);
    out["material_loading"] = std::move(loaded);
    return out;
}
} // namespace p3d
