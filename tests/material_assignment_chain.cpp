#include <p3d/material_assignment_table.hpp>
#include <stdexcept>
unsigned material_assignment_chain_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        if (!ok)
            throw std::runtime_error(why);
    };
    auto table = [](const std::string &layer, unsigned id = 9, const std::string &name = "Paint") {
        return decode_native_material_assignment_table(
            "<table><assignments><a id='" + std::to_string(id) + "' materialName='" + name +
            "'><l cMsk0='1'>" + layer + "</l></a></assignments></table>");
    };
    NativeAssignmentQuery query;
    query.layer_name = u"x";
    query.color = 0;
    query.text.uppercase = [](char16_t c) -> std::optional<char16_t> {
        return c >= u'a' && c <= u'z' ? char16_t(c - 32) : c;
    };
    NativeAssignmentModelChainOptions options;
    options.use_master_material = false;
    options.ancestry_complete = true;
    auto scope = [&](const std::string &key, std::optional<Json> value) {
        NativeAssignmentModelScope out;
        out.source_key = key;
        out.table_known = true;
        out.table = std::move(value);
        return out;
    };
    std::vector<NativeAssignmentModelScope> scopes = {scope("child", table("other")),
                                                      scope("parent", table("x"))};
    auto result = lookup_native_material_assignment_model_chain(scopes, query, options);
    check(result["status"] == "matched" && result["scope_index"] == 1 &&
              result["visited"].size() == 2,
          "native model ancestry continues only after a table rule miss");
    scopes[0].table = table("*");
    result = lookup_native_material_assignment_model_chain(scopes, query, options);
    check(result["scope_index"] == 0 && result["rule"]["pass"] == "pattern",
          "child pattern rule wins before any exact rule in a parent table");
    options.use_master_material = true;
    result = lookup_native_material_assignment_model_chain(scopes, query, options);
    check(result["scope_index"] == 1 && result["visited"].size() == 1,
          "master material setting starts at the actual terminal native model");
    scopes[0].table_known = false;
    check(lookup_native_material_assignment_model_chain(scopes, query, options)["status"] ==
              "matched",
          "master mode does not inspect an irrelevant child table");
    options.ancestry_complete = false;
    check(lookup_native_material_assignment_model_chain(scopes, query, options)["status"] ==
              "unresolved",
          "master mode never treats the last known ancestor as a confirmed root");
    options.use_master_material = false;
    check(lookup_native_material_assignment_model_chain(scopes, query, options)["status"] ==
              "unresolved",
          "unknown child table cannot be skipped in favor of a known parent match");
    scopes[0].table_known = true;
    scopes[0].table = table("x");
    check(lookup_native_material_assignment_model_chain(scopes, query, options)["scope_index"] == 0,
          "local rule match needs no unused remaining ancestry");
    scopes.resize(1);
    scopes[0].table = table("other");
    check(lookup_native_material_assignment_model_chain(scopes, query, options)["status"] ==
              "unresolved",
          "incomplete ancestry miss is not a confirmed absence of assignment");
    options.ancestry_complete = true;
    check(lookup_native_material_assignment_model_chain(scopes, query, options)["status"] ==
              "not_found",
          "complete ancestry with no matching rule is a known miss");
    scopes[0].table.reset();
    check(lookup_native_material_assignment_model_chain(
              scopes, query, options)["visited"][0]["status"] == "table_absent",
          "known absent model table is distinct from unknown table state");
    options.use_master_material.reset();
    check(lookup_native_material_assignment_model_chain(scopes, query, options)["status"] ==
              "not_found",
          "one complete scope has the same result independently of the master setting");
    scopes.push_back(scope("parent", table("x")));
    check(lookup_native_material_assignment_model_chain(scopes, query, options)["status"] ==
              "unresolved",
          "unknown master setting does not default to child-first search");
    options.use_master_material = false;
    scopes[0].table = table("other");
    options.max_scopes = 1;
    check(lookup_native_material_assignment_model_chain(scopes, query, options)["status"] ==
              "unresolved",
          "model table visit budget does not publish an unchecked later match");
    options.use_master_material = true;
    check(lookup_native_material_assignment_model_chain(scopes, query, options)["scope_index"] == 1,
          "master query charges only its visited prepared scope");
    options.max_scopes = 0;
    check(lookup_native_material_assignment_model_chain(scopes, query, options)["status"] ==
              "unresolved",
          "zero table visit budget is rejected explicitly");
    options.max_scopes = 100;
    options.use_master_material = false;
    scopes[0] = scope("same", std::nullopt);
    scopes[1].source_key = "same";
    check(lookup_native_material_assignment_model_chain(scopes, query, options)["scope_index"] == 1,
          "equal caller labels never merge independent native model scopes");
    scopes[0].table = Json{{"status", "unresolved"}};
    check(lookup_native_material_assignment_model_chain(scopes, query, options)["status"] ==
              "unresolved",
          "unresolved normalized table blocks fallback");
    const Json current = {{"kind", "current_resource_context"}};
    const Json catalog = {
        {"status", "resolved"},
        {"current_context", current},
        {"entries",
         {{{"id", 9}, {"name", "Paint"}, {"resource", {{"primary_context", current}}}}}}};
    std::vector<unsigned> calls;
    scopes = {scope("child", table("x")), scope("parent", table("x"))};
    for (unsigned i = 0; i < scopes.size(); ++i) {
        scopes[i].registered_catalog = catalog;
        scopes[i].material.id.load = [&, i](std::size_t) {
            calls.push_back(i);
            return GraphicsMaterialResult{true, 100 + i};
        };
    }
    result = resolve_native_assignment_material_model_chain(scopes, query, options);
    check(result["status"] == "loaded" && result["material_index"] == 100 &&
              calls == std::vector<unsigned>{0},
          "selected model rule uses that model's resource loader exactly once");
    calls.clear();
    scopes[0].table.reset();
    result = resolve_native_assignment_material_model_chain(scopes, query, options);
    check(result["material_index"] == 101 && calls == std::vector<unsigned>{1},
          "parent rule loads from its own catalog context rather than the child context");
    calls.clear();
    scopes[0].table = table("x");
    scopes[0].material.id.load = [&](std::size_t) {
        calls.push_back(0);
        return GraphicsMaterialResult{true, {}};
    };
    result = resolve_native_assignment_material_model_chain(scopes, query, options);
    check(result["status"] == "load_failed" && result["native_diagnostic_code"] == 4 &&
              calls == std::vector<unsigned>{0} && result["rule_selection"]["visited"].size() == 1,
          "provider failure after a rule match does not resume parent table traversal");
    calls.clear();
    scopes[0].material.id.load = [&](std::size_t) {
        calls.push_back(0);
        return GraphicsMaterialResult{false, {}};
    };
    check(resolve_native_assignment_material_model_chain(scopes, query, options)["status"] ==
                  "unresolved" &&
              calls == std::vector<unsigned>{0},
          "unknown selected provider is not treated as a rule miss");
    calls.clear();
    scopes[0].table = table("x", 99, "");
    result = resolve_native_assignment_material_model_chain(scopes, query, options);
    check(result["status"] == "not_found" && calls.empty() &&
              result["rule_selection"]["visited"].size() == 1,
          "known ID and name misses within selected rule do not search a parent table");
    scopes[0].table = table("other");
    scopes[1].table = table("other");
    check(resolve_native_assignment_material_model_chain(scopes, query, options)["status"] ==
                  "rule_not_found" &&
              calls.empty(),
          "complete rule miss never accesses material providers");
    scopes[0].table = table("X");
    scopes.resize(1);
    unsigned comparisons = 0;
    query.text.uppercase = [&](char16_t c) -> std::optional<char16_t> {
        ++comparisons;
        return c >= u'a' && c <= u'z' ? char16_t(c - 32) : c;
    };
    lookup_native_material_assignment_model_chain(scopes, query, options);
    const auto once = comparisons;
    comparisons = 0;
    scopes[0].material.id.load = [](std::size_t) { return GraphicsMaterialResult{true, 7}; };
    result = resolve_native_assignment_material_model_chain(scopes, query, options);
    check(result["status"] == "loaded" && once > 0 && comparisons == once,
          "rule loading reuses selection without repeating source-locale pattern evaluation");
    options.ancestry_complete = true;
    check(resolve_native_assignment_material_model_chain({}, query, options)["status"] ==
              "rule_not_found",
          "known empty model ancestry cannot manufacture a material");
    return checks;
}
