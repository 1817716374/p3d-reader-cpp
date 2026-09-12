#include "internal.hpp"

unsigned material_catalog_registration_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    Json selection = {{"status", "selected"}, {"table", {{"members", Json::array()}}}};
    Json sources = Json::array();
    auto add = [&](std::uint64_t id, const std::string &name,
                   const std::string &resource = "Palette") {
        const auto i = sources.size();
        selection["table"]["members"].push_back(
            {{"native_record_index", i},
             {"input_occurrence_index", 100 + i},
             {"assigned_id", id},
             {"material_attribute",
              {{"status", "selected"}, {"attachment_index", i}, {"source_ordinal", 0}}}});
        sources.push_back({{"native_record_index", i},
                           {"catalog_name", {{"reader", {{"status", "success"}, {"value", name}}}}},
                           {"resource_reference",
                            {{"interpretation",
                              {{"status", "decoded"},
                               {"primary_context", "current_resource_context"},
                               {"descriptor_mode", 0},
                               {"member_name", resource}}}}}});
    };
    auto reset = [&] {
        selection["table"]["members"] = Json::array();
        sources = Json::array();
    };
    auto run = [&](const MaterialCatalogOptions &o = MaterialCatalogOptions{}) {
        return native_material_catalog_registration(selection, sources, o);
    };
    add(1, "Steel");
    add(2, "sTEEL");
    add(1, "Concrete", "Different");
    add(3, "Stone");
    auto r = run();
    check(r["status"] == "resolved" && r["entries"].size() == 2 && r["entries"][1]["id"] == 3,
          "registration retains accepted input order and never replaces earlier entries");
    check(r["input"][1]["action"] == "rejected_duplicate_name_resource" &&
              r["input"][1]["existing_entry_index"] == 0,
          "case-insensitive name and member duplicates reject even with distinct native IDs");
    check(r["input"][2]["action"] == "rejected_duplicate_id_context",
          "ID/context conflicts are checked after name/resource conflicts");
    reset();
    add(1, "A");
    add(1, "a");
    check(run()["input"][1]["action"] == "rejected_duplicate_name_resource",
          "name/resource conflict wins when both rejection conditions apply");
    reset();
    add(0, "");
    add(1, "");
    add(0, "");
    add(UINT64_MAX, "");
    add(UINT64_MAX, "");
    r = run();
    check(r["entries"].size() == 4 && r["input"][2]["action"] == "rejected_duplicate_id_context",
          "empty names bypass name lookup, zero IDs remain indexed and sentinel IDs bypass ID "
          "lookup");
    reset();
    add(UINT64_MAX, "A");
    add(UINT64_MAX, "A");
    check(run()["entries"].size() == 1,
          "sentinel ID does not bypass name/resource duplicate detection");
    reset();
    add(1, "A");
    add(2, "A");
    sources[1]["resource_reference"]["interpretation"]["descriptor_mode"] = 1;
    check(run()["entries"].size() == 2,
          "descriptor mode participates in resource duplicate identity");
    reset();
    add(1, "A");
    add(2, "A", "Other");
    check(run()["entries"].size() == 2,
          "different resource member names preserve same-named entries");
    reset();
    add(1, "混凝土");
    add(2, "混凝土");
    add(3, "钢");
    check(run()["entries"].size() == 2,
          "C-locale equality preserves non-ASCII source text and exact duplicates");
    MaterialCatalogOptions exact;
    exact.case_equal = [](const auto &a, const auto &b) { return a == b; };
    reset();
    add(1, "A");
    add(2, "a");
    check(run(exact)["entries"].size() == 2 && run(exact)["string_comparison"] == "caller_supplied",
          "caller comparison changes registration without altering source names");
    reset();
    add(1, "missing");
    add(2, "badresource");
    add(3, "noattribute");
    add(4, "valid");
    sources[0]["catalog_name"]["reader"]["status"] = "missing";
    sources[1]["resource_reference"]["interpretation"]["status"] = "rejected_extension";
    selection["table"]["members"][2]["material_attribute"]["status"] = "absent";
    r = run();
    check(r["entries"].size() == 1 && r["entries"][0]["id"] == 4 && r["input"].size() == 4,
          "failed name/resource readers and missing attributes skip without terminating later "
          "members");
    reset();
    add(1, "A");
    add(2, "B");
    add(3, "C");
    sources[1]["resource_reference"]["interpretation"]["primary_context"] = nullptr;
    r = run();
    check(r["status"] == "partial" && r["entries"].size() == 1 && r["unvisited_member_count"] == 1,
          "unknown descriptor stops later registration instead of guessing duplicates");
    MaterialCatalogOptions resolved;
    resolved.resolve_resource = [](const Json &,
                                   const Json &) -> std::optional<MaterialCatalogResource> {
        return MaterialCatalogResource{"Palette", 0, std::nullopt};
    };
    check(run(resolved)["entries"].size() == 3,
          "caller-resolved descriptors resume complete registration");
    reset();
    add(1, "A");
    add(2, "A");
    sources[1]["resource_reference"]["interpretation"]["primary_context"] = nullptr;
    resolved.resolve_resource = [](const Json &,
                                   const Json &) -> std::optional<MaterialCatalogResource> {
        return MaterialCatalogResource{"Palette", 0, MaterialCatalogContext{"external", "file"}};
    };
    check(run(resolved)["status"] == "partial",
          "symbolic current context cannot be assumed unequal to an external context");
    resolved.current_context = MaterialCatalogContext{"current", "FILE"};
    check(run(resolved)["input"][1]["action"] == "rejected_duplicate_name_resource",
          "name/resource comparison uses fallback comparison keys, not ID-index keys");
    sources[1]["catalog_name"]["reader"]["value"] = "B";
    selection["table"]["members"][1]["assigned_id"] = 1;
    resolved.current_context = MaterialCatalogContext{"EXTERNAL", "another"};
    check(run(resolved)["input"][1]["action"] == "rejected_duplicate_id_context",
          "ID/context comparison uses index keys even when context comparison keys differ");
    resolved.current_context = MaterialCatalogContext{"other", "another"};
    check(run(resolved)["entries"].size() == 2, "same IDs in unequal context keys remain separate");
    resolved.resolve_resource = [](const Json &,
                                   const Json &) -> std::optional<MaterialCatalogResource> {
        return MaterialCatalogResource{std::string("bad\0name", 8), 0, std::nullopt};
    };
    check(run(resolved)["status"] == "partial",
          "callback embedded NUL does not create a misleading identity");
    check(native_material_catalog_registration({{"status", "absent"}}, sources, {})["status"] ==
              "absent",
          "absent table and unresolved table remain distinguishable");
    check(native_material_catalog_registration({{"status", "unresolved"}}, sources, {})["status"] ==
              "unresolved",
          "unresolved table cannot imply empty successful registration");

    // The native table iterator reads name/resource linkages from every accepted
    // direct member; source element type 49 is not a registration precondition.
    Bytes body(36);
    body[4] = 37;
    body[8] = 16;
    body[12] = 16;
    auto rows = parse_native(body);
    const std::vector<std::size_t> member_indices = {0};
    check(native_material_catalog_records(rows).empty() &&
              native_material_catalog_records(rows, &member_indices).size() == 1,
          "table member extraction is not restricted to the source catalog-record type");
    return checks;
}
