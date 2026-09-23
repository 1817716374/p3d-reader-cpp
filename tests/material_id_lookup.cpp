#include "internal.hpp"
#include <p3d/entity_material.hpp>
#include <p3d/material_lookup.hpp>

unsigned material_id_lookup_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) {
        ++checks;
        require(value, message);
    };
    const Json current = {{"kind", "current_resource_context"}};
    auto key = [](const std::string &s) {
        return Json{{"kind", "explicit_context_keys"},
                    {"index_key", s},
                    {"comparison_key", "not the ID key"}};
    };
    auto entry = [&](std::uint64_t id, const Json &resource) {
        return Json{{"id", id},
                    {"name", "Same name"},
                    {"resource",
                     {{"primary_context", resource},
                      {"member_name", "same member"},
                      {"descriptor_mode", 0}}}};
    };
    auto catalog = [&](const Json &entries, const Json &resource) {
        return Json{{"status", "resolved"}, {"current_context", resource}, {"entries", entries}};
    };
    const auto source = catalog(
        Json::array({entry(7, current), entry(7, key("Other")), entry(8, current)}), key("Main"));
    const auto original = source.dump();
    NativeMaterialIdContext context;
    std::vector<std::size_t> loads;
    unsigned comparisons = 0;
    // Explicit fixture locale; not a library assumption about the source process.
    context.compare = [&](std::u16string a, std::u16string b) -> std::optional<int> {
        ++comparisons;
        for (auto *s : {&a, &b})
            for (auto &c : *s)
                if (c >= u'A' && c <= u'Z')
                    c += 32;
        return a.compare(b);
    };
    context.load = [&](std::size_t i) {
        loads.push_back(i);
        return GraphicsMaterialResult{true, 100 + i};
    };
    auto result = lookup_native_material_id(7, source, context);
    check(result.at("status") == "loaded" && result.at("material_index") == 100 &&
              loads == std::vector<std::size_t>{0},
          "ID query binds the indexed source entry in current resource");
    check(result.at("selected_catalog_index") == 0 &&
              result.at("id_candidate_indices") == Json::array({0, 1}) &&
              result.at("matching_catalog_indices") == Json::array({0}) && comparisons == 1,
          "same numeric IDs remain separate resource candidates");
    loads.clear();
    context.resource_index_key = u"OTHER";
    result = lookup_native_material_id(7, source, context);
    check(result.at("material_index") == 101 && loads == std::vector<std::size_t>{1},
          "source resource index key, not material name/member/comparison key, selects material");
    context.resource_index_key = std::u16string{u'O', u't', u'h', u'e', u'r', 0, u'X'};
    check(lookup_native_material_id(7, source, context).at("selected_catalog_index") == 1,
          "query context uses native first-NUL boundary");
    context.resource_index_key = u"missing";
    loads.clear();
    result = lookup_native_material_id(7, source, context);
    check(result.at("status") == "not_found" && result.at("native_diagnostic_code") == 3 &&
              loads.empty(),
          "same ID in other contexts is not a match");

    const auto symbolic = catalog(Json::array({entry(0, current)}), current);
    NativeMaterialIdContext simple;
    simple.load = [&](std::size_t i) {
        check(i == 0, "symbolic current entry index");
        return GraphicsMaterialResult{true, 0};
    };
    result = lookup_native_material_id(0, symbolic, simple);
    check(result.at("status") == "loaded" && result.at("material_index") == 0 &&
              result.at("comparisons")[0].at("method") == "same_current_resource_context",
          "zero ID and material index zero remain valid; same symbolic context needs no fabricated "
          "path");
    simple.resource_index_key = u"";
    result = lookup_native_material_id(0, symbolic, simple);
    check(result.at("status") == "unresolved" && result.at("loads").empty(),
          "explicit empty resource key differs from unknown symbolic current context");
    const auto explicit_empty = catalog(Json::array({entry(0, current)}), key(""));
    check(lookup_native_material_id(0, explicit_empty, simple).at("status") == "loaded",
          "known empty context string is a real index key");
    simple.resource_index_key = u"Main";
    const auto explicit_main = catalog(Json::array({entry(0, key("Main"))}), current);
    check(lookup_native_material_id(0, explicit_main, simple).at("status") == "loaded",
          "explicit matching keys do not depend on unused current context or comparator");
    simple.resource_index_key = u"main";
    result = lookup_native_material_id(0, explicit_main, simple);
    check(result.at("status") == "unresolved" &&
              result.at("reason") == "native_resource_key_comparator_required",
          "different concrete context keys require confirmed source comparison");
    simple.compare = [](const auto &, const auto &) -> std::optional<int> { return {}; };
    check(lookup_native_material_id(0, explicit_main, simple).at("status") == "unresolved",
          "unknown key comparison cannot become a miss");

    NativeMaterialIdContext unloaded;
    result = lookup_native_material_id(0, symbolic, unloaded);
    check(result.at("status") == "unresolved" && result.at("selected_catalog_index") == 0 &&
              result.at("native_diagnostic_code").is_null(),
          "selected catalog declaration alone is not a loaded material");
    unloaded.load = [](std::size_t) { return GraphicsMaterialResult{true, {}}; };
    result = lookup_native_material_id(0, symbolic, unloaded);
    check(result.at("status") == "load_failed" && result.at("native_diagnostic_code") == 4 &&
              result.at("loads").size() == 1,
          "one failed indexed load is distinct from no matching entry");
    unloaded.load = [](std::size_t) { return GraphicsMaterialResult{}; };
    result = lookup_native_material_id(0, symbolic, unloaded);
    check(result.at("status") == "unresolved" && result.at("loads")[0].at("status") == "unresolved",
          "unknown loading result retains exact attempted index");
    unloaded.load = [](std::size_t) { return GraphicsMaterialResult{false, 12}; };
    check(lookup_native_material_id(0, symbolic, unloaded).at("reason") ==
              "unknown_material_has_identity",
          "inconsistent loader result cannot publish material identity");
    unloaded.load = [](std::size_t) -> GraphicsMaterialResult {
        throw std::runtime_error("provider error");
    };
    result = lookup_native_material_id(0, symbolic, unloaded);
    check(result.at("status") == "unresolved" && result.at("reason") == "provider error" &&
              result.at("selected_catalog_index") == 0,
          "provider exception preserves selected source without retry");

    result = lookup_native_material_id(UINT64_MAX, Json(), {});
    check(result.at("status") == "invalid_identifier" && result.at("native_diagnostic_code") == 1,
          "ID-only sentinel fails before catalog/context access");
    result = lookup_native_material_id(1, {{"status", "absent"}}, {});
    check(result.at("status") == "not_found" && result.at("native_diagnostic_code") == 3,
          "confirmed absent prepared catalog has no entry");
    check(lookup_native_material_id(1, catalog(Json::array(), Json()), {}).at("status") ==
              "not_found",
          "empty complete catalog does not require resource keys");
    auto partial = symbolic;
    partial["status"] = "partial";
    check(lookup_native_material_id(0, partial, context).at("status") == "unresolved",
          "known prefix of a catalog is not a complete material index");
    for (const auto id : {std::uint64_t(0xfedcba9876543210), UINT64_MAX - 1}) {
        auto full = catalog(Json::array({entry(id, current)}), current);
        simple.resource_index_key.reset();
        check(lookup_native_material_id(id, full, simple).at("status") == "loaded",
              "high-bit material ID is not truncated or treated as signed");
    }
    auto invalid = symbolic;
    for (const auto &id : {Json(-1), Json(0.5), Json("0")}) {
        invalid["entries"][0]["id"] = id;
        check(lookup_native_material_id(0, invalid, context).at("status") == "unresolved",
              "invalid catalog ID cannot hide a candidate");
    }
    loads.clear();
    context.resource_index_key.reset();
    const auto duplicate =
        catalog(Json::array({entry(7, current), entry(7, key("MAIN"))}), key("Main"));
    result = lookup_native_material_id(7, duplicate, context);
    check(result.at("status") == "unresolved" &&
              result.at("reason") == "duplicate_registered_material_id_context" && loads.empty(),
          "contradictory registered ID keys are not repaired by deduplication or first-wins");
    const auto duplicate_sentinel = catalog(
        Json::array({entry(UINT64_MAX, current), entry(UINT64_MAX, current), entry(7, current)}),
        current);
    check(lookup_native_material_id(7, duplicate_sentinel, context).at("selected_catalog_index") ==
              2,
          "unindexed sentinel entries do not renumber the source catalog");
    auto invalid_key = catalog(Json::array({entry(0, key(std::string("A\0B", 3)))}), current);
    simple.resource_index_key = u"A";
    check(lookup_native_material_id(0, invalid_key, simple).at("status") == "unresolved",
          "edited catalog keys cannot change registration by embedded NUL");
    invalid_key["entries"][0]["resource"]["primary_context"]["index_key"] = std::string("\xff");
    check(lookup_native_material_id(0, invalid_key, simple).at("status") == "unresolved",
          "invalid catalog UTF-8 is not an empty key");
    simple.resource_index_key = std::u16string{0x4e2d, 0xd83c, 0xdf32};
    const auto unicode =
        catalog(Json::array({entry(0, key("\xe4\xb8\xad\xf0\x9f\x8c\xb2"))}), current);
    check(lookup_native_material_id(0, unicode, simple).at("status") == "loaded",
          "explicit UTF-8 catalog key maps to full UTF-16 source identity");

    // Use the existing registration pipeline, including rejected members and
    // input ID assignment, to prove the selected index belongs to its output.
    Json selection = {{"status", "selected"}, {"table", {{"members", Json::array()}}}};
    Json sources = Json::array();
    for (unsigned i = 0; i < 3; ++i) {
        selection["table"]["members"].push_back({{"native_record_index", i},
                                                 {"assigned_id", i == 1 ? 7 : 9},
                                                 {"material_attribute", {{"status", "selected"}}}});
        sources.push_back(
            {{"native_record_index", i},
             {"catalog_name",
              {{"reader", {{"status", "success"}, {"value", i == 1 ? "Target" : "Duplicate"}}}}},
             {"resource_reference",
              {{"interpretation",
                {{"status", "decoded"},
                 {"primary_context", "current_resource_context"},
                 {"descriptor_mode", 0},
                 {"member_name", "Palette"}}}}}});
    }
    const auto registered = native_material_catalog_registration(selection, sources, {});
    check(registered.at("entries").size() == 2,
          "registration fixture contains a rejected duplicate member");
    result = lookup_native_material_id(7, registered, context);
    check(result.at("selected_catalog_index") == 1 && result.at("material_index") == 101 &&
              registered.at("entries")[1].at("member").at("native_record_index") == 1,
          "registration source identity reaches loaded catalog index");

    // Both native entity ID routes can use the new prepared-index query. A
    // provider miss on the primary route must issue the second route, not name.
    Bytes id_payload{0x0e, 0, 1, 0, 7, 0, 0, 0, 0, 0, 0, 0};
    Json record = {
        {"links", Json::array({Json{
                      {"header", 0x1007}, {"app", 0x41}, {"payload", rawbytes(id_payload)}}})}};
    NativeEntityMaterialContext entity;
    entity.entity_model_resource_available = true;
    unsigned primary_calls = 0, fallback_calls = 0;
    entity.lookup_id = [&](std::uint64_t id, NativeEntityMaterialIdQuery route) {
        NativeMaterialIdContext lookup;
        const bool primary = route == NativeEntityMaterialIdQuery::primary_resource_update_true;
        lookup.load = [&](std::size_t i) {
            check(i == 1, "entity callback receives source registered index");
            if (primary) {
                ++primary_calls;
                return GraphicsMaterialResult{true, {}};
            }
            ++fallback_calls;
            return GraphicsMaterialResult{true, 42};
        };
        const auto found = lookup_native_material_id(id, registered, lookup);
        if (found.at("status") == "loaded")
            return GraphicsMaterialResult{true, found.at("material_index").get<std::size_t>()};
        if (found.at("status") == "load_failed" || found.at("status") == "not_found")
            return GraphicsMaterialResult{true, {}};
        return GraphicsMaterialResult{};
    };
    const auto bound = resolve_native_entity_material(record, entity);
    check(bound.material.material_index == 42 && primary_calls == 1 && fallback_calls == 1,
          "source entity to registered catalog to loaded material retains two ID routes");
    check(source.dump() == original, "query never modifies or reorders the registered catalog");
    return checks;
}
