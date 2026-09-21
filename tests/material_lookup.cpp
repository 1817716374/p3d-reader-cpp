#include "internal.hpp"
#include <p3d/material_lookup.hpp>

unsigned material_lookup_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    NativeMaterialNameContext context;
    auto query = [&](const std::u16string &name = u"Steel") {
        return lookup_native_material_name(name, context);
    };
    auto result = query(u"");
    check(result.at("status") == "not_found" && result.at("native_diagnostic_code") == 3,
          "empty name skips unavailable catalog and callbacks");
    check(query(std::u16string(u"\0Steel", 6)).at("status") == "not_found",
          "name lookup uses its first NUL boundary");
    check(query().at("status") == "unresolved", "partial catalog cannot imply a native miss");
    context.catalog_complete = true;
    check(query().at("native_diagnostic_code") == 3, "empty complete catalog needs no callbacks");
    context.names = {u"Steel"};
    check(query().at("reason") == "native_material_name_comparator_required",
          "source locale comparison is not guessed");
    std::vector<std::string> events;
    context.compare = [&](const std::u16string &a, const std::u16string &b) -> std::optional<int> {
        events.push_back("compare");
        auto lower = [](std::u16string s) {
            for (auto &c : s)
                if (c >= u'A' && c <= u'Z')
                    c += 32;
            return s;
        };
        return lower(a).compare(lower(b)); // Explicit fixture locale, not library policy.
    };
    check(query().at("reason") == "native_material_loader_required",
          "matching declaration is not a loaded material");
    context.names = {u"STEEL", u"Wood", u"steel", u"Steel"};
    context.load = [&](std::size_t i) {
        events.push_back("load" + std::to_string(i));
        return GraphicsMaterialResult{true, i == 0 ? std::optional<std::size_t>{}
                                                   : std::optional<std::size_t>{7}};
    };
    events.clear();
    result = query();
    check(events == std::vector<std::string>{"compare", "compare", "compare", "compare", "load0",
                                             "load2", "load3"},
          "all candidates are collected before any load and loads continue after success");
    check(result.at("candidate_indices") == Json::array({0, 2, 3}) &&
              result.at("material_indices") == Json::array({7, 7}),
          "duplicate names and repeated native loaded identities retain their order");
    check(result.at("first_catalog_index") == 2 && result.at("first_material_index") == 7 &&
              result.at("native_return_code") == 0 && result.at("native_diagnostic_code") == 0,
          "failed first candidate does not hide later native loaded material");
    context.load = [](std::size_t) { return GraphicsMaterialResult{true, {}}; };
    result = query();
    check(result.at("status") == "load_failed" && result.at("native_diagnostic_code") == 4 &&
              result.at("loads").size() == 3 && result.at("first_result_status") == "not_found",
          "all matching resources failing differs from absent names");
    events.clear();
    result = query(u"Glass");
    check(result.at("status") == "not_found" && result.at("loads").empty(),
          "unmatched catalog entries do not invoke loaders");
    context.names = {std::u16string(u"Steel\0hidden", 12)};
    context.compare = [&](const std::u16string &a, const std::u16string &b) -> std::optional<int> {
        check(a == u"Steel" && b == u"Steel",
              "comparison receives C string prefixes in source/query order");
        return 0;
    };
    context.load = [](std::size_t) { return GraphicsMaterialResult{true, 0}; };
    result = query(std::u16string(u"Steel\0other", 11));
    check(result.at("first_material_index") == 0, "material index zero is a successful identity");
    context.names = {u"first", u"second"};
    unsigned comparisons = 0, loads = 0;
    context.compare = [&](const auto &, const auto &) -> std::optional<int> {
        if (++comparisons == 2)
            return {};
        return 0;
    };
    context.load = [&](std::size_t) {
        ++loads;
        return GraphicsMaterialResult{true, 9};
    };
    result = query();
    check(result.at("status") == "unresolved" && loads == 0 &&
              result.at("candidate_indices") == Json::array({0}),
          "unknown comparison never starts an incomplete candidate load sequence");
    context.compare = [](const auto &, const auto &) -> std::optional<int> { return 0; };
    context.load = [](std::size_t i) {
        return i == 0 ? GraphicsMaterialResult{true, 9} : GraphicsMaterialResult{};
    };
    result = query();
    check(result.at("status") == "unresolved" && result.at("native_return_code").is_null() &&
              result.at("first_result_status") == "matched" &&
              result.at("first_material_index") == 9,
          "later unknown loading does not erase a known prefix or claim full completion");
    context.load = [&](std::size_t) {
        ++loads;
        return GraphicsMaterialResult{};
    };
    loads = 0;
    result = query();
    check(loads == 1 && result.at("first_result_status") == "unresolved" &&
              result.at("material_indices").empty(),
          "unknown earlier load stops before callbacks whose cache state may depend on it");
    context.load = [](std::size_t) { return GraphicsMaterialResult{false, 4}; };
    check(query().at("reason") == "unknown_material_has_identity",
          "unknown loader cannot provide a confirmed identity");
    context.load = [](std::size_t) -> GraphicsMaterialResult {
        throw std::runtime_error("provider unavailable");
    };
    result = query();
    check(result.at("reason") == "provider unavailable" &&
              result.at("unresolved_phase") == "candidate_loading",
          "provider exception is not reported as a confirmed native null result");
    context.compare = [](const auto &, const auto &) -> std::optional<int> {
        throw std::runtime_error("locale unavailable");
    };
    check(query().at("reason") == "locale unavailable", "comparison exceptions remain unresolved");
    // Integrate an actual group-2 name payload with the rebuilt-entry resolver.
    context.names = {u"Steel", u"Steel"};
    context.compare = [](const auto &a, const auto &b) -> std::optional<int> {
        return a.compare(b);
    };
    std::vector<std::size_t> loaded;
    context.load = [&](std::size_t i) {
        loaded.push_back(i);
        return GraphicsMaterialResult{true, i + 10};
    };
    GraphicsMaterialContext graphics;
    graphics.native_entity_material = {true, {}};
    graphics.lookup_name = [&](const std::u16string &name, GraphicsMaterialNameQuery kind) {
        check(kind == GraphicsMaterialNameQuery::legacy_part,
              "legacy material query retains owning-file scope");
        const auto found = lookup_native_material_name(name, context);
        require(found.at("status") == "loaded", "fixture material loading failed");
        return GraphicsMaterialResult{true, found.at("first_material_index").get<std::size_t>()};
    };
    Bytes payload;
    for (char16_t unit : std::u16string(u"Steel")) {
        payload.push_back(std::uint8_t(unit));
        payload.push_back(std::uint8_t(unit >> 8));
    }
    auto resolved = resolve_rebuilt_graphics_materials(
        Json::array({{{"inline_material", nullptr}}}),
        Json::array({{{"group", 2}, {"key", 10001}, {"index", 0}, {"payload", rawbytes(payload)}}}),
        graphics);
    check(resolved.at("status") == "resolved" &&
              resolved.at("entries")[0].at("material_index") == 10 &&
              loaded == std::vector<std::size_t>{0, 1},
          "part selects first result while full native query loads all candidates");
    return checks;
}
