#include "internal.hpp"
#include <future>

unsigned component_material_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        ++checks;
        require(value, why);
    };
    auto source = [](const Bytes &name) {
        // A complete empty-instance ParaCmptInstance with a variable footer.
        Bytes b(15);
        auto u32 = [&](std::uint32_t value) {
            for (unsigned i = 0; i < 4; ++i)
                b.push_back(static_cast<std::uint8_t>(value >> (i * 8)));
        };
        u32(static_cast<std::uint32_t>(name.size()));
        b.insert(b.end(), name.begin(), name.end());
        b.resize(b.size() + 21);
        auto field = decode_binary_field("ParaCmptInstance", b);
        require(!field.contains("decode_error"), "component material source fixture");
        return field.at("decoded");
    };
    ComponentMaterialContext context;
    context.noumenon_material_known = true;
    context.material_list_complete = true;
    context.material_names = {u"Outer", u"Inner", u"Outer", u"outer"};
    context.noumenon_material_name = Bytes{'I', 'n', 'n', 'e', 'r'};
    const auto instance = source(Bytes{'O', 'u', 't', 'e', 'r'});
    const auto original = instance.dump();
    auto result = resolve_component_material_overrides(instance, context);
    check(result.at("status") == "override_selected" && result.at("material_index") == 0 &&
              result.at("lookup_index") == 1 && result.at("lookups")[0].at("material_index") == 1,
          "outer footer wins over inner material; first same-name material wins");
    check(result.at("applies_to") == "all_rebuilt_graphics_entries" &&
              !result.contains("entry_indices"),
          "override does not invent serialized-entry to rebuilt-part mapping");
    auto miss = source(Bytes{'M', 'i', 's', 's'});
    result = resolve_component_material_overrides(miss, context);
    check(result.at("status") == "override_selected" && result.at("material_index") == 1 &&
              result.at("lookup_index") == 0 && result.at("lookups")[1].at("status") == "not_found",
          "outer miss preserves inner hit");
    result = resolve_component_material_overrides(source(Bytes{'o', 'u', 't', 'e', 'r'}), context);
    check(result.at("material_index") == 3, "case-distinct native materials are distinct keys");
    result = resolve_component_material_overrides(source(Bytes{'O', 'U', 'T', 'E', 'R'}), context);
    check(result.at("material_index") == 1 && result.at("lookup_index") == 0,
          "component lookup does not inherit case-insensitive element lookup");
    context.noumenon_material_name.reset();
    result = resolve_component_material_overrides(miss, context);
    check(result.at("status") == "preserve_entry_materials" &&
              result.at("lookups")[0].at("status") == "absent" &&
              !result.contains("material_index"),
          "known absence and complete-list miss leave original materials intact");
    context.noumenon_material_known = false;
    result = resolve_component_material_overrides(miss, context);
    check(result.at("status") == "unresolved", "unknown inner context is not property absence");
    result = resolve_component_material_overrides(instance, context);
    check(result.at("status") == "override_selected" && result.at("material_index") == 0,
          "confirmed outer hit resolves final uniform override despite unknown inner stage");
    context.noumenon_material_known = true;
    context.material_names.push_back(u"");
    result = resolve_component_material_overrides(source({}), context);
    check(result.at("status") == "override_selected" && result.at("material_index") == 4,
          "empty footer name may match empty loaded material name");
    context.material_names.pop_back();
    result = resolve_component_material_overrides(source({}), context);
    check(result.at("status") == "preserve_entry_materials", "empty footer misses ordinary names");
    result = resolve_component_material_overrides(source(Bytes{'O', 'u', 't', 'e', 'r', 0, 0xff}),
                                                  context);
    check(result.at("material_index") == 0 &&
              result.at("lookups")[1].at("lookup_name_bytes").at("bytes") == 5,
          "footer NUL truncation precedes encoding and lookup");
    context.material_names.push_back(std::u16string{u'O', u'u', u't', u'e', u'r', 0, u'x'});
    result = resolve_component_material_overrides(instance, context);
    check(result.at("material_index") == 0, "loaded material names use full UTF-16 length");
    context.material_list_complete = false;
    result = resolve_component_material_overrides(instance, context);
    check(result.at("status") == "override_selected" && result.at("material_index") == 0,
          "hit in known ordered prefix is definitive");
    result = resolve_component_material_overrides(miss, context);
    check(result.at("status") == "unresolved" &&
              result.at("lookups")[1].at("reason") == "material_list_incomplete",
          "missing name in incomplete list is not a lookup miss");
    context.noumenon_material_name = Bytes{'I', 'n', 'n', 'e', 'r'};
    result = resolve_component_material_overrides(miss, context);
    check(result.at("status") == "unresolved" && !result.contains("material_index") &&
              result.at("lookups")[0].at("status") == "matched",
          "unknown outer stage suppresses an otherwise confirmed inner override");
    context.material_list_complete = true;
    context.noumenon_material_name.reset();
    result = resolve_component_material_overrides(source(Bytes{0xff}), context);
    check(result.at("status") == "unresolved" &&
              result.at("lookups")[1].at("reason") == "requires_ansi_decoder",
          "non-ASCII source is never guessed as UTF-8 or a fixed ANSI code page");
    context.material_names.push_back(u"\u6728\u6750");
    Bytes observed;
    context.ansi_decoder = [&](const Bytes &b) {
        observed = b;
        return std::u16string(u"\u6728\u6750");
    };
    result = resolve_component_material_overrides(source(Bytes{0x81, 0x82, 0, 0xff}), context);
    check(result.at("status") == "override_selected" && result.at("material_index") == 5 &&
              observed == Bytes({0x81, 0x82}),
          "caller conversion receives exact NUL-excluded bytes and selects Unicode material");
    context.material_names.push_back(std::u16string{char16_t(0xd800)});
    context.ansi_decoder = [](const Bytes &) { return std::u16string{char16_t(0xd800)}; };
    result = resolve_component_material_overrides(instance, context);
    check(result.at("material_index") == 6 &&
              result.at("lookups")[1].at("lookup_name_utf16_code_units")[0] == 0xd800,
          "UTF-16 identity retains raw code units without Unicode normalization");
    context.ansi_decoder = [](const Bytes &) -> std::u16string {
        throw std::runtime_error("test conversion failed");
    };
    result = resolve_component_material_overrides(instance, context);
    check(result.at("status") == "unresolved" &&
              result.at("lookups")[1].at("reason") == "test conversion failed",
          "conversion failure never becomes missing material");
    context.ansi_decoder = {};
    context.noumenon_material_name = Bytes{'I', 'n', 'n', 'e', 'r', 0, 0xff};
    result = resolve_component_material_overrides(miss, context);
    check(result.at("material_index") == 1, "Noumenon material string has the same NUL boundary");
    result = resolve_component_material_overrides(Json::object(), context);
    check(result.at("status") == "unresolved" && !result.contains("material_index"),
          "unavailable footer does not imply a harmless outer miss");
    auto changed_display = instance;
    changed_display["footer"]["material_name_reference"]["lookup_name"] = "Inner";
    result = resolve_component_material_overrides(changed_display, context);
    check(result.at("material_index") == 0, "lookup uses source bytes rather than display text");
    check(instance.dump() == original, "resolution does not mutate source material or geometry");
    const auto expected = resolve_component_material_overrides(instance, context);
    std::vector<std::future<Json>> tasks;
    for (unsigned i = 0; i < 4; ++i)
        tasks.push_back(std::async(std::launch::async, [&] {
            return resolve_component_material_overrides(instance, context);
        }));
    for (auto &task : tasks)
        check(task.get() == expected, "concurrent resolution has no shared mutable lookup cache");
    return checks;
}
