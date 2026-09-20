#include "internal.hpp"
#include <p3d/graphics_material.hpp>
#include <codecvt>
#include <locale>

namespace p3d {
namespace {
Json binding(const GraphicsMaterialResult &value) {
    require(value.known || !value.material_index, "unknown_material_has_identity");
    Json out = {{"status", !value.known           ? "unresolved"
                           : value.material_index ? "matched"
                                                  : "not_found"}};
    if (value.material_index)
        out["material_index"] = *value.material_index;
    return out;
}
std::u16string wide_name(const Bytes &bytes) {
    // The native reader appends two zero bytes before scanning UTF-16 words.
    // An odd final byte therefore forms a word with a zero high byte.
    std::u16string out;
    for (std::size_t i = 0; i < bytes.size(); i += 2) {
        const auto word =
            unsigned(bytes[i]) | (i + 1 < bytes.size() ? unsigned(bytes[i + 1]) << 8 : 0);
        if (!word)
            break;
        out.push_back(static_cast<char16_t>(word));
    }
    return out;
}
std::u16string utf8_name(std::string text) {
    text.resize(text.find('\0') == std::string::npos ? text.size() : text.find('\0'));
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> codec;
    auto out = codec.from_bytes(text);
    require(codec.converted() == text.size(), "material_name_utf8_conversion_incomplete");
    return out;
}
} // namespace

Json resolve_rebuilt_graphics_materials(const Json &entries, const Json &attributes,
                                        const GraphicsMaterialContext &context) {
    Json out = {{"status", "unresolved"},
                {"scope", "rebuilt_graphics_entries_with_available_builder"},
                {"part_index_basis", "zero_based_rebuilt_graphics_entry_order"},
                {"geometry_mapping_status", "caller_supplied_rebuilt_entry_sequence"},
                {"entries", Json::array()}};
    try {
        require(entries.is_array() && attributes.is_array(), "material_inputs_must_be_arrays");
        require(entries.size() <= std::size_t(INT32_MAX) + 1,
                "rebuilt_part_index_exceeds_signed_int");
        const auto lookup = native_attribute_lookup(attributes);
        out["attribute_lookup"] = lookup;
        require(lookup.at("status") == "resolved", "material_attribute_lookup_unresolved");
        std::map<std::pair<unsigned, std::uint32_t>, std::size_t> selected;
        for (const auto &key : lookup.at("keys"))
            if (key.at("key") == 10001 && (key.at("group") == 2 || key.at("group") == 4))
                selected.emplace(std::make_pair(key.at("group").get<unsigned>(),
                                                key.at("index").get<std::uint32_t>()),
                                 key.at("selected_source_ordinal").get<std::size_t>());
        auto attribute = [&](unsigned group, std::uint32_t index) -> const Json * {
            auto found = selected.find({group, index});
            return found == selected.end() ? nullptr : &attributes.at(found->second);
        };
        auto name_lookup = [&](const std::u16string &name, GraphicsMaterialNameQuery query) {
            Json step = {{"status", "unresolved"},
                         {"lookup_name_utf16_code_units",
                          std::vector<std::uint16_t>(name.begin(), name.end())},
                         {"query", query == GraphicsMaterialNameQuery::legacy_part
                                       ? "owning_file_update_false"
                                       : "current_project_update_true"}};
            if (!context.lookup_name)
                step["reason"] = "material_name_loader_unavailable";
            else
                step.update(binding(context.lookup_name(name, query)));
            return step;
        };
        auto wide_lookup = [&](unsigned group, std::uint32_t index,
                               GraphicsMaterialNameQuery query) {
            Json step = {{"status", "not_found"}};
            const auto *a = attribute(group, index);
            if (!a)
                return step;
            step["attribute_ordinal"] = selected.at({group, index});
            try {
                step.update(name_lookup(wide_name(bytesof(a->at("payload"))), query));
            } catch (const std::exception &e) {
                step.update({{"status", "unresolved"}, {"reason", e.what()}});
            }
            return step;
        };
        Json element;
        try {
            element = binding(context.native_entity_material);
            element["source"] = "native_entity_material";
            if (element.at("status") == "not_found") {
                element = wide_lookup(4, 0, GraphicsMaterialNameQuery::advanced_entity);
                element["source"] = "advanced_entity_name";
            }
        } catch (const std::exception &e) {
            element = {{"status", "unresolved"}, {"reason", e.what()}};
        }
        out["entity_material"] = element;

        // Parse once. A rejected input stays unknown: JsonCpp also accepts syntax
        // outside the strict JSON subset, so our parse failure is not a native miss.
        Json part_names, part_table = {{"status", "absent"}};
        if (const auto *a = attribute(4, 1)) {
            part_table["attribute_ordinal"] = selected.at({4, 1});
            try {
                const auto bytes = bytesof(a->at("payload"));
                const auto end = std::find(bytes.begin(), bytes.end(), std::uint8_t(0));
                part_names = Json::parse(bytes.begin(), end);
                require(part_names.is_object() || part_names.is_null(),
                        "part_material_json_not_object_or_null");
                part_table["status"] = "parsed";
                part_table["input_bytes"] = std::size_t(end - bytes.begin());
            } catch (const std::exception &e) {
                part_table.update({{"status", "unresolved"}, {"reason", e.what()}});
            }
        }
        out["part_name_table"] = part_table;
        bool complete = true;
        for (std::size_t i = 0; i < entries.size(); ++i) {
            Json item = {{"entry_index", i}, {"part_index", i}, {"lookups", Json::array()}};
            auto apply = [&](Json step, const char *source) {
                step["source"] = source;
                const auto state = step.at("status");
                item["lookups"].push_back(step);
                if (state == "not_found")
                    return false;
                item["status"] = state == "matched" ? "resolved" : "unresolved";
                item["source"] = source;
                if (state == "matched")
                    item["material_index"] = step.at("material_index");
                else
                    complete = false;
                return true;
            };
            Json advanced = {{"status", "not_found"}};
            try {
                require(part_table.at("status") != "unresolved", "part_name_table_unresolved");
                auto name = part_names.find(std::to_string(i));
                if (part_names.is_object() && name != part_names.end()) {
                    // Other JsonCpp asString conversions require their native numeric profile.
                    require(name->is_string(),
                            "part_material_value_requires_native_string_conversion");
                    advanced = name_lookup(utf8_name(name->get<std::string>()),
                                           GraphicsMaterialNameQuery::advanced_part);
                }
            } catch (const std::exception &e) {
                advanced = {{"status", "unresolved"}, {"reason", e.what()}};
            }
            if (!apply(advanced, "advanced_part_name") &&
                !apply(wide_lookup(2, static_cast<std::uint32_t>(i),
                                   GraphicsMaterialNameQuery::legacy_part),
                       "legacy_part_name") &&
                !apply(element, "entity_material")) {
                Json inline_result;
                try {
                    const auto &material = entries.at(i).at("inline_material");
                    if (material.is_null())
                        inline_result = {{"status", "not_found"}};
                    else if (context.load_inline_material)
                        inline_result = binding(context.load_inline_material(material));
                    else
                        inline_result = {{"status", "unresolved"},
                                         {"reason", "inline_material_loader_unavailable"}};
                } catch (const std::exception &e) {
                    inline_result = {{"status", "unresolved"}, {"reason", e.what()}};
                }
                if (!apply(inline_result, "graphics_entry_material")) {
                    item["status"] = "unassigned";
                    item["source"] = nullptr;
                }
            }
            out["entries"].push_back(std::move(item));
        }
        out["status"] = complete ? "resolved" : "unresolved";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
