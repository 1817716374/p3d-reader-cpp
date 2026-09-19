#include "material_catalog_read.hpp"
#include <pugixml/pugixml.hpp>

namespace p3d {
namespace {
bool same_context(const Json &resource, const MaterialCatalogOptions &options) {
    const auto &context = resource.at("primary_context");
    if (context.at("kind") == "current_resource_context")
        return true;
    require(context.at("kind") == "explicit_context_keys" && options.current_context,
            "current_material_resource_context_keys_required");
    const auto a = context.at("comparison_key").get<std::string>();
    const auto &b = options.current_context->comparison_key;
    if (options.case_equal)
        return options.case_equal(b, a); // Native current-context.compare(resource-context).
    if (a.size() != b.size())
        return false;
    auto lower = [](unsigned char c) { return c >= 'A' && c <= 'Z' ? c + 32 : c; };
    for (std::size_t i = 0; i < a.size(); ++i)
        if (lower(a[i]) != lower(b[i]))
            return false;
    return true;
}
Json source_attribute(const Json &attachment, std::size_t ordinal) {
    auto attribute = attachment.at("attributes").at(ordinal);
    attribute["stream"] = attachment.at("stream");
    attribute["record_offset"] = attachment.at("offset");
    return attribute;
}
Json supplementary_input(const Json &attachment) {
    Json inputs = Json::array();
    for (auto key : {20091u, 22906u}) {
        Json input = {{"group", 0}, {"key", key}, {"index", 0}, {"status", "absent"}};
        for (const auto &lookup : attachment.at("lookup").at("keys")) {
            if (lookup.at("group") != 0 || lookup.at("key") != key || lookup.at("index") != 0)
                continue;
            const auto ordinal = lookup.at("selected_source_ordinal").get<std::size_t>();
            input["status"] = "selected";
            input["source_ordinal"] = ordinal;
            input["source_attribute"] = source_attribute(attachment, ordinal);
            break;
        }
        inputs.push_back(std::move(input));
    }
    return inputs;
}
Json primary_xml(const Json &attribute) {
    const auto &decoded = attribute.at("decoded");
    require(decoded.contains("data"), "material_attribute_codec_unavailable");
    const auto bytes = bytesof(decoded.at("data"));
    std::size_t end = 0;
    while (end + 1 < bytes.size() && (bytes[end] != 0 || bytes[end + 1] != 0))
        end += 2;
    // P3DXmlDom receives a UTF-16 pointer and length zero. Its input ends at
    // the first zero word, not at the attribute's declared decompressed length.
    require(end + 1 < bytes.size(), "native_material_xml_terminator_outside_payload");
    const auto xml = utf16(slice(bytes, 0, end));
    pugi::xml_document document;
    const auto parsed = document.load_buffer(
        xml.data(), xml.size(), pugi::parse_full | pugi::parse_ws_pcdata | pugi::parse_fragment,
        pugi::encoding_utf8);
    require(bool(parsed), std::string("material_xml: ") + parsed.description());
    unsigned roots = 0;
    for (const auto &node : document.children()) {
        require(node.type() != pugi::node_doctype, "material_xml_dtd_requires_native_reader");
        if (node.type() == pugi::node_element)
            ++roots;
        else if (node.type() == pugi::node_pcdata || node.type() == pugi::node_cdata)
            require(std::string(node.value()).find_first_not_of(" \t\r\n") == std::string::npos,
                    "material_xml_text_outside_root");
    }
    require(roots == 1, "material_xml_requires_one_root");
    auto tree = xml_tree(xml);
    auto settings = material_settings(tree);
    auto textures = material_texture_references(tree, settings);
    return {{"xml", xml},
            {"tree", std::move(tree)},
            {"settings", std::move(settings)},
            {"texture_references", std::move(textures)},
            {"xml_profile", "single_root_without_dtd"},
            {"utf16_input_byte_count", end},
            {"terminator_byte_offset", end},
            {"ignored_suffix", rawbytes(slice(bytes, end + 2, bytes.size() - end - 2))}};
}
} // namespace

Json native_catalog_material_read(const Json &catalog, const Json &container, const Json &records,
                                  const MaterialCatalogOptions &options) {
    Json out = {{"reader_profile", "bimbase_2025_default_material_provider_input"},
                {"scope", "initial_system_input_without_material_cache"},
                {"status", "unresolved"},
                {"entries", Json::array()},
                {"decoded_entry_indices", Json::array()},
                {"runtime_material_enumeration", "not_performed"},
                {"texture_file_resolution", "caller_responsibility"}};
    if (catalog.value("status", Json()) == "absent") {
        out["status"] = "absent";
        return out;
    }
    try {
        require(catalog.at("scope") == "single_system_table_with_empty_material_catalog",
                "initial_system_catalog_required");
        require(catalog.at("status") == "resolved" || catalog.at("status") == "partial",
                "registered_catalog_prefix_required");
        const auto &ids = container.at("system_id_assignments");
        require(ids.at("status") == "resolved", "complete_system_registry_required");
        const auto &attributes = container.at("initial_attribute_input");
        const auto as = attributes.at("status");
        require(as == "resolved" || as == "absent" || as == "not_loaded",
                "complete_system_attribute_input_required");
        std::map<std::uint64_t, const Json *> by_id;
        for (const auto &target : ids.at("registry"))
            require(by_id.emplace(target.at("id").get<std::uint64_t>(), &target).second,
                    "ambiguous_system_registry");
        std::map<std::size_t, Json> source_fields;
        out["status"] = catalog.at("status");
        for (std::size_t i = 0; i < catalog.at("entries").size(); ++i) {
            const auto &entry = catalog.at("entries")[i];
            Json read = {{"catalog_entry_index", i}, {"status", "unresolved"}};
            try {
                // Other resource contexts require their own registry and input
                // collections. Never search unrelated local records by source ID.
                require(same_context(entry.at("resource"), options),
                        "external_material_resource_context_required");
                const auto id = entry.at("id").get<std::uint64_t>();
                read["lookup_id"] = id;
                const auto target = by_id.find(id);
                if (target == by_id.end()) {
                    read["status"] = "not_found";
                    read["reason"] = "material_id_not_in_system_registry";
                    out["entries"].push_back(std::move(read));
                    continue;
                }
                read["target"] = *target->second;
                const auto ni = target->second->at("native_record_index").get<std::size_t>();
                if (!source_fields.count(ni)) {
                    const std::vector<std::size_t> selected = {ni};
                    source_fields.emplace(
                        ni, native_material_catalog_records(records, &selected).at(0));
                }
                const auto &source = source_fields.at(ni);
                read["source_record"] = {{"native_record_index", ni},
                                         {"stream", source.at("stream")},
                                         {"record_offset", source.at("record_offset")},
                                         {"source_id", source.at("record_id")}};
                const auto selection = native_material_attribute(*target->second, attributes);
                read["material_attribute"] = selection;
                require(selection.at("status") == "selected", "material_attribute_unavailable");
                const auto &attachment =
                    attributes.at("attachments")
                        .at(selection.at("attachment_index").get<std::size_t>());
                auto attribute =
                    source_attribute(attachment, selection.at("source_ordinal").get<std::size_t>());
                require(attribute.at("group") == 0 && attribute.at("key") == 20014 &&
                            attribute.at("index") == 0,
                        "inconsistent_material_attribute_selection");
                read["source_attribute"] = attribute;
                auto material = primary_xml(attribute);
                const auto &name = source.at("catalog_name").at("reader");
                read["object_name_reader"] = name;
                require(name.at("status") == "success" && name.at("value").is_string(),
                        "material_object_name_reader_failed");
                read["object_resource_reference"] = source.at("resource_reference");
                const auto &resource = source.at("resource_reference").at("interpretation");
                require(resource.at("status") == "decoded", "material_object_resource_unresolved");
                // The object reader first applies the record's name; its caller
                // then overwrites name and ID with the actual catalog entry.
                auto final_name = entry.at("name").get<std::string>();
                final_name.resize(final_name.find('\0') == std::string::npos
                                      ? final_name.size()
                                      : final_name.find('\0'));
                material["name"] = final_name;
                material["id"] = id;
                material["object_name"] = name.at("value");
                material["name_source"] = "catalog_entry";
                material["supplementary_attributes"] = supplementary_input(attachment);
                material["texture_postprocessing"] = "not_performed";
                read["material"] = std::move(material);
                read["status"] = "decoded";
                out["decoded_entry_indices"].push_back(i);
            } catch (const std::exception &e) {
                read["reason"] = e.what();
                out["status"] = "partial";
            }
            out["entries"].push_back(std::move(read));
        }
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
