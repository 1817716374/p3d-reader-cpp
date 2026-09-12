#include "internal.hpp"

namespace p3d {
namespace {
bool c_equal(const std::string &a, const std::string &b) {
    if (a.size() != b.size())
        return false;
    auto lower = [](unsigned char c) { return c >= 'A' && c <= 'Z' ? c + 32 : c; };
    for (std::size_t i = 0; i < a.size(); ++i)
        if (lower(a[i]) != lower(b[i]))
            return false;
    return true;
}
void valid_text(const std::string &s) {
    // Reuse strict UTF-8 validation, also excluding embedded NUL from callback keys.
    require(s.find('\0') == std::string::npos, "catalog string contains embedded NUL");
    Json(s).dump();
}
Json context_json(const std::optional<MaterialCatalogContext> &context) {
    if (!context)
        return {{"kind", "current_resource_context"}};
    return {{"kind", "explicit_context_keys"},
            {"index_key", context->index_key},
            {"comparison_key", context->comparison_key}};
}
} // namespace

Json native_material_catalog_registration(const Json &selection, const Json &sources,
                                          const MaterialCatalogOptions &options) {
    Json out = {{"reader_profile", "bimbase_2025_material_catalog_registration"},
                {"scope", "single_system_table_with_empty_material_catalog"},
                {"string_comparison", options.case_equal ? "caller_supplied" : "crt_c_locale"},
                {"status", "unresolved"},
                {"entries", Json::array()},
                {"input", Json::array()}};
    out["current_context"] = context_json(options.current_context);
    if (selection.value("status", Json()) == "absent") {
        out["status"] = "absent";
        return out;
    }
    if (selection.value("status", Json()) != "selected") {
        out["reason"] = "selected_system_material_table_required";
        return out;
    }
    const auto &members = selection.at("table").at("members");
    std::map<std::size_t, const Json *> by_source;
    for (const auto &source : sources)
        by_source.emplace(source.at("native_record_index").get<std::size_t>(), &source);
    struct Registered {
        std::uint64_t id;
        std::string name;
        MaterialCatalogResource resource;
    };
    std::vector<Registered> entries;
    const auto equal = options.case_equal ? options.case_equal : c_equal;
    auto context_equal = [&](const MaterialCatalogResource &a, const MaterialCatalogResource &b,
                             bool index) -> bool {
        if (!a.primary_context && !b.primary_context)
            return true;
        auto ac = a.primary_context ? a.primary_context : options.current_context;
        auto bc = b.primary_context ? b.primary_context : options.current_context;
        require(ac && bc, "current resource context keys required for cross-context comparison");
        return equal(index ? ac->index_key : ac->comparison_key,
                     index ? bc->index_key : bc->comparison_key);
    };
    out["status"] = "resolved";
    for (std::size_t mi = 0; mi < members.size(); ++mi) {
        const auto &member = members[mi];
        Json step = {{"member_index", mi}, {"member", member}};
        try {
            const auto it = by_source.find(member.at("native_record_index").get<std::size_t>());
            require(it != by_source.end(), "material member source fields unavailable");
            const auto &source = *it->second;
            const auto &name_reader = source.at("catalog_name").at("reader");
            const auto name_status = name_reader.at("status");
            if (name_status == "missing" || name_status == "failure") {
                step["action"] = "skipped_name_reader_failure";
                out["input"].push_back(std::move(step));
                continue;
            }
            require(name_status == "success" && name_reader.at("value").is_string(),
                    "material name reader unresolved");
            const auto name = name_reader.at("value").get<std::string>();
            valid_text(name);
            const auto &reference = source.at("resource_reference").at("interpretation");
            const auto reference_status = reference.at("status").get<std::string>();
            if (reference_status.compare(0, 9, "rejected_") == 0) {
                step["action"] = "skipped_resource_reader_failure";
                out["input"].push_back(std::move(step));
                continue;
            }
            std::optional<MaterialCatalogResource> resource;
            bool caller_resolved = false;
            if (reference_status == "decoded" &&
                reference.value("primary_context", Json()) == "current_resource_context" &&
                reference.contains("descriptor_mode") && reference["descriptor_mode"].is_number()) {
                const auto field =
                    reference.contains("member_name") ? "member_name" : "project_member_name";
                if (reference.contains(field) && reference[field].is_string())
                    resource = MaterialCatalogResource{
                        reference[field].get<std::string>(),
                        reference["descriptor_mode"].get<std::uint32_t>(), std::nullopt};
            }
            if (!resource && options.resolve_resource) {
                resource = options.resolve_resource(member, source);
                caller_resolved = resource.has_value();
            }
            require(resource.has_value(), "resource descriptor requires external resolution facts");
            valid_text(resource->member_name);
            if (resource->primary_context) {
                valid_text(resource->primary_context->index_key);
                valid_text(resource->primary_context->comparison_key);
            }
            if (options.current_context) {
                valid_text(options.current_context->index_key);
                valid_text(options.current_context->comparison_key);
            }
            const auto &attribute = member.at("material_attribute");
            if (attribute.at("status") == "absent") {
                step["action"] = "skipped_missing_material_attribute";
                out["input"].push_back(std::move(step));
                continue;
            }
            require(attribute.at("status") == "selected",
                    "material attribute selection unresolved");
            const auto id = member.at("assigned_id").get<std::uint64_t>();
            step["name"] = name;
            step["resource"] = {
                {"member_name", resource->member_name},
                {"source", caller_resolved ? "caller_resolved" : "source_reference"},
                {"descriptor_mode", resource->descriptor_mode},
                {"primary_context", context_json(resource->primary_context)}};
            std::optional<std::size_t> duplicate;
            if (!name.empty()) {
                for (std::size_t i = 0; i < entries.size(); ++i) {
                    const auto &e = entries[i];
                    if (equal(e.name, name) && context_equal(e.resource, *resource, false) &&
                        e.resource.descriptor_mode == resource->descriptor_mode &&
                        equal(e.resource.member_name, resource->member_name) &&
                        context_equal(*resource, e.resource, false)) {
                        duplicate = i;
                        break;
                    }
                }
            }
            if (duplicate) {
                step["action"] = "rejected_duplicate_name_resource";
            } else if (id != UINT64_MAX) {
                for (std::size_t i = 0; i < entries.size(); ++i)
                    if (entries[i].id == id &&
                        context_equal(entries[i].resource, *resource, true)) {
                        duplicate = i;
                        break;
                    }
                if (duplicate)
                    step["action"] = "rejected_duplicate_id_context";
            }
            if (duplicate) {
                step["existing_entry_index"] = *duplicate;
                step["native_registration_return_code"] = 1;
            } else {
                const auto ei = entries.size();
                entries.push_back({id, name, *resource});
                step["action"] = "registered";
                step["entry_index"] = ei;
                step["native_registration_return_code"] = 0;
                out["entries"].push_back(
                    {{"member_index", mi},
                     {"member", member},
                     {"id", id},
                     {"name", name},
                     {"resource", step["resource"]},
                     {"source_resource_reference", source.at("resource_reference")},
                     {"material_attribute", attribute}});
            }
        } catch (const std::exception &e) {
            step["action"] = "unresolved";
            step["reason"] = e.what();
            out["status"] = "partial";
            out["input"].push_back(std::move(step));
            break; // Unknown registration can change every later duplicate decision.
        }
        out["input"].push_back(std::move(step));
    }
    out["unvisited_member_count"] = members.size() - out["input"].size();
    return out;
}

Json Document::native_material_catalog(const MaterialCatalogOptions &options) const {
    Json out = Json::array();
    for (const auto &container : native_input_containers()) {
        if (container.at("kind") != "P3D-SSYS")
            continue;
        const auto &selection = container.at("initial_material_table");
        std::vector<std::size_t> indices;
        if (selection.at("status") == "selected")
            for (const auto &member : selection.at("table").at("members"))
                indices.push_back(member.at("native_record_index").get<std::size_t>());
        const auto sources = native_material_catalog_records(native_records(), &indices);
        auto result = native_material_catalog_registration(selection, sources, options);
        result["container"] = container.at("container");
        for (auto &entry : result["entries"]) {
            const auto &reference = entry.at("material_attribute");
            const auto &attachment = container.at("initial_attribute_input")
                                         .at("attachments")
                                         .at(reference.at("attachment_index").get<std::size_t>());
            auto attribute =
                attachment.at("attributes").at(reference.at("source_ordinal").get<std::size_t>());
            attribute["stream"] = attachment.at("stream");
            attribute["record_offset"] = attachment.at("offset");
            entry["source_attribute"] = std::move(attribute);
            // Inputs to the later material reader, using this exact initial
            // collection. Registration alone does not execute that reader.
            entry["supplementary_attributes"] = Json::array();
            for (const auto key : {20091u, 22906u}) {
                Json extra = {{"group", 0}, {"key", key}, {"index", 0}, {"status", "absent"}};
                for (const auto &lookup : attachment.at("lookup").at("keys")) {
                    if (lookup.at("group") != 0 || lookup.at("key") != key ||
                        lookup.at("index") != 0)
                        continue;
                    const auto ordinal = lookup.at("selected_source_ordinal").get<std::size_t>();
                    extra["status"] = "selected";
                    extra["source_attribute"] = attachment.at("attributes").at(ordinal);
                    extra["source_attribute"]["stream"] = attachment.at("stream");
                    extra["source_attribute"]["record_offset"] = attachment.at("offset");
                    break;
                }
                entry["supplementary_attributes"].push_back(std::move(extra));
            }
        }
        out.push_back(std::move(result));
    }
    return out;
}
} // namespace p3d
