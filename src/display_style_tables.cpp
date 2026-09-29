#include "internal.hpp"
#include "display_style_xml.hpp"

namespace p3d {
namespace {
const Json *attachment_for(const Json &input, const Json &occurrence, std::size_t &index) {
    const Json *found = nullptr;
    for (std::size_t i = 0; i < input.at("attachments").size(); ++i) {
        const auto &a = input.at("attachments")[i];
        if (a.at("target").at("input_occurrence_index") != occurrence)
            continue;
        require(!found, "ambiguous_initial_attribute_attachment");
        found = &a;
        index = i;
    }
    return found;
}

const Json *lookup_key(const Json &attachment, unsigned key, unsigned index) {
    const auto &lookup = attachment.at("lookup");
    require(lookup.at("status") == "resolved", "initial_attribute_lookup_required");
    for (const auto &k : lookup.at("keys"))
        if (k.at("group") == 0 && k.at("key") == key && k.at("index") == index)
            return &k;
    return nullptr;
}

Json slots(const Json &record, const Json &attachment) {
    Json out = {{"status", "unresolved"}, {"scope", "builtin_22903_attribute_slot_layout"},
                {"object_construction", "not_evaluated"}, {"entries", Json::array()},
                {"missing_attribute_ranges", Json::array()}};
    try {
        // P3DKJ 0xa9890 reads native header +0x24 (source prefix adds 4).
        const auto bytes = bytesof(record.at("data"));
        require(bytes.size() >= 44, "truncated_style_table_maximum");
        const auto maximum = Reader(bytes, 40).i32();
        out["maximum_entry_index"] = maximum;
        // 0xdcdb0 increments a signed DWORD, then compares <= maximum.
        require(maximum != std::numeric_limits<std::int32_t>::max(),
                "native_int32_slot_loop_wrap");
        const auto count = maximum < 0 ? 0ull : std::uint64_t(maximum) + 1;
        out["slot_count"] = count;
        std::map<std::uint32_t, const Json *> keys;
        if (!attachment.is_null()) {
            require(attachment.at("lookup").at("status") == "resolved",
                    "initial_attribute_lookup_required");
            for (const auto &key : attachment.at("lookup").at("keys")) {
                const auto index = key.at("index").get<std::uint32_t>();
                if (key.at("group") == 0 && key.at("key") == 22903 && index < count)
                    require(keys.emplace(index, &key).second, "ambiguous_style_slot_key");
            }
        }
        std::uint64_t next = 0;
        for (const auto &[index, key] : keys) {
            if (next < index)
                out["missing_attribute_ranges"].push_back({{"first", next}, {"last", index - 1}});
            next = std::uint64_t(index) + 1;
            const auto ordinal = key->at("selected_source_ordinal").get<std::size_t>();
            const auto &a = attachment.at("attributes").at(ordinal);
            Json entry = {{"slot_index", index}, {"attribute_index", index},
                          {"source_ordinal", ordinal}, {"attribute_offset", a.at("offset")},
                          {"matching_source_ordinals", key->at("matching_source_ordinals")},
                          {"status", "unresolved_payload"}};
            const auto &decoded = a.at("decoded");
            if (decoded.contains("tree")) {
                entry["xml_tree"] = decoded.at("tree");
                entry["native_xml_import"] = decode_display_style_xml(decoded.at("tree"));
                entry["status"] = "xml_input";
            }
            out["entries"].push_back(std::move(entry));
        }
        if (next < count)
            out["missing_attribute_ranges"].push_back({{"first", next}, {"last", count - 1}});
        // Missing attributes append null; a compatible handler appends even a
        // null import result. Under the built-in registry, all exact 22903
        // keys use 0x62cd20 -> 0xa9c70. Registry replacement is outside scope.
        out["missing_attribute_slot_value"] = nullptr;
        out["status"] = "resolved";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json select_table(const Json &list, const Json &records, const Json &ids,
                  const Json &input, std::uint32_t wanted, DisplayStyleHandlerProfile profile) {
    Json out = {{"kind", wanted == 0x006f0000u ? "common" : "lite"},
                {"registration_key", wanted}, {"status", "unresolved"},
                {"selected_table", nullptr}, {"scan", Json::array()}};
    try {
        require(list.at("scope") == "empty_list_before_runtime_registration" &&
                    list.at("status") == "resolved" && list.at("system_bootstrap_required") == true &&
                    list.at("system_bootstrap_found") == true,
                "complete_fresh_system_list_required");
        require(ids.at("scope") == "first_system_input_with_empty_id_registry" &&
                    ids.at("status") == "resolved", "complete_system_id_assignments_required");
        const auto input_status = input.at("status");
        require(input_status == "resolved" || input_status == "absent" || input_status == "not_loaded",
                "complete_initial_attribute_input_required");
        require(list.at("roots").size() == ids.at("roots").size(), "system_input_order_mismatch");
        unsigned state = 0;
        for (std::size_t ri = 0; ri < list.at("roots").size(); ++ri) {
            const auto &root = list.at("roots")[ri];
            const auto &assigned = ids.at("roots")[ri];
            require(root.at("native_record_index") == assigned.at("native_record_index") &&
                        root.at("headers").size() == assigned.at("records").size(),
                    "system_input_order_mismatch");
            for (std::size_t hi = 0; hi < root.at("headers").size(); ++hi) {
                const auto &header = root.at("headers")[hi];
                const auto &item = assigned.at("records")[hi];
                const auto ni = header.at("native_record_index").get<std::size_t>();
                require(header.at("status") == "resolved" && item.at("native_record_index") == ni,
                        "prepared_system_header_required");
                const auto &n = records.at(ni);
                const auto type = n.at("element_type").get<unsigned>();
                const auto flags = header.at("output_element_flags").get<unsigned>();
                const auto words = header.at("output_record_word_count").get<std::uint32_t>();
                Json step = {{"native_record_index", ni},
                             {"input_occurrence_index", item.at("input_occurrence_index")},
                             {"assigned_id", item.at("assigned_id")}};
                // 0x456e68 checks length before the hierarchy/type mask.
                if (!type || words < 16 || words > 65535) {
                    step["action"] = "stop_invalid_header";
                    out["scan"].push_back(std::move(step));
                    out.update({{"status", "absent"}, {"stop_reason", "invalid_scan_header"},
                                {"native_scan_return_code", 0x44}});
                    return out;
                }
                const bool compound = type >= 10 && type <= 32 ? true :
                                      type >= 33 && type <= 63 ? false : (flags & 0x40) != 0;
                if (!(flags & 0x80))
                    state = 0;
                if (state == 3 || (state != 1 && type != 92)) {
                    step["action"] = state == 3 ? "skip_rejected_subtree" : "skip_type";
                    if (compound)
                        state = 3;
                    out["scan"].push_back(std::move(step));
                    continue;
                }
                if (compound)
                    state = 1;
                // 0x4576c0 with flags 0x400c0: only the type mask applies.
                // Accepted compound descendants bypass that mask (0x456ebc).
                std::size_t attachment_index = 0;
                const auto *attachment = attachment_for(input, item.at("input_occurrence_index"),
                                                        attachment_index);
                const auto *key = attachment ? lookup_key(*attachment, 22900, 1) : nullptr;
                if (!key) {
                    // The built-in type-92 fallback (0x399830) is not either
                    // style singleton. Other accepted descendant types could
                    // invoke factories that are not modeled here.
                    step["action"] = type == 92 ? "non_style_default_handler" : "unresolved_handler";
                    out["scan"].push_back(std::move(step));
                    require(type == 92, "descendant_handler_fallback_unmodeled");
                    continue;
                }
                const auto ordinal = key->at("selected_source_ordinal").get<std::size_t>();
                const auto &a = attachment->at("attributes").at(ordinal);
                step["attachment_index"] = attachment_index;
                step["source_ordinal"] = ordinal;
                step["matching_source_ordinals"] = key->at("matching_source_ordinals");
                const auto payload = bytesof(a.at("payload"));
                if (payload.size() < 8) {
                    step["action"] = type == 92 ? "short_marker_default_handler" : "unresolved_handler";
                    out["scan"].push_back(std::move(step));
                    require(type == 92, "descendant_handler_fallback_unmodeled");
                    continue;
                }
                const auto registration = Reader(payload).u32();
                step["registration_key"] = registration;
                if (profile == DisplayStyleHandlerProfile::BuiltinDefaultService &&
                    (registration == 0x58740000u || registration == 0x58740001u)) {
                    // Original 0x394620/0x3946e0 observations after core init:
                    // neither key is registered. The default service at
                    // vtable 0x544390 does not add it. Resolver 0x39fe20,
                    // with caller mask zero, then uses the type fallback.
                    step["registration_lookup"] = "absent_after_default_service";
                    step["action"] = type == 92 ? "missing_registration_default_handler" :
                                                  "unresolved_handler";
                    if (type == 92)
                        step["handler_vtable_rva"] = 0x54f230;
                    out["scan"].push_back(std::move(step));
                    require(type == 92, "descendant_handler_fallback_unmodeled");
                    continue;
                }
                if (registration != 0x006f0000u && registration != 0x597e0000u) {
                    step["action"] = "unresolved_handler";
                    out["scan"].push_back(std::move(step));
                    throw std::runtime_error("other_handler_registration_unmodeled");
                }
                if (registration != wanted) {
                    step["action"] = "other_style_handler";
                    out["scan"].push_back(std::move(step));
                    continue;
                }
                step["action"] = "first_matching_handler";
                out["scan"].push_back(step);
                step.erase("action");
                step["root_index"] = ri;
                step["source_id"] = item.at("source_id");
                step["block_number"] = root.at("block_number");
                step["attribute_stream"] = attachment->at("stream");
                out["selected_table"] = std::move(step);
                out["status"] = "selected";
                out["stop_reason"] = "first_matching_handler";
                out["native_scan_return_code"] = 0xb;
                if (item.at("assigned_id") == 0) {
                    // dd1f0 re-looks up the callback's ID; an unindexed zero
                    // ID does not cause the scanner to try the next table.
                    out["table_load"] = {{"status", "empty"}, {"reason", "selected_id_not_indexed"}};
                } else {
                    out["table_load"] = slots(n, *attachment);
                }
                return out;
            }
        }
        out.update({{"status", "absent"}, {"stop_reason", "end_of_initial_system_list"}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json slot_query(std::int32_t index, const Json &lite, const Json &plan) {
    Json out = {{"status", "unresolved"}, {"runtime_application", "not_evaluated"}};
    try {
        // 5fac0 sign-extends object+148; 60000 compares the resulting uint64
        // with the vector length. A negative value cannot select a slot.
        if (index < 0) {
            out.update({{"status", "resolved"}, {"outcome", "negative_index"}});
            return out;
        }
        require(plan.at("status") == "conditional" || plan.at("status") == "layout_resolved",
                "lite_initialization_unresolved");
        const auto count = plan.at("slot_count").get<std::uint64_t>();
        out["slot_count"] = count;
        out["status"] = plan.at("status") == "conditional" ? "conditional" : "resolved";
        if (plan.contains("conditions")) out["conditions"] = plan.at("conditions");
        if (std::uint64_t(index) >= count) {
            out["outcome"] = "out_of_range";
            return out;
        }
        const bool copied = plan.at("decision") == "copy_common";
        const auto &entries = copied ? plan.at("entries") : lite.at("table_load").at("entries");
        out["list_source"] = copied ? "common_copy_plan" : "lite_table_load";
        const Json *selected = nullptr;
        for (const auto &entry : entries)
            if (entry.at("slot_index") == index) {
                require(!selected, "ambiguous_lite_slot");
                selected = &entry;
            }
        if (!selected) {
            out["outcome"] = "null_slot";
            return out;
        }
        out["entry"] = *selected;
        if (copied) {
            out["outcome"] = "projected_style_input";
        } else {
            require(selected->contains("native_xml_import"), "style_xml_input_unresolved");
            const auto &imported = selected->at("native_xml_import");
            out["status"] = "conditional";
            out["conditions"] = Json::array({"native_xml_import_matches_retained_tree"});
            out["outcome"] = imported.at("status") == "rejected" ? "null_slot" : "selected_style_input";
            require(imported.at("status") == "rejected" || imported.at("status") == "partial",
                    "style_xml_input_unresolved");
            // Retain the import's unresolved handler/usage state. Selecting
            // an input is not evidence that its native object was constructed.
            out["object_construction"] = "not_evaluated";
        }
    } catch (const std::exception &e) {
        out["status"] = "unresolved";
        out.erase("outcome");
        out["reason"] = e.what();
    }
    return out;
}

Json reference_inputs(const Json &records, const Json &ids, const Json &input,
                      const Json &lite, const Json &plan) {
    Json out = {{"status", "unresolved"}, {"scope", "attached_20080_input_queries"},
                {"native_reader_dispatch", "not_evaluated"}, {"references", Json::array()}};
    try {
        require(ids.at("status") == "resolved" &&
                    ids.at("scope") == "first_system_input_with_empty_id_registry",
                "complete_system_id_assignments_required");
        require(input.at("status") == "resolved" || input.at("status") == "absent" ||
                    input.at("status") == "not_loaded", "complete_initial_attribute_input_required");
        for (const auto &root : ids.at("roots")) {
            for (const auto &item : root.at("records")) {
                std::size_t ai = 0;
                const auto *a = attachment_for(input, item.at("input_occurrence_index"), ai);
                if (!a) continue;
                // Inventory attached collections containing this key, even
                // when only a nonzero index exists. Do not imply that an
                // arbitrary native record dispatches through 4b46b0/4b4220.
                bool relevant = false;
                for (const auto &attribute : a->at("attributes"))
                    relevant |= attribute.at("group") == 0 && attribute.at("key") == 20080;
                if (!relevant) continue;
                const auto ni = item.at("native_record_index").get<std::size_t>();
                Json ref = {{"native_record_index", ni}, {"element_type", records.at(ni).at("element_type")},
                            {"input_occurrence_index", item.at("input_occurrence_index")},
                            {"source_id", item.at("source_id")}, {"assigned_id", item.at("assigned_id")},
                            {"attachment_index", ai}, {"attribute_stream", a->at("stream")},
                            {"status", "unresolved"}};
                try {
                    const auto *key = lookup_key(*a, 20080, 0);
                    std::int32_t value = -1;
                    std::string action = "keep_default_missing_attribute";
                    if (key) {
                        const auto ordinal = key->at("selected_source_ordinal").get<std::size_t>();
                        const auto &attribute = a->at("attributes").at(ordinal);
                        ref["source_ordinal"] = ordinal;
                        ref["attribute_offset"] = attribute.at("offset");
                        ref["matching_source_ordinals"] = key->at("matching_source_ordinals");
                        const auto payload = bytesof(attribute.at("payload"));
                        ref["payload_size"] = payload.size();
                        action = "keep_default_invalid_length";
                        // 4b4737 initializes +148 to -1. 4b4342 queries exact
                        // key/index first; 4b43c5 only writes on length == 4.
                        if (payload.size() == 4) {
                            value = Reader(payload).i32();
                            action = "read_selected_int32";
                        }
                    }
                    ref["reader_projection"] = {{"status", "conditional"},
                        {"condition", "fresh_object_uses_R1.18_4b46b0_and_4b4220_reader"},
                        {"initial_value", -1}, {"display_style_index", value}, {"action", action}};
                    ref["lite_slot_query"] = slot_query(value, lite, plan);
                    ref["status"] = "resolved";
                } catch (const std::exception &e) {
                    ref["reason"] = e.what();
                }
                out["references"].push_back(std::move(ref));
            }
        }
        out["status"] = "resolved";
        for (const auto &ref : out.at("references"))
            if (ref.at("status") != "resolved") out["status"] = "partial";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace

Json initial_native_display_style_tables(const Json &list, const Json &records,
                                        const Json &ids, const Json &input,
                                        DisplayStyleHandlerProfile profile) {
    Json out = {{"scope", "fresh_system_style_tables_before_runtime_callbacks"},
            {"handler_profile", profile == DisplayStyleHandlerProfile::BuiltinDefaultService
                                    ? "bimbase_r1_18_builtin_registry_and_default_service"
                                    : "bimbase_r1_18_builtin_style_handlers_without_replacement"},
            {"host_service", profile == DisplayStyleHandlerProfile::BuiltinDefaultService
                                 ? "original_default_service" : "unspecified"},
            {"runtime_resolution", "not_evaluated"},
            {"common_to_lite_conversion", "not_evaluated"},
            {"common", select_table(list, records, ids, input, 0x006f0000u, profile)},
            {"lite", select_table(list, records, ids, input, 0x597e0000u, profile)}};
    out["lite_initialization"] = plan_initial_lite_style_list(out.at("common"), out.at("lite"));
    out["reference_inputs"] = reference_inputs(records, ids, input, out.at("lite"),
                                               out.at("lite_initialization"));
    return out;
}

Json Document::initial_display_style_tables(DisplayStyleHandlerProfile profile) const {
    Json out = Json::array();
    for (const auto &container : native_input_containers()) {
        // The native lookup uses the file's SSYS list, not a model namespace.
        if (container.at("kind") != "P3D-SSYS" || container.at("container").size() != 2)
            continue;
        auto tables = initial_native_display_style_tables(
            container.at("list_preparation"), native_records(),
            container.at("system_id_assignments"), container.at("initial_attribute_input"), profile);
        tables["system_container"] = container.at("container");
        out.push_back(std::move(tables));
    }
    return out;
}
} // namespace p3d
