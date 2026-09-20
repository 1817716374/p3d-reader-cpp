#include "internal.hpp"
#include <p3d/reference_model.hpp>

namespace p3d {
namespace {
template <class T> T known(const std::optional<T> &value, const char *why) {
    require(value.has_value(), why);
    return *value;
}
std::int32_t signed_id(std::uint32_t value) {
    return value <= INT32_MAX
               ? static_cast<std::int32_t>(value)
               : static_cast<std::int32_t>(static_cast<std::int64_t>(value) - INT64_C(0x100000000));
}
} // namespace

Json select_reference_model(const ReferenceModelQuery &q, const ReferenceModelFileContext &file) {
    Json out = {{"status", "unresolved"}, {"scope", "reference_target_id_after_file_acquisition"}};
    try {
        const auto flags = known(q.secondary_flags, "reference_secondary_flags_required");
        out["load_mask_after_selection"] = flags & 0x8000u ? 0u : q.requested_load_mask;
        std::uint32_t id;
        if (flags & 0x8000u) {
            out["selection"] = "forced_file_default";
            id = known(file.default_model_id, "file_default_model_id_required");
        } else if (known(q.runtime_flags, "reference_runtime_flags_required") & 0x1000u) {
            out["selection"] = "explicit_model_id";
            id = known(q.explicit_model_id, "explicit_reference_model_id_required");
        } else {
            const auto input = known(q.requested_name, "requested_reference_model_name_required");
            const auto name = input.substr(0, input.find(u'\0'));
            if (name.empty()) {
                out["selection"] = "empty_name_file_default";
                id = known(file.default_model_id, "file_default_model_id_required");
            } else {
                out["selection"] = "model_name";
                require(!file.directory_known_absent || !file.directory,
                        "conflicting_model_directory_state");
                if (file.directory_known_absent) {
                    out.update(
                        {{"status", "missing"}, {"reason", "target_model_directory_absent"}});
                    return out;
                }
                require(file.directory, "current_model_directory_required");
                out["directory_lookup"] =
                    lookup_native_model_directory(*file.directory, name, file.equal);
                const auto &lookup = out.at("directory_lookup");
                if (lookup.at("status") == "missing") {
                    out.update({{"status", "missing"}, {"reason", "target_model_name_missing"}});
                    return out;
                }
                require(lookup.at("status") == "matched", "target_model_name_lookup_unresolved");
                const auto &value = lookup.at("model_id");
                require(value.is_number_unsigned() ||
                            (value.is_number_integer() && value.get<std::int64_t>() >= 0),
                        "directory_model_id_must_be_uint32");
                require(value.get<std::uint64_t>() <= UINT32_MAX,
                        "directory_model_id_must_be_uint32");
                id = value.get<std::uint32_t>();
                if (id == 0xfffffffeu) {
                    out.update({{"status", "missing"}, {"reason", "directory_model_id_minus_two"}});
                    return out;
                }
            }
        }
        out.update({{"status", "selected"}, {"model_id", id}, {"signed_model_id", signed_id(id)}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

ReferenceModelQuery initial_reference_model_query(const Json &record, std::uint32_t mask) {
    require(record.at("element_type") == 13, "type_13_reference_record_required");
    require(record.at("reference_input").at("status") == "decoded",
            "decoded_reference_input_required");
    const auto &target = record.at("reference_target");
    const auto &secondary = target.at("initial_loaded_flags").at("secondary");
    require(secondary.at("status") == "decoded", "initial_reference_secondary_flags_required");
    ReferenceModelQuery q;
    q.requested_load_mask = mask;
    q.secondary_flags = secondary.at("value").get<std::uint32_t>();
    if (*q.secondary_flags & 0x8000u)
        return q;
    const auto &runtime = target.at("initial_runtime_state");
    require(runtime.at("status") == "decoded", "initial_reference_runtime_state_unresolved");
    q.runtime_flags = runtime.at("runtime_flags").get<std::uint32_t>();
    if (*q.runtime_flags & 0x1000u) {
        q.explicit_model_id = runtime.at("model_id").get<std::uint32_t>();
        return q;
    }
    const auto key = *q.secondary_flags & 0x20000u ? 36 : 21;
    for (const auto &slot : target.at("strings")) {
        if (slot.at("key") != key)
            continue;
        require(slot.at("status") == "decoded" || slot.at("status") == "absent",
                "selected_reference_model_name_unresolved");
        std::u16string name;
        for (const auto &unit : slot.at("utf16_code_units")) {
            require(unit.is_number_unsigned() ||
                        (unit.is_number_integer() && unit.get<std::int64_t>() >= 0),
                    "invalid_reference_model_name_code_unit");
            const auto value = unit.get<std::uint64_t>();
            require(value <= 0xffff, "invalid_reference_model_name_code_unit");
            if (!value)
                break;
            name.push_back(static_cast<char16_t>(value));
        }
        q.requested_name = std::move(name);
        return q;
    }
    throw std::runtime_error("selected_reference_model_name_slot_required");
}

Json lookup_native_model(std::uint32_t id, const NativeModelLookupContext &c) {
    const auto key = signed_id(id);
    Json out = {{"status", "unresolved"},
                {"scope", "native_file_model_lookup"},
                {"model_id", id},
                {"signed_model_id", key},
                {"creation_and_callbacks", "not_executed"}};
    auto finish = [&](std::optional<std::size_t> model, const char *source) {
        out.update({{"status", "resolved"}, {"found", model.has_value()}, {"source", source}});
        out["object_index"] = model ? Json(*model) : Json();
        return out;
    };
    try {
        if (key < -1)
            return finish(std::nullopt, "invalid_signed_model_id");
        if (key == -1) {
            require(c.special_model_known, "special_model_slot_required");
            if (c.special_model)
                return finish(c.special_model, "special_model_slot");
        } else {
            const auto at = c.model_cache.find(key);
            if (at != c.model_cache.end()) {
                if (at->second)
                    return finish(at->second, "model_cache");
            } else {
                require(c.model_cache_complete, "remaining_model_cache_required");
            }
        }
        if (c.suppressed_model_ids.count(key))
            return finish(std::nullopt, "suppressed_model_id");
        require(c.suppressed_model_ids_complete, "remaining_suppressed_model_ids_required");
        out["creation_required"] = true;
        require(c.creation_result_known, "complete_model_creation_result_required");
        return finish(c.created_model, "model_creation_result");
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json resolve_reference_model(const ReferenceModelQuery &q, const ReferenceModelFileContext &file,
                             const NativeModelLookupContext &models) {
    Json out = {{"status", "unresolved"},
                {"scope", "reference_model_target_before_attachment"},
                {"attachment_and_loading", "not_executed"},
                {"selection", select_reference_model(q, file)}};
    const auto &selection = out.at("selection");
    if (selection.at("status") == "unresolved") {
        out["reason"] = selection.at("reason");
        return out;
    }
    out["load_mask_after_selection"] = selection.at("load_mask_after_selection");
    if (selection.at("status") == "selected") {
        out["lookup"] = lookup_native_model(selection.at("model_id").get<std::uint32_t>(), models);
        const auto &lookup = out.at("lookup");
        if (lookup.at("status") != "resolved") {
            out["reason"] = lookup.at("reason");
            return out;
        }
        if (lookup.at("found") == true) {
            out.update({{"status", "matched"}, {"object_index", lookup.at("object_index")}});
            return out;
        }
    }
    // The connection path sets runtime bit6 and primary bit9 on model failure.
    // These are effects at this point, not final flags after all other branches.
    out.update({{"status", "missing"},
                {"native_return", 0x11010},
                {"runtime_flags_set_mask", 0x40},
                {"primary_flags_set_mask", 0x200}});
    return out;
}
} // namespace p3d
