#include "internal.hpp"
#include <p3d/reference_loading.hpp>

namespace p3d {
namespace {
template <class T> T known(const std::optional<T> &value, const char *reason) {
    require(value.has_value(), reason);
    return *value;
}
} // namespace

Json reference_host_loading_filter(std::uint32_t primary, std::uint32_t secondary,
                                   const ReferenceHostFilterContext &c) {
    Json out = {{"status", "unresolved"},
                {"scope", "explicit_loaded_reference_host_filter"},
                {"examined_host_models", 0}};
    auto finish = [&](bool excluded, const char *reason) {
        out.update({{"status", "resolved"}, {"excluded", excluded}, {"reason", reason}});
        return out;
    };
    try {
        if (!known(c.host_valid, "host_validity_required"))
            return finish(false, "invalid_host");
        if (!known(c.host_reference_present, "host_reference_presence_required"))
            return finish(false, "host_has_no_reference");
        if (!known(c.host_reference_valid, "host_reference_validity_required"))
            return finish(false, "invalid_host_reference");
        if (!known(c.host_reference_parent_present, "host_reference_parent_presence_required"))
            return finish(false, "host_reference_has_no_parent");
        if (primary & 0x10000000u) {
            // Native model query 9 returns 1 for a null root, otherwise bit 9
            // of its model-info flags. It examines no more than two hosts.
            require(!c.host_models.empty(), "current_host_model_state_required");
            for (std::size_t i = 0; i < 2; ++i) {
                if (i == c.host_models.size()) {
                    require(c.complete_host_chain, "next_host_model_state_required");
                    break;
                }
                out["examined_host_models"] = i + 1;
                const auto &model = c.host_models[i];
                if (!known(model.root_present, "host_root_presence_required"))
                    return finish(false, "host_model_query_9_positive");
                if (known(model.root_model_flags, "host_root_model_flags_required") & 0x200u)
                    return finish(false, "host_model_query_9_positive");
            }
        } else {
            if (known(c.host_reference_secondary_flags, "host_reference_secondary_flags_required") &
                0x400000u)
                return finish(false, "host_reference_secondary_bit22");
            if (known(c.same_file_object, "host_file_identity_comparison_required") &&
                known(c.parent_root_present, "parent_root_presence_required") &&
                known(c.parent_model_kind, "parent_model_kind_required") == 1)
                return finish(false, "same_file_parent_model_kind_1");
        }
        return finish(bool(secondary & 0x80000u), "candidate_secondary_bit19");
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json reference_loading_decision(std::uint16_t type, const ReferenceLoadingDecisionContext &c) {
    Json out = {{"status", "unresolved"},
                {"scope", "ordinary_reference_loading_branch"},
                {"input_and_callbacks", "not_executed"},
                {"list_mutation", "not_executed"}};
    auto finish = [&](int state, std::int32_t code, bool transferred, const char *list) {
        out.update({{"status", "resolved"},
                    {"loading_state", state},
                    {"native_return", code},
                    {"reference_transferred", transferred},
                    {"requested_list", list}});
        return out;
    };
    try {
        if (type != 13)
            return finish(4, 1, false, "none");
        if (c.require_source_bit14 &&
            !(known(c.source_primary_flags, "source_primary_flags_required") & 0x4000u))
            return finish(7, 0, false, "none");
        const auto input_status =
            known(c.native_input_status, "native_reference_input_status_required");
        if (input_status != 0)
            return finish(4, input_status, false, "none");
        if (known(c.loaded_primary_flags, "loaded_primary_flags_required") & 0x80u)
            return finish(5, 0, false, "none");
        if (known(c.host_filter_excluded, "host_loading_filter_result_required"))
            return finish(6, 0, false, "none");
        if (known(c.source_runtime_deleted, "source_runtime_deleted_state_required"))
            return finish(9, 0, true, "secondary");
        if (known(c.descendant_filter_required, "descendant_filter_gate_required") &&
            known(c.descendant_match, "descendant_match_result_required")) {
            if (known(c.keep_descendant_match, "keep_descendant_match_required"))
                return finish(9, 0, true, "secondary");
            return finish(3, 0, false, "none");
        }
        if (known(c.ancestor_repeated, "ancestor_repetition_result_required")) {
            if (known(c.keep_ancestor_repetition, "keep_ancestor_repetition_required"))
                return finish(9, 0, true, "secondary");
            return finish(8, 0, false, "none");
        }
        return finish(1, 0, true, "active");
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json reference_link_insertion(ReferenceLinkList list, const ReferenceLinkInsertionContext &c) {
    Json out = {{"status", "unresolved"},
                {"scope", "ordinary_reference_loading_list_insertion"},
                {"examined_entries", 0}};
    auto finish = [&](bool append, const char *reason) {
        out.update({{"status", "resolved"}, {"append", append}, {"reason", reason}});
        return out;
    };
    try {
        require(list == ReferenceLinkList::active || list == ReferenceLinkList::secondary,
                "invalid_reference_list_kind");
        out["list"] = list == ReferenceLinkList::active ? "active" : "secondary";
        if (known(c.runtime_byte_4e9_nonzero, "runtime_byte_4e9_state_required"))
            return finish(false, "runtime_byte_4e9_nonzero");
        if (known(c.runtime_byte_4e8_nonzero, "runtime_byte_4e8_state_required"))
            return finish(false, "runtime_byte_4e8_nonzero");
        if (list == ReferenceLinkList::active)
            return finish(true, "active_list_appends_without_key_comparison");
        for (std::size_t i = 0; i < c.existing_entries.size(); ++i) {
            out["examined_entries"] = i + 1;
            require(c.existing_entries[i].present, "null_secondary_list_entry");
            const auto key =
                known(c.existing_entries[i].secondary_key, "existing_secondary_key_required");
            if (key == known(c.new_secondary_key, "new_secondary_key_required")) {
                out["matching_entry_index"] = i;
                return finish(false, "secondary_list_key_already_present");
            }
        }
        require(c.existing_entries_complete, "remaining_secondary_list_required");
        return finish(true, "secondary_list_has_no_matching_key");
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
