#include "internal.hpp"
#include <p3d/reference_recursion.hpp>

namespace p3d {
namespace {
std::u16string c_string(const std::u16string &value) {
    return value.substr(0, value.find(u'\0'));
}
std::u16string model_slot(const Json &target, unsigned key) {
    for (const auto &slot : target.at("strings")) {
        if (slot.at("key") != key)
            continue;
        require(slot.at("status") == "decoded" || slot.at("status") == "absent",
                "repetition_model_name_unresolved");
        std::u16string result;
        for (const auto &value : slot.at("utf16_code_units")) {
            require(value.is_number_integer() && value >= 0 && value <= 65535,
                    "invalid_repetition_name_code_unit");
            const auto unit = value.get<std::uint16_t>();
            if (!unit)
                break;
            result.push_back(static_cast<char16_t>(unit));
        }
        return result;
    }
    throw std::runtime_error("repetition_model_name_slot_required");
}
} // namespace

Json reference_ancestor_repetition(std::uint32_t primary, std::uint32_t secondary,
                                   const Json &target, const ReferenceRepetitionContext &context) {
    Json out = {{"status", "unresolved"},
                {"scope", "explicit_loaded_reference_ancestor_repetition"},
                {"examined_ancestors", 0},
                {"matching_ancestor_indices", Json::array()},
                {"native_post_match_callbacks", "not_evaluated"}};
    auto finish = [&](bool repeated, const char *reason) {
        out.update({{"status", "resolved"}, {"repeated", repeated}, {"reason", reason}});
        return out;
    };
    try {
        if (primary & 0x200u)
            return finish(false, "reference_flag_disables_check");
        const bool gate = context.native_setting_enabled.value_or(false) ||
                          context.host_file_fallback_gate.value_or(false);
        if (!gate) {
            require(context.native_setting_enabled.has_value() &&
                        context.host_file_fallback_gate.has_value(),
                    "reference_repetition_gate_required");
            return finish(false, "native_gate_disabled");
        }
        std::u16string file;
        if (secondary & 0x20000u) {
            require(context.alternate_file_reference.has_value(),
                    "alternate_repetition_file_state_required");
            file = c_string(*context.alternate_file_reference);
        }
        if (file.empty()) {
            require(context.lookup_reference.has_value(), "repetition_lookup_reference_required");
            file = c_string(*context.lookup_reference);
            out["file_source"] = "file_specification_lookup_reference";
        } else
            out["file_source"] = "alternate_runtime_file_reference";
        const unsigned name_key = secondary & 0x20000u ? 36 : 21;
        const auto name = model_slot(target, name_key);
        out["model_name_key"] = name_key;
        out["limit"] = context.limit;
        auto equal = [&](const std::u16string &a, const std::u16string &b) {
            if (a == b)
                return true;
            require(bool(context.equal), "native_wide_case_comparison_required");
            return context.equal(a, b);
        };
        std::uint32_t matches = 0;
        for (std::size_t i = 0; i < context.ancestors.size(); ++i) {
            const auto &ancestor = context.ancestors[i];
            out["examined_ancestors"] = i + 1;
            if (!ancestor.valid)
                return finish(false, "invalid_ancestor_terminates_chain");
            require(ancestor.reference_known_absent || ancestor.reference_primary_flags.has_value(),
                    "ancestor_reference_state_required");
            require(!(ancestor.reference_known_absent && ancestor.reference_primary_flags),
                    "conflicting_ancestor_reference_state");
            if (ancestor.reference_primary_flags && (*ancestor.reference_primary_flags & 0x200u))
                continue;
            require(!(ancestor.file_known_absent && ancestor.file),
                    "conflicting_ancestor_file_state");
            if (ancestor.file_known_absent)
                continue;
            require(ancestor.file.has_value(), "ancestor_file_reference_required");
            auto ancestor_file = c_string(ancestor.file->lookup_reference);
            if (ancestor_file.empty())
                ancestor_file = c_string(ancestor.file->stored_reference);
            if (!equal(ancestor_file, file))
                continue;
            bool model_matches = false;
            if (name.empty()) {
                require(ancestor.is_default_model.has_value(),
                        "ancestor_default_model_state_required");
                model_matches = *ancestor.is_default_model;
            }
            if (!model_matches) {
                require(ancestor.model_name.has_value(), "ancestor_model_name_required");
                model_matches = equal(c_string(*ancestor.model_name), name);
            }
            if (!model_matches)
                continue;
            out["matching_ancestor_indices"].push_back(i);
            // Native adds depth and the PRE-increment match count in eax,
            // then uses a signed comparison with the caller's limit.
            const std::uint32_t bits = static_cast<std::uint32_t>(i) + matches;
            ++matches;
            const auto score = bits < 0x80000000u ? static_cast<std::int64_t>(bits)
                                                  : static_cast<std::int64_t>(bits) - 0x100000000LL;
            if (score >= context.limit) {
                out["triggering_ancestor_index"] = i;
                out["native_comparison_value"] = score;
                return finish(true, "native_repetition_threshold_met");
            }
        }
        require(context.complete_ancestor_chain, "remaining_ancestor_chain_required");
        return finish(false, "complete_chain_below_repetition_threshold");
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
