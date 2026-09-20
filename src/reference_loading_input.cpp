#include "internal.hpp"
#include <p3d/reference_loading.hpp>

namespace p3d {
Json initial_reference_loading_decision(const Json &record,
                                        const ReferenceLoadingDecisionContext &context) {
    try {
        const auto type = record.at("element_type").get<std::uint16_t>();
        auto c = context;
        c.source_primary_flags.reset();
        c.loaded_primary_flags.reset();
        if (type == 13 && c.require_source_bit14 && record.contains("reference_input")) {
            const auto &input = record.at("reference_input");
            if (input.contains("origin_inputs") &&
                input.at("origin_inputs").contains("primary_flags"))
                c.source_primary_flags =
                    input.at("origin_inputs").at("primary_flags").get<std::uint32_t>();
        }
        // Preserve the native early exits: loaded flags are not required when
        // source-bit filtering or an input failure has already selected a result.
        const bool needs_loaded =
            type == 13 &&
            (!c.require_source_bit14 ||
             (c.source_primary_flags && (*c.source_primary_flags & 0x4000u))) &&
            c.native_input_status && *c.native_input_status == 0;
        if (needs_loaded && record.contains("reference_target")) {
            const auto &target = record.at("reference_target");
            if (target.contains("initial_loaded_flags")) {
                const auto &flags = target.at("initial_loaded_flags").at("primary");
                if (flags.at("status") == "decoded")
                    c.loaded_primary_flags = flags.at("value").get<std::uint32_t>();
            }
        }
        auto out = reference_loading_decision(type, c);
        out["flags_source"] = "initial_reference_record";
        return out;
    } catch (const std::exception &e) {
        return {{"status", "unresolved"},
                {"scope", "initial_reference_loading_branch"},
                {"reason", e.what()}};
    }
}
} // namespace p3d
