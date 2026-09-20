#pragma once
#include <p3d/reader.hpp>

namespace p3d {
struct ReferenceHostModelState {
    std::optional<bool> root_present;
    std::optional<std::uint32_t> root_model_flags;
};
struct ReferenceHostFilterContext {
    std::optional<bool> host_valid;
    std::optional<bool> host_reference_present;
    std::optional<bool> host_reference_valid;
    std::optional<bool> host_reference_parent_present;
    // Current host followed by its parent; at most two levels are examined.
    std::vector<ReferenceHostModelState> host_models;
    bool complete_host_chain = false;
    std::optional<std::uint32_t> host_reference_secondary_flags;
    // Compare the file object identities reached through the host reference and
    // its parent. Two null file pointers compare equal; names are not identities.
    std::optional<bool> same_file_object;
    std::optional<bool> parent_root_present;
    std::optional<std::int32_t> parent_model_kind;
};
Json reference_host_loading_filter(std::uint32_t primary_flags, std::uint32_t secondary_flags,
                                   const ReferenceHostFilterContext &context);

struct ReferenceLoadingDecisionContext {
    bool require_source_bit14 = false;
    std::optional<std::uint32_t> source_primary_flags;
    // Return code and flags AFTER the native input operation and its linkages.
    std::optional<std::int32_t> native_input_status;
    std::optional<std::uint32_t> loaded_primary_flags;
    std::optional<bool> host_filter_excluded;
    // Runtime bit 3 of the source record object, not the saved header flags.
    std::optional<bool> source_runtime_deleted;
    // Whether the full host/parent/reference-bit-28 gate selected this search.
    std::optional<bool> descendant_filter_required;
    std::optional<bool> descendant_match;
    std::optional<bool> keep_descendant_match;
    std::optional<bool> ancestor_repeated;
    std::optional<bool> keep_ancestor_repetition;
};
// Evaluates the ordinary loading branch after each required operation's result
// is supplied. Does not run input callbacks, build a graph, or mutate lists.
Json reference_loading_decision(std::uint16_t record_type,
                                const ReferenceLoadingDecisionContext &context);

enum class ReferenceLinkList { active, secondary };
struct ReferenceLinkListEntry {
    bool present = true;
    std::optional<std::uint32_t> secondary_key;
};
struct ReferenceLinkInsertionContext {
    // Native loaded-reference bytes; their broader role is not inferred here.
    std::optional<bool> runtime_byte_4e9_nonzero;
    std::optional<bool> runtime_byte_4e8_nonzero;
    std::optional<std::uint32_t> new_secondary_key;
    std::vector<ReferenceLinkListEntry> existing_entries;
    bool existing_entries_complete = false;
};
Json reference_link_insertion(ReferenceLinkList list, const ReferenceLinkInsertionContext &context);
} // namespace p3d
