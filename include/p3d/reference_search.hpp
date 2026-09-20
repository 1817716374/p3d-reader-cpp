#pragma once
#include <p3d/reference_recursion.hpp>

namespace p3d {
struct ReferenceDescendantGateContext {
    std::optional<bool> host_valid;
    std::optional<bool> host_parent_present;
    std::optional<bool> candidate_valid;
    std::optional<bool> candidate_reference_present;
    std::optional<std::uint32_t> candidate_primary_flags;
    // Starts at the candidate's parent. Only validity/reference/flags are used.
    std::vector<ReferenceAncestorState> candidate_hosts;
    bool complete_candidate_host_chain = false;
};
Json reference_descendant_search_gate(const ReferenceDescendantGateContext &context);

struct ReferenceSearchQuery {
    bool reference_present = true;
    std::optional<std::u16string> stored_file_reference;
    std::optional<std::u16string> lookup_file_reference;
    // Loaded reference's separate runtime flag word (native +1b0).
    std::optional<std::uint32_t> runtime_flags;
    std::optional<std::uint32_t> model_id;
    // Loaded PRIMARY name (+200). The alternate name is not used by this search.
    std::optional<std::u16string> model_name;
    NativeModelNameEqual equal;
};
struct ReferenceSearchModel {
    bool valid = true;
    std::optional<bool> root_present;
    bool file_known_absent = false;
    std::optional<NativeFileReference> file;
    std::optional<std::uint32_t> model_id;
    std::optional<std::u16string> model_name;
    std::optional<bool> is_default_model;
    // Fields needed only when this object is inspected as an active-list entry.
    std::optional<std::int32_t> native_kind;
    std::optional<bool> reference_present;
    std::optional<std::uint32_t> reference_runtime_flags;
    bool active_list_known_absent = false;
    std::vector<std::optional<std::size_t>> active_links;
    bool active_links_complete = false;
};
Json match_reference_model(const ReferenceSearchQuery &query, const ReferenceSearchModel &model);
struct ReferenceSearchContext {
    // Indices express native object identity; file/model IDs may repeat.
    std::vector<ReferenceSearchModel> models;
    std::optional<std::size_t> start;
    bool start_known = false;
};
// Includes the starting model, then eligible direct active references recursively
// in native order. This is not a traversal of the document's structural tree.
Json search_reference_descendants(const ReferenceSearchQuery &query,
                                  const ReferenceSearchContext &context);
} // namespace p3d
