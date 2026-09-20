#pragma once
#include <p3d/reference_recursion.hpp>
#include <unordered_map>

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
// Builds the search input for a successful ordinary native input operation,
// before outer loading or callbacks change the reference. file_specification
// must be the prepared specification, including any host/service fallback.
// Invalid records or unresolved runtime words throw; an unresolved primary name
// remains absent and is required by the matcher only when it actually uses it.
ReferenceSearchQuery initial_reference_search_query(const Json &reference_record,
                                                    const NativeFileReference &file_specification,
                                                    NativeModelNameEqual equal = {});
enum class ReferenceObjectDispatch { unknown, model, reference };
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
    // Confirmed ordinary native implementation, not a saved model-type value.
    ReferenceObjectDispatch object_dispatch = ReferenceObjectDispatch::unknown;
    std::optional<std::size_t> parent;
    bool parent_known = false;
    std::optional<std::uint32_t> reference_primary_flags;
    std::optional<std::uint16_t> reference_nest_depth;
    // Assigned input-record ID stored by the native loaded reference (+260),
    // distinct from the target model ID and the 32-bit secondary insertion key.
    std::optional<std::uint64_t> reference_link_id;
    bool secondary_list_known_absent = false;
    std::vector<std::optional<std::size_t>> secondary_links;
    bool secondary_links_complete = false;
};
// A freshly constructed ordinary reference after successful initial input,
// before connecting its target or loading its own active links. parent is the
// required host identity in the caller's graph, not a saved model/file ID.
// assigned_link_id must come from the actual input registration; omission keeps
// it unknown rather than assuming the saved source ID survived registration.
ReferenceSearchModel
initial_reference_graph_node(const Json &reference_record, std::size_t parent,
                             std::optional<std::uint64_t> assigned_link_id = std::nullopt);
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
// Native parent-root query: ordinary model returns itself, ordinary reference
// forwards to its valid parent. This does not query a reference's connected root.
Json reference_parent_root(const ReferenceSearchContext &context,
                           std::optional<std::size_t> object_index);
// Computes the native ancestor-adjusted signed depth. A positive cap limits
// each reference's uint16 depth; zero/negative cap does not impose that limit.
// A model without a reference gives 1. Unknown parents and cycles are unresolved.
Json reference_nesting_depth(const ReferenceSearchContext &context,
                             std::optional<std::size_t> object_index, std::int32_t cap = 0);
// Gate inside the ordinary collection loop, after its earlier host/input guards.
// force_input bypasses depth evaluation; it does not prove the host was loaded.
Json reference_link_loading_policy(const ReferenceSearchContext &context,
                                   std::optional<std::size_t> host, bool force_input = false,
                                   std::int32_t cap = 0);
// Plans a single native reference request. registry must describe the complete
// current host registry (same resolved/registry/id shape as initial_model_link_registry).
// A hit returns the existing graph object identity without re-input or list mutation.
// A miss describes registration and input still required; it does not execute them.
Json reference_link_request(const ReferenceSearchContext &context, std::size_t host,
                            std::uint64_t assigned_link_id, const Json &registry);
// Validated ID-membership snapshot for repeated requests against one registry.
// Rebuild after registry changes; source pointers and loading states are not
// copied because this pre-input branch does not inspect them. Invalid JSON throws.
class ReferenceLinkRegistryIndex {
  public:
    explicit ReferenceLinkRegistryIndex(const Json &registry);
    std::optional<std::size_t> entry_index(std::uint64_t id) const;

  private:
    std::unordered_map<std::uint64_t, std::size_t> indices_;
};
Json reference_link_request(const ReferenceSearchContext &context, std::size_t host,
                            std::uint64_t assigned_link_id,
                            const ReferenceLinkRegistryIndex &registry);
// Host and candidate are known object identities (nullopt means known absent).
// Derives the gate and search start from this graph; context.start is not used.
Json evaluate_reference_descendant_filter(const ReferenceSearchQuery &query,
                                          const ReferenceSearchContext &context,
                                          std::optional<std::size_t> host,
                                          std::optional<std::size_t> candidate);
} // namespace p3d
