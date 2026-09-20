#pragma once
#include <p3d/reader.hpp>

namespace p3d {
struct ViewSequenceLink {
    std::uint64_t id = 0;
    bool present = true;
    // The native link's runtime state bit 14 excludes it from reconciliation,
    // but not from the unfiltered link count or default sequence construction.
    bool excluded_from_reconciliation = false;
};
struct ViewSequenceContext {
    // Preserve native list order, duplicates, null slots and excluded links.
    std::vector<ViewSequenceLink> links;
    bool links_complete = false;
    // Document::models()[key].view_state.current_model_last (header bit 11).
    // Missing means the selected model's position has not been established.
    std::optional<bool> current_model_last;
    // nullopt differs from an allocated, empty sequence.
    std::optional<std::vector<std::uint64_t>> previous_sequence;
    // Explicitly invoke the native default builder if no sequence was allocated.
    // links must then be in the order at that call site (after any caller sorting).
    bool initialize_default = false;
};
// saved_sequence is native_records()[i].view_link_sequence, or null when absent.
// Resolves loading/reconciliation and optional explicit default initialization.
// This does not reproduce the surrounding model-loading and link-sorting pipeline.
Json resolve_view_link_sequence(const Json &saved_sequence, const ViewSequenceContext &context);

struct ViewSequenceCandidate {
    std::uint64_t link_id = 0;
    // Native pointer identity with the current model, not link_id == 0.
    bool is_current_model = false;
};
// Candidates must already be collected for the selected model. Result indices
// refer to this input array; no model discovery or visibility filtering occurs.
Json order_view_link_candidates(const std::vector<std::uint64_t> &entry_ids,
                                const std::vector<ViewSequenceCandidate> &candidates);

enum class ViewCandidateNodeKind { unknown, model, reference };
struct ViewCandidateNode {
    ViewCandidateNodeKind kind = ViewCandidateNodeKind::unknown;
    bool valid = true;
    // Indices represent native object identity, independently of serialized IDs.
    // A model terminates ancestry traversal; only references follow parent.
    std::optional<std::size_t> parent;
    bool parent_known = false;
    std::optional<std::uint64_t> link_id;
};
struct ViewCandidateContext {
    std::vector<ViewCandidateNode> nodes;
    std::size_t current_model = 0;
    // Result of the selected object's native root-model query, not its parent.
    std::optional<bool> root_model_available;
    std::vector<std::optional<std::size_t>> links;
    bool links_complete = false;
    // nullopt selects the include_* path; an empty array is an explicit filter.
    std::optional<std::vector<std::optional<std::size_t>>> provided_candidates;
    bool include_current_model = true;
    bool include_links = true;
    bool apply_sequence = true;
    // The already resolved sequence of the selected object's root model.
    std::optional<std::vector<std::uint64_t>> sequence;
};
// Reproduces native candidate collection, ancestry filtering and optional swaps
// in the supplied object graph. Does not open files or construct runtime links.
// On an unresolved result, no final candidate list is returned.
Json collect_view_link_candidates(const ViewCandidateContext &context);

// Fresh control-list registration, before runtime flag changes or callbacks.
// model_input is Document::native_model_id_assignments(...); native_records are
// from the same document. Only root objects of the control list participate.
Json initial_model_link_registry(const Json &model_input, const Json &native_records);
Json initial_model_link_registry(const Document &document, const StreamPath &model_storage,
                                 std::uint64_t initial_id_counter);
} // namespace p3d
