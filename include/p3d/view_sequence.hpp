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
} // namespace p3d
