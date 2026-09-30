#pragma once
#include <p3d/reader.hpp>
#include <optional>

namespace p3d {
struct NativeDependencyLoadEntity {
    // Final ID after input preparation and collision handling, not the source ID.
    std::uint64_t assigned_id = 0;
    std::optional<std::uint32_t> runtime_flags_10;
    // All 56d0 user-link payloads in original order, excluding the four-byte
    // linkage header. Other linkage applications are not supplied here.
    std::vector<Bytes> dependency_payloads;
};
struct NativeDependencySystemTarget {
    std::uint64_t assigned_id = 0;
    std::optional<std::uint32_t> runtime_flags_10;
};
struct NativeDependencyLoadInput {
    std::vector<NativeDependencyLoadEntity> entities;
    // Each batch is one accepted root subtree in recursive input order. All
    // its entities are registered before any of its file callbacks execute.
    // Every entity must occur exactly once; indices are occurrence identities.
    std::vector<std::vector<std::size_t>> batches;
    bool input_complete = false;
    // Used after a local miss when no complete system_registry is supplied.
    bool system_registry_known_empty = false;
    // Required only when both current-model and system lookup miss.
    bool file_fallback_disabled = false;
    bool monitored_entity_set_known_empty = false;
    std::size_t max_work_items = 1000000;
    // Complete, already registered targets in the distinct system model.
    // nullopt means unknown unless system_registry_known_empty is true.
    // Vector positions are system occurrence identities; ID 0 is unregistered.
    // Targets/flags remain fixed throughout this model's input batches.
    std::optional<std::vector<NativeDependencySystemTarget>> system_registry;
};
struct NativeDependencyEdge {
    std::size_t target_entity = 0;
    std::size_t dependent_entity = 0;
};
struct NativeDependencyLoadBatchResult {
    // Original callback order. Each edge prepends the dependent to the target's
    // native reverse list; duplicates and self references are retained.
    std::vector<NativeDependencyEdge> added_edges;
    std::vector<std::size_t> newly_pending_entities;
    // target_entity indexes system_registry, dependent_entity indexes entities.
    std::vector<NativeDependencyEdge> added_system_edges;
};
struct NativeDependencyLoadResult {
    bool resolved = false;
    std::string reason;
    std::optional<std::size_t> failed_batch;
    std::optional<std::size_t> failed_entity;
    std::vector<std::vector<std::size_t>> dependents;
    // Set membership in stable input-index order, not native pointer ordering.
    std::vector<std::size_t> pending_entities;
    std::vector<NativeDependencyLoadBatchResult> batches;
    // New reverse-list prefixes contributed by this load to system targets.
    // Entries are local entities indices, not system indices or persisted IDs.
    // Qualify them with the current model identity before prepending each
    // prefix to any pre-existing system reverse list spanning other models.
    std::vector<std::vector<std::size_t>> system_dependents;
};
// R1.18 initial file callbacks (mode 1), fresh current-model ID registry, empty
// service monitored set. Local lookup precedes the supplied system registry;
// file fallback context is required only if both miss. Resolves
// direct IDs in formats 0/1; format 0 owner=10000/relation=4 is a separate path
// program and remains unsupported. Disabled/empty/out-of-range formats skip.
// This does not run later pending-resolution passes, cross-model callbacks,
// typed-handler work queues or geometry regeneration. Failure publishes no
// partial graph. A resolved result may still contain pending entities.
NativeDependencyLoadResult project_native_dependency_load(const NativeDependencyLoadInput &input);

struct NativeDependencyRetryInput {
    std::vector<NativeDependencyLoadEntity> entities;
    // Complete existing reverse lists, in native head-to-tail order.
    std::vector<std::vector<std::size_t>> dependents;
    // Complete pending set, in the order used by this retry. Native uses entity
    // pointer order, which cannot be inferred from persisted IDs or indices.
    std::vector<std::size_t> pending_iteration_order;
    // Whether this model enters the notification set (model138->byte14 != 0
    // or model byte154 != 0). Unknown must not be treated as false.
    std::optional<bool> model_notified;
    bool input_complete = false;
    bool system_registry_known_empty = false;
    bool file_fallback_disabled = false;
    bool monitored_entity_set_known_empty = false;
    std::size_t max_work_items = 1000000;
};
struct NativeDependencyRetryResult {
    bool resolved = false;
    std::string reason;
    std::optional<std::size_t> failed_entity;
    std::vector<std::vector<std::size_t>> dependents;
    std::vector<std::size_t> pending_entities;
    std::vector<std::size_t> monitored_entities;
    std::vector<NativeDependencyEdge> added_edges;
    // Distinct target/dependent pairs queued for subsequent processing, sorted
    // by indices; added_edges retains every insertion including duplicates.
    std::vector<NativeDependencyEdge> scheduled_pairs;
};
// R1.18 complete 1e9640 pending retry core after model notification, mode 2,
// fresh auxiliary queues, registry byte c = 0, single model. This does not run
// the enclosing 1eb240/1f1a90 service flush or geometry updates. Deleted pending
// entities are discarded; excluded targets now requeue the dependent. Missing
// IDs remain pending and become monitored. Hits only register when notified.
// Active pending entities with attribute-provider flag 0x100000 are unsupported.
NativeDependencyRetryResult project_native_dependency_retry(const NativeDependencyRetryInput &input);

struct NativeDependencyNormalizationInput {
    std::vector<std::vector<std::size_t>> dependents;
    std::vector<NativeDependencyEdge> scheduled_pairs;
    bool input_complete = false;
    // All target entities use the standard entity interface (virtual +38 = 1).
    bool standard_entities_known = false;
    // registry+110 has no separate removed-dependency work.
    bool removal_work_known_empty = false;
    std::size_t max_work_items = 1000000;
};
struct NativeDependencyNormalizationResult {
    bool resolved = false;
    std::string reason;
    std::vector<std::vector<std::size_t>> dependents;
    // Positions in each input list, preserving node occurrence identity.
    std::vector<std::vector<std::size_t>> retained_positions;
};
// R1.18 1ec3e0 after work-set exchange: only scheduled pairs are normalized.
// Keep the first occurrence of each scheduled dependent and all occurrences
// of other dependents, in unchanged head-to-tail order. No geometry refresh.
NativeDependencyNormalizationResult project_native_dependency_normalization(const NativeDependencyNormalizationInput &input);

struct NativeDependencyCycleInput : NativeDependencyRetryInput {
    // At entry, all work queues except the supplied pending set are empty.
    bool other_work_queues_known_empty = false;
    bool standard_entities_known = false;
    // Original +a8/+b0 callbacks take/release a nested hold; final unload is
    // outside this profile and must not be silently treated as a no-op.
    bool caller_holds_model = false;
    bool link_update_handlers_known_absent = false;
};
struct NativeDependencyCycleResult {
    bool resolved = false;
    std::string reason;
    std::optional<std::size_t> failed_entity;
    std::vector<std::vector<std::size_t>> dependents;
    // This iteration consumes pending work even when IDs remain missing.
    std::vector<std::size_t> pending_entities;
    // Missing/excluded targets survive here; an empty pending queue does not
    // mean every reference resolved. Membership uses stable input indices.
    std::vector<std::size_t> monitored_entities;
    std::vector<NativeDependencyEdge> scheduled_pairs;
};
// Complete R1.18 1eb240 iteration in a single caller-held standard model:
// retry, selective duplicate removal, remaining link callbacks and cleanup.
// Requires absent link-update handlers; active flag 0x4000 is unsupported.
// Shares one max_work_items budget across all phases. Does not run outer
// 1f1a90 host transactions, model unload, cross-model or attribute callbacks.
// resolved describes this bounded projection, not a fully resolved graph.
NativeDependencyCycleResult project_native_dependency_cycle(const NativeDependencyCycleInput &input);
} // namespace p3d
