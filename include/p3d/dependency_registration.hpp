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
struct NativeDependencyLoadInput {
    std::vector<NativeDependencyLoadEntity> entities;
    // Each batch is one accepted root subtree in recursive input order. All
    // its entities are registered before any of its file callbacks execute.
    // Every entity must occur exactly once; indices are occurrence identities.
    std::vector<std::vector<std::size_t>> batches;
    bool input_complete = false;
    bool system_registry_known_empty = false;
    bool file_fallback_disabled = false;
    bool monitored_entity_set_known_empty = false;
    std::size_t max_work_items = 1000000;
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
};
// R1.18 initial file callbacks (mode 1), fresh current-model ID registry, empty
// system registry, no file fallback, empty service monitored set. Resolves
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
} // namespace p3d
