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
} // namespace p3d
