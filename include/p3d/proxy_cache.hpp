#pragma once
#include <p3d/reader.hpp>

namespace p3d {
struct ProxyCacheLimits {
    std::size_t max_depth = 64;
    std::size_t max_entries = 1000000;
};
// Reads one length-prefixed proxy registry inside an already decompressed
// model edge cache. This is not the complete attribute or model-cache header.
// Keeps source occurrences and the native last-value registry separately.
// Decoded command streams use the same schema as parse_commands(). Some
// component fields and display flags retain explicitly unnamed source values.
Json decode_native_proxy_registry(const Bytes &, ProxyCacheLimits = {});

struct ModelEdgeCacheLimits {
    ProxyCacheLimits proxies;
    std::size_t max_models = 10000;
    std::size_t max_model_bytes = 64u * 1024 * 1024;
    std::size_t max_total_model_bytes = 256u * 1024 * 1024;
};
// Input is the complete, actually attached attribute collection, using the
// group/key/index/payload schema of graphics_records(). The native duplicate
// lookup depends on the entire collection; do not prefilter it.
// Reads index 0 and the version-51 model tree in preorder starting at index 1.
// Models retain source link IDs, not resolved runtime model identities.
// A decoded result is structural: runtime attachment and some fields remain
// explicitly unevaluated. Truncated/unsafe source extents are rejected.
Json decode_native_model_edge_cache(const Json &attributes, ModelEdgeCacheLimits = {});

// Reads the separately compressed index-65535 association attribute from the
// same complete attribute collection. Paths are ordered from the actual host
// toward descendant references; entity IDs are local to that resolved model.
// Association links address source entry ordinals, not model or entity IDs.
// This preserves associations without inventing runtime targets or validity.
Json decode_native_edge_cache_associations(const Json &attributes, ModelEdgeCacheLimits = {});

struct EdgeCacheEntityStateContext {
    std::optional<bool> entity_found;
    std::optional<std::uint32_t> runtime_flags;
    std::optional<double> last_modified_milliseconds;
};
// Tests the entity-existence/flags/modification-time gate, before the native
// display callback. Saved time is exactly eight bytes of IEEE-754 binary64.
// Flags are from the resolved runtime entity, not a persisted record header.
Json compare_native_edge_cache_entity_state(const Bytes &saved_modification_time,
                                            const EdgeCacheEntityStateContext &);

struct EdgeCacheAssociationTarget {
    // Whether the actual reference/model lookup and ALL entity state/display
    // checks created this source association. Unknown is not false.
    std::optional<bool> created;
    // Caller tokens identifying actual resolved objects. Equal objects require
    // equal tokens; unrelated objects require distinct tokens. File IDs alone
    // are not suitable. Tokens are never dereferenced or persisted as pointers.
    std::optional<std::uint64_t> reference_identity;
    std::vector<std::uint64_t> entity_identities;
    bool entity_identities_complete = false;
};
// Applies native link-set cardinality and group equivalence to the decoded
// association result and actual targets in matching source order. Returns
// selected/retained SOURCE ordinals; does not invent native pointer ordering,
// regenerate hashes, resolve targets, merge geometry or evaluate display.
Json select_native_edge_cache_associations(const Json &decoded,
                                           const std::vector<EdgeCacheAssociationTarget> &targets,
                                           ProxyCacheLimits = {});

// A native cache hash occupies exactly 32 bytes. Exported mode 0 uses MD5;
// nonzero modes select SHA-1. The source marker does not prove hash success.
Json decode_native_cache_hash(const Bytes &);
// Native equality compares mode, declared length (at most 20), and only that
// many digest bytes. It ignores marker/padding/unused digest storage. This
// does not regenerate the current runtime hash or validate the whole cache.
Json compare_native_cache_hash(const Bytes &saved, const Bytes &current);
} // namespace p3d
