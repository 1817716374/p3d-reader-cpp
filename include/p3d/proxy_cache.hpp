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
} // namespace p3d
