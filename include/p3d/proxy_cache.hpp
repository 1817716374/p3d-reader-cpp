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
} // namespace p3d
