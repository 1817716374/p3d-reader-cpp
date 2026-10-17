#pragma once
#include <p3d/reader.hpp>

namespace p3d {
struct PropertyBlockLimits {
    std::size_t max_tokens = 1000000;
    std::size_t max_fields = 1000000;
    std::size_t max_array_elements = 1000000;
    std::size_t max_depth = 32;
    std::size_t max_decoded_bytes = 256u * 1024 * 1024;
};
// Reads the native typed property-block archive. Keeps ordered tokens, source
// bytes and an additional field/array structure; does not instantiate schema
// classes, evaluate defaults, assign units or follow entity references.
// Invalid/truncated typed streams and exhausted token/byte budgets throw.
// Unsupported/inconsistent field layouts retain tokens and report a partial
// structure. Field value token indices always refer to that block's tokens.
// Known native binary field contracts also produce binary_value_views, without
// selecting active graphics or resolving entity references. Their parse status
// is separate from the outer wire layout; unknown contracts retain raw values.
// max_decoded_bytes also bounds bytes passed to these nested field decoders.
Json decode_property_data_block(const Bytes &, const PropertyBlockLimits & = {});
} // namespace p3d
