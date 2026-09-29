#pragma once
#include <p3d/reader.hpp>
#include <array>
#include <optional>

namespace p3d {
// Persisted extended-header fields only. Input includes the four-byte stream
// prefix and ends at the base boundary; linkage bytes cannot supply the range.
// These fields alone do not establish runtime inclusion or a model range.
Json decode_native_record_bounds_header(const Bytes &record_base);

struct NativeModelBoundsRecord {
    // Loaded/prepared record bytes, including a four-byte prefix. Callers must
    // account for input conversion and registration before using source bytes.
    Bytes record_base;
    std::optional<std::uint32_t> runtime_flags_10;
    // Result of the original ordered pointer-key lookup for RVA 643e52.
    // Known absence/null value selects the shifted header range; unknown lookup
    // is not absence. Present range is the six int64 values at value+8.
    bool override_lookup_known = false;
    std::optional<std::array<std::int64_t,6>> override_range;
};
struct NativeModelBoundsProviderInput {
    std::vector<NativeModelBoundsRecord> records;
    // Flattened model+158 list blocks, retaining order and repeated identities.
    std::vector<std::size_t> record_indices;
    bool record_list_complete = false;
    // Controls per-record validity, independently of final model byte +0x78.
    std::optional<bool> model_query_68;
    std::size_t max_record_visits = 1000000;
};
struct NativeModelBoundsProviderResult {
    bool resolved = false;
    std::string reason;
    std::optional<std::size_t> failed_record;
    std::optional<std::array<std::int64_t,6>> integer_range;
    std::vector<std::size_t> included_record_indices;
};
// R1.18 initial df270/deeb0 provider population. Computes its numeric range,
// not the spatial index tree/capacity or later insert/delete/dirty refresh.
// Feed a resolved integer_range to project_native_cached_model_bounds with the
// separately known model spatial_byte_78; then union child/reference models.
NativeModelBoundsProviderResult project_native_model_bounds_provider(
    const NativeModelBoundsProviderInput &input);
} // namespace p3d
