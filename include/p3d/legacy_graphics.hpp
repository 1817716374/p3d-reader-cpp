#pragma once
#include <p3d/reader.hpp>

namespace p3d {
// The installed native writer and reader disagree on nonempty FloatRgb and
// edge-chain storage. Select a contract explicitly; this is not auto-detection.
enum class LegacyGraphicsProfile { writer_2025, reader_2025 };
struct LegacyGraphicsLimits {
    std::size_t max_input_bytes = 256u * 1024 * 1024;
    std::size_t max_entries = 1000000;
    std::size_t max_values = 10000000;
    std::size_t max_work = 10000000;
    std::size_t max_depth = 32;
    std::size_t max_string_units = 1000000;
    std::size_t max_interpretations = 16;
};
// Decodes the version-3.1 graphics archive, not BGFB. Preserves entry order,
// source storage and every supported mesh channel. Unknown kinds/layouts are
// not skipped. Status is parsed, ambiguous or not_parsed; parsed requires a
// unique full-buffer interpretation and a completed search. Ambiguous outputs
// expose hypotheses instead of choosing one. Limits apply across all attempts.
// Direct fields include curves/Bsplines, nine solid parameter layouts and text.
// Nested CSG has independent member decode statuses. Known native omissions
// report geometry_status=not_serialized; a parsed archive is not proof that
// every source geometry exists or is valid. Input too large omits source_storage.
// Source-cache decoding does not select active geometry or bind materials.
Json decode_legacy_graphics(const Bytes &,
                            LegacyGraphicsProfile = LegacyGraphicsProfile::writer_2025,
                            const LegacyGraphicsLimits & = {});
} // namespace p3d
