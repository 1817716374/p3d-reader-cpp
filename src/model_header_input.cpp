#include "internal.hpp"
#include <p3d/model_header_input.hpp>

namespace p3d {
Json initial_native_model_header_input(const Bytes &source) {
    Json result = {{"scope", "initial_native_model_header_input"},
                   {"status", "not_evaluated"},
                   {"source_record", rawbytes(source)},
                   {"loaded_header", nullptr},
                   {"loaded_offsets_include_stream_prefix", false}};
    auto unresolved = [&](const char *reason) {
        result["reason"] = reason;
        return result;
    };
    if (source.size() < 500)
        return unresolved("truncated_model_header_record");
    Reader r(source);
    const auto words = r.at<std::uint32_t>(8), base = r.at<std::uint32_t>(12);
    if (std::uint64_t(words) * 2 + 4 != source.size() || base < 248 || base > words)
        return unresolved("invalid_model_header_record_length");
    if (r.at<std::uint16_t>(4) != 47 || r.at<std::uint32_t>(16) != 32)
        return unresolved("unsupported_model_header_record_type");
    const auto version = r.at<std::uint16_t>(36);
    result["source_version"] = version;
    if (version != 7 && version != 8)
        return unresolved("unsupported_legacy_model_header_conversion");
    if (version == 7) {
        // 105020: avoid context-dependent unit defaults and orientation repair.
        // These gates are a supported subset, not native rejection conditions.
        for (auto offset : {0xe4u, 0x16cu, 0x174u}) {
            const auto value = r.at<double>(offset);
            if (!std::isfinite(value) || value <= 0)
                return unresolved("legacy_scale_repair_requires_context");
        }
        for (unsigned i = 0; i < 9; ++i)
            if (r.at<double>(0x124 + 8 * i) != (i % 4 == 0 ? 1. : 0.))
                return unresolved("legacy_orientation_repair_not_evaluated");
    }
    auto loaded = slice(source, 4, source.size() - 4);
    if (version == 7) {
        // 1052c4 and 105615. Offsets here exclude the stream prefix.
        const float one = 1;
        std::memcpy(loaded.data() + 0x30, &one, sizeof one);
        loaded[0x20] = 8;
        loaded[0x21] = 0;
    }
    result.update({{"status", "resolved"}, {"loaded_version", 8},
                   {"conversion", version == 8 ? "current_header_copy" : "version_7_supported_subset"},
                   {"loaded_header", rawbytes(loaded)},
                   {"spatial", bool(r.at<std::uint32_t>(72) & 1)}});
    return result;
}
} // namespace p3d
