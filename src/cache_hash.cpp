#include "internal.hpp"
#include <p3d/proxy_cache.hpp>

namespace p3d {
Json decode_native_cache_hash(const Bytes &bytes) {
    Json out = {{"status", "not_evaluated"}, {"scope", "native_cache_hash_storage"}};
    try {
        require(bytes.size() == 32, "cache_hash_requires_32_bytes");
        Reader r(bytes);
        const auto marker = r.u8();
        const auto padding = r.take(3);
        const auto mode = r.u32(), size = r.u32();
        const auto data = r.take(20);
        out.update({{"status", "decoded"},
                    {"source_marker", marker},
                    {"padding", rawbytes(padding)},
                    {"mode", mode},
                    {"algorithm", mode == 0 ? "md5" : "sha1"},
                    {"algorithm_id", mode == 0 ? 0x8003 : 0x8004},
                    {"declared_digest_bytes", size},
                    {"expected_algorithm_bytes", mode == 0 ? 16 : 20},
                    {"size_matches_algorithm", size == (mode == 0 ? 16u : 20u)},
                    {"digest_storage", rawbytes(data)},
                    {"digest", nullptr},
                    {"unused_digest_storage", nullptr},
                    {"hash_generation_status", "not_evaluated"}});
        if (size <= 20) {
            out["digest_status"] = "decoded";
            out["digest"] = rawbytes(slice(data, 0, size));
            out["digest_hex"] = hex(slice(data, 0, size));
            out["unused_digest_storage"] = rawbytes(slice(data, size, 20 - size));
        } else {
            out["digest_status"] = "declared_size_exceeds_storage";
        }
        // The native writer sets the marker before querying the hash. Neither
        // this marker nor a conventional length proves generation succeeded.
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json compare_native_cache_hash(const Bytes &saved, const Bytes &current) {
    Json out = {{"status", "not_evaluated"},
                {"scope", "native_cache_hash_comparison_only"},
                {"match", nullptr},
                {"cache_validity_status", "not_evaluated"}};
    try {
        require(saved.size() == 32 && current.size() == 32, "cache_hash_requires_32_bytes");
        auto result = [&](bool match, const char *reason) {
            out.update({{"status", "resolved"}, {"match", match}, {"reason", reason}});
            return out;
        };
        if (Reader(saved, 4).u32() != Reader(current, 4).u32())
            return result(false, "mode_mismatch");
        const auto size = Reader(saved, 8).u32();
        if (size != Reader(current, 8).u32())
            return result(false, "digest_size_mismatch");
        if (size > 20)
            return result(false, "digest_size_exceeds_storage");
        for (std::uint32_t i = 0; i < size; ++i)
            if (saved[12 + i] != current[12 + i])
                return result(false, "digest_mismatch");
        return result(true, "compared_fields_equal");
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
